#pragma once
#include "common/angle.h"
#include "common/earth.h"
#include "common/types.h"
#include "fileio/fileloader.h"
#include "fileio/sensors_provider.hpp"
#include <array>
#include <cmath>
#include <sstream>
#include <string>
#include <vector>

class KsxtFileLoader : public IGnssFileLoader {
public:
    KsxtFileLoader() = delete;
    explicit KsxtFileLoader(const string &filename, int columns = 19) {
        open(filename, columns, FileLoader::TEXT);
    }
    struct temper {
        double sow;
        int week;
        int status;
        int nsat;
        std::array<double, 3> blh;
        std::array<double, 3> std;
        std::array<double, 3> vel;
        std::array<double, 3> vstd;
        double undulation;

    } temper;
    const GNSS &next() {
        if (load_()) {
            gnss_.week = temper.week;
            gnss_.time = temper.sow;
            gnss_.blh  = {temper.blh[0] * D2R, temper.blh[1] * D2R, temper.blh[2] + temper.undulation};
            gnss_.std  = {temper.std[0], temper.std[1], temper.std[2]};
            gnss_.vel  = {temper.vel[0], temper.vel[1], -temper.vel[2]};
            gnss_.vstd = {fabs(temper.vstd[0]), fabs(temper.vstd[1]), fabs(temper.vstd[2])};
        }
        if (temper.status > 1 && temper.nsat > 8) {
            gnss_.isPosValid = true;
            gnss_.isVelValid = true;
        } else {
            gnss_.isPosValid = false;
            gnss_.isVelValid = false;
        }
        return gnss_;
    }

private:
    GNSS gnss_;
    std::string next_line_;

    std::vector<std::string> split(const std::string &s, char delimiter) {
        std::vector<std::string> tokens;
        std::string token;
        std::istringstream tokenStream(s);
        while (std::getline(tokenStream, token, delimiter)) {
            tokens.push_back(token);
        }
        if (!s.empty() && s.back() == delimiter) {
            tokens.push_back("");
        }
        return tokens;
    }

    double dm2deg(double dm) {
        int d    = (int) (dm / 100.0);
        double m = dm - d * 100.0;
        return d + m / 60.0;
    }

    void parse_ksxt_time(const std::string &datetime) {
        if (datetime.size() < 14)
            return;

        int y      = std::stoi(datetime.substr(0, 4));
        int m      = std::stoi(datetime.substr(4, 2));
        int d      = std::stoi(datetime.substr(6, 2));
        int h      = std::stoi(datetime.substr(8, 2));
        int min    = std::stoi(datetime.substr(10, 2));
        double sec = std::stod(datetime.substr(12));

        if (m <= 2) {
            y--;
            m += 12;
        }
        int A    = y / 100;
        int B    = 2 - A + A / 4;
        long JD  = (long) (365.25 * (y + 4716)) + (long) (30.6001 * (m + 1)) + d + B - 1524;
        double t = (JD - 2444245.0) * 86400.0 + h * 3600.0 + min * 60.0 + sec;

        // GPS time is ahead of UTC by leap seconds (18s currently)
        // If the time in KSXT is UTC, we should add leap seconds to get GPS time
        t += 18.0;

        temper.week = (int) (t / 604800.0);
        temper.sow  = t - temper.week * 604800.0;
    }

    bool load_() {
        std::string line;
        bool ksxt_found = false;

        while (true) {
            if (!next_line_.empty()) {
                line = next_line_;
                next_line_.clear();
            } else {
                if (filefp_.eof())
                    break;
                std::getline(filefp_, line);
                if (line.empty())
                    continue;
            }
            if (!line.empty() && line.back() == '\r')
                line.pop_back();

            std::vector<std::string> tokens = split(line, ',');
            if (tokens.empty())
                continue;

            if (line.find("KSXT") != std::string::npos) {
                if (ksxt_found) {
                    next_line_ = line;
                    return true;
                }
                ksxt_found = true;

                // $KSXT,20260108100703.000,114.35629225,30.52830161,21.4361,,,335.97,0.03,,3,0,38,0,205.183,-643.136,-0.588,-0.003,0.007,-0.028,,*01
                if (tokens.size() > 19) {
                    parse_ksxt_time(tokens[1]);

                    if (!tokens[2].empty())
                        temper.blh[1] = std::stod(tokens[2]); // Lon
                    if (!tokens[3].empty())
                        temper.blh[0] = std::stod(tokens[3]); // Lat
                    if (!tokens[4].empty())
                        temper.blh[2] = std::stod(tokens[4]); // Height
                    temper.undulation = 0.0; // KSXT doesn't provide undulation, assume height is ready to use

                    if (!tokens[10].empty())
                        temper.status = std::stoi(tokens[10]); // posQual
                    if (!tokens[12].empty())
                        temper.nsat = std::stoi(tokens[12]); // satNumMaster (using master)

                    // Vel in KSXT: velEast, velNorth, velUp (indices 17, 18, 19)
                    // Config temper.vel to be {North, East, Up} so generated gnss_.vel (NED) is correct
                    // gnss_.vel = {temper.vel[0], temper.vel[1], -temper.vel[2]};
                    // So temper.vel[0] = North, temper.vel[1] = East, temper.vel[2] = Up
                    if (!tokens[17].empty()) {
                        temper.vel[1]  = std::stod(tokens[17]);
                        temper.vstd[1] = 0.03;
                    } // East
                    if (!tokens[18].empty()) {
                        temper.vel[0]  = std::stod(tokens[18]);
                        temper.vstd[0] = 0.03;
                    } // North
                    if (!tokens[19].empty()) {
                        temper.vel[2]  = std::stod(tokens[19]);
                        temper.vstd[2] = 0.06;
                    } // Up
                }
            } else if (line.find("GST") != std::string::npos) {
                std::string last_token = tokens.back();
                size_t star_pos        = last_token.find('*');
                if (star_pos != std::string::npos) {
                    tokens.back() = last_token.substr(0, star_pos);
                }
                if (tokens.size() >= 9) {
                    if (!tokens[6].empty())
                        temper.std[0] = std::stod(tokens[6]);
                    if (!tokens[7].empty())
                        temper.std[1] = std::stod(tokens[7]);
                    if (!tokens[8].empty())
                        temper.std[2] = std::stod(tokens[8]);
                }
            }
        }
        return ksxt_found;
    }
};