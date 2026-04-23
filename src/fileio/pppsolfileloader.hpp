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

class PppsolFileLoader : public IGnssFileLoader {
public:
    PppsolFileLoader() = delete;
    explicit PppsolFileLoader(const string &filename, int columns = 25) {
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

            if (temper.status >= 3 && temper.status <= 4 && temper.nsat > 5) {
                gnss_.isPosValid = true;
                gnss_.isVelValid = true;
            } else {
                gnss_.isPosValid = false;
                gnss_.isVelValid = false;
            }
        }
        return gnss_;
    }

private:
    GNSS gnss_;

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

    void parse_utc_time(const std::string &datetime) {
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
        t += 18.0;

        temper.week = (int) (t / 604800.0);
        temper.sow  = t - temper.week * 604800.0;
    }

    bool load_() {
        std::string line;
        bool pppsol_found = false;

        while (true) {
            if (filefp_.eof())
                break;
            std::getline(filefp_, line);
            if (line.empty())
                continue;

            if (!line.empty() && line.back() == '\r')
                line.pop_back();

            size_t pos = line.find("$PPPSOL");
            if (pos != std::string::npos) {
                std::string pppsol_line         = line.substr(pos);
                std::vector<std::string> tokens = split(pppsol_line, ',');
                if (tokens.size() >= 17) {
                    pppsol_found = true;

                    parse_utc_time(tokens[1]);

                    if (!tokens[2].empty())
                        temper.status = std::stoi(tokens[2]);
                    if (!tokens[3].empty())
                        temper.nsat = std::stoi(tokens[3]);

                    if (!tokens[4].empty())
                        temper.blh[1] = std::stod(tokens[4]); // Lon
                    if (!tokens[5].empty())
                        temper.std[1] = std::stod(tokens[5]); // East STD

                    if (!tokens[6].empty())
                        temper.blh[0] = std::stod(tokens[6]); // Lat
                    if (!tokens[7].empty())
                        temper.std[0] = std::stod(tokens[7]); // North STD

                    if (!tokens[8].empty())
                        temper.blh[2] = std::stod(tokens[8]); // Height
                    if (!tokens[9].empty())
                        temper.std[2] = std::stod(tokens[9]); // Height STD

                    if (!tokens[10].empty())
                        temper.undulation = std::stod(tokens[10]); // Undulation

                    if (!tokens[11].empty())
                        temper.vel[1] = std::stod(tokens[11]); // Vel E
                    if (!tokens[12].empty())
                        temper.vstd[1] = std::stod(tokens[12]); // Vel STD E

                    if (!tokens[13].empty())
                        temper.vel[0] = std::stod(tokens[13]); // Vel N
                    if (!tokens[14].empty())
                        temper.vstd[0] = std::stod(tokens[14]); // Vel STD N

                    if (!tokens[15].empty())
                        temper.vel[2] = std::stod(tokens[15]); // Vel U
                    if (!tokens[16].empty())
                        temper.vstd[2] = std::stod(tokens[16]); // Vel STD U

                    return true;
                }
            }
        }
        return pppsol_found;
    }
};
