#pragma once
#include "common/angle.h"
#include "common/earth.h"
#include "common/types.h"
#include "fileio/fileloader.h"
#include "fileio/sensors_provider.hpp"
#include <array>
#include <cmath>
#include <exception>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

struct PppsolOptions {
    Eigen::Vector3d spp_pos_std_scale = Eigen::Vector3d::Ones();
    double spp_vel_std_scale          = 1.0;
    Eigen::Vector3d spp_pos_std_max = Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity());
    double spp_vel_std_min          = 0.0;
    int spp_recovery_confirm_epochs = 1;
    bool spp_pos_std_gate_enabled   = false;
    bool spp_vel_std_gate_enabled   = false;
};

class PppsolFileLoader : public IGnssFileLoader {
public:
    PppsolFileLoader() = delete;
    explicit PppsolFileLoader(const string &filename, int columns = 25, PppsolOptions options = {})
        : options_(options) {
        invalidateGnss();
        open(filename, columns, FileLoader::TEXT);
    }
    struct temper {
        double sow = 0.0;
        int week   = 0;
        int status = 0;
        int nsat   = 0;
        std::array<double, 3> blh{};
        std::array<double, 3> std{};
        std::array<double, 3> vel{};
        std::array<double, 3> vstd{};
        double undulation = 0.0;
    } temper;
    const GNSS &next() {
        if (!load_()) {
            invalidateGnss();
            return gnss_;
        }

        gnss_.week = temper.week;
        gnss_.time = temper.sow;
        gnss_.blh  = {temper.blh[0] * D2R, temper.blh[1] * D2R, temper.blh[2] + temper.undulation};
        gnss_.std  = {temper.std[0], temper.std[1], temper.std[2]};
        gnss_.vel  = {temper.vel[0], temper.vel[1], -temper.vel[2]};
        gnss_.vstd = {fabs(temper.vstd[0]), fabs(temper.vstd[1]), fabs(temper.vstd[2])};

        if (temper.status == 3) {
            gnss_.std = gnss_.std.cwiseProduct(options_.spp_pos_std_scale);
            gnss_.vstd *= options_.spp_vel_std_scale;
        }

        if (temper.status == 4 && temper.nsat > 5) {
            gnss_.isPosValid = true;
            gnss_.isVelValid = true;
            resetSppRecovery();
        } else if (temper.status == 3 && temper.nsat > 5) {
            updateSppValidity();
        } else {
            gnss_.isPosValid = false;
            gnss_.isVelValid = false;
            resetSppRecovery();
        }
        gnss_.isvalid = gnss_.isPosValid || gnss_.isVelValid;
        return gnss_;
    }

private:
    GNSS gnss_;
    PppsolOptions options_;
    double last_spp_epoch_ = -1.0;
    int spp_pos_good_epochs_ = 0;
    int spp_vel_good_epochs_ = 0;

    void resetSppRecovery() {
        last_spp_epoch_ = -1.0;
        spp_pos_good_epochs_ = 0;
        spp_vel_good_epochs_ = 0;
    }

    void updateSppValidity() {
        double epoch = gnss_.week * 604800.0 + gnss_.time;
        double dt = epoch - last_spp_epoch_;
        if (last_spp_epoch_ < 0.0 || dt <= 0.0 || dt > 1.5) {
            spp_pos_good_epochs_ = 0;
            spp_vel_good_epochs_ = 0;
        }
        last_spp_epoch_ = epoch;

        bool pos_quality_ok = true;
        bool vel_quality_ok = true;
        for (int i = 0; i < 3; ++i) {
            if (options_.spp_pos_std_gate_enabled) {
                pos_quality_ok = pos_quality_ok && std::isfinite(temper.std[i]) && temper.std[i] > 0.0 &&
                                 temper.std[i] <= options_.spp_pos_std_max[i];
            }
            if (options_.spp_vel_std_gate_enabled) {
                vel_quality_ok = vel_quality_ok && std::isfinite(temper.vstd[i]) &&
                                 fabs(temper.vstd[i]) >= options_.spp_vel_std_min;
            }
        }

        spp_pos_good_epochs_ = pos_quality_ok ? spp_pos_good_epochs_ + 1 : 0;
        spp_vel_good_epochs_ = vel_quality_ok ? spp_vel_good_epochs_ + 1 : 0;
        gnss_.isPosValid = spp_pos_good_epochs_ >= options_.spp_recovery_confirm_epochs;
        gnss_.isVelValid = spp_vel_good_epochs_ >= options_.spp_recovery_confirm_epochs;
    }

    void invalidateGnss() {
        gnss_.week = 0;
        gnss_.time = -1.0;
        gnss_.blh.setZero();
        gnss_.std.setZero();
        gnss_.vel.setZero();
        gnss_.vstd.setZero();
        gnss_.isvalid       = false;
        gnss_.isPosValid    = false;
        gnss_.isVelValid    = false;
    }

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

    bool parse_utc_time(const std::string &datetime) {
        if (datetime.size() < 14) {
            return false;
        }

        try {
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
        } catch (const std::exception &) {
            return false;
        }
        return true;
    }

    bool load_() {
        std::string line;

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
                    temper = {};
                    bool has_required_fields = true;
                    for (size_t i = 1; i <= 16; ++i) {
                        if (tokens[i].empty()) {
                            has_required_fields = false;
                            break;
                        }
                    }
                    if (!has_required_fields) {
                        continue;
                    }
                    if (!parse_utc_time(tokens[1])) {
                        continue;
                    }

                    try {
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
                    } catch (const std::exception &) {
                        temper = {};
                        continue;
                    }

                    return true;
                }
            }
        }
        return false;
    }
};
