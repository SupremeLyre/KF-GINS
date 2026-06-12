#pragma once
#include "common/angle.h"
#include "common/earth.h"
#include "common/types.h"
#include "fileio/fileloader.h"
#include "fileio/sensors_provider.hpp"
#include <array>
#include <cmath>
#include <exception>
#include <sstream>
#include <string>
#include <vector>

class KsxtFileLoader : public IGnssFileLoader {
public:
    KsxtFileLoader() = delete;
    explicit KsxtFileLoader(const string &filename, int columns = 19) {
        invalidateGnss();
        open(filename, columns, FileLoader::TEXT);
    }
    struct temper {
        double sow        = 0.0;
        double utc_daysec = 0.0;
        int week          = 0;
        int status        = 0;
        int nsat          = 0;
        std::array<double, 3> blh{};
        std::array<double, 3> std{1.0, 1.0, 2.0};
        std::array<double, 3> vel{};
        std::array<double, 3> vstd{0.3, 0.3, 0.5};
        double undulation = 0.0;
        bool has_vel      = false;

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

        bool solution_valid = temper.status > 0 && temper.nsat > 8;
        gnss_.isPosValid    = solution_valid;
        gnss_.isVelValid    = solution_valid && temper.has_vel;
        gnss_.isvalid       = gnss_.isPosValid || gnss_.isVelValid;
        return gnss_;
    }

private:
    GNSS gnss_;
    std::string next_line_;
    bool pending_gst_valid_ = false;
    double pending_gst_time_ = 0.0;
    std::array<double, 3> pending_gst_std_{};

    void invalidateGnss() {
        gnss_.week = 0;
        gnss_.time = -1.0;
        gnss_.blh.setZero();
        gnss_.std.setZero();
        gnss_.vel.setZero();
        gnss_.vstd.setZero();
        gnss_.isvalid    = false;
        gnss_.isPosValid = false;
        gnss_.isVelValid = false;
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

    double dm2deg(double dm) {
        int d    = (int) (dm / 100.0);
        double m = dm - d * 100.0;
        return d + m / 60.0;
    }

    bool parse_ksxt_time(const std::string &datetime) {
        if (datetime.size() < 14)
            return false;

        try {
            int y      = std::stoi(datetime.substr(0, 4));
            int m      = std::stoi(datetime.substr(4, 2));
            int d      = std::stoi(datetime.substr(6, 2));
            int h      = std::stoi(datetime.substr(8, 2));
            int min    = std::stoi(datetime.substr(10, 2));
            double sec = std::stod(datetime.substr(12));
            double utc_seconds  = secondsFromGpsEpoch(y, m, d, h, min, sec);
            double gpst_seconds = utc_seconds + gpsUtcOffset(y, m, d);

            temper.week       = (int) (gpst_seconds / 604800.0);
            temper.sow        = gpst_seconds - temper.week * 604800.0;
            temper.utc_daysec = h * 3600.0 + min * 60.0 + sec;
        } catch (const std::exception &) {
            return false;
        }
        return true;
    }

    bool parse_gst_time(const std::string &time, double &daysec) {
        if (time.size() < 6)
            return false;

        try {
            int h      = std::stoi(time.substr(0, 2));
            int min    = std::stoi(time.substr(2, 2));
            double sec = std::stod(time.substr(4));
            daysec     = h * 3600.0 + min * 60.0 + sec;
        } catch (const std::exception &) {
            return false;
        }
        return true;
    }

    long julianDay(int year, int month, int day) {
        if (month <= 2) {
            year--;
            month += 12;
        }
        int A = year / 100;
        int B = 2 - A + A / 4;
        return (long) (365.25 * (year + 4716)) + (long) (30.6001 * (month + 1)) + day + B - 1524;
    }

    double secondsFromGpsEpoch(int year, int month, int day, int hour, int minute, double second) {
        constexpr long GPS_EPOCH_JD = 2444245;
        return (julianDay(year, month, day) - GPS_EPOCH_JD) * 86400.0 + hour * 3600.0 + minute * 60.0 + second;
    }

    int gpsUtcOffset(int year, int month, int day) {
        struct LeapSecond {
            int year;
            int month;
            int day;
            int offset;
        };
        static const std::array<LeapSecond, 18> leap_seconds{{
            {1981, 7, 1, 1},  {1982, 7, 1, 2},  {1983, 7, 1, 3},  {1985, 7, 1, 4},  {1988, 1, 1, 5},
            {1990, 1, 1, 6},  {1991, 1, 1, 7},  {1992, 7, 1, 8},  {1993, 7, 1, 9},  {1994, 7, 1, 10},
            {1996, 1, 1, 11}, {1997, 7, 1, 12}, {1999, 1, 1, 13}, {2006, 1, 1, 14}, {2009, 1, 1, 15},
            {2012, 7, 1, 16}, {2015, 7, 1, 17}, {2017, 1, 1, 18},
        }};

        int offset = 0;
        for (const auto &leap : leap_seconds) {
            if (year > leap.year ||
                (year == leap.year && (month > leap.month || (month == leap.month && day >= leap.day)))) {
                offset = leap.offset;
            }
        }
        return offset;
    }

    bool same_epoch_time(double lhs, double rhs) {
        double diff = fabs(lhs - rhs);
        return diff < 0.5 || fabs(diff - 86400.0) < 0.5;
    }

    void apply_gst_std(const std::array<double, 3> &std) {
        temper.std = std;
    }

    bool parse_ksxt_tokens(const std::vector<std::string> &tokens) {
        if (tokens.size() <= 19)
            return false;
        if (tokens[1].empty() || tokens[2].empty() || tokens[3].empty() || tokens[4].empty() || tokens[10].empty() ||
            tokens[12].empty()) {
            return false;
        }

        temper = {};
        if (!parse_ksxt_time(tokens[1]))
            return false;

        try {
            temper.blh[1] = std::stod(tokens[2]);  // Lon
            temper.blh[0] = std::stod(tokens[3]);  // Lat
            temper.blh[2] = std::stod(tokens[4]);  // Height
            temper.status = std::stoi(tokens[10]); // posQual
            temper.nsat   = std::stoi(tokens[12]); // satNumMaster

            // KSXT velocity fields are East, North, Up in km/h.
            constexpr double KMH_TO_MPS = 1.0 / 3.6;
            if (!tokens[17].empty() && !tokens[18].empty() && !tokens[19].empty()) {
                temper.vel[1]  = std::stod(tokens[17]) * KMH_TO_MPS; // East
                temper.vel[0]  = std::stod(tokens[18]) * KMH_TO_MPS; // North
                temper.vel[2]  = std::stod(tokens[19]) * KMH_TO_MPS; // Up
                temper.has_vel = true;
            }
        } catch (const std::exception &) {
            temper = {};
            return false;
        }

        if (pending_gst_valid_ && same_epoch_time(pending_gst_time_, temper.utc_daysec)) {
            apply_gst_std(pending_gst_std_);
            pending_gst_valid_ = false;
        }
        return true;
    }

    bool parse_gst_tokens(const std::vector<std::string> &tokens, double &daysec, std::array<double, 3> &std) {
        if (tokens.size() < 9)
            return false;
        if (!parse_gst_time(tokens[1], daysec))
            return false;

        std::vector<std::string> clean_tokens = tokens;
        std::string last_token                = clean_tokens.back();
        size_t star_pos                       = last_token.find('*');
        if (star_pos != std::string::npos) {
            clean_tokens.back() = last_token.substr(0, star_pos);
        }

        try {
            if (clean_tokens[6].empty() || clean_tokens[7].empty() || clean_tokens[8].empty())
                return false;
            std[0] = std::stod(clean_tokens[6]);
            std[1] = std::stod(clean_tokens[7]);
            std[2] = std::stod(clean_tokens[8]);
        } catch (const std::exception &) {
            return false;
        }
        return true;
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

                // $KSXT,20260108100703.000,114.35629225,30.52830161,21.4361,,,335.97,0.03,,3,0,38,0,205.183,-643.136,-0.588,-0.003,0.007,-0.028,,*01
                if (parse_ksxt_tokens(tokens))
                    ksxt_found = true;
            } else if (line.find("GST") != std::string::npos) {
                double gst_time = 0.0;
                std::array<double, 3> gst_std{};
                if (parse_gst_tokens(tokens, gst_time, gst_std)) {
                    if (ksxt_found && same_epoch_time(gst_time, temper.utc_daysec)) {
                        apply_gst_std(gst_std);
                    } else {
                        pending_gst_time_  = gst_time;
                        pending_gst_std_   = gst_std;
                        pending_gst_valid_ = true;
                    }
                }
            }
        }
        return ksxt_found;
    }
};
