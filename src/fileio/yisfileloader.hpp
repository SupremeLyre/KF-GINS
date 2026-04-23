#pragma once
#include "absl/strings/str_split.h"
#include "common/angle.h"
#include "common/time.hpp"
#include "common/types.h"
#include "fileio/fileloader.h"
#include "fileio/sensors_provider.hpp"
class YisFileLoader : public IImuFileLoader {
public:
    YisFileLoader() = delete;
    explicit YisFileLoader(const string &filename, int columns = 22, int rate = 100) {
        open(filename, columns_, FileLoader::TEXT);
        dt_       = 1.0 / (double) rate;
        imu_.time = 0;
    }
    const IMU &next() {
        imu_pre_ = imu_;
        if (YisFileLoader::load_()) {
            imu_.week = temper.week;
            imu_.time = temper.tow;
            double dt = imu_.time - imu_pre_.time;
            imu_.dt = dt_;
#if 1
            imu_.dtheta << temper.gyr[0] * D2R * imu_.dt, -temper.gyr[1] * D2R * imu_.dt,
                -temper.gyr[2] * D2R * imu_.dt;
            imu_.dvel << temper.acc[0] * imu_.dt, -temper.acc[1] * imu_.dt, -temper.acc[2] * imu_.dt;
            imu_.accel << temper.acc[0], -temper.acc[1], -temper.acc[2];
            imu_.omega << temper.gyr[0] * D2R, -temper.gyr[1] * D2R, -temper.gyr[2] * D2R;
#else
            imu_.dtheta << temper.gyr[1] * D2R, temper.gyr[0] * D2R, -temper.gyr[2] * D2R;
            imu_.dvel << temper.acc[1], temper.acc[0], -temper.acc[2];
#endif
        }
        return imu_;
    }
    double starttime() {

        double starttime;
        std::streampos sp = filefp_.tellg();

        filefp_.seekg(0, std::ios_base::beg);
        // starttime = load().front();
        if (YisFileLoader::load_()) {
            starttime = temper.tow;
        }
        filefp_.seekg(sp, std::ios_base::beg);
        return starttime;
    }

    double endtime() {

        double endtime    = -1;
        std::streampos sp = filefp_.tellg();

        if (filetype_ == TEXT) {
            filefp_.seekg(-2, std::ios_base::end);
            char byte = 0;
            auto pos  = filefp_.tellg();
            do {
                pos -= 1;
                filefp_.seekg(pos);
                filefp_.read(&byte, 1);
            } while (byte != '\n');
        } else {
            filefp_.seekg(-columns_ * sizeof(double), std::ios_base::end);
        }
        // endtime = load().front();
        if (YisFileLoader::load_()) {
            endtime = temper.tow;
        }
        filefp_.seekg(sp, std::ios_base::beg);
        return endtime;
    }

private:
    IMU imu_, imu_pre_;
    double dt_;
    bool load_() {
        if (isEof())
            return false;
        string line;
        std::getline(filefp_, line);
        std::stringstream iss(line);
        char comma;
        iss >> temper.tid >> comma >> temper.acc[0] >> comma >> temper.acc[1] >> comma >> temper.acc[2] >> comma >>
            temper.gyr[0] >> comma >> temper.gyr[1] >> comma >> temper.gyr[2] >> comma >> temper.rpy[0] >> comma >>
            temper.rpy[1] >> comma >> temper.rpy[2] >> comma >> temper.qbn[0] >> comma >> temper.qbn[1] >> comma >>
            temper.qbn[2] >> comma >> temper.qbn[3] >> comma >> temper.blh[0] >> comma >> temper.blh[1] >> comma >>
            temper.blh[2] >> comma >> temper.vn[0] >> comma >> temper.vn[1] >> comma >> temper.vn[2] >> comma >>
            temper.time_str[0] >> temper.time_str[1] >> comma >> temper.status >> comma;
        vector<double> ep;
        vector<string> splits = absl::StrSplit(temper.time_str[0], absl::ByAnyChar("/ :,-"), absl::SkipWhitespace());
        for (const auto &s : splits) {
            ep.push_back(std::stod(s));
        }
        splits.clear();
        splits = absl::StrSplit(temper.time_str[1], absl::ByAnyChar("/ :"), absl::SkipWhitespace());
        for (auto &split : splits) {
            ep.push_back(strtod(split.data(), nullptr));
        }
        ep[0] += 2000;
        int week{0};
        double tow{0.0};
        bool stat{Time::epoch2gpst(ep, week, tow)};
        tow += 18.0;
        tow -= 8 * 3600;
        if (!stat) {
            return false;
        }
        temper.week = week;
        temper.tow  = tow;

        return true;
    }
    struct temp {
        int tid;
        double acc[3];
        double gyr[3];
        double rpy[3];
        double qbn[4];
        double blh[3];
        double vn[3];
        string time_str[2];
        double tow;
        int week;
        int status;
    } temper;
};