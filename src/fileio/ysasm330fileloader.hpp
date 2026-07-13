#pragma once

#include "common/angle.h"
#include "common/types.h"
#include "fileio/fileloader.h"
#include "fileio/sensors_provider.hpp"

#include <cstdio>

#define YSASM330_DPS 1

class YsAsm330FileLoader : public IImuFileLoader {
public:
    YsAsm330FileLoader() = delete;
    explicit YsAsm330FileLoader(const string &filename, int columns = 9, int rate = 110) {
        open(filename, columns, FileLoader::TEXT);
        dt_ = 1.0 / static_cast<double>(rate);
        resetImu(imu_);
        resetImu(imu_pre_);
    }

    double starttime() {
        double starttime  = -1.0;
        std::streampos sp = filefp_.tellg();

        filefp_.seekg(0, std::ios_base::beg);
        if (YsAsm330FileLoader::load_()) {
            starttime = temper.sow;
        }
        filefp_.seekg(sp, std::ios_base::beg);
        return starttime;
    }

    double endtime() {
        double endtime    = -1.0;
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

        if (YsAsm330FileLoader::load_()) {
            endtime = temper.sow;
        }
        filefp_.seekg(sp, std::ios_base::beg);
        return endtime;
    }

    const IMU &next() {
        while (YsAsm330FileLoader::load_()) {
            double time = temper.sow;
            // time += 18;
            // time --;
            double dt = has_valid_imu_ ? time - imu_.time : dt_;
            if (has_valid_imu_ && (dt <= 0.0 || dt > 2.1 * dt_)) {
                continue;
            }

            imu_pre_  = imu_;
            imu_.week = temper.week;
            imu_.time = time;
            imu_.dt   = dt;
#ifdef YSASM330_DPS
            imu_.accel << temper.acc[0] * 9.80665, temper.acc[1] * 9.80665, temper.acc[2] * 9.80665;
            imu_.omega << temper.gyr[0] * D2R, temper.gyr[1] * D2R, temper.gyr[2] * D2R;
#else
            imu_.accel << temper.acc[0], temper.acc[1], temper.acc[2];
            imu_.omega << temper.gyr[0], temper.gyr[1], temper.gyr[2];
#endif
            imu_.dtheta = imu_.omega * imu_.dt;
            imu_.dvel   = imu_.accel * imu_.dt;

            has_valid_imu_ = true;
            return imu_;
        }

        return imu_;
    }

private:
    struct Temp {
        int week   = 0;
        double sow = 0.0;
        double gyr[3]{};
        double acc[3]{};
        double temperature = 0.0;
    } temper;

    double dt_          = 0.0;
    bool has_valid_imu_ = false;
    IMU imu_, imu_pre_;

    void resetImu(IMU &imu) {
        imu.week = 0;
        imu.time = -1.0;
        imu.dt   = 0.0;
        imu.dtheta.setZero();
        imu.dvel.setZero();
        imu.accel.setZero();
        imu.omega.setZero();
        imu.odovel = 0.0;
    }

    bool load_() {
        string line;
        while (std::getline(filefp_, line)) {
            if (line.empty()) {
                continue;
            }

            Temp parsed{};
            int n = sscanf(line.c_str(), "%d,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf", &parsed.week, &parsed.sow,
                           &parsed.gyr[0], &parsed.gyr[1], &parsed.gyr[2], &parsed.acc[0], &parsed.acc[1],
                           &parsed.acc[2], &parsed.temperature);
            if (n != 9) {
                continue;
            }

            temper = parsed;
            return true;
        }

        return false;
    }
};
