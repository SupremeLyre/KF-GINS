#pragma once
#include "common/angle.h"
#include "common/types.h"
#include "fileio/adisfileloader.hpp"
#include "fileio/sensors_provider.hpp"
#include <cstdio>
#define ASM330RATE
class Asm330FileLoader : public AdisFileLoader {
public:
    Asm330FileLoader() = delete;
    explicit Asm330FileLoader(const string &filename, int columns = 13, int rate = 10)
        : AdisFileLoader(filename, columns, rate) {
        dt_ = 1.0 / sample_rate;
        resetImu(imu_);
        resetImu(imu_pre_);
    }
    double starttime() {

        double starttime = -1.0;
        std::streampos sp = filefp_.tellg();
        filefp_.seekg(0, std::ios_base::beg);
        // starttime = load().front();
        if (Asm330FileLoader::load_()) {
            starttime = temper.tow + 18.0; // leap seconds
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
        if (Asm330FileLoader::load_()) {
            endtime = temper.tow + 18.0; // leap seconds
        }
        filefp_.seekg(sp, std::ios_base::beg);
        return endtime;
    }
    const IMU &next() {
        while (Asm330FileLoader::load_()) {
            double time = temper.tow + 18.0; // leap seconds
            double dt   = has_valid_imu_ ? time - imu_.time : dt_;
            if (has_valid_imu_ && (dt <= 0.0 || dt > 2.0 * dt_)) {
                continue;
            }

            imu_pre_  = imu_;
            imu_.week = temper.week;
            imu_.time = time;
            imu_.dt   = dt;
#ifndef ASM330RATE
            imu_.dtheta << temper.gyr[1] * D2R, temper.gyr[0] * D2R, -temper.gyr[2] * D2R;
            imu_.dvel << temper.acc[1], temper.acc[0], -temper.acc[2];
            imu_.accel = imu_.dvel / imu_.dt;
            imu_.omega = imu_.dtheta / imu_.dt;
#else
            imu_.accel << temper.acc[1], temper.acc[0], -temper.acc[2];
            imu_.omega << temper.gyr[1] * D2R, temper.gyr[0] * D2R, -temper.gyr[2] * D2R;
            imu_.dtheta = imu_.omega * imu_.dt;
            imu_.dvel   = imu_.accel * imu_.dt;
#endif
            has_valid_imu_ = true;
            return imu_;
        }
        return imu_;
    }

private:
    double dt_;
    bool has_valid_imu_ = false;
#ifdef ASM330MEAN10HZ
    const double sample_rate = (208.0 / 21.0);
#elif defined(ASM330RATE)
    const double sample_rate = 104.0;
#endif
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
            temp parsed{};
            int n = sscanf(line.c_str(), "%c,%d,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf", &parsed.type, &parsed.week,
                           &parsed.tow, &parsed.temprature, &parsed.acc[0], &parsed.acc[1], &parsed.acc[2],
                           &parsed.gyr[0], &parsed.gyr[1], &parsed.gyr[2], &parsed.bt);
            if (n != 11 || parsed.type != 'I') {
                continue;
            }
            temper = parsed;
            return true;
        }

        return false;
    }
    struct temp {
        char type = '\0';
        int week  = 0;
        double tow = 0.0;
        double temprature = 0.0;
        double gyr[3]{};
        double acc[3]{};
        double bt = 0.0;
    } temper;
    IMU imu_, imu_pre_;
};
