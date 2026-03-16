#pragma once
#include "common/angle.h"
#include "common/types.h"
#include "fileio/adisfileloader.hpp"
class Asm330FileLoader : public AdisFileLoader {
public:
    Asm330FileLoader() = delete;
    explicit Asm330FileLoader(const string &filename, int columns = 13, int rate = 10)
        : AdisFileLoader(filename, columns, rate) {
        dt_ = 1.0 / sample_rate;
    }
    double starttime() {

        double starttime;
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
        imu_pre_ = imu_;
        if (Asm330FileLoader::load_()) {
            imu_.week = temper.week;
            imu_.time = temper.tow + 18.0; // leap seconds
            imu_.dt   = dt_;
            imu_.dtheta << temper.gyr[1] * D2R, temper.gyr[0] * D2R, -temper.gyr[2] * D2R;
            imu_.dvel << temper.acc[1], temper.acc[0], -temper.acc[2];
            imu_.accel << temper.acc[1] * sample_rate, temper.acc[0] * sample_rate, -temper.acc[2] * sample_rate;
            imu_.omega << temper.gyr[1] * D2R * sample_rate, temper.gyr[0] * D2R * sample_rate,
                -temper.gyr[2] * D2R * sample_rate;
        }
        return imu_;
    }

private:
    double dt_;
    const double sample_rate = (208.0 / 21.0);
    bool load_() {
        if (isEof())
            return false;
        string line;
        std::getline(filefp_, line);
        sscanf(line.c_str(), "%c,%d,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf", &temper.type, &temper.week, &temper.tow,
               &temper.temprature, &temper.acc[0], &temper.acc[1], &temper.acc[2], &temper.gyr[0], &temper.gyr[1],
               &temper.gyr[2], &temper.bt);

        return true;
    }
    struct temp {
        char type;
        int week;
        double tow;
        double temprature;
        double gyr[3];
        double acc[3];
        double bt;
    } temper;
    IMU imu_, imu_pre_;
};
