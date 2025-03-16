#include "adisfileloader.hpp"
#include "common/angle.h"
#include "common/earth.h"
#include "fileio/fileloader.h"
#include <cstdio>
#include <sstream>
#include <string>
AdisFileLoader::AdisFileLoader(const string &filename, int columns, int rate) {
    open(filename, columns_, FileLoader::TEXT);
    dt_       = 1.0 / (double) rate;
    imu_.time = 0;
}
const IMU &AdisFileLoader::next() {
    imu_pre_ = imu_;
    if (AdisFileLoader::load_()) {
        imu_.week = temper.week;
        imu_.time = temper.tow;
        double dt = imu_.time - imu_pre_.time;
        if (dt < 0.1) {
            imu_.dt = dt;
        } else {
            imu_.dt = dt_;
        }
        imu_.dtheta << temper.gyr[1] * D2R, temper.gyr[0] * D2R, -temper.gyr[2] * D2R;
        imu_.dvel << temper.acc[1], temper.acc[0], -temper.acc[2];
    }
    return imu_;
}
bool AdisFileLoader::load_() {
    if (isEof())
        return false;
    string line;
    std::getline(filefp_, line);
    sscanf(line.c_str(), "%c,%lf,%d,%d,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%d,%d", &temper.type, &temper.tow, &temper.week,
           &temper.leap, &temper.temprature, &temper.gyr[0], &temper.gyr[1], &temper.gyr[2], &temper.acc[0],
           &temper.acc[1], &temper.acc[2], &temper.stamp, &temper.cap);
    return true;
}