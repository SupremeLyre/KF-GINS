#pragma once
#include "common/angle.h"
#include "common/crc32.h"
#include "common/types.h"
#include "fileio/fileloader.h"
#include <array>
#include <cstdint>
#include <fstream>
#include <string>
class AdisBinaryLoader : public FileLoader {
public:
    AdisBinaryLoader() = delete;
    explicit AdisBinaryLoader(const string &filename, int columns = 13, int rate = 100) {
        open(filename, columns, FileLoader::BINARY);
        dt_       = 1.0 / (double) rate;
        imu_.time = 0;
    }
    double starttime() {

        double starttime;
        std::streampos sp = filefp_.tellg();

        filefp_.seekg(0, std::ios_base::beg);
        // starttime = load().front();
        if (AdisBinaryLoader::load_()) {
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
            filefp_.seekg(-BUFFER_SIZE, std::ios_base::end);
        }
        // endtime = load().front();
        if (AdisBinaryLoader::load_()) {
            endtime = temper.tow;
        }
        filefp_.seekg(sp, std::ios_base::beg);
        return endtime;
    }
    const IMU &next() {
        imu_pre_ = imu_;
        if (load_()) {
            imu_.week = temper.week;
            imu_.time = temper.tow;
            double dt = imu_.time - imu_pre_.time;
            if (dt > 0.003) {
                imu_.dt = dt;
            } else {
                imu_.dt = dt_;
            }
            imu_.dtheta << temper.gyr[1] * D2R * imu_.dt, temper.gyr[0] * D2R * imu_.dt, -temper.gyr[2] * D2R * imu_.dt;
            imu_.dvel << temper.acc[1] * 9.80665 * imu_.dt, temper.acc[0] * 9.80665 * imu_.dt,
                -temper.acc[2] * 9.80665 * imu_.dt;
            imu_.accel << temper.acc[1] * 9.80665, temper.acc[0] * 9.80665, -temper.acc[2] * 9.80665;
            imu_.omega << temper.gyr[1] * D2R, temper.gyr[0] * D2R, -temper.gyr[2] * D2R;
        }
        return imu_;
    }

private:
    double dt_;
    IMU imu_;
    IMU imu_pre_;
    const size_t BUFFER_SIZE = 1 + 8 + 2 + 2 + 7 * 8 + 2 * 4 + 4;
    bool load_() {
        bool stat = false;
        // 用filefp_找0x49
        uint8_t buffer[BUFFER_SIZE + 1];
        filefp_.read((char *) buffer, sizeof(uint8_t));
        size_t offset = 1;
        if (buffer[0] == 0x49) {
            filefp_.read((char *) buffer + offset, sizeof(double));
            memcpy(&temper.tow, (char *) buffer + offset, sizeof(double));
            offset += sizeof(double);
            filefp_.read((char *) buffer + offset, sizeof(uint16_t));
            memcpy(&temper.week, (char *) buffer + offset, sizeof(uint16_t));
            offset += sizeof(uint16_t);
            filefp_.read((char *) buffer + offset, sizeof(uint16_t));
            memcpy(&temper.leap, (char *) buffer + offset, sizeof(uint16_t));
            offset += sizeof(uint16_t);
            filefp_.read((char *) buffer + offset, sizeof(double));
            memcpy(&temper.temperature, (char *) buffer + offset, sizeof(double));
            offset += sizeof(double);
            filefp_.read((char *) buffer + offset, sizeof(double));
            memcpy(&temper.acc[0], (char *) buffer + offset, sizeof(double));
            offset += sizeof(double);
            filefp_.read((char *) buffer + offset, sizeof(double));
            memcpy(&temper.acc[1], (char *) buffer + offset, sizeof(double));
            offset += sizeof(double);
            filefp_.read((char *) buffer + offset, sizeof(double));
            memcpy(&temper.acc[2], (char *) buffer + offset, sizeof(double));
            offset += sizeof(double);
            filefp_.read((char *) buffer + offset, sizeof(double));
            memcpy(&temper.gyr[0], (char *) buffer + offset, sizeof(double));
            offset += sizeof(double);
            filefp_.read((char *) buffer + offset, sizeof(double));
            memcpy(&temper.gyr[1], (char *) buffer + offset, sizeof(double));
            offset += sizeof(double);
            filefp_.read((char *) buffer + offset, sizeof(double));
            memcpy(&temper.gyr[2], (char *) buffer + offset, sizeof(double));
            offset += sizeof(double);
            filefp_.read((char *) buffer + offset, sizeof(int));
            memcpy(&temper.ts, (char *) buffer + offset, sizeof(int));
            offset += sizeof(int);
            filefp_.read((char *) buffer + offset, sizeof(int));
            memcpy(&temper.tt, (char *) buffer + offset, sizeof(int));
            offset += sizeof(int);
            filefp_.read((char *) buffer + offset, sizeof(uint32_t));
            memcpy(&temper.crc, (char *) buffer + offset, sizeof(uint32_t));
            // 校验crc
            uint32_t file_crc = crc32(&buffer[1], BUFFER_SIZE - 1 - 4, 0);
            if (file_crc == temper.crc) {
                stat = true;
            }
        }
        return stat;
    }
    struct temper {
        double tow;
        uint16_t week;
        uint16_t leap;
        double temperature;
        std::array<double, 3> acc;
        std::array<double, 3> gyr;
        int ts;
        int tt;
        uint32_t crc;
    } temper;
};