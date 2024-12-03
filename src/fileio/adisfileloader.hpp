#pragma once
#include "common/types.h"
#include "fileio/fileloader.h"
class AdisFileLoader : public FileLoader {
public:
    AdisFileLoader() = delete;
    explicit AdisFileLoader(const string &filename, int columns = 13, int rate = 100);
    const IMU &next();
    double starttime() {

        double starttime;
        std::streampos sp = filefp_.tellg();

        filefp_.seekg(0, std::ios_base::beg);
        // starttime = load().front();
        if (AdisFileLoader::load_()) {
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
        if (AdisFileLoader::load_()) {
            endtime = temper.tow;
        }
        filefp_.seekg(sp, std::ios_base::beg);
        return endtime;
    }

private:
    IMU imu_, imu_pre_;
    double dt_;
    bool load_();
    struct temp {
        char type;
        int week;
        double tow;
        int leap;
        double temprature;
        double gyr[3];
        double acc[3];
        int stamp;
        int cap;
    } temper;
};