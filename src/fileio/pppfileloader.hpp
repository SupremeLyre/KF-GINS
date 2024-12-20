#pragma once
#include "common/angle.h"
#include "common/types.h"
#include "fileio/fileloader.h"
#include <vector>
class PPPFileLoader : public FileLoader {
public:
    PPPFileLoader() = delete;
    explicit PPPFileLoader(const string &filename, int columns = 31) {
        open(filename, columns, FileLoader::TEXT);
    }
    const GNSS &next() {

        if (PPPFileLoader::load_()) {
            gnss_.week = temper.week;
            gnss_.time = temper.tow;
            gnss_.blh << temper.blh[0], temper.blh[1], temper.blh[2];
            gnss_.std << temper.std_ned[0], temper.std_ned[1], temper.std_ned[2];
            gnss_.vel << temper.vned[0], temper.vned[1], temper.vned[2];
            gnss_.vstd << temper.std_vned[0], temper.std_vned[1], temper.std_vned[2];
        }
        return gnss_;
    }

private:
    GNSS gnss_;
    vector<double> data_;
    struct temp {
        int week;
        double tow;
        string time_str[2];
        double xyz[3];
        double blh[3];
        int stat;
        int ns;
        double enu[3];
        double age;
        double std_xyz[3];
        double vxyz[3];
        double std_vxyz[3];
        double acc[3];
        int stat_dpos;
        double dpos[3];
        double qdpos[3];
        double std_ned[3];
        double std_vned[3];
        double vned[3];
        string time_acc;
    } temper;
    bool load_();
};