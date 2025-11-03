#pragma once
#include "common/angle.h"
#include "common/types.h"
#include "fileio/fileloader.h"
#include <array>
#include <string>
class PvtFileLoader : public FileLoader {
public:
    PvtFileLoader() = delete;
    explicit PvtFileLoader(const string &filename, int columns = 19) {
        open(filename, columns, FileLoader::TEXT);
    }
    struct temper {
        char type;
        double sow;
        int week;
        int leap;
        int status;
        int nsat;
        std::array<double, 3> blh;
        std::array<double, 3> std;
        std::array<double, 3> vel;
        std::array<double, 3> vstd;
        double undulation;

    } temper;
    const GNSS &next() {
        if (load_()) {
            gnss_.week = temper.week;
            gnss_.time = temper.sow;
            gnss_.blh  = {temper.blh[0] * D2R, temper.blh[1] * D2R, temper.blh[2] + temper.undulation};
            gnss_.std  = {temper.std[0], temper.std[1], temper.std[2]};
            gnss_.vel  = {temper.vel[0], temper.vel[1], -temper.vel[2]};
            gnss_.vstd = {fabs(temper.vstd[0]), fabs(temper.vstd[1]), fabs(temper.vstd[2])};
        }
        return gnss_;
    }

private:
    GNSS gnss_;
    bool load_() {
        if (isEof())
            return false;
        string line;
        std::getline(filefp_, line);
        sscanf(line.c_str(), "%c,%lf,%d,%d,%d,%d,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf", &temper.type,
               &temper.sow, &temper.week, &temper.leap, &temper.status, &temper.nsat, &temper.blh[1], &temper.std[1],
               &temper.blh[0], &temper.std[0], &temper.blh[2], &temper.std[2], &temper.undulation, &temper.vel[1],
               &temper.vstd[1], &temper.vel[0], &temper.vstd[0], &temper.vel[2], &temper.vstd[2]);
        if (temper.status >=16 && temper.status <= 50) {
            return true;
        } else {
            return false;
        }
    }
};