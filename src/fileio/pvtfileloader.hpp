#pragma once
#include "common/angle.h"
#include "common/earth.h"
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
        std::array<double, 3> xyz;
        std::array<double, 3> stdxyz;
        std::array<double, 3> vxyz;
        std::array<double, 3> stdvxyz;
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
#if 1
        sscanf(line.c_str(), "%c,%lf,%d,%d,%d,%d,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf", &temper.type,
               &temper.sow, &temper.week, &temper.leap, &temper.status, &temper.nsat, &temper.blh[1], &temper.std[1],
               &temper.blh[0], &temper.std[0], &temper.blh[2], &temper.std[2], &temper.undulation, &temper.vel[1],
               &temper.vstd[1], &temper.vel[0], &temper.vstd[0], &temper.vel[2], &temper.vstd[2]);
#else
        sscanf(line.c_str(), "%c,%lf,%d,%d,%d,%d,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf", &temper.type,
               &temper.sow, &temper.week, &temper.leap, &temper.status, &temper.nsat, &temper.xyz[0], &temper.stdxyz[0],
               &temper.xyz[1], &temper.stdxyz[1], &temper.xyz[2], &temper.stdxyz[2], &temper.undulation,
               &temper.vxyz[0], &temper.vstd[0], &temper.vxyz[1], &temper.vstd[1], &temper.vxyz[2], &temper.vstd[2]);
        Vector3d blh = Earth::ecef2blh(Vector3d(temper.xyz[0], temper.xyz[1], temper.xyz[2]));
        temper.blh   = {blh[0] * R2D, blh[1] * R2D, blh[2]};
        Matrix3d Conv_, Conv_v;
        Conv_ << temper.stdxyz[0], 0, 0, 0, temper.stdxyz[1], 0, 0, 0, temper.stdxyz[2];
        Conv_v << temper.vstd[0], 0, 0, 0, temper.vstd[1], 0, 0, 0, temper.vstd[2];
        Vector3d stdned  = (Earth::cne(blh).transpose() * Conv_ * Earth::cne(blh)).diagonal();
        Vector3d vstdned = (Earth::cne(blh).transpose() * Conv_v * Earth::cne(blh)).diagonal();
        temper.std       = {stdned[1], stdned[0], stdned[2]};
        temper.vstd      = {vstdned[1], vstdned[0], vstdned[2]};
        Vector3d vned    = Earth::cne(blh).transpose() * Vector3d(temper.vxyz[0], temper.vxyz[1], temper.vxyz[2]);
        temper.vel       = {vned[0], vned[1], -vned[2]};
#endif
        if (temper.status > 34 && temper.status <= 50) {
            return true;
        } else {
            return false;
        }
    }
};