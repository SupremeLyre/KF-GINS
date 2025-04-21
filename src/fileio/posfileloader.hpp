#pragma once
#include "common/types.h"
#include "fileio/fileloader.h"
class PosFileLoader : public FileLoader {
public:
    PosFileLoader() = delete;
    explicit PosFileLoader(const string &filename, int columns = 34) {
        open(filename, columns, FileLoader::TEXT);
    }
    const GNSS &next();

private:
    GNSS gnss_;
    bool load_();

    struct temp {
        int week;
        double tow;
        double xyz[3];
        double blh[3];
        int stat;
        int ns;
        double std_xyz[3];
        double cov_xyz[3];
        double age;
        double ratio;
        double vxyz[3];
        double std_vxyz[3];
        double cov_vxyz[3];
        int stat_dp;
        double dpos[3];
        double qdpos[3];
        double cov_dpos[3];
        double std_ned[3];
        double std_vned[3];
        double vned[3];
        int nsat[4];
    } temper;
};