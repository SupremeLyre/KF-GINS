#include "posfileloader.hpp"
#include "Eigen/src/Core/Matrix.h"
#include "common/earth.h"
#include "common/types.h"
#include <sstream>
#include <string>
bool PosFileLoader::load_() {
    if (isEof())
        return false;
    string line;
    std::getline(filefp_, line);
    std::stringstream iss;
    sscanf(line.c_str(),
           "%d%lf%lf%lf%lf%d%d%lf%lf%lf%lf%lf%lf%lf%lf%lf%lf%lf%lf%lf%lf%lf%lf%lf%d%lf%lf%lf%lf%lf%lf%lf%lf%lf",
           &temper.week, &temper.tow, &temper.xyz[0], &temper.xyz[1], &temper.xyz[2], &temper.stat, &temper.ns,
           &temper.std_xyz[0], &temper.std_xyz[1], &temper.std_xyz[2], &temper.cov_xyz[0], &temper.cov_xyz[1],
           &temper.cov_xyz[2], &temper.age, &temper.ratio, &temper.vxyz[0], &temper.vxyz[1], &temper.vxyz[2],
           &temper.std_vxyz[0], &temper.std_vxyz[1], &temper.std_vxyz[2], &temper.cov_vxyz[0], &temper.cov_vxyz[1],
           &temper.cov_vxyz[2], &temper.stat_dp, &temper.dpos[0], &temper.dpos[1], &temper.dpos[2], &temper.qdpos[0],
           &temper.qdpos[1], &temper.qdpos[2], &temper.cov_dpos[0], &temper.cov_dpos[1], &temper.cov_dpos[2]);
    Eigen::Vector3d xyz, blh;
    xyz << temper.xyz[0], temper.xyz[1], temper.xyz[2];
    blh = Earth::ecef2blh(xyz);

    temper.blh[0] = blh[0];
    temper.blh[1] = blh[1];
    temper.blh[2] = blh[2];
    return true;
}
const GNSS &PosFileLoader::next() {
    if (PosFileLoader::load_()) {
        gnss_.week = temper.week;
        gnss_.time = temper.tow;
        gnss_.blh << temper.blh[0], temper.blh[1], temper.blh[2];
        gnss_.std << temper.std_xyz[0], temper.std_xyz[2], temper.std_xyz[1];
        gnss_.vel << temper.dpos[0], temper.dpos[1], temper.dpos[2];
        gnss_.vstd << temper.qdpos[0], temper.qdpos[2], temper.qdpos[1];
    }
    return gnss_;
}