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
    Eigen::Matrix3d Conv_, Conv_v;
    Conv_ << temper.std_xyz[0], temper.cov_xyz[0], temper.cov_xyz[2], temper.cov_xyz[0], temper.std_xyz[1],
        temper.cov_xyz[1], temper.cov_xyz[2], temper.cov_xyz[1], temper.std_xyz[2];
    Conv_v << temper.std_vxyz[0], temper.cov_vxyz[0], temper.cov_vxyz[2], temper.cov_dpos[0], temper.cov_vxyz[1],
        temper.cov_vxyz[1], temper.cov_vxyz[2], temper.cov_vxyz[1], temper.std_vxyz[2];
    Eigen::Vector3d stdned  = (Earth::cne(blh).transpose() * Conv_ * Earth::cne(blh)).diagonal();
    Eigen::Vector3d vstdned = (Earth::cne(blh).transpose() * Conv_v * Earth::cne(blh)).diagonal();
    Eigen::Vector3d vned;
    vned << temper.dpos[0], temper.dpos[1], temper.dpos[2];
    vned = Earth::cne(blh).transpose() * vned;
    for (int i = 0; i < 3; i++) {
        temper.blh[i]      = blh[i];
        temper.std_ned[i]  = stdned[i];
        temper.std_vned[i] = vstdned[i];
        temper.vned[i]     = vned[i];
    }
    return true;
}
const GNSS &PosFileLoader::next() {
    if (PosFileLoader::load_()) {
        gnss_.week = temper.week;
        gnss_.time = temper.tow;
        gnss_.blh << temper.blh[0], temper.blh[1], temper.blh[2];
        gnss_.std << temper.std_ned[0], temper.std_ned[1], temper.std_ned[2];
        gnss_.vel << temper.vned[0], temper.vned[1], temper.vned[2];
        gnss_.vstd << temper.std_vned[0], temper.std_vned[1], temper.std_vned[2];
    }
    return gnss_;
}