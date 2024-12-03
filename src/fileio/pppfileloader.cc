#include "pppfileloader.hpp"
#include "Eigen/src/Core/Matrix.h"
#include "common/earth.h"
#include "common/time.hpp"
#include <absl/strings/str_split.h>
#include <array>
#include <sstream>
#include <string>
#include <vector>
bool PPPFileLoader::load_() {
    if (isEof())
        return false;
    data_.resize(columns_);
    string line;
    std::getline(filefp_, line);
    std::stringstream iss(line);
    iss >> temper.time_str[0] >> temper.time_str[1] >> temper.xyz[0] >> temper.xyz[1] >> temper.xyz[2] >> temper.stat >>
        temper.ns >> temper.enu[0] >> temper.enu[1] >> temper.enu[2] >> temper.age >> temper.std_xyz[0] >>
        temper.std_xyz[1] >> temper.std_xyz[2] >> temper.vxyz[0] >> temper.vxyz[1] >> temper.vxyz[2] >>
        temper.std_vxyz[0] >> temper.std_vxyz[1] >> temper.std_vxyz[2] >> temper.acc[0] >> temper.acc[1] >>
        temper.acc[2] >> temper.stat_dpos >> temper.dpos[0] >> temper.dpos[1] >> temper.dpos[2] >> temper.qdpos[0] >>
        temper.qdpos[1] >> temper.qdpos[2] >> temper.time_acc;
    vector<double> ep;
    vector<string> splits = absl::StrSplit(temper.time_str[0], absl::ByAnyChar("/ :"), absl::SkipWhitespace());
    for (auto &split : splits) {
        ep.push_back(strtod(split.data(), nullptr));
    }
    splits.clear();
    splits = absl::StrSplit(temper.time_str[1], absl::ByAnyChar("/ :"), absl::SkipWhitespace());
    for (auto &split : splits) {
        ep.push_back(strtod(split.data(), nullptr));
    }
    int week{0};
    double tow{0.0};
    bool stat{Time::epoch2gpst(ep, week, tow)};
    temper.week = week;
    temper.tow  = tow;

    Eigen::Vector3d ecef, blh;
    ecef << temper.xyz[0], temper.xyz[1], temper.xyz[2];
    blh = Earth::ecef2blh(ecef);

    temper.blh[0] = blh[0];
    temper.blh[1] = blh[1];
    temper.blh[2] = blh[2];
    return stat;
}