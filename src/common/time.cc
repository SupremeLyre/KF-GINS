#include "time.hpp"
#include <cmath>
#include <ctime>
Time::gtime_t Time::epoch2time(vector<double> ep) {
    gtime_t time_now;
    int year{static_cast<int>(ep[0])}, month{static_cast<int>(ep[1])}, day{static_cast<int>(ep[2])};
    int days{(year - 1970) * 365 + (year - 1969) / 4 + doy[month - 1] + day - 2 +
             (year % 4 == 0 && month >= 3 ? 1 : 0)};
    int sec{static_cast<int>(floor(ep[5]))};
    time_now.time =
        static_cast<std::time_t>(days) * 86400 + static_cast<int>(ep[3]) * 3600 + static_cast<int>(ep[4]) * 60 + sec;
    time_now.sec = ep[5] - sec;
    return time_now;
}
bool Time::epoch2gpst(const vector<double> ep, int &week, double &tow) {
    if (ep.size() != 6)
        return false;
    gtime_t time_now = epoch2time(ep);
    gtime_t time0    = epoch2time(gpst0);
    std::time_t sec  = time_now.time - time0.time;
    week             = static_cast<int>(sec / (86400 * 7));
    tow              = static_cast<double>(sec - static_cast<double>(week) * 86400 * 7) + time_now.sec;
    return true;
}