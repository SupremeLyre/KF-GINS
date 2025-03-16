#include <array>
#include <ctime>
#include <vector>
using std::vector;
namespace Time {
const int doy[12]{1, 32, 60, 91, 121, 152, 182, 213, 244, 274, 305, 335};
const vector<double> gpst0{1980, 1, 6, 0, 0, 0};
struct gtime_t {
    std::time_t time;
    double sec;
};
gtime_t epoch2time(vector<double> ep);
bool epoch2gpst(vector<double> ep, int &week, double &tow);
} // namespace Time