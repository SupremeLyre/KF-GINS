/*
 * KF-GINS: An EKF-Based GNSS/INS Integrated Navigation System
 *
 * Copyright (C) 2022 i2Nav Group, Wuhan University
 *
 *     Author : Liqiang Wang
 *    Contact : wlq@whu.edu.cn
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#include "common/angle.h"
#include "fileio/filesaver.h"
#include <Eigen/Dense>
#include <condition_variable>
#include <format>
#include <iostream>
#include <mutex>
#include <thread>
#include <unistd.h>
#include <yaml-cpp/exceptions.h>
#include <yaml-cpp/yaml.h>

#include "common/logging.h"
#include "kf-gins/gi_engine.h"
#include "kf-gins/kf_gins_types.h"
#define EIGEN_USE_BLAS
#define EIGEN_USE_LAPACKE

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
    double temprature;
    double gyr[3];
    double acc[3];
    int stamp;
    int cap;

} temper;

static volatile bool updatedImu            = false;
static volatile bool updatedGnss           = false;
static volatile bool wait_integer_gnss_out = false;
static volatile bool read_finished         = false;
static int end_week                        = 0;
static double end_time                     = 0.0;
// === 新增：通用“带时间戳的数据”模板和 GNSS/IMU 统一事件队列 ===
// 事件类型
enum class SensorEventKind { IMU, GNSS };

// 使用 std::variant 存 IMU 或 GNSS
struct SensorEvent {
    SensorEventKind kind;
    double time;
    int week;
    GNSS gnssdata;
    IMU imudata;
};
// 按时间顺序的事件队列
static std::deque<SensorEvent> sensor_events;

static std::mutex m;
static std::condition_variable cv;
// 比较两个事件的时间（先比周，再比秒）
static inline bool sensor_event_earlier(const SensorEvent &a, const SensorEvent &b) {
    if (a.week != b.week)
        return a.week < b.week;
    return fabs(a.time - b.time) < 0.1;
}
// 按时间有序插入到 deque 中
void push_sensor_event_sorted(const SensorEvent &ev) {
    // 队列大部分时间是“尾部追加”，用逆向遍历效率更高
    auto it = sensor_events.end();
    while (it != sensor_events.begin()) {
        auto prev = it;
        --prev;
        if (sensor_event_earlier(*prev, ev)) {
            // 找到比 ev 早的，插在它后面
            sensor_events.insert(it, ev);
            return;
        }
        it = prev;
    }
    // 所有元素都不早于 ev，插到最前面
    sensor_events.push_front(ev);
}

bool loadConfig(YAML::Node &config, GINSOptions &options);
int PPIGINSFormat(std::fstream &fp_ppi, NavState &navstate, int week, double time, Eigen::MatrixXd &Cov) {
    char buf[256]{};
    sprintf(buf,
            "%c,%16.8f,%4d,%2d,%16.8f,%16.8f,%8.4f,%8.4f,%8.4f,%8.4f,%16.8f,%16.8f,%16.8f,%11.3f,%11.3f,%11.3f,%11.3f,%"
            "11.3f,%11.3f\n",
            'M', time, week, navstate.status, navstate.pos[0] * R2D, navstate.pos[1] * R2D, navstate.pos[2],
            navstate.vel[0], navstate.vel[1], navstate.vel[2], navstate.euler[0] * R2D, navstate.euler[1] * R2D,
            navstate.euler[2] * R2D, navstate.imuerror.gyrbias[0] * (R2D * 3600),
            navstate.imuerror.gyrbias[1] * (R2D * 3600), navstate.imuerror.gyrbias[2] * (R2D * 3600),
            navstate.imuerror.accbias[0] * 1e5, navstate.imuerror.accbias[1] * 1e5, navstate.imuerror.accbias[2] * 1e5);
    fp_ppi << buf;
    if (time - round(time) > 0 && time - round(time) < 0.1) {
        std::cout << buf;
    }
    fp_ppi.sync();
    fp_ppi.flush();
    return 1;
}

int main(int argc, char *argv[]) {
    Logging::initialization(argv);

    if (argc != 2) {
        std::cout << "usage: KF-GINS kf-gins.yaml" << std::endl;
        return -1;
    }

    // std::cout << std::endl << "KF-GINS: An EKF-Based GNSS/INS Integrated Navigation System" << std::endl <<
    // std::endl;

    // 加载配置文件
    // load configuration file
    YAML::Node config;
    try {
        config = YAML::LoadFile(argv[1]);
    } catch (YAML::Exception &exception) {
        std::cout << "Failed to read configuration file. Please check the path and format of the configuration file!"
                  << std::endl;
        return -1;
    }

    // 读取配置参数到GINSOptioins中，并构造GIEngine
    // load configuration parameters to GINSOptioins
    GINSOptions options{};
    if (!loadConfig(config, options)) {
        std::cout << "Error occurs in the configuration file!" << std::endl;
        return -1;
    }

    // 读取文件路径配置
    // load filepath configuration
    std::string imupath, outputpath;
    try {
        imupath    = config["imupath"].as<std::string>();
        outputpath = config["outputpath"].as<std::string>();
    } catch (YAML::Exception &exception) {
        std::cout << "Failed when loading configuration. Please check the file path and output path!" << std::endl;
        return -1;
    }
    // 构造输出文件
    // construct output file
    std::fstream fp_ppi(outputpath + "/KF_GINS_PPI.txt", std::ios::out);
    if (!fp_ppi.is_open()) {
        std::cout << "Failed to open output file. Please check the path and format of the output file!" << std::endl;
        return -1;
    }
    // 构造GIEngine
    // Construct GIEngine
    GIEngine giengine(options);
    GNSS gnss{};
    IMU imu_cur{};
    IMU imu_pre{};
    std::mutex timer_lock;
    std::jthread process_thread([&]() {
        IMU imu_pre{}, imu_cur{};
        GNSS gnss{};
        for (;;) {
            std::unique_lock<std::mutex> lk(m);
            cv.wait(lk, [&] { return !sensor_events.empty() || read_finished; });
            if (sensor_events.empty() && read_finished) {
                fp_ppi.close();
                break;
            }
            auto ev = sensor_events.front();
            sensor_events.pop_front();
            lk.unlock();

            if (!giengine.isAligned) {
                if (ev.kind == SensorEventKind::GNSS) {
                    giengine.alignProcess();
                    if (giengine.isAligned) {
                        int week_           = giengine.alignedWeek;
                        double time_        = giengine.alignedTime;
                        NavState navstate_  = giengine.getNavState();
                        Eigen::MatrixXd Cov = giengine.getCovariance();
                        PPIGINSFormat(fp_ppi, navstate_, week_, time_, Cov);
                    }
                }
            } else {
                if (ev.kind == SensorEventKind::IMU) {
                    imu_pre = imu_cur;
                    // 下一秒imu已经来了却还在等gnss输出，输出一下imu的结果
                    if (fabs(imu_pre.time - round(imu_pre.time)) < 0.01 && wait_integer_gnss_out) {
                        giengine.checkCov();
                        NavState navstate_  = giengine.getNavState();
                        Eigen::MatrixXd Cov = giengine.getCovariance();
                        PPIGINSFormat(fp_ppi, navstate_, imu_cur.week, imu_cur.time, Cov);
                        wait_integer_gnss_out = false;
                    }
                    imu_cur = ev.imudata;
                    giengine.updatePva();
                    giengine.insPropagation(imu_pre, imu_cur);
                    updatedImu = false;
                } else if (ev.kind == SensorEventKind::GNSS) {
                    gnss = ev.gnssdata;
                    printf("Processing GNSS TIME: %.3f,imu_pre.time=%.3f,imu_cur.time=%.3f\n", ev.time, imu_pre.time,
                           imu_cur.time);
                    giengine.gnssUpdate(gnss);
                    giengine.stateFeedback();
                    updatedGnss = false;
                }
                giengine.checkCov();
                NavState navstate_  = giengine.getNavState();
                Eigen::MatrixXd Cov = giengine.getCovariance();
                if (fabs(imu_cur.time - round(imu_cur.time)) < 0.01) {
                    // 整秒时刻等待GNSS输出，只输出组合结果
                    if (navstate_.status & 0b0001) {
                        PPIGINSFormat(fp_ppi, navstate_, imu_cur.week, imu_cur.time, Cov);
                        wait_integer_gnss_out = false;
                    } else {
                        wait_integer_gnss_out = true;
                    }
                } else {
                    PPIGINSFormat(fp_ppi, navstate_, imu_cur.week, imu_cur.time, Cov);
                }
            }
        }
    });
    std::jthread imu_thread([&]() {
        std::string line;
        IMU imu_cur_{}, imu_pre_{};
        std::fstream obsfile(imupath, std::ios::in);
        if (!obsfile.is_open()) {
            std::cout << "Failed to open imu file. Please check the path and format of the imu file!" << std::endl;
            return;
        }
        while (std::getline(obsfile, line)) {
            SensorEvent ev{};
            if (line.empty())
                continue;
            if (line[0] == 'G') {
                sscanf(line.c_str(), "%c,%lf,%d,%d,%d,%d,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf",
                       &temper.type, &temper.sow, &temper.week, &temper.leap, &temper.status, &temper.nsat,
                       &temper.xyz[0], &temper.stdxyz[0], &temper.xyz[1], &temper.stdxyz[1], &temper.xyz[2],
                       &temper.stdxyz[2], &temper.undulation, &temper.vxyz[0], &temper.vstd[0], &temper.vxyz[1],
                       &temper.vstd[1], &temper.vxyz[2], &temper.vstd[2]);
                Vector3d blh = Earth::ecef2blh(Vector3d(temper.xyz[0], temper.xyz[1], temper.xyz[2]));
                temper.blh   = {blh[0] * R2D, blh[1] * R2D, blh[2]};
                Matrix3d Conv_, Conv_v;
                Conv_ << temper.stdxyz[0], 0, 0, 0, temper.stdxyz[1], 0, 0, 0, temper.stdxyz[2];
                Conv_v << temper.vstd[0], 0, 0, 0, temper.vstd[1], 0, 0, 0, temper.vstd[2];
                Vector3d stdned  = (Earth::cne(blh).transpose() * Conv_ * Earth::cne(blh)).diagonal();
                Vector3d vstdned = (Earth::cne(blh).transpose() * Conv_v * Earth::cne(blh)).diagonal();
                temper.std       = {stdned[1], stdned[0], stdned[2]};
                temper.vstd      = {vstdned[1], vstdned[0], vstdned[2]};
                Vector3d vned = Earth::cne(blh).transpose() * Vector3d(temper.vxyz[0], temper.vxyz[1], temper.vxyz[2]);
                temper.vel    = {vned[1], vned[0], -vned[2]};
                if (temper.status > 34 && temper.status <= 50 && temper.nsat >= 10) {
                    gnss.week   = temper.week;
                    gnss.time   = temper.sow;
                    gnss.blh    = {temper.blh[0] * D2R, temper.blh[1] * D2R, temper.blh[2] + temper.undulation};
                    gnss.std    = {temper.std[1], temper.std[0], temper.std[2]};
                    gnss.vel    = {temper.vel[1], temper.vel[0], -temper.vel[2]};
                    gnss.vstd   = {fabs(temper.vstd[1]), fabs(temper.vstd[0]), fabs(temper.vstd[2])};
                    updatedGnss = true;
                    giengine.addGnssData(gnss);
                    ev.kind     = SensorEventKind::GNSS;
                    ev.time     = gnss.time;
                    ev.week     = gnss.week;
                    ev.gnssdata = gnss;
                    // sensor_events.push_back(ev);
                    {
                        std::lock_guard lk(m);
                        push_sensor_event_sorted(ev);
                    }
                    cv.notify_one();
                }
            } else if (line[0] == 'I') {
                sscanf(line.c_str(), "%c,%lf,%d,%d,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%d,%d", &temper.type, &temper.sow,
                       &temper.week, &temper.leap, &temper.temprature, &temper.acc[0], &temper.acc[1], &temper.acc[2],
                       &temper.gyr[0], &temper.gyr[1], &temper.gyr[2], &temper.stamp, &temper.cap);
                imu_pre_      = imu_cur_;
                imu_cur_.week = temper.week;
                imu_cur_.time = temper.sow;
                double dt     = imu_cur_.time - imu_pre_.time;
                double dt_    = 1.0 / options.sample_rate;
                if (dt > 0.003 && dt < 0.2) {
                    imu_cur_.dt = dt;
                } else {
                    imu_cur_.dt = dt_;
                }
                imu_cur_.accel << temper.acc[1] * 9.8, temper.acc[0] * 9.8, -temper.acc[2] * 9.8;
                imu_cur_.omega << temper.gyr[1] * D2R, temper.gyr[0] * D2R, -temper.gyr[2] * D2R;
                imu_cur_.dvel << temper.acc[1] * 9.8 * imu_cur_.dt, temper.acc[0] * 9.8 * imu_cur_.dt,
                    -temper.acc[2] * 9.8 * imu_cur_.dt;
                imu_cur_.dtheta << temper.gyr[1] * D2R * imu_cur_.dt, temper.gyr[0] * D2R * imu_cur_.dt,
                    -temper.gyr[2] * D2R * imu_cur_.dt;
                imu_cur_.time = temper.sow;
                imu_cur_.week = temper.week;
                giengine.addImuData(imu_cur_);
                updatedImu = true;
                ev.kind    = SensorEventKind::IMU;
                ev.time    = imu_cur_.time;
                ev.week    = imu_cur_.week;
                ev.imudata = imu_cur_;
                {
                    std::lock_guard lk(m);
                    sensor_events.push_back(ev);
                }
                cv.notify_one();
                // push_sensor_event_sorted(ev);
            }
        }
        {
            std::lock_guard lk(m);
            read_finished = true;
        }
        cv.notify_all();
        obsfile.close();
    });
    // imu_thread.join();
    // process_thread.join();
    // fp_ppi.close();
}

/**
 * @brief 从配置文件中读取GIEngine相关的初始状态，并转换为标准单位
 *        Load initial states of GIEngine from configuration file and convert them to standard units
 * */
bool loadConfig(YAML::Node &config, GINSOptions &options) {

    try {
        options.sample_rate = config["sample_rate"].as<int>();
    } catch (YAML::Exception &exception) {
        std::cout << "Failed when loading configuration. Please check sample rate!" << std::endl;
        return false;
    }
    // 读取初始位置(纬度 经度 高程)、(北向速度 东向速度 垂向速度)、姿态(欧拉角，ZYX旋转顺序, 横滚角、俯仰角、航向角)
    // load initial position(latitude longitude altitude)
    //              velocity(speeds in the directions of north, east and down)
    //              attitude(euler angle, ZYX, roll, pitch and yaw)
    std::vector<double> vec1, vec2, vec3, vec4, vec5, vec6;
    // 读取初始位置、速度、姿态(欧拉角)的标准差
    // load initial position std, velocity std and attitude(euler angle) std
    try {
        vec1 = config["initposstd"].as<std::vector<double>>();
        vec2 = config["initvelstd"].as<std::vector<double>>();
        vec3 = config["initattstd"].as<std::vector<double>>();
    } catch (YAML::Exception &exception) {
        std::cout << "Failed when loading configuration. Please check initial std of position, velocity, and attitude!"
                  << std::endl;
        return false;
    }
    for (int i = 0; i < 3; i++) {
        options.initstate_std.pos[i]   = vec1[i];
        options.initstate_std.vel[i]   = vec2[i];
        options.initstate_std.euler[i] = vec3[i] * D2R;
    }

    // 读取IMU噪声参数
    // load imu noise parameters
    try {
        vec1 = config["imunoise"]["arw"].as<std::vector<double>>();
        vec2 = config["imunoise"]["vrw"].as<std::vector<double>>();
        vec3 = config["imunoise"]["gbstd"].as<std::vector<double>>();
        vec4 = config["imunoise"]["abstd"].as<std::vector<double>>();
        vec5 = config["imunoise"]["gsstd"].as<std::vector<double>>();
        vec6 = config["imunoise"]["asstd"].as<std::vector<double>>();

        options.imunoise.corr_time = config["imunoise"]["corrtime"].as<double>();
    } catch (YAML::Exception &exception) {
        std::cout << "Failed when loading configuration. Please check IMU noise!" << std::endl;
        return false;
    }
    for (int i = 0; i < 3; i++) {
        options.imunoise.gyr_arw[i]      = vec1[i];
        options.imunoise.acc_vrw[i]      = vec2[i];
        options.imunoise.gyrbias_std[i]  = vec3[i];
        options.imunoise.accbias_std[i]  = vec4[i];
        options.imunoise.gyrscale_std[i] = vec5[i];
        options.imunoise.accscale_std[i] = vec6[i];
    }

    // 读取IMU误差初始标准差,如果配置文件中没有设置，则采用IMU噪声参数中的零偏和比例因子的标准差
    // Load initial imu bias and scale std, set to bias and scale instability std if load failed
    try {
        vec1 = config["initbgstd"].as<std::vector<double>>();
    } catch (YAML::Exception &exception) {
        vec1 = {options.imunoise.gyrbias_std.x(), options.imunoise.gyrbias_std.y(), options.imunoise.gyrbias_std.z()};
    }
    try {
        vec2 = config["initbastd"].as<std::vector<double>>();
    } catch (YAML::Exception &exception) {
        vec2 = {options.imunoise.accbias_std.x(), options.imunoise.accbias_std.y(), options.imunoise.accbias_std.z()};
    }
    try {
        vec3 = config["initsgstd"].as<std::vector<double>>();
    } catch (YAML::Exception &exception) {
        vec3 = {options.imunoise.gyrscale_std.x(), options.imunoise.gyrscale_std.y(),
                options.imunoise.gyrscale_std.z()};
    }
    try {
        vec4 = config["initsastd"].as<std::vector<double>>();
    } catch (YAML::Exception &exception) {
        vec4 = {options.imunoise.accscale_std.x(), options.imunoise.accscale_std.y(),
                options.imunoise.accscale_std.z()};
    }
    // IMU初始误差转换为标准单位
    // convert initial imu errors' units to standard units
    for (int i = 0; i < 3; i++) {
        options.initstate_std.imuerror.gyrbias[i]  = vec1[i] * D2R / 3600.0;
        options.initstate_std.imuerror.accbias[i]  = vec2[i] * 1e-5;
        options.initstate_std.imuerror.gyrscale[i] = vec3[i] * 1e-6;
        options.initstate_std.imuerror.accscale[i] = vec4[i] * 1e-6;
    }

    // IMU噪声参数转换为标准单位
    // convert imu noise parameters' units to standard units
    options.imunoise.gyr_arw *= (D2R / 60.0);
    options.imunoise.acc_vrw /= 60.0;
    options.imunoise.gyrbias_std *= (D2R / 3600.0);
    options.imunoise.accbias_std *= 1e-5;
    options.imunoise.gyrscale_std *= 1e-6;
    options.imunoise.accscale_std *= 1e-6;
    options.imunoise.corr_time *= 3600;

    // GNSS天线杆臂, GNSS天线相位中心在IMU坐标系下位置
    // gnss antenna leverarm, position of GNSS antenna phase center in IMU frame
    try {
        vec1 = config["antlever"].as<std::vector<double>>();
    } catch (YAML::Exception &exception) {
        std::cout << "Failed when loading configuration. Please check antenna leverarm!" << std::endl;
        return false;
    }
    options.antlever = Eigen::Vector3d(vec1.data());
    GIEngineOpt opt1;
    try {
        opt1.estimate_scale  = config["options"]["estimate_scale"].as<bool>();
        opt1.enable_gnss_pos = config["options"]["enable_gnss_pos"].as<bool>();
        opt1.enable_gnss_vel = config["options"]["enable_gnss_vel"].as<bool>();
        opt1.enable_nhc      = config["options"]["enable_nhc"].as<bool>();
        opt1.enable_zupt     = config["options"]["enable_zupt"].as<bool>();
    } catch (YAML::Exception &exception) {
        std::cout << "Failed when loading configuration. Please check engine options!" << std::endl;
        return false;
    }
    try {
        opt1.zuptopt.interval      = config["zupt"]["interval"].as<double>();
        opt1.zuptopt.vel_threshold = config["zupt"]["vel_threshold"].as<double>();
        opt1.zuptopt.wib_threshold = config["zupt"]["wib_threshold"].as<double>();
        opt1.zuptopt.fb_threshold  = config["zupt"]["fb_threshold"].as<double>();
    } catch (YAML::Exception &exception) {
        std::cout << "Failed when loading configuration. Please check zupt options!" << std::endl;
        return false;
    }
    try {
        auto vec = config["imu_misalign"].as<std::vector<double>>();

        options.imu_misalign = Eigen::Vector3d(vec[0], vec[1], vec[2]) * D2R;
    } catch (YAML::Exception &exception) {
        std::cout << "Missing imu_misalign configuration!" << std::endl;
        return false;
    }
    options.engineopt = opt1;
    try {
        options.processNoise_pos = config["processnoise"]["pos"].as<double>();
    } catch (YAML::Exception &exception) {
        std::cout << "Missing process noise configuration!" << std::endl;
        return false;
    }
    return true;
}
