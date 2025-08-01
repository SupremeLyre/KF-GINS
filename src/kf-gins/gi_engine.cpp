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
#include "gi_engine.h"
#include "common/angle.h"
#include "common/earth.h"
#include "common/logging.h"
#include "common/rotation.h"
#include "common/types.h"
#include "insmech.h"
#include "kf-gins/kf_gins_types.h"
#include <Eigen/Eigen>
#include <format>

GIEngine::GIEngine(GINSOptions &options) {

    this->options_ = options;
    // options_.print_options();
    week_      = 0;
    timestamp_ = 0;

    // 设置协方差矩阵，系统噪声阵和系统误差状态矩阵大小
    // resize covariance matrix, system noise matrix, and system error state matrix
    Cov_.resize(RANK, RANK);
    Qc_.resize(NOISERANK, NOISERANK);
    dx_.resize(RANK, 1);
    Cov_.setZero();
    Qc_.setZero();
    dx_.setZero();

    // 初始化系统噪声阵
    // initialize noise matrix
    auto imunoise                   = options_.imunoise;
    Qc_.block(ARW_ID, ARW_ID, 3, 3) = imunoise.gyr_arw.cwiseProduct(imunoise.gyr_arw).asDiagonal();
    Qc_.block(VRW_ID, VRW_ID, 3, 3) = imunoise.acc_vrw.cwiseProduct(imunoise.acc_vrw).asDiagonal();
    Qc_.block(BGSTD_ID, BGSTD_ID, 3, 3) =
        2 / imunoise.corr_time * imunoise.gyrbias_std.cwiseProduct(imunoise.gyrbias_std).asDiagonal();
    Qc_.block(BASTD_ID, BASTD_ID, 3, 3) =
        2 / imunoise.corr_time * imunoise.accbias_std.cwiseProduct(imunoise.accbias_std).asDiagonal();
    if (engineopt_.estimate_scale) {
        Qc_.block(SGSTD_ID, SGSTD_ID, 3, 3) =
            2 / imunoise.corr_time * imunoise.gyrscale_std.cwiseProduct(imunoise.gyrscale_std).asDiagonal();
        Qc_.block(SASTD_ID, SASTD_ID, 3, 3) =
            2 / imunoise.corr_time * imunoise.accscale_std.cwiseProduct(imunoise.accscale_std).asDiagonal();
    }

    // 设置系统状态(位置、速度、姿态和IMU误差)初值和初始协方差
    // set initial state (position, velocity, attitude and IMU error) and covariance
    initialize(options_.initstate, options_.initstate_std);
    engineopt_ = options_.engineopt;
    C_imu_body = Rotation::euler2matrix(options_.imu_misalign);
}

void GIEngine::initialize(const NavState &initstate, const NavState &initstate_std) {

    // 初始化位置、速度、姿态
    // initialize position, velocity and attitude
    pvacur_.pos       = initstate.pos;
    pvacur_.vel       = initstate.vel;
    pvacur_.att.euler = initstate.euler;
    pvacur_.att.cbn   = Rotation::euler2matrix(pvacur_.att.euler);
    pvacur_.att.qbn   = Rotation::euler2quaternion(pvacur_.att.euler);
    // 初始化IMU误差
    // initialize imu error
    imuerror_ = initstate.imuerror;

    // 给上一时刻状态赋同样的初值
    // set the same value to the previous state
    pvapre_ = pvacur_;

    // 初始化协方差
    // initialize covariance
    ImuError imuerror_std            = initstate_std.imuerror;
    Cov_.block(P_ID, P_ID, 3, 3)     = initstate_std.pos.cwiseProduct(initstate_std.pos).asDiagonal();
    Cov_.block(V_ID, V_ID, 3, 3)     = initstate_std.vel.cwiseProduct(initstate_std.vel).asDiagonal();
    Cov_.block(PHI_ID, PHI_ID, 3, 3) = initstate_std.euler.cwiseProduct(initstate_std.euler).asDiagonal();
    Cov_.block(BG_ID, BG_ID, 3, 3)   = imuerror_std.gyrbias.cwiseProduct(imuerror_std.gyrbias).asDiagonal();
    Cov_.block(BA_ID, BA_ID, 3, 3)   = imuerror_std.accbias.cwiseProduct(imuerror_std.accbias).asDiagonal();
    if (engineopt_.estimate_scale) {
        Cov_.block(SG_ID, SG_ID, 3, 3) = imuerror_std.gyrscale.cwiseProduct(imuerror_std.gyrscale).asDiagonal();
        Cov_.block(SA_ID, SA_ID, 3, 3) = imuerror_std.accscale.cwiseProduct(imuerror_std.accscale).asDiagonal();
    }
}

void GIEngine::newImuProcess() {

    // 当前IMU时间作为系统当前状态时间,
    // set current IMU time as the current state time
    week_      = imucur_.week;
    timestamp_ = imucur_.time;

    // 如果GNSS有效，则将更新时间设置为GNSS时间
    // set update time as the gnss time if gnssdata is valid
    double updatetime = gnssdata_.isvalid ? gnssdata_.time : -1;

    // 判断是否需要进行GNSS更新
    // determine if we should do GNSS update
    int res = isToUpdate(imupre_.time, imucur_.time, updatetime);

    if (res == 0) {
        // 只传播导航状态
        // only propagate navigation state
        insPropagation(imupre_, imucur_);
    } else if (res == 1) {
        // GNSS数据靠近上一历元，先对上一历元进行GNSS更新
        // gnssdata is near to the previous imudata, we should firstly do gnss update
        gnssUpdate(gnssdata_);
        stateFeedback();
        pvapre_ = pvacur_;
        insPropagation(imupre_, imucur_);
    } else if (res == 2) {
        // GNSS数据靠近当前历元，先对当前IMU进行状态传播
        // gnssdata is near current imudata, we should firstly propagate navigation state
        insPropagation(imupre_, imucur_);
        gnssUpdate(gnssdata_);
        stateFeedback();
    } else {
        // GNSS数据在两个IMU数据之间(不靠近任何一个), 将当前IMU内插到整秒时刻
        // gnssdata is between the two imudata, we interpolate current imudata to gnss time
        IMU midimu;
        imuInterpolate(imupre_, imucur_, updatetime, midimu);

        // 对前一半IMU进行状态传播
        // propagate navigation state for the first half imudata
        insPropagation(imupre_, midimu);
        // 整秒时刻进行GNSS更新，并反馈系统状态
        // do GNSS position update at the whole second and feedback system states
        gnssUpdate(gnssdata_);
        stateFeedback();

        // 对后一半IMU进行状态传播
        // propagate navigation state for the second half imudata
        pvapre_ = pvacur_;
        insPropagation(midimu, imucur_);
    }
    // 使用NHC添加约束
    // add constraint using NHC
    // || (imucur_.time > 188723 && imucur_.time < 188771)
    if (engineopt_.enable_nhc &&
        ((imucur_.time > 188431 && imucur_.time < 188540))) {
        int nv = nhc(pvacur_);
    }
    if (engineopt_.enable_zupt && isStatic()) {
        int nv = zupt(pvacur_);
    }
    // 检查协方差矩阵对角线元素
    // check diagonal elements of current covariance matrix
    checkCov();

    // 更新上一时刻的状态和IMU数据
    // update system state and imudata at the previous epoch
    pvapre_ = pvacur_;
    imupre_ = imucur_;
}

void GIEngine::imuCompensate(IMU &imu) {

    // 补偿IMU零偏
    // compensate the imu bias
    imu.dtheta -= imuerror_.gyrbias * imu.dt;
    imu.dvel -= imuerror_.accbias * imu.dt;

    // 补偿IMU比例因子
    // compensate the imu scale
    Eigen::Vector3d gyrscale, accscale;
    gyrscale   = Eigen::Vector3d::Ones() + imuerror_.gyrscale;
    accscale   = Eigen::Vector3d::Ones() + imuerror_.accscale;
    imu.dtheta = imu.dtheta.cwiseProduct(gyrscale.cwiseInverse());
    imu.dvel   = imu.dvel.cwiseProduct(accscale.cwiseInverse());
}

void GIEngine::insPropagation(IMU &imupre, IMU &imucur) {

    // 对当前IMU数据(imucur)补偿误差, 上一IMU数据(imupre)已经补偿过了
    // compensate imu error to 'imucur', 'imupre' has been compensated
    imuCompensate(imucur);
    // IMU状态更新(机械编排算法)
    // update imustate(mechanization)
    INSMech::insMech(pvapre_, pvacur_, imupre, imucur);

    // 系统噪声传播，姿态误差采用phi角误差模型
    // system noise propagate, phi-angle error model for attitude error
    Eigen::MatrixXd Phi, F, Qd, G;

    // 初始化Phi阵(状态转移矩阵)，F阵，Qd阵(传播噪声阵)，G阵(噪声驱动阵)
    // initialize Phi (state transition), F matrix, Qd(propagation noise) and G(noise driven) matrix
    Phi.resizeLike(Cov_);
    F.resizeLike(Cov_);
    Qd.resizeLike(Cov_);
    G.resize(RANK, NOISERANK);
    Phi.setIdentity();
    F.setZero();
    Qd.setZero();
    G.setZero();

    // 使用上一历元状态计算状态转移矩阵
    // compute state transition matrix using the previous state
    Eigen::Vector2d rmrn;
    Eigen::Vector3d wie_n, wen_n;
    double gravity;
    rmrn    = Earth::meridianPrimeVerticalRadius(pvapre_.pos[0]);
    gravity = Earth::gravity(pvapre_.pos);
    wie_n << WGS84_WIE * cos(pvapre_.pos[0]), 0, -WGS84_WIE * sin(pvapre_.pos[0]);
    wen_n << pvapre_.vel[1] / (rmrn[1] + pvapre_.pos[2]), -pvapre_.vel[0] / (rmrn[0] + pvapre_.pos[2]),
        -pvapre_.vel[1] * tan(pvapre_.pos[0]) / (rmrn[1] + pvapre_.pos[2]);

    Eigen::Matrix3d temp;
    Eigen::Vector3d accel, omega;
    double rmh, rnh;

    rmh   = rmrn[0] + pvapre_.pos[2];
    rnh   = rmrn[1] + pvapre_.pos[2];
    accel = imucur.dvel / imucur.dt;
    omega = imucur.dtheta / imucur.dt;

    // 位置误差
    // position error
    temp.setZero();
    temp(0, 0)                = -pvapre_.vel[2] / rmh;
    temp(0, 2)                = pvapre_.vel[0] / rmh;
    temp(1, 0)                = pvapre_.vel[1] * tan(pvapre_.pos[0]) / rnh;
    temp(1, 1)                = -(pvapre_.vel[2] + pvapre_.vel[0] * tan(pvapre_.pos[0])) / rnh;
    temp(1, 2)                = pvapre_.vel[1] / rnh;
    F.block(P_ID, P_ID, 3, 3) = temp;
    F.block(P_ID, V_ID, 3, 3) = Eigen::Matrix3d::Identity();

    // 速度误差
    // velocity error
    temp.setZero();
    temp(0, 0) = -2 * pvapre_.vel[1] * WGS84_WIE * cos(pvapre_.pos[0]) / rmh -
                 pow(pvapre_.vel[1], 2) / rmh / rnh / pow(cos(pvapre_.pos[0]), 2);
    temp(0, 2) = pvapre_.vel[0] * pvapre_.vel[2] / rmh / rmh - pow(pvapre_.vel[1], 2) * tan(pvapre_.pos[0]) / rnh / rnh;
    temp(1, 0) = 2 * WGS84_WIE * (pvapre_.vel[0] * cos(pvapre_.pos[0]) - pvapre_.vel[2] * sin(pvapre_.pos[0])) / rmh +
                 pvapre_.vel[0] * pvapre_.vel[1] / rmh / rnh / pow(cos(pvapre_.pos[0]), 2);
    temp(1, 2) = (pvapre_.vel[1] * pvapre_.vel[2] + pvapre_.vel[0] * pvapre_.vel[1] * tan(pvapre_.pos[0])) / rnh / rnh;
    temp(2, 0) = 2 * WGS84_WIE * pvapre_.vel[1] * sin(pvapre_.pos[0]) / rmh;
    temp(2, 2) = -pow(pvapre_.vel[1], 2) / rnh / rnh - pow(pvapre_.vel[0], 2) / rmh / rmh +
                 2 * gravity / (sqrt(rmrn[0] * rmrn[1]) + pvapre_.pos[2]);
    F.block(V_ID, P_ID, 3, 3) = temp;
    temp.setZero();
    temp(0, 0)                  = pvapre_.vel[2] / rmh;
    temp(0, 1)                  = -2 * (WGS84_WIE * sin(pvapre_.pos[0]) + pvapre_.vel[1] * tan(pvapre_.pos[0]) / rnh);
    temp(0, 2)                  = pvapre_.vel[0] / rmh;
    temp(1, 0)                  = 2 * WGS84_WIE * sin(pvapre_.pos[0]) + pvapre_.vel[1] * tan(pvapre_.pos[0]) / rnh;
    temp(1, 1)                  = (pvapre_.vel[2] + pvapre_.vel[0] * tan(pvapre_.pos[0])) / rnh;
    temp(1, 2)                  = 2 * WGS84_WIE * cos(pvapre_.pos[0]) + pvapre_.vel[1] / rnh;
    temp(2, 0)                  = -2 * pvapre_.vel[0] / rmh;
    temp(2, 1)                  = -2 * (WGS84_WIE * cos(pvapre_.pos(0)) + pvapre_.vel[1] / rnh);
    F.block(V_ID, V_ID, 3, 3)   = temp;
    F.block(V_ID, PHI_ID, 3, 3) = Rotation::skewSymmetric(pvapre_.att.cbn * accel);
    F.block(V_ID, BA_ID, 3, 3)  = pvapre_.att.cbn;
    if (engineopt_.estimate_scale) {
        F.block(V_ID, SA_ID, 3, 3) = pvapre_.att.cbn * (accel.asDiagonal());
    }

    // 姿态误差
    // attitude error
    temp.setZero();
    temp(0, 0) = -WGS84_WIE * sin(pvapre_.pos[0]) / rmh;
    temp(0, 2) = pvapre_.vel[1] / rnh / rnh;
    temp(1, 2) = -pvapre_.vel[0] / rmh / rmh;
    temp(2, 0) = -WGS84_WIE * cos(pvapre_.pos[0]) / rmh - pvapre_.vel[1] / rmh / rnh / pow(cos(pvapre_.pos[0]), 2);
    temp(2, 2) = -pvapre_.vel[1] * tan(pvapre_.pos[0]) / rnh / rnh;
    F.block(PHI_ID, P_ID, 3, 3) = temp;
    temp.setZero();
    temp(0, 1)                    = 1 / rnh;
    temp(1, 0)                    = -1 / rmh;
    temp(2, 1)                    = -tan(pvapre_.pos[0]) / rnh;
    F.block(PHI_ID, V_ID, 3, 3)   = temp;
    F.block(PHI_ID, PHI_ID, 3, 3) = -Rotation::skewSymmetric(wie_n + wen_n);
    F.block(PHI_ID, BG_ID, 3, 3)  = -pvapre_.att.cbn;
    if (engineopt_.estimate_scale) {
        F.block(PHI_ID, SG_ID, 3, 3) = -pvapre_.att.cbn * (omega.asDiagonal());
    }

    // IMU零偏误差和比例因子误差，建模成一阶高斯-马尔科夫过程
    // imu bias error and scale error, modeled as the first-order Gauss-Markov process
    F.block(BG_ID, BG_ID, 3, 3) = -1 / options_.imunoise.corr_time * Eigen::Matrix3d::Identity();
    F.block(BA_ID, BA_ID, 3, 3) = -1 / options_.imunoise.corr_time * Eigen::Matrix3d::Identity();
    if (engineopt_.estimate_scale) {
        F.block(SG_ID, SG_ID, 3, 3) = -1 / options_.imunoise.corr_time * Eigen::Matrix3d::Identity();
        F.block(SA_ID, SA_ID, 3, 3) = -1 / options_.imunoise.corr_time * Eigen::Matrix3d::Identity();
    }

    // 系统噪声驱动矩阵
    // system noise driven matrix
    G.block(V_ID, VRW_ID, 3, 3)    = pvapre_.att.cbn;
    G.block(PHI_ID, ARW_ID, 3, 3)  = pvapre_.att.cbn;
    G.block(BG_ID, BGSTD_ID, 3, 3) = Eigen::Matrix3d::Identity();
    G.block(BA_ID, BASTD_ID, 3, 3) = Eigen::Matrix3d::Identity();
    if (engineopt_.estimate_scale) {
        G.block(SG_ID, SGSTD_ID, 3, 3) = Eigen::Matrix3d::Identity();
        G.block(SA_ID, SASTD_ID, 3, 3) = Eigen::Matrix3d::Identity();
    }

    // 状态转移矩阵
    // compute the state transition matrix
    Phi.setIdentity();
    Phi = Phi + F * imucur.dt;

    // 计算系统传播噪声
    // compute system propagation noise
    Qd = G * Qc_ * G.transpose() * imucur.dt;
    Qd = (Phi * Qd * Phi.transpose() + Qd) / 2;

    // EKF预测传播系统协方差和系统误差状态
    // do EKF predict to propagate covariance and error state
    EKFPredict(Phi, Qd);
}

void GIEngine::gnssUpdate(GNSS &gnssdata) {

    // IMU位置转到GNSS天线相位中心位置
    // convert IMU position to GNSS antenna phase center position
    Eigen::Vector3d antenna_pos, antenna_vel;
    Eigen::Matrix3d Dr, Dr_inv;
    Dr_inv      = Earth::DRi(pvacur_.pos);
    Dr          = Earth::DR(pvacur_.pos);
    antenna_pos = pvacur_.pos + Dr_inv * pvacur_.att.cbn * options_.antlever;
    antenna_vel = pvacur_.vel -
                  Rotation::skewSymmetric(Earth::iewn(pvacur_.pos[0])) * (pvacur_.att.cbn * options_.antlever) -
                  pvacur_.att.cbn * (Rotation::skewSymmetric(options_.antlever) * (imucur_.dtheta / imucur_.dt));
    // GNSS位置测量新息
    // compute GNSS position innovation
    Eigen::MatrixXd dz;
    dz = Dr * (antenna_pos - gnssdata.blh);

    // 构造GNSS位置观测矩阵
    // construct GNSS position measurement matrix
    Eigen::MatrixXd H_gnsspos;
    H_gnsspos.resize(3, Cov_.rows());
    H_gnsspos.setZero();
    H_gnsspos.block(0, P_ID, 3, 3)   = Eigen::Matrix3d::Identity();
    H_gnsspos.block(0, PHI_ID, 3, 3) = Rotation::skewSymmetric(pvacur_.att.cbn * options_.antlever);

    // 位置观测噪声阵
    // construct measurement noise matrix
    Eigen::MatrixXd R_gnsspos;
    R_gnsspos = gnssdata.std.cwiseProduct(gnssdata.std).asDiagonal();

    // EKF更新协方差和误差状态
    // do EKF update to update covariance and error state
    if (engineopt_.enable_gnss_pos) {
        EKFUpdate(dz, H_gnsspos, R_gnsspos, KFFilterType::EKF);
    }

    // GNSS速度观测矩阵
    Eigen::MatrixXd H_gnssvel;
    H_gnssvel.resize(3, Cov_.rows());
    H_gnssvel.setZero();
    H_gnssvel.block(0, V_ID, 3, 3) = Eigen::Matrix3d::Identity();
    H_gnssvel.block(0, PHI_ID, 3, 3) =
        -(Rotation::skewSymmetric(Earth::iewn(pvacur_.pos[0])) *
              Rotation::skewSymmetric(pvacur_.att.cbn * options_.antlever) +
          Rotation::skewSymmetric(pvacur_.att.cbn *
                                  (Rotation::skewSymmetric(options_.antlever) * (imucur_.dtheta / imucur_.dt))));
    H_gnssvel.block(0, BG_ID, 3, 3) = -Rotation::skewSymmetric(pvacur_.att.cbn * options_.antlever);
    if (engineopt_.estimate_scale) {
        H_gnssvel.block(0, SG_ID, 3, 3) =
            -Rotation::skewSymmetric(pvacur_.att.cbn * options_.antlever) * (imucur_.dtheta / imucur_.dt).asDiagonal();
    }
    // GNSS速度观测噪声矩阵
    Eigen::MatrixXd R_gnssvel;
    R_gnssvel = gnssdata.vstd.cwiseProduct(gnssdata.vstd).asDiagonal();
    // GNSS速度量测新息
    Eigen::MatrixXd dz_vel;
    dz_vel = antenna_vel - gnssdata.vel;
    if (engineopt_.enable_gnss_vel) {
        EKFUpdate(dz_vel, H_gnssvel, R_gnssvel, KFFilterType::EKF);
    }
    // GNSS更新之后设置为不可用
    // Set GNSS invalid after update
    gnssdata.isvalid = false;
}

int GIEngine::isToUpdate(double imutime1, double imutime2, double updatetime) const {

    if (abs(imutime1 - updatetime) < TIME_ALIGN_ERR) {
        // 更新时间靠近imutime1
        // updatetime is near to imutime1
        return 1;
    } else if (abs(imutime2 - updatetime) <= TIME_ALIGN_ERR) {
        // 更新时间靠近imutime2
        // updatetime is near to imutime2
        return 2;
    } else if (imutime1 < updatetime && updatetime < imutime2) {
        // 更新时间在imutime1和imutime2之间, 但不靠近任何一个
        // updatetime is between imutime1 and imutime2, but not near to either
        return 3;
    } else {
        // 更新时间不在imutimt1和imutime2之间，且不靠近任何一个
        // updatetime is not bewteen imutime1 and imutime2, and not near to either.
        return 0;
    }
}

void GIEngine::EKFPredict(Eigen::MatrixXd &Phi, Eigen::MatrixXd &Qd) {

    assert(Phi.rows() == Cov_.rows());
    assert(Qd.rows() == Cov_.rows());

    // 传播系统协方差和误差状态
    // propagate system covariance and error state
    Cov_ = Phi * Cov_ * Phi.transpose() + Qd;
    dx_  = Phi * dx_;
}

void GIEngine::EKFUpdate(Eigen::MatrixXd &dz, Eigen::MatrixXd &H, Eigen::MatrixXd &R, KFFilterType type) {

    assert(H.cols() == Cov_.rows());
    assert(dz.rows() == H.rows());
    assert(dz.rows() == R.rows());
    assert(dz.cols() == 1);
    // 抗差处理：计算标准化残差并调整观测值噪声矩阵
    Eigen::MatrixXd adj_R = R;
    switch (type) {
        case KFFilterType::Huber: {
            double k0 = 2.0;

            Eigen::MatrixXd HPHt = H * Cov_ * H.transpose();
            for (int i = 0; i < dz.rows(); i++) {
                double sigma = sqrt((HPHt + R)(i, i));
                if (sigma < 1e-12)
                    continue;
                double v_normalized = dz(i, 0) / sigma;
                if (fabs(v_normalized) > k0) {
                    adj_R(i, i) *= fabs(v_normalized / k0); // 调整等价权
                }
            }
            break;
        }
        case KFFilterType::EKF: {
            break;
        }
        case KFFilterType::IGG3: {
            break;
        }
    }
    // 计算Kalman增益
    // Compute Kalman Gain
    auto temp         = H * Cov_ * H.transpose() + adj_R;
    Eigen::MatrixXd K = Cov_ * H.transpose() * temp.inverse();

    // 更新系统误差状态和协方差
    // update system error state and covariance
    Eigen::MatrixXd I;
    I.resizeLike(Cov_);
    I.setIdentity();
    I = I - K * H;
    // 如果每次更新后都进行状态反馈，则更新前dx_一直为0，下式可以简化为：dx_ = K * dz;
    // if state feedback is performed after every update, dx_ is always zero before the update
    // the following formula can be simplified as : dx_ = K * dz;
    dx_  = dx_ + K * (dz - H * dx_);
    Cov_ = I * Cov_ * I.transpose() + K * R * K.transpose();
}

void GIEngine::stateFeedback() {

    Eigen::Vector3d vectemp;

    // 位置误差反馈
    // posisiton error feedback
    Eigen::Vector3d delta_r = dx_.block(P_ID, 0, 3, 1);
    Eigen::Matrix3d Dr_inv  = Earth::DRi(pvacur_.pos);
    pvacur_.pos -= Dr_inv * delta_r;

    // 速度误差反馈
    // velocity error feedback
    vectemp = dx_.block(V_ID, 0, 3, 1);
    pvacur_.vel -= vectemp;

    // 姿态误差反馈
    // attitude error feedback
    vectemp                = dx_.block(PHI_ID, 0, 3, 1);
    Eigen::Quaterniond qpn = Rotation::rotvec2quaternion(vectemp);
    pvacur_.att.qbn        = qpn * pvacur_.att.qbn;
    pvacur_.att.cbn        = Rotation::quaternion2matrix(pvacur_.att.qbn);
    pvacur_.att.euler      = Rotation::matrix2euler(pvacur_.att.cbn);

    // IMU零偏误差反馈
    // IMU bias error feedback
    vectemp = dx_.block(BG_ID, 0, 3, 1);
    imuerror_.gyrbias += vectemp;
    vectemp = dx_.block(BA_ID, 0, 3, 1);
    imuerror_.accbias += vectemp;

    // IMU比例因子误差反馈
    // IMU sacle error feedback

    if (engineopt_.estimate_scale) {
        vectemp = dx_.block(SG_ID, 0, 3, 1);
        imuerror_.gyrscale += vectemp;
        vectemp = dx_.block(SA_ID, 0, 3, 1);
        imuerror_.accscale += vectemp;
    }

    // 误差状态反馈到系统状态后,将误差状态清零
    // set 'dx' to zero after feedback error state to system state
    dx_.setZero();
}

NavState GIEngine::getNavState() {

    NavState state;

    state.pos      = pvacur_.pos;
    state.vel      = pvacur_.vel;
    state.euler    = pvacur_.att.euler;
    state.imuerror = imuerror_;

    return state;
}
int GIEngine::nhc(PVA pvacur_) {
    // 理论上载体系下只有前向速度，没有右向和下向速度。
    Eigen::Matrix3d Cnb = pvacur_.att.cbn.transpose();
    Eigen::Vector3d T1, T2, T3, vb;
    vb         = Cnb * pvacur_.vel;
    Matrix3d T = -Cnb * Rotation::skewSymmetric(pvacur_.vel);
    Eigen::Vector3d C1, C2, C3;
    C1 = Cnb.row(0);
    C2 = Cnb.row(1);
    C3 = Cnb.row(2);
    T1 = T.row(0);
    T2 = T.row(1);
    T3 = T.row(2);
    // Logging::printMatrix(vb);
    // 设计矩阵
    Eigen::MatrixXd H;
    // 新息向量
    Eigen::MatrixXd dz;
    // 观测噪声
    Eigen::MatrixXd R;
    std::cout << std::format("{:10.3f} {:8.3f} {:8.3f} {:8.3f} {:8.3f}\n", imucur_.time, vb[0], vb[1], vb[2],
                             imucur_.dtheta.norm() * R2D);
    // Logging::printMatrix(Cnb);
    // Logging::printMatrix(C1);
    // Logging::printMatrix(C2);
    // Logging::printMatrix(C3);
    // Logging::printMatrix(T);
    // Logging::printMatrix(T1);
    // Logging::printMatrix(T2);
    // Logging::printMatrix(T3);
    dz.resize(2, 1);
    R.resize(2, 2);
    R.setZero();
    H.resize(2, Cov_.rows());
    H.setZero();
    int nv = 0;
    if (fabs(vb[0]) > 2) {
        for (int i = 1; i < 3; i++) {
#if 0
            if (fabs(vb[i]) > 0.5) {
                continue;
            }
            if (fabs(imucur_.dtheta.norm()) > 30 * D2R) {
                continue;
            }
#endif
            Vector3d Tx = i == 1 ? T2 : T3;
            Vector3d Cx = i == 1 ? C2 : C3;

            H.block(nv, V_ID, 1, 3)   = Cx.transpose();
            H.block(nv, PHI_ID, 1, 3) = Tx.transpose();

            dz(nv) = vb[i];

            R(nv, nv) = pow(0.05, 2);
            nv++;
        }
    }
    // Logging::printMatrix(H);

    if (nv > 1) {
        EKFUpdate(dz, H, R, KFFilterType::EKF);
        stateFeedback();
        std::cout << std::format("{:10.3f} nhc=1\n", imucur_.time);
    }
    return nv;
}
int GIEngine::zupt(PVA pvacur_) {
    Eigen::Matrix3d Cnb = pvacur_.att.cbn.transpose();
    Eigen::MatrixXd T1, T2, T3;

    T1 = Cnb.row(0);
    T2 = Cnb.row(1);
    T3 = Cnb.row(2);

    Eigen::Vector3d vn = pvacur_.vel;
    Eigen::Vector3d vb = Cnb * vn;
    Eigen::MatrixXd H, R, dz;
    dz.resize(3, 1);
    R.resize(3, 3);
    R.setZero();
    H.resize(3, Cov_.rows());
    H.setZero();
    H.block(0, V_ID, 3, 3) << Cnb;
    H.block(0, PHI_ID, 3, 3) << Cnb * Rotation::skewSymmetric(vn);
    dz(0)   = T1(0) * vn(0) + T1(1) * vn(1) + T1(2) * vn(2);
    dz(1)   = T2(0) * vn(0) + T2(1) * vn(1) + T2(2) * vn(2);
    dz(2)   = T3(0) * vn(0) + T3(1) * vn(1) + T3(2) * vn(2);
    R(0, 0) = pow(0.1, 2);
    R(1, 1) = pow(0.1, 2);
    R(2, 2) = pow(0.1, 2);
    EKFUpdate(dz, H, R, KFFilterType::EKF);
    stateFeedback();
    return 0;
}

bool GIEngine::isStatic() {
    bool isZupt{false};
    bool flag_t{false};
    Eigen::Vector3d accmean{0.0, 0.0, 0.0};
    Eigen::Vector3d gyromean{0.0, 0.0, 0.0};
    Eigen::Vector3d accstd{0.0, 0.0, 0.0};
    Eigen::Vector3d gyrstd{0.0, 0.0, 0.0};
    Eigen::Vector3d resacc{0.0, 0.0, 0.0};
    Eigen::Vector3d resgyr{0.0, 0.0, 0.0};
    // GNSS速度小于阈值，直接成立
    if (gnssdata_.vel.norm() != 0.0 && gnssdata_.vel.norm() < options_.engineopt.zuptopt.vel_threshold) {
        isZupt = true;
    }
    // 利用双向队列存储imu原始数据，用于进行静止判断
    if (imuwindow_.empty() || imuwindow_.back().time - imuwindow_.front().time <= options_.engineopt.zuptopt.interval) {
        imuwindow_.push_back(imucur_);
    } else {
        // 退一个头部元素，在尾部插入一个新元素
        imuwindow_.pop_front();
        imuwindow_.push_back(imucur_);
        // 计算IMU数据的平均加速度和加速度标准差
        for (auto imu : imuwindow_) {
            accmean += imu.dvel / imu.dt;
        }
        accmean /= imuwindow_.size();
        // 计算IMU数据的平均角速度和角速度标准差
        for (auto imu : imuwindow_) {
            gyromean += imu.dtheta / imu.dt;
        }
        gyromean /= imuwindow_.size();
        for (auto imu : imuwindow_) {
            accstd += (imu.dvel - accmean).cwiseProduct(imu.dvel - accmean);
        }
        accstd /= imuwindow_.size();
        // 开方
        accstd = accstd.cwiseSqrt();
        for (auto imu : imuwindow_) {
            gyrstd += (imu.dtheta - gyromean).cwiseProduct(imu.dtheta - gyromean);
        }
        gyrstd /= imuwindow_.size();
        gyrstd = gyrstd.cwiseSqrt();
        // 计算当前历元加速度角速度与均值的差
        resacc = imucur_.dvel - accmean;
        resgyr = imucur_.dtheta - gyromean;
        // 3-sigma准则检验当前IMU数值是否异常
        if (resacc.norm() < 3 * accstd.norm() && resgyr.norm() < 3 * gyrstd.norm()) {
            flag_t = true;
        }
        if (flag_t && accstd.norm() < options_.engineopt.zuptopt.fb_threshold &&
            gyrstd.norm() * R2D < options_.engineopt.zuptopt.wib_threshold) {
            isZupt = true;
        }
        //
        // LOGI << "It's static now:" << std::format("{:.4f}", imucur_.time) << std::endl;
    }
    return isZupt;
}