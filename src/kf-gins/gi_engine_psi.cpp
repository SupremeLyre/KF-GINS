#include "gi_engine_psi.h"
#include "Eigen/Geometry"
#include "common/earth.h"
void GIEngine_PSI::insPropagation(IMU &imupre, IMU &imucur) {

    // 对当前IMU数据(imucur)补偿误差, 上一IMU数据(imupre)已经补偿过了
    // compensate imu error to 'imucur', 'imupre' has been compensated
    imuCompensate(imucur);
    // IMU状态更新(机械编排算法)
    // update imustate(mechanization)
    INSMech::insMech(pvapre_, pvacur_, imupre, imucur);
    pvacur_.status |= 0b1000;
    // 系统噪声传播，姿态误差采用phi角误差模型
    // system noise propagate, phi-angle error model for attitude error
    Eigen::MatrixXd Phi, F, Qd, G;

    // 初始化Phi阵(状态转移矩阵)，F阵，Qd阵(传播噪声阵)，G阵(噪声驱动阵)
    // initialize Phi (state transition), F matrix, Qd(propagation noise) and G(noise driven) matrix
    Phi.resizeLike(Cov_);
    F.resizeLike(Cov_);
    Qd.resizeLike(Cov_);
    if (engineopt_.estimate_scale) {
        G.resize(RANK, NOISERANK);
    } else {
        G.resize(RANKLITE, NOISERANKLITE);
    }
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
    wie_n   = Earth::wien(pvapre_.pos[0]);
    wen_n   = Earth::wenn(rmrn, pvapre_.pos, pvapre_.vel);

    Eigen::Matrix3d temp;
    Eigen::Vector3d accel, omega;
    double rmh, rnh;

    rmh   = rmrn[0] + pvapre_.pos[2];
    rnh   = rmrn[1] + pvapre_.pos[2];
    accel = imucur.accel;
    omega = imucur.omega;

    // 位置误差
    // position error
    temp                      = -Rotation::skewSymmetric(wen_n);
    F.block(P_ID, P_ID, 3, 3) = temp;
    F.block(P_ID, V_ID, 3, 3) = Eigen::Matrix3d::Identity();

    // 速度误差
    // velocity error
    temp.setZero();
    double sqrt_RmRn          = sqrt(rmrn[0] * rmrn[1]);
    temp(0, 0)                = -gravity * (rmrn[0] + pvapre_.pos[2]);
    temp(1, 1)                = -gravity * (rmrn[1] + pvapre_.pos[2]);
    temp(2, 2)                = -2 * gravity * (sqrt_RmRn + pvapre_.pos[2]);
    F.block(V_ID, P_ID, 3, 3) = temp;
    temp                      = Rotation::skewSymmetric(-2 * wie_n + wen_n);
    F.block(V_ID, V_ID, 3, 3) = temp;
    // F_v_phi=fc× n系的比力积分项变化率的反对称阵
    F.block(V_ID, PHI_ID, 3, 3) = Rotation::skewSymmetric(pvapre_.att.cbn * accel);
    F.block(V_ID, BA_ID, 3, 3)  = pvapre_.att.cbn;
    if (engineopt_.estimate_scale) {
        F.block(V_ID, SA_ID, 3, 3) = pvapre_.att.cbn * (accel.asDiagonal());
    }

    // 姿态误差
    // attitude error
    temp.setZero();
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
    if (engineopt_.enable_zupt && isStatic(average)) {
        int nv = zupt(pvacur_);
        // imuerror_.gyrbias[0] = average[0];
        // imuerror_.gyrbias[1] = average[1];
        // imuerror_.gyrbias[2] = average[2];
        pvacur_.status |= 0b0100;
    }
    // 使用NHC添加约束
    // add constraint using NHC
    // || (imucur_.time > 188723 && imucur_.time < 188771)
    // if (engineopt_.enable_nhc && ((imucur_.time > 188431 && imucur_.time < 188540))) {
    if (engineopt_.enable_nhc && (fabs(imucur_.time - updatetime) >= 1.0 || gnssdata_.std.norm() > 4.0)) {
        // if (engineopt_.enable_nhc) {
        int nv = nhc(pvacur_);
        pvacur_.status |= 0b0010;
    }
}
void GIEngine_PSI::stateFeedback() {
    Eigen::Vector3d vectemp;

    // 位置误差反馈
    // posisiton error feedback
    Eigen::Vector3d delta_r = dx_.block(P_ID, 0, 3, 1);
    Eigen::Matrix3d Dr_inv  = Earth::DRi(pvacur_.pos);
    Eigen::Vector3d dpos    = Dr_inv * delta_r;
    // 计算n系和c系的误差角theta的等效旋转矢量
    Eigen::Vector3d delta_theta = Earth::dpos2rvec(pvacur_.pos, dpos[0], dpos[1]);
    Eigen::Quaterniond qn       = Rotation::rotvec2quaternion(-delta_theta);
    Eigen::Quaterniond qne      = Earth::qne(pvacur_.pos);
    qne                         = qne * qn;
    pvacur_.pos                 = Earth::blh(qne, pvacur_.pos[2] - dpos[2]);

    // 速度误差反馈
    // velocity error feedback
    vectemp = dx_.block(V_ID, 0, 3, 1);
    Eigen::Matrix3d Ccn = Eigen::Matrix3d::Identity();
    Ccn += Rotation::skewSymmetric(delta_theta);
    pvacur_.vel = Ccn * (pvacur_.vel - vectemp);

    // 姿态误差反馈
    // attitude error feedback
    vectemp                = dx_.block(PHI_ID, 0, 3, 1) + delta_theta;
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