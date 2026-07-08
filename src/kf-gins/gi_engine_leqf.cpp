#include "gi_engine_leqf.h"

#include "common/angle.h"
#include "common/earth.h"
#include "common/rotation.h"
#include "kf-gins/insmech.h"

#include <cmath>

namespace {
using Matrix96d = Eigen::Matrix<double, 9, 6>;
using Matrix9d  = Eigen::Matrix<double, 9, 9>;

enum LieNavID {
    LPHI_ID = 0,
    LV_ID   = 3,
    LP_ID   = 6,
};

Eigen::Matrix3d gammaFunc(const Eigen::Vector3d &phi, int order) {
    const Eigen::Matrix3d I  = Eigen::Matrix3d::Identity();
    const Eigen::Matrix3d S  = Rotation::skewSymmetric(phi);
    const Eigen::Matrix3d S2 = S * S;
    const double theta       = phi.norm();

    if (theta < 1e-6) {
        switch (order) {
            case 0:
                return I + S + 0.5 * S2;
            case 1:
                return I + 0.5 * S + (1.0 / 6.0) * S2;
            case 2:
                return 0.5 * I + (1.0 / 6.0) * S + (1.0 / 24.0) * S2;
            case 3:
                return (1.0 / 6.0) * I + (1.0 / 24.0) * S + (1.0 / 120.0) * S2;
            default:
                return I;
        }
    }

    const double theta2 = theta * theta;
    const double theta3 = theta2 * theta;
    const double theta4 = theta2 * theta2;
    const double theta5 = theta4 * theta;

    switch (order) {
        case 0:
            return I + (std::sin(theta) / theta) * S + ((1.0 - std::cos(theta)) / theta2) * S2;
        case 1:
            return I + ((1.0 - std::cos(theta)) / theta2) * S + ((theta - std::sin(theta)) / theta3) * S2;
        case 2:
            return 0.5 * I + ((theta - std::sin(theta)) / theta3) * S +
                   ((theta2 + 2.0 * std::cos(theta) - 2.0) / (2.0 * theta4)) * S2;
        case 3:
            return (1.0 / 6.0) * I + ((theta2 + 2.0 * std::cos(theta) - 2.0) / (2.0 * theta4)) * S +
                   ((theta3 + 6.0 * std::sin(theta) - 6.0 * theta) / (6.0 * theta5)) * S2;
        default:
            return I;
    }
}

Matrix9d leftNavTransition(const IMU &imu) {
    const double dt           = imu.dt;
    const Eigen::Vector3d phi = imu.dtheta;
    const Eigen::Vector3d dv  = imu.dvel;

    const Eigen::Matrix3d Gamma0     = gammaFunc(phi, 0);
    const Eigen::Matrix3d Gamma1     = gammaFunc(phi, 1);
    const Eigen::Matrix3d Gamma2     = gammaFunc(phi, 2);
    const Eigen::Matrix3d Gamma0_inv = Gamma0.transpose();

    const Eigen::Vector3d upsilon_v = Gamma1 * dv;
    const Eigen::Vector3d upsilon_r = Gamma2 * dv * dt;

    Matrix9d F                  = Matrix9d::Identity();
    F.block<3, 3>(LP_ID, LV_ID) = dt * Eigen::Matrix3d::Identity();

    Matrix9d Ad_upsilon_inv                      = Matrix9d::Zero();
    Ad_upsilon_inv.block<3, 3>(LPHI_ID, LPHI_ID) = Gamma0_inv;
    Ad_upsilon_inv.block<3, 3>(LV_ID, LPHI_ID)   = -Gamma0_inv * Rotation::skewSymmetric(upsilon_v);
    Ad_upsilon_inv.block<3, 3>(LV_ID, LV_ID)     = Gamma0_inv;
    Ad_upsilon_inv.block<3, 3>(LP_ID, LPHI_ID)   = -Gamma0_inv * Rotation::skewSymmetric(upsilon_r);
    Ad_upsilon_inv.block<3, 3>(LP_ID, LP_ID)     = Gamma0_inv;

    return Ad_upsilon_inv * F;
}

Matrix96d localNoiseJacobian(const IMU &imu) {
    const double dt             = imu.dt;
    const Eigen::Vector3d phi   = imu.dtheta;
    const Eigen::Vector3d accel = imu.accel;

    const Eigen::Matrix3d Gamma0_inv = gammaFunc(phi, 0).transpose();
    const Eigen::Matrix3d Gamma1     = gammaFunc(phi, 1);
    const Eigen::Matrix3d Gamma2     = gammaFunc(phi, 2);
    const Eigen::Matrix3d Gamma2_neg = gammaFunc(-phi, 2);
    const Eigen::Matrix3d Gamma3_neg = gammaFunc(-phi, 3);
    const Eigen::Matrix3d accel_skew = Rotation::skewSymmetric(accel);

    Matrix96d G_lie               = Matrix96d::Zero();
    G_lie.block<3, 3>(LPHI_ID, 0) = Gamma0_inv * Gamma1 * dt;
    G_lie.block<3, 3>(LV_ID, 0)   = Gamma0_inv * (-Gamma1 * accel_skew * Gamma2_neg * dt * dt);
    G_lie.block<3, 3>(LV_ID, 3)   = Gamma0_inv * Gamma1 * dt;
    G_lie.block<3, 3>(LP_ID, 0)   = Gamma0_inv * (-Gamma2 * accel_skew * Gamma3_neg * dt * dt * dt);
    G_lie.block<3, 3>(LP_ID, 3)   = Gamma0_inv * Gamma2 * dt * dt;
    return G_lie;
}

void copyLieNavToEngineOrder(const Matrix9d &lie, Eigen::MatrixXd &target, int row, int col) {
    target.block(row + 0, col + 0, 3, 3) = lie.block(LP_ID, LP_ID, 3, 3);
    target.block(row + 0, col + 3, 3, 3) = lie.block(LP_ID, LV_ID, 3, 3);
    target.block(row + 0, col + 6, 3, 3) = lie.block(LP_ID, LPHI_ID, 3, 3);
    target.block(row + 3, col + 0, 3, 3) = lie.block(LV_ID, LP_ID, 3, 3);
    target.block(row + 3, col + 3, 3, 3) = lie.block(LV_ID, LV_ID, 3, 3);
    target.block(row + 3, col + 6, 3, 3) = lie.block(LV_ID, LPHI_ID, 3, 3);
    target.block(row + 6, col + 0, 3, 3) = lie.block(LPHI_ID, LP_ID, 3, 3);
    target.block(row + 6, col + 3, 3, 3) = lie.block(LPHI_ID, LV_ID, 3, 3);
    target.block(row + 6, col + 6, 3, 3) = lie.block(LPHI_ID, LPHI_ID, 3, 3);
}

void copyLieNoiseToEngineOrder(const Matrix96d &lie, Eigen::MatrixXd &target, int row, int col) {
    target.block(row + 0, col, 3, 6) = lie.block(LP_ID, 0, 3, 6);
    target.block(row + 3, col, 3, 6) = lie.block(LV_ID, 0, 3, 6);
    target.block(row + 6, col, 3, 6) = lie.block(LPHI_ID, 0, 3, 6);
}
} // namespace

void GIEngine_LEQF::insPropagation(IMU &imupre, IMU &imucur) {
    imuCompensate(imucur);

    INSMech::insMech(pvapre_, pvacur_, imupre, imucur);
    pvacur_.status |= 0b1000;

    Eigen::MatrixXd Phi, Qd;
    Phi.resizeLike(Cov_);
    Qd.resizeLike(Cov_);
    Phi.setIdentity();
    Qd.setZero();

    Matrix9d Phi_lie = leftNavTransition(imucur);
    Matrix96d G_lie  = localNoiseJacobian(imucur);
    Eigen::MatrixXd G_nav(Cov_.rows(), 6);
    G_nav.setZero();

    copyLieNavToEngineOrder(Phi_lie, Phi, P_ID, P_ID);
    copyLieNoiseToEngineOrder(G_lie, Phi, P_ID, BG_ID);
    copyLieNoiseToEngineOrder(G_lie, G_nav, P_ID, 0);

    const double bias_phi         = std::exp(-imucur.dt / options_.imunoise.corr_time);
    Phi.block(BG_ID, BG_ID, 3, 3) = bias_phi * Eigen::Matrix3d::Identity();
    Phi.block(BA_ID, BA_ID, 3, 3) = bias_phi * Eigen::Matrix3d::Identity();

    Eigen::Matrix<double, 6, 6> Qimu = Eigen::Matrix<double, 6, 6>::Zero();
    Qimu.block<3, 3>(0, 0)           = Qc_.block(ARW_ID, ARW_ID, 3, 3);
    Qimu.block<3, 3>(3, 3)           = Qc_.block(VRW_ID, VRW_ID, 3, 3);
    Qd += G_nav * (Qimu / imucur.dt) * G_nav.transpose();
    Qd.block(BG_ID, BG_ID, 3, 3) += Qc_.block(BGSTD_ID, BGSTD_ID, 3, 3) * imucur.dt;
    Qd.block(BA_ID, BA_ID, 3, 3) += Qc_.block(BASTD_ID, BASTD_ID, 3, 3) * imucur.dt;

    if (engineopt_.estimate_scale) {
        Eigen::MatrixXd G_scale(Cov_.rows(), 6);
        G_scale.setZero();
        G_scale.block(P_ID, 0, 9, 3)  = G_nav.block(P_ID, 0, 9, 3) * imucur.omega.asDiagonal();
        G_scale.block(P_ID, 3, 9, 3)  = G_nav.block(P_ID, 3, 9, 3) * imucur.accel.asDiagonal();
        Phi.block(P_ID, SG_ID, 9, 3)  = G_scale.block(P_ID, 0, 9, 3);
        Phi.block(P_ID, SA_ID, 9, 3)  = G_scale.block(P_ID, 3, 9, 3);
        Phi.block(SG_ID, SG_ID, 3, 3) = bias_phi * Eigen::Matrix3d::Identity();
        Phi.block(SA_ID, SA_ID, 3, 3) = bias_phi * Eigen::Matrix3d::Identity();
        Qd.block(SG_ID, SG_ID, 3, 3) += Qc_.block(SGSTD_ID, SGSTD_ID, 3, 3) * imucur.dt;
        Qd.block(SA_ID, SA_ID, 3, 3) += Qc_.block(SASTD_ID, SASTD_ID, 3, 3) * imucur.dt;
    } else if (engineopt_.estimate_mount_angle) {
        Qd.block(MOUNT_ID, MOUNT_ID, 3, 3) += Qc_.block(MOUNTSTD_ID, MOUNTSTD_ID, 3, 3) * imucur.dt;
    }

    EKFPredict(Phi, Qd);

    if (engineopt_.enable_zupt && isStatic(average)) {
        leqfZupt(pvacur_);
        pvacur_.status |= 0b0100;
    }

    if (engineopt_.enable_nhc && (fabs(imucur_.time - updatetime) >= 1.0 || gnssdata_.std.norm() > 4.0)) {
        leqfNhc(pvacur_);
        pvacur_.status |= 0b0010;
    }
}

void GIEngine_LEQF::stateFeedback() {
    Eigen::Vector3d vectemp;

    vectemp = dx_.block(PHI_ID, 0, 3, 1);
    if (vectemp.norm() > 1e-15) {
        Eigen::Quaterniond qbp = Rotation::rotvec2quaternion(-vectemp);
        pvacur_.att.qbn        = (pvacur_.att.qbn * qbp).normalized();
        pvacur_.att.cbn        = Rotation::quaternion2matrix(pvacur_.att.qbn);
        pvacur_.att.euler      = Rotation::matrix2euler(pvacur_.att.cbn);
    }

    vectemp = dx_.block(V_ID, 0, 3, 1);
    pvacur_.vel -= pvacur_.att.cbn * vectemp;

    vectemp = dx_.block(P_ID, 0, 3, 1);
    pvacur_.pos -= Earth::DRi(pvacur_.pos) * (pvacur_.att.cbn * vectemp);

    vectemp = dx_.block(BG_ID, 0, 3, 1);
    imuerror_.gyrbias += vectemp;
    vectemp = dx_.block(BA_ID, 0, 3, 1);
    imuerror_.accbias += vectemp;

    if (engineopt_.estimate_scale) {
        vectemp = dx_.block(SG_ID, 0, 3, 1);
        imuerror_.gyrscale += vectemp;
        vectemp = dx_.block(SA_ID, 0, 3, 1);
        imuerror_.accscale += vectemp;
    } else if (engineopt_.estimate_mount_angle) {
        vectemp = dx_.block(MOUNT_ID, 0, 3, 1);
        if (vectemp.norm() > 1e-15) {
            Eigen::Quaterniond qpv      = Rotation::rotvec2quaternion(vectemp);
            Eigen::Vector3d delta_angle = Rotation::quaternion2euler(qpv);
            delta_angle[0]              = 0;
            qpv                         = Rotation::euler2quaternion(delta_angle);
            pvacur_.att.qbv             = (qpv * pvacur_.att.qbv).normalized();
            pvacur_.att.cbv             = Rotation::quaternion2matrix(pvacur_.att.qbv);
        }
    }

    dx_.setZero();
}

void GIEngine_LEQF::gnssUpdate(GNSS &gnssdata) {
    Eigen::Vector3d antenna_pos, antenna_vel;
    Eigen::Matrix3d Dr, Dr_inv;
    Dr_inv               = Earth::DRi(pvacur_.pos);
    Dr                   = Earth::DR(pvacur_.pos);
    antenna_pos          = pvacur_.pos + Dr_inv * pvacur_.att.cbn * options_.antlever;
    Eigen::Vector2d rmrn = Earth::meridianPrimeVerticalRadius(pvacur_.pos[0]);
    Eigen::Vector3d winn = Earth::wien(pvacur_.pos[0]) + Earth::wenn(rmrn, pvacur_.pos, pvacur_.vel);
    antenna_vel          = pvacur_.vel - Rotation::skewSymmetric(winn) * (pvacur_.att.cbn * options_.antlever) -
                  pvacur_.att.cbn * (Rotation::skewSymmetric(options_.antlever) * imucur_.omega);

    if (engineopt_.enable_gnss_pos && gnssdata.isPosValid) {
        Eigen::MatrixXd dz = pvacur_.att.cbn.transpose() * (Dr * (antenna_pos - gnssdata.blh));

        Eigen::MatrixXd H;
        H.resize(3, Cov_.rows());
        H.setZero();
        H.block(0, P_ID, 3, 3)   = Eigen::Matrix3d::Identity();
        H.block(0, PHI_ID, 3, 3) = -Rotation::skewSymmetric(options_.antlever);

        Eigen::MatrixXd R = gnssdata.std.cwiseProduct(gnssdata.std).asDiagonal();
        R                 = pvacur_.att.cbn.transpose() * R * pvacur_.att.cbn;

        EKFUpdate(dz, H, R, configuredFilterType(engineopt_.kf_gnss_pos_type, KFFilterType::EKF),
                  engineopt_.kf_enable_adaptive);
    }

    if (engineopt_.enable_gnss_vel && gnssdata.isVelValid) {
        Eigen::Matrix3d Cec      = Earth::cne(pvacur_.pos).transpose();
        Eigen::Matrix3d Cne      = Earth::cne(gnssdata.blh);
        Eigen::Matrix3d Cnc      = Cec * Cne;
        Eigen::Vector3d gnss_vel = Cnc * gnssdata.vel;
        Eigen::MatrixXd dz       = pvacur_.att.cbn.transpose() * (antenna_vel - gnss_vel);

        Eigen::MatrixXd H;
        H.resize(3, Cov_.rows());
        H.setZero();
        H.block(0, V_ID, 3, 3) = Eigen::Matrix3d::Identity();
        H.block(0, PHI_ID, 3, 3) =
            Rotation::skewSymmetric(pvacur_.att.cbn.transpose() * winn) * Rotation::skewSymmetric(options_.antlever) +
            Rotation::skewSymmetric(Rotation::skewSymmetric(options_.antlever) * imucur_.omega);
        H.block(0, BG_ID, 3, 3) = -Rotation::skewSymmetric(options_.antlever);
        if (engineopt_.estimate_scale) {
            H.block(0, SG_ID, 3, 3) = -Rotation::skewSymmetric(options_.antlever) * imucur_.omega.asDiagonal();
        }

        Eigen::MatrixXd R = gnssdata.vstd.cwiseProduct(gnssdata.vstd).asDiagonal();
        R                 = pvacur_.att.cbn.transpose() * R * pvacur_.att.cbn;

        EKFUpdate(dz, H, R, configuredFilterType(engineopt_.kf_gnss_vel_type, KFFilterType::EKF),
                  engineopt_.kf_enable_adaptive);
    }

    gnssdata.isvalid = false;
    if (gnssdata.isPosValid || gnssdata.isVelValid) {
        pvacur_.status |= 0b0001;
    }
}

int GIEngine_LEQF::leqfZupt(const PVA &pvacur) {
    Eigen::MatrixXd H, R, dz;
    dz = pvacur.att.cbn.transpose() * pvacur.vel;

    H.resize(3, Cov_.rows());
    H.setZero();
    H.block(0, V_ID, 3, 3)   = Eigen::Matrix3d::Identity();
    H.block(0, PHI_ID, 3, 3) = Rotation::skewSymmetric(dz);

    Eigen::Vector3d noise_std = engineopt_.zuptopt.vel_std;
    R                         = noise_std.cwiseProduct(noise_std).asDiagonal();
    R                         = pvacur.att.cbn.transpose() * R * pvacur.att.cbn;

    EKFUpdate(dz, H, R, configuredFilterType(engineopt_.kf_zupt_type, KFFilterType::EKF),
              engineopt_.kf_enable_adaptive);
    stateFeedback();
    return 3;
}

int GIEngine_LEQF::leqfNhc(const PVA &pvacur) {
    Eigen::Matrix3d Cbn = pvacur.att.cbn.transpose();
    Eigen::Matrix3d Cvb = pvacur.att.cbv;
    Eigen::Vector3d v_b = Cbn * pvacur.vel;
    Eigen::Vector3d v_v = Cvb * v_b;

    Eigen::MatrixXd H;
    Eigen::MatrixXd dz;
    Eigen::MatrixXd R;
    H.resize(2, Cov_.rows());
    dz.resize(2, 1);
    R.resize(2, 2);
    H.setZero();
    dz.setZero();
    R.setZero();

    int nv = 0;
    if (fabs(v_v[0]) > 2) {
        for (int i = 1; i < 3; i++) {
            if (fabs(v_v[i]) > 0.5) {
                continue;
            }
            if (fabs(imucur_.dtheta.norm()) > 30 * D2R) {
                continue;
            }

            H.block(nv, V_ID, 1, 3)   = Cvb.row(i);
            H.block(nv, PHI_ID, 1, 3) = (Cvb * Rotation::skewSymmetric(v_b)).row(i);
            if (engineopt_.estimate_mount_angle) {
                H.block(nv, MOUNT_ID, 1, 3) = Rotation::skewSymmetric(v_v).row(i);
            }

            dz(nv)    = v_v[i];
            R(nv, nv) = i == 1 ? pow(engineopt_.nhcopt.lateral_cov, 2) : pow(engineopt_.nhcopt.vertical_cov, 2);
            nv++;
        }
    }

    if (nv > 1) {
        EKFUpdate(dz, H, R, configuredFilterType(engineopt_.kf_nhc_type, KFFilterType::EKF),
                  engineopt_.kf_enable_adaptive);
        stateFeedback();
    }
    return nv;
}
