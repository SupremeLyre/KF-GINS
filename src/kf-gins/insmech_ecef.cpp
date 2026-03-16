#include "insmech_ecef.hpp"
#include "Eigen/Core"
#include "common/earth.h"
#include "common/rotation.h"
void INSMechECEF::velUpdateECEF(const PVA &pvapre, PVA &pvacur, const IMU &imupre, const IMU &imucur) {

    Eigen::Vector3d d_vfb, d_vfe, d_vge, gl, midvel, midpos, midxyz;
    Eigen::Vector3d temp1, temp2, temp3;
    Eigen::Matrix3d cee, I33 = Eigen::Matrix3d::Identity();
    Eigen::Quaterniond qee, qbb;

    // 旋转效应和双子样划桨效应
    // rotational and sculling motion
    temp1 = imucur.dtheta.cross(imucur.dvel) / 2;
    temp2 = imupre.dtheta.cross(imucur.dvel) / 12;
    temp3 = imupre.dvel.cross(imucur.dtheta) / 12;

    // b系比力积分项
    // velocity increment due to the specific force
    d_vfb = imucur.dvel + temp1 + temp2 + temp3;

    // 比力积分项投影到e系
    // velocity increment dut to the specfic force projected to the e-frame
    temp1 << 0, 0, WGS84_WIE * imucur.dt;
    cee   = I33 - Rotation::skewSymmetric(temp1);
    d_vfe = cee * pvapre.att.cbe * d_vfb;

    // 计算重力/哥式积分项
    // velocity increment due to the gravity and Coriolis force
    Eigen::Vector3d wie_e;
    double gravity = Earth::gravity(pvapre.pos);
    gl << 0, 0, gravity;
    wie_e << 0, 0, WGS84_WIE;
    d_vge = (Earth::cne(pvapre.pos) * gl - (2 * wie_e).cross(pvapre.vel)) * imucur.dt;

    // 得到中间时刻速度和位置
    // velocity and ECEF position at k-1/2
    midvel = pvapre.vel + (d_vfe + d_vge) / 2;
    midxyz = pvapre.xyz + midvel * imucur.dt / 2;
    midpos = Earth::ecef2blh(midxyz);

    // 重新计算重力、哥式积分项
    // recompute d_vgn
    gl << 0, 0, Earth::gravity(midpos);
    d_vge = (Earth::cne(midpos) * gl - (2 * wie_e).cross(midvel)) * imucur.dt;

    // 速度更新完成
    // velocity update finish
    pvacur.vel_ecef = pvapre.vel_ecef + d_vfe + d_vge;
}
void INSMechECEF::posUpdateECEF(const PVA &pvapre, PVA &pvacur, const IMU &imupre, const IMU &imucur) {

    Eigen::Vector3d temp1, temp2, midvel_ecef, midpos;
    Eigen::Quaterniond qne, qee, qnn;

    // 计算中间时刻的速度
    // compute velocity at k-1/2
    midvel_ecef = (pvacur.vel_ecef + pvapre.vel_ecef) / 2;
    pvacur.xyz  = pvapre.xyz + midvel_ecef * imucur.dt;
}
void INSMechECEF::attUpdateECEF(const PVA &pvapre, PVA &pvacur, const IMU &imupre, const IMU &imucur) {

    Eigen::Quaterniond qee, qbb;
    Eigen::Vector3d temp1;

    // 计算k-1到k时刻e系转动等效旋转矢量，然后转换为四元数
    // caculate e-frame rotation quaternion (e(k) with respect to e(k-1)-frame)
    temp1 << 0, 0, WGS84_WIE * imucur.dt;
    qee = Rotation::rotvec2quaternion(temp1);

    // 计算b系旋转四元数 补偿二阶圆锥误差
    // b-frame rotation vector (b(k) with respect to b(k-1)-frame)
    // compensate the second-order coning correction term.
    temp1 = imucur.dtheta + imupre.dtheta.cross(imucur.dtheta) / 12;
    qbb   = Rotation::rotvec2quaternion(temp1);

    // 姿态更新完成
    // attitude update finish
    pvacur.att.qbe = (qee * pvapre.att.qbe * qbb).normalized();
    pvacur.att.cbe = Rotation::quaternion2matrix(pvacur.att.qbe);
}
void INSMechECEF::updateStateNED(const PVA &pvapre, PVA &pvacur) {
    pvacur.pos       = Earth::ecef2blh(pvacur.xyz);
    pvacur.vel       = Earth::cne(pvacur.pos).transpose() * pvacur.vel_ecef;
    pvacur.att.qbn   = Earth::qne(pvacur.pos).inverse() * pvacur.att.qbe;
    pvacur.att.cbn   = Earth::cne(pvacur.pos).transpose() * pvacur.att.cbe;
    pvacur.att.euler = Rotation::matrix2euler(pvacur.att.cbn);
}