#pragma once
#include "common/types.h"

#include "kf_gins_types.h"

class INSMechECEF {
public:
    static void insMechECEF(const PVA &pvapre, PVA &pvacur, const IMU &imupre, const IMU &imucur) {
        attUpdateECEF(pvapre, pvacur, imupre, imucur);
        velUpdateECEF(pvapre, pvacur, imupre, imucur);
        posUpdateECEF(pvapre, pvacur, imupre, imucur);
        updateStateNED(pvapre, pvacur);
    }

private:
    static void posUpdateECEF(const PVA &pvapre, PVA &pvacur, const IMU &imupre, const IMU &imucur);
    static void velUpdateECEF(const PVA &pvapre, PVA &pvacur, const IMU &imupre, const IMU &imucur);
    static void attUpdateECEF(const PVA &pvapre, PVA &pvacur, const IMU &imupre, const IMU &imucur);
    static void updateStateNED(const PVA &pvapre, PVA &pvacur);
};