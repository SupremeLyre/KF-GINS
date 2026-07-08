/*
 * KF-GINS: An EKF-Based GNSS/INS Integrated Navigation System
 *
 * Copyright (C) 2022 i2Nav Group, Wuhan University
 *
 *     Author : Liqiang Wang
 *    Contact : wlq@whu.edu.cn
 * Modified by : SupremeLyre(Leran Fu), Wuhan University
 * Reference :
 * Yarong Luo, Fei Lu, Chi Guo, and Jingnan Liu,
 * "Matrix Lie Group-Based Extended Kalman Filtering for
 * Inertial-Integrated Navigation in the Navigation Frame", 2024
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 */
#pragma once

#include "gi_engine.h"

class GIEngine_LEQF : public GIEngine {
    using GIEngine::GIEngine;

private:
    void insPropagation(IMU &imupre, IMU &imucur) override;
    void stateFeedback() override;
    void gnssUpdate(GNSS &gnssdata) override;

    int leqfZupt(const PVA &pvacur);
    int leqfNhc(const PVA &pvacur);
};
