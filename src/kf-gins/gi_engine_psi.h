/*
 * KF-GINS: An EKF-Based GNSS/INS Integrated Navigation System
 *
 * Copyright (C) 2022 i2Nav Group, Wuhan University
 *
 *     Author : Liqiang Wang
 *    Contact : wlq@whu.edu.cn
 * Modified by : SupremeLyre(Leran Fu), Wuhan University
 * Reference :
 * Eun-Hwan Shin, "Estimation Techniques for Low-Cost Inertial Navigation", 2005
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
#pragma once
#include "gi_engine.h"
#include "insmech.h"
class GIEngine_PSI : public GIEngine {
    using GIEngine::GIEngine;

private:
    void insPropagation(IMU &imupre, IMU &imucur) override;
    void stateFeedback() override;
};