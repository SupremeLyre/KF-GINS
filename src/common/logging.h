/*
 * OB_GINS: An Optimization-Based GNSS/INS Integrated Navigation System
 *
 * Copyright (C) 2022 i2Nav Group, Wuhan University
 *
 *     Author : Hailiang Tang
 *    Contact : thl@whu.edu.cn
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

#ifndef LOGGING_H
#define LOGGING_H

#include <Eigen/Geometry>
#include <absl/strings/str_format.h>
#include <iostream>
#include <string>

using std::string;

class Logging {

public:
    static void initialization(char **argv, bool logtostderr = true, bool logtofile = true) {
        (void) argv;
        (void) logtostderr;
        (void) logtofile;
    }

    template <typename T, int Rows, int Cols>
    static void printMatrix(const Eigen::Matrix<T, Rows, Cols> &matrix, const string &prefix = "Matrix: ") {
        std::cout << prefix << matrix.rows() << "x" << matrix.cols() << std::endl;
        if (matrix.cols() == 1) {
            std::cout << matrix.transpose() << std::endl;
        } else {
            std::cout << matrix << std::endl;
        }
    }

    static string doubleData(double data) {
        return absl::StrFormat("%0.6lf", data);
    }
};

#endif // LOGGING_H
