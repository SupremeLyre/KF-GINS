#pragma once
#include "common/angle.h"
#include "common/types.h"
#include "fileloader.h"

class ResPppFileLoader : public FileLoader {

public:
    ResPppFileLoader() = delete;
    explicit ResPppFileLoader(const string &filename, int columns = 14) {
        open(filename, columns, FileLoader::TEXT);
    }

    const GNSS &next() {
        data_ = load();
        gnss_.week = data_[0];
        gnss_.time = data_[1];
        memcpy(gnss_.blh.data(), &data_[2], 3 * sizeof(double));
        memcpy(gnss_.std.data(), &data_[5], 3 * sizeof(double));
        memcpy(gnss_.vel.data(), &data_[8], 3 * sizeof(double));
        memcpy(gnss_.vstd.data(), &data_[11], 3 * sizeof(double));
        gnss_.blh[0] *= D2R;
        gnss_.blh[1] *= D2R;

        return gnss_;
    }

private:
    GNSS gnss_;
    vector<double> data_;
};