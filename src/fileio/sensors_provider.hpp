#pragma once
#include "fileloader.h"
#include "common/types.h"
// 针对 GNSS 提供者的接口
class IGnssFileLoader : public FileLoader {
public:
    virtual ~IGnssFileLoader() = default;
    virtual const GNSS &next() = 0;
};

// 针对 IMU 提供者的接口
class IImuFileLoader : public FileLoader {
public:
    virtual ~IImuFileLoader()  = default;
    virtual const IMU &next()  = 0;
    virtual double starttime() = 0;
    virtual double endtime()   = 0;
};