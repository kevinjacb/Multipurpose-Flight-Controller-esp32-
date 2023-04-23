#ifndef IMU_H
#define IMU_H

#include <Arduino.h>
#include "MPU9250.h"

class IMU
{
public:
    void getAngles(float &pitch, float &roll, float &yaw);
    static IMU &getInstance();

private:
    MPU9250 mpu;
    MPU9250Setting settings;
    bool initialized;

    static IMU instance;

    IMU(const IMU &) = delete;
    IMU();
};

#endif // IMU_H
