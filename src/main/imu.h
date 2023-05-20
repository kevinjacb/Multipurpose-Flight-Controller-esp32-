#ifndef IMU_H
#define IMU_H

#include <Arduino.h>
#include "MPU9250.h"

class IMU
{
public:
    void getAngles(float &pitch, float &roll, float &yaw);
    static IMU &getInstance();
    void calibrate(float &pitch_offset, float &roll_offset, float &yaw_offset);
    void begin();

private:
    MPU9250 mpu;
    MPU9250Setting settings;
    bool initialized;

    IMU(const IMU &) = delete;
    IMU();
};

#endif // IMU_H
