#include "imu.h"
#include "MPU9250.h"
#include "globals.h"

// SECTION 1: IMU

IMU::IMU()
{

    Globals &gInstance = Globals::getInstance();

    // configure the IMU
    settings.accel_fs_sel = ACCEL_FS_SEL::A8G;
    settings.gyro_fs_sel = GYRO_FS_SEL::G250DPS;
    settings.gyro_dlpf_cfg = GYRO_DLPF_CFG::DLPF_20HZ;
    settings.accel_dlpf_cfg = ACCEL_DLPF_CFG::DLPF_21HZ;

    initialized = true;

    // initialize the IMU
    gInstance.setError(2, 1);

    long start_time = millis();
    while (!mpu.setup(ADDR, settings))
    {
        if (millis() - start_time > 10000)
        {
            initialized = false;
            break;
        }
    }

    // setError based on the initialization
    if (initialized)
        gInstance.setError(0, 1);
    else
    {
        gInstance.setError(1, 1);
        return;
    }

    // TODO : make dynamic
    // mpu.setAccBias(753.27, -446.93, 1055.02);
    // mpu.setGyroBias(41.95, 303.32, 113.12);
    // mpu.setMagBias(184.57, 336.55, 24.76);

    mpu.setAccBias(ACC_X_BIAS_ADDR, ACC_Y_BIAS_ADDR, ACC_Z_BIAS_ADDR);
    mpu.setGyroBias(GYRO_X_BIAS_ADDR, GYRO_Y_BIAS_ADDR, GYRO_Z_BIAS_ADDR);
    mpu.setMagBias(MAG_X_BIAS_ADDR, MAG_Y_BIAS_ADDR, MAG_Z_BIAS_ADDR);
}

// return state of initialization

// return the instance of the IMU
IMU &IMU::getInstance()
{
    static IMU instance;
    return instance;
}

// get the angles from the IMU
void IMU::getAngles(float &pitch, float &roll, float &yaw)
{
    mpu.update();

    pitch = mpu.getPitch();
    roll = mpu.getRoll();
    yaw = mpu.getYaw();
}

void IMU::callibrate()
{
    // TODO
}