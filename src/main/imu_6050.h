#ifndef IMU_H_6050
#define IMU_H_6050

#include <Arduino.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "globals.h"
#include "errorNotifier.h"
#include <Wire.h>

// Section 1

class IMU_6050
{

private:
    MPU6050 mpu;
    bool dmpReady = false;
    uint8_t mpuIntStatus;
    uint8_t devStatus;
    uint16_t packetSize;
    uint16_t fifoCount;
    uint8_t fifoBuffer[64];

    Quaternion q;
    VectorInt16 aa;
    VectorInt16 aaReal;
    VectorInt16 aaWorld;
    VectorFloat gravity;
    float euler[3];
    float ypr[3];
    float prev_yaw = 0;
    long last_read = 0;

    static volatile bool mpuInterrupt;

    static void dmpDataReady()
    {
        mpuInterrupt = true;
    }

    IMU_6050(const IMU_6050 &) = delete;
    IMU_6050() {}

public:
    void getAngles(float &pitch, float &roll, float &yaw, float &yaw_angle)
    {
        if (!dmpReady)
            return;

        // display Euler angles in degrees
        if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer))
        {
            int16_t ax, ay, az, gx, gy, gz;
            mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            pitch = (float)ypr[1] * 180.0 / (float)M_PI;
            roll = (float)ypr[2] * 180.0 / (float)M_PI;
            yaw_angle = (float)ypr[0] * 180.0 / (float)M_PI;
            yaw = (float)gz / 131.0f;

            // if (millis() - last_read > 4)
            // {
            //     prev_yaw = (float)ypr[0] * 180.0 / (float)M_PI;
            //     last_read = millis();
            // }

#if defined(ENABLE_DEBUG)
            if (false)
            {
                Serial.print("pitch:\t");
                Serial.print(pitch);
                Serial.print("\troll:\t");
                Serial.print(roll);
                Serial.print("\tyaw:\t");
                Serial.println(yaw);
            }
#endif
        }
    }

    static IMU_6050 &getInstance()
    {
        static IMU_6050 instance;
        return instance;
    }
    void calibrate(float &offset_pitch, float &offset_roll, float &offset_yaw)
    {
        Globals &instance = Globals::getInstance();
        instance.setError(2, 1);
        float pitch, roll, yaw, yaw_angle;
        for (int i = 0; i < 100; i++)
        {
            getAngles(pitch, roll, yaw, yaw_angle);
            offset_pitch += pitch;
            offset_roll += roll;
            offset_yaw += yaw;
            delay(10);
        }
        offset_pitch /= 100;
        offset_roll /= 100;
        offset_yaw /= 100;
        instance.setError(0, 1);
    }

    void begin()
    {
        Globals &instance = Globals::getInstance();
        ErrorNotifier &errorNotifier = ErrorNotifier::getInstance();
        instance.setError(2, 1);
        errorNotifier.notifyError();

        Wire.begin();
        Wire.setClock(400000);
        // initialize device
        Serial.println(F("Initializing I2C devices..."));
        mpu.initialize();
        pinMode(INTERRUPT_PIN, INPUT);
        // verify connection
        Serial.println(F("Testing device connections..."));
        Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
        // load and configure the DMP
        Serial.println(F("Initializing DMP..."));
        delay(2000);
        devStatus = mpu.dmpInitialize();
        // supply your own gyro offsets here, scaled for min sensitivity
        mpu.setXGyroOffset(188);
        mpu.setYGyroOffset(-28);
        mpu.setZGyroOffset(44);
        mpu.setXAccelOffset(-2180);
        mpu.setYAccelOffset(371);
        mpu.setZAccelOffset(-40); // 1688 factory default for my test chip
        //-2180	371	-40	188	-28	44
        // make sure it worked (returns 0 if so)
        mpuInterrupt = false;

        if (devStatus == 0)
        {
            // turn on the DMP, now that it's ready
            Serial.println(F("Enabling DMP..."));
            mpu.setDMPEnabled(true);
            // enable Arduino interrupt detection
            Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
            attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
            mpuIntStatus = mpu.getIntStatus();
            // set our DMP Ready flag so the main loop() function knows it's okay to use it
            Serial.println(F("DMP ready! Waiting for first interrupt..."));
            dmpReady = true;
            // get expected DMP packet size for later comparison
            packetSize = mpu.dmpGetFIFOPacketSize();
        }
        else
        {
            // ERROR!
            // 1 = initial memory load failed
            // 2 = DMP configuration updates failed
            // (if it's going to break, usually the code will be 1)
            Serial.print(F("DMP Initialization failed (code "));
            Serial.print(devStatus);
            Serial.println(F(")"));
        }

        instance.setError(0, 1);
        errorNotifier.notifyError();
    }

    void setOffsets(int16_t x, int16_t y, int16_t z)
    {
        mpu.setXAccelOffset(x);
        mpu.setYAccelOffset(y);
        mpu.setZAccelOffset(z);
    }
};

volatile bool IMU_6050::mpuInterrupt = false;

#endif // IMU_H_6050