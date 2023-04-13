#ifndef GLOBALS_H
#define GLOBALS_H

#include <stdint.h>
#include <mutex>

// LED Pins and Error Codes
#define RED_L 34
#define GREEN_L 25
#define BLUE_L 26
#define IMU_ERROR 1

// IMU Address
#define ADDR 0x68

// PID Gains
#define Kp 0.5
#define Ki 0.005
#define Kd 0.1
#define dt 0.01

class Globals
{
private:
    uint8_t error;
    uint8_t handler;
    std::mutex errorMutex;
    static Globals instance;

public:
    static constexpr uint8_t LED_PINS[3] = {RED_L, GREEN_L, BLUE_L};
    static Globals &getInstance();
    uint8_t getError();
    void setError(uint8_t error, uint8_t section);
};

#endif GLOBALS_H