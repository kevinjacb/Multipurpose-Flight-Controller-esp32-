#ifndef MOTORS_H
#define MOTORS_H

#include <Arduino.h>
#include "globals.h"
#include <ESP32Servo.h>

class Outputs
{
private:
    Outputs();
    Outputs(const Outputs &) = delete; // disable copy constructor
    uint8_t *motorPins;
    Servo motors[NUM_MOTORS];

public:
    void setOutputs(volatile output_t &outputs);
    static Outputs &getInstance();
    void begin();
};

#endif