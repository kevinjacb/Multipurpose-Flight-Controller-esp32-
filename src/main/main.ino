#include <Arduino.h>
#include "imu.h"
#include "errorNotifier.h"
#include "globals.h"
#include "pid.h"

volatile bool ready = false;

ErrorNotifier &errorNotifier = ErrorNotifier::getInstance();
IMU &imu = IMU::getInstance();
PIDController &pid = PIDController::getInstance();

void setup()
{
    if (imu.isInitialized())
        ready = true;
}

void loop()
{
    errorNotifier.notifyError();
}
