#include <Arduino.h>
#include "imu.h"
#include "errorNotifier.h"
#include "globals.h"
#include "pid.h"
#include "wifi_bluetooth.h"
#include "motors.h"
#include <BluetoothSerial.h>

volatile bool ready = false;

ErrorNotifier &errorNotifier = ErrorNotifier::getInstance();
IMU &imu = IMU::getInstance();
PIDController &pid = PIDController::getInstance();
Globals &global = Globals::getInstance();
WiFiBluetooth &WBController = WiFiBluetooth::getInstance();
Outputs &out = Outputs::getInstance();

control_t currControls = IDLE;
output_t outputs = {0, 0, 0, 0, 0, 0};

float pitch = 0.0f, roll = 0.0f, yaw = 0.0f;

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

BluetoothSerial SerialBT;

void setup()
{
    SerialBT.begin("HAZARDOUZ"); // Bluetooth device name
}

void loop()
{
    errorNotifier.notifyError();
    if (global.getError() != 0)
    { // TODO disarm drone when error occurs
        return;
    }
#if defined(USE_WIFI) // switch between wifi and rc TODO
    currControls = WBController.receive(currControls);
#endif
}

void setup2() {}

void loop2()
{
    if (global.getError() != 0)
    { // TODO disarm drone when error occurs
        return;
    }
    imu.getAngles(pitch, roll, yaw);
    pid.update(outputs, currControls, pitch, roll, yaw);
    out.setOutputs(outputs);
}
