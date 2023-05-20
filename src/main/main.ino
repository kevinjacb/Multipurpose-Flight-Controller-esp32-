#include <Arduino.h>

#include "globals.h"
#include "errorNotifier.h"
#include "pid.h"
#include "wifi_bluetooth.h"
#include "motors.h"

#if defined(USE_RC)
#include "rc_reciever.h"
#endif
// #include <Wire.h>
#if defined(SEL_MPU9250)
#include "imu.h"
#elif defined(SEL_MPU6050)
#include "imu_6050.h"
#endif
// #include <BluetoothSerial.h>

#define print(x) Serial.print(x)
#define println(x) Serial.println(x)

volatile bool ready = false;

Globals &global = Globals::getInstance();
EEPROMHandler &eeprom = EEPROMHandler::getInstance();
ErrorNotifier &errorNotifier = ErrorNotifier::getInstance();
#if defined(SEL_MPU9250)
IMU &imu = IMU::getInstance();
#elif defined(SEL_MPU6050)
IMU_6050 &imu = IMU_6050::getInstance();
#endif
PIDController &pid = PIDController::getInstance();
WiFiBluetooth &WBController = WiFiBluetooth::getInstance();
Outputs &out = Outputs::getInstance();
#if defined(USE_RC)
RCReciever &reciever = RCReciever::getInstance();
#endif

control_t currControls = IDLE;
output_t outputs = {0, 0, 0, 0, 0, 0};
state_t state = {false, false, false, false, false, pre_Kp, pre_Kd, pre_Ki};

float pitch = 0.0f, roll = 0.0f, yaw = 0.0f;
float pitch_offset = 0.0f, roll_offset = 0.0f, yaw_offset = 0.0f;
float throttle_percent = 0.0f;

// #if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
// #error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
// #endif

// BluetoothSerial SerialBT;

TaskHandle_t core2;

void setup()
{
    Serial.begin(115200);
    imu.begin();
    pid.begin();
    errorNotifier.begin();
    eeprom.begin();
    out.begin();
    WBController.begin();
#if defined(USE_RC)
    reciever.begin();
#endif
    imu.calibrate(pitch_offset, roll_offset, yaw_offset);
    currControls.yaw = yaw_offset;
    ready = true;
    xTaskCreatePinnedToCore(main_process, "main_process", 10000, NULL, 1, &core2, 1);
}

#if defined(ENABLE_DEBUG)
long last_debug_print = 0;
#endif

void loop()
{ // handles wifi and rc
    errorNotifier.notifyError();

#if defined(ENABLE_DEBUG)
    if (millis() - last_debug_print > 200 && false)
    {
        print("Controls: ");
        print(currControls.throttle);
        print(" ");
        print(currControls.pitch);
        print(" ");
        print(currControls.roll);
        print(" ");
        print(currControls.yaw);
        println();
        // print("Error: ");
        // println(global.getError());
        last_debug_print = millis();
    }

#endif
#if defined(USE_WIFI) // switch between wifi and rc TODO
    currControls = WBController.receive(currControls, state);
#endif

#if defined(USE_RC)
    currControls = reciever.receive(currControls, state);
#endif

    if (state.calibrate)
    {
        imu.calibrate(pitch_offset, roll_offset, yaw_offset);
        state.calibrate = false;
    }
#if !defined(USE_RC)
    throttle_percent = currControls.throttle / 65535.0f * 100.0f;
#else
    throttle_percent = currControls.throttle / 2000.0f * 100.0f;
#endif
    if (global.getError() != 0)
    { // TODO disarm drone when error occurs
        ready = false;
        return;
    }
    else
        ready = true;
}

long last_debug_print2 = millis();
void main_process(void *parameter)
{
    // get imu data
    while (true)
    {
        imu.getAngles(pitch, roll, yaw);

        // set a threshold for throttle at which the drone will start
        if (!ready)
            currControls.throttle = 0;
        if (throttle_percent > 20)
        {
            // calculate pid
            pid.update(outputs, state, currControls, pitch - pitch_offset, roll - roll_offset, yaw);
        }
        else
            outputs = IDLE;
        // set outputs
        out.setOutputs(outputs);

#if defined(ENABLE_DEBUG)
        if (millis() - last_debug_print2 > 200)
        {
            print("Outputs: ");
            print(outputs.motor1);
            print(" ");
            print(outputs.motor2);
            print(" ");
            print(outputs.motor3);
            print(" ");
            print(outputs.motor4);
            println();
            print("Controls: ");
            print(currControls.throttle);
            print(" ");
            print(currControls.pitch);
            print(" ");
            print(currControls.roll);
            print(" ");
            print(currControls.yaw);
            println();
            last_debug_print2 = millis();
        }
#endif
    }
}
