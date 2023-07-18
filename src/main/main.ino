#include <Arduino.h>

#include "globals.h"
#include "errorNotifier.h"
#include "pid.h"
#include "wifi_bluetooth.h"
#include "motors.h"
#include "altitude.h"
#include "stdio.h"

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
#if defined(HCSR04)
Ultrasonic &ultraSonic = Ultrasonic::getInstance();
#endif
#if defined(BARO)
BMPSeries &barometer = BMPSeries::getInstance();
#endif
Battery &battery = Battery::getInstance();

volatile control_t currControls = {IDLE_VALUE, IDLE_VALUE, IDLE_VALUE, IDLE_VALUE, 0, 0, 0, IDLE_VALUE, IDLE_VALUE};
volatile output_t outputs = {IDLE_VALUE, IDLE_VALUE, IDLE_VALUE, IDLE_VALUE, IDLE_VALUE, IDLE_VALUE};
volatile state_t state = {false, false, false, false, false, pre_Kp, pre_Kd, pre_Ki};

float pitch = 0.0f, roll = 0.0f, yaw = 0.0f, yaw_angle = 0.0f, altitude = 0.0f;
float pitch_offset = 0.0f, roll_offset = 0.0f, yaw_offset = 0.0f;
float takeoff_voltage = 0.0f, voltage = 0.0f;
volatile float throttle_percent = 0.0f;

// #if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
// #error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
// #endif

// BluetoothSerial SerialBT;

TaskHandle_t core2 = NULL;

void setup()
{
    Serial.begin(115200);
    imu.begin();
    pid.begin();
    errorNotifier.begin();
    eeprom.begin();
    out.begin();
    out.setOutputs(outputs);
    WBController.begin();
    initStateFromEEPROM();
#if defined(HCSR04)
    ultraSonic.begin();
#endif
#if defined(BARO)
    barometer.begin();
#endif

#if defined(USE_RC)
    reciever.begin();
#endif
    battery.begin();
    // imu.calibrate(pitch_offset, roll_offset, yaw_offset);
    currControls.yaw = yaw_offset;
    ready = true;
    xTaskCreatePinnedToCore(main_process, "main_process", 10000, NULL, 1, &core2, 1);
}

#if defined(ENABLE_DEBUG)
long last_debug_print = 0;
#endif

long last_motor_update = 0, last_data_update = 0;

void loop()
{ // handles wifi and rc
    errorNotifier.notifyError();

#if defined(ENABLE_DEBUG)
    if (millis() - last_debug_print > 200 && false)
    {
        print("Error: ");
        println(global.getError());
        print("Kp : ");
        print(state._Kp);
        print("\tKi : ");
        print(state._Ki);
        print("\tKd : ");
        println(state._Kd);
        last_debug_print = millis();
    }

#endif
#if defined(USE_WIFI) // switch between wifi and rc TODO
    WBController.receive(currControls, state);
#endif

#if defined(USE_RC)
    reciever.receive(currControls, state);
#endif

    if (state.calibrate)
    {
        imu.calibrate(pitch_offset, roll_offset, yaw_offset);
        state.calibrate = false;
    }
    if (state.stop)
    { // safety measure during debugging TODO
        ready = false;
#if defined(USE_WIFI)
        if (currControls.throttle < 100)
        {
            ready = true;
            state.stop = false;
        }
#endif
        return;
    }

#if !defined(USE_RC)
    throttle_percent = currControls.throttle / 65535.0f * 100.0f;
#else
    throttle_percent = (currControls.throttle - 1000) / 1000.0f * 100.0f;
#endif
    // send pitch, roll, yaw data to app if connected
    if (throttle_percent < 5 && millis() - last_data_update > 10)
    {
        last_data_update = millis();
        char buffer[50];
        sprintf(buffer, "P:%.2f,R:%.2f,Y:%.2f\n", CORRECTED_IMU_DATA);
        WBController.send(buffer);
    }

    if (global.getError() != 0)
    { // TODO disarm drone when error occurs
#if defined(USE_RC)
        if (global.getError() == 3)
        {
            ready = true;
            return;
        }
#endif
        ready = false;
        return;
    }
    else
        ready = true;
}

long last_debug_print2 = millis(), last_alt_measure = millis();
void main_process(void *parameter)
{
    // get imu data
    while (true)
    {
        imu.getAngles(IMU_DATA, yaw_angle);
#if MODE == PLANE
        if (currControls.aux1 < 1500)
        { // turn off gyro
            if (currControls.aux2 < 1500)
            {
                idle();
                continue;
            }
            pass_through();
#if defined(ENABLE_DEBUG)
            debug_print(1);
#endif
            continue;
        }
#endif

        // set a threshold for throttle at which the drone will start
        altitude = calculate_altitude();
#ifdef VOLTAGE_MONITOR
        voltage = battery.getVoltage();
#endif

        if (!ready)
        {
            idle();
            continue;
        }
#if MODE != PLANE
        if (throttle_percent > 15 && currControls.aux2 > 1500)
        {
            // calculate pid
            // pid.update(outputs, state, currControls, pitch - pitch_offset, roll - roll_offset, yaw);
            // calculate pid with alt hold
            // pid.update(outputs, state, currControls, pitch - pitch_offset, roll - roll_offset, yaw, altitude);
            pid.update(outputs, state, currControls, CORRECTED_IMU_DATA, altitude, takeoff_voltage, voltage);
        }
#else
        if (currControls.aux2 > 1500)
        {
            pid.update(outputs, state, currControls, CORRECTED_IMU_DATA);
        }
#endif
        else
        {
            takeoff_voltage = voltage;
            pid.resetAll();
            idle();
        }
        // set outputs
        setOutputs();
#if defined(ENABLE_DEBUG)
        debug_print(1);
#endif
    }
}

void initStateFromEEPROM()
{
    // get data from EEPROM during initialization
    try
    {
        EEPROMHandler &eeprom = EEPROMHandler::getInstance();
        state._Kp = eeprom.read(KP_ADDR);
        state._Kd = eeprom.read(KD_ADDR);
        state._Ki = eeprom.read(KI_ADDR);
        state._Yaw_Kp = eeprom.read(YAW_KP_ADDR);
        state._Yaw_Kd = eeprom.read(YAW_KD_ADDR);
        state._Yaw_Ki = eeprom.read(YAW_KI_ADDR);
        state.inverted_pitch = eeprom.read(PITCH_INVERT_ADDR);
        state.inverted_roll = eeprom.read(ROLL_INVERT_ADDR);
        state.inverted_yaw = eeprom.read(YAW_INVERT_ADDR);
    }
    catch (...)
    {
        println("Error in initStateFromEEPROM");
    }
}

// 0 - ultra sonic, 1 - barometer
float calculate_altitude()
{
    float altitude = -1;

#if defined(HCSR04)
    if (millis() - last_alt_measure > 20)
    {
        ultraSonic.getAlt(altitude, pitch - pitch_offset, roll - roll_offset);
        last_alt_measure = millis();
        if (altitude > 210)
            altitude = -1;
        // print("Altitude: ");
        // println(altitude);
    }
#endif
#if defined(BARO)
    if (millis() - last_alt_measure > 20)
    {
        barometer.getAlt(altitude);
        last_alt_measure = millis();
        altitude *= 100;
        if (altitude < 0)
            altitude = 0;
    }
#endif

    return altitude;
}

void debug_print(int id)
{
    switch (id)
    {
    case 0: // general
        print("Pitch: ");
        print(pitch);
        print("\tRoll: ");
        print(roll);
        print("\tYaw: ");
        print(yaw);
        print("\tAltitude: ");
        print(altitude);
        print("\tVoltage: ");
        print(voltage);
        print("\tThrottle: ");
        print(throttle_percent);
        println();
        break;
    case 1: // channels
        print("Throttle: ");
        print(currControls.throttle);
        print("\tAileron: ");
        print(currControls.aileron);
        print("\tElevator: ");
        print(currControls.elevator);
        print("\tRudder: ");
        print(currControls.rudder);
        print("\tAux1: ");
        print(currControls.aux1);
        print("\tAux2: ");
        print(currControls.aux2);
        println();
        break;
    }
}

void setOutputs()
{
    if (millis() - last_motor_update > 10)
    {
        last_motor_update = millis();
        out.setOutputs(outputs);
    }
}

void idle()
{
    outputs.motor1 = IDLE_VALUE;
#if MODE == PLANE
    outputs.motor2 = SERVO_IDLE;
    outputs.motor3 = SERVO_IDLE;
    outputs.motor4 = SERVO_IDLE;
    outputs.motor5 = SERVO_IDLE;
#else
    outputs.motor2 = IDLE_VALUE;
    outputs.motor3 = IDLE_VALUE;
    outputs.motor4 = IDLE_VALUE;
#endif
    out.setOutputs(outputs);
}

#if MODE == PLANE
void pass_through() // only for planes
{
    outputs.motor1 = currControls.throttle;
    outputs.motor2 = currControls.aileron;
    outputs.motor3 = currControls.aileron;
    outputs.motor4 = currControls.elevator;
    outputs.motor5 = currControls.rudder;
    setOutputs();
}
#endif
