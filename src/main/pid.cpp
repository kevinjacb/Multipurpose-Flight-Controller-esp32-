#include "pid.h"
#include "globals.h"
#include <Arduino.h>
#include "altitude.h"

#define print(x) Serial.print(x)
#define println(x) Serial.println(x)

void PIDController::begin()
{
    // debug
    // EEPROMHandler &eeprom = EEPROMHandler::getInstance();
    // _kp = eeprom.read(KP_ADDR);
    // _ki = eeprom.read(KI_ADDR);
    // _kd = eeprom.read(KD_ADDR);
    // _yaw_kp = eeprom.read(YAW_KP_ADDR);
    // _yaw_ki = eeprom.read(YAW_KI_ADDR);
    // _yaw_kd = eeprom.read(YAW_KD_ADDR);
}

PIDController::PIDController()
{
}

PIDController::PIDCalculator::PIDCalculator()
{ // TODO get values from eeprom
    // pitch and roll settings
    _kp = pre_Kp;
    _ki = pre_Ki;
    _kd = pre_Kd;
    _dt = pre_dt;
    // yaw settings
    _yaw_kp = 3.0;
    _yaw_ki = 0.0004;
    _yaw_kd = 0.0;
    _error = 0.0f;
    _error_sum = 0.0f;
    _last_error = 0.0f;     // last error without the setpoint (input)
    _last_act_error = 0.0f; // last_actual error with the setpoint
}

float PIDController::PIDCalculator::calculate(float setpoint, float input)
{
    _error = setpoint - input;
    if (millis() - lastCorrection > 1000 * _dt)
    {
        _error_sum = _error_sum + (_error + _last_act_error) * _dt / 2;
        _last_act_error = _error;
        error_delta = -(input - _last_error) / _dt;
        _last_error = input;
        lastCorrection = millis();
    }

    return constrain(_kp * _error, -PID_LIMIT, PID_LIMIT) + constrain(_ki * _error_sum, -INTEGRAL_WINDUP_LIMIT, INTEGRAL_WINDUP_LIMIT) + constrain(_kd * error_delta, -PID_LIMIT, PID_LIMIT); // prevents integral windup
}

void PIDController::update(volatile output_t &outputs, volatile state_t &state, volatile control_t &controls, float pitch, float roll, float yaw) // PID Controller
{
    // check if any of the gains have changed
    if (state._Kp != _pitch._kp || state._Ki != _pitch._ki || state._Kd != _pitch._kd)
    {
        _pitch.setGains(state._Kp, state._Ki, state._Kd);
        _roll.setGains(state._Kp, state._Ki, state._Kd);
    }
    else if (state._Yaw_Kp != _yaw._kp || state._Yaw_Ki != _yaw._ki || state._Yaw_Kd != _yaw._kd)
    {
        _yaw.setGains(state._Yaw_Kp, state._Yaw_Ki, state._Yaw_Kd);
    }

    float pitchPID = -constrain(_pitch.calculate(controls.pitch, pitch), -PID_LIMIT, PID_LIMIT);
    float rollPID = -constrain(_roll.calculate(controls.roll, roll), -PID_LIMIT, PID_LIMIT); // inverted roll
    float yawPID = -constrain(_yaw.calculate(0, controls.yaw + yaw), -PID_LIMIT, PID_LIMIT);

    // yawPID = 0; // debug
#if MODE == PLANE
    outputs.motor1 = controls.throttle;
    outputs.motor2 = constrain(SERVO_IDLE + rollPID, 900, 2100);  // Aileron1
    outputs.motor3 = constrain(SERVO_IDLE + rollPID, 900, 2100);  // Aileron2
    outputs.motor4 = constrain(SERVO_IDLE + pitchPID, 900, 2100); // Elevator
    outputs.motor5 = constrain(SERVO_IDLE + yawPID, 900, 2100);   // Rudder
#elif MODE == QUAD_X                                              // TODO add hexa and octo and other types
    outputs.motor1 = constrain(controls.throttle + pitchPID + rollPID + yawPID, 1000, 2000);
    outputs.motor2 = constrain(controls.throttle + pitchPID - rollPID - yawPID, 1000, 2000);
    outputs.motor3 = constrain(controls.throttle - pitchPID - rollPID + yawPID, 1000, 2000);
    outputs.motor4 = constrain(controls.throttle - pitchPID + rollPID - yawPID, 1000, 2000);
#elif MODE == MICRO_QUAD_X
    outputs.motor1 = constrain(controls.throttle + pitchPID + rollPID + yawPID, 0, 65535);
    outputs.motor2 = constrain(controls.throttle + pitchPID - rollPID - yawPID, 0, 65535);
    outputs.motor3 = constrain(controls.throttle - pitchPID - rollPID + yawPID, 0, 65535);
    outputs.motor4 = constrain(controls.throttle - pitchPID + rollPID - yawPID, 0, 65535);
#endif
}

void PIDController::update(volatile output_t &outputs, volatile state_t &state, volatile control_t &controls, float pitch, float roll, float yaw, float altitude, float takeoff_voltage, float voltage) // PID Controller
{
    _pitch.setGains(state._Kp, state._Ki, state._Kd);
    _roll.setGains(state._Kp, state._Ki, state._Kd);
    _yaw.setGains(state._Yaw_Kp, state._Yaw_Ki, state._Yaw_Kd);
    _throttle.setGains(2, 0.0001, 0.2);

    float pitchPID = constrain(_pitch.calculate(controls.pitch, pitch), -PID_LIMIT, PID_LIMIT);
    float rollPID = constrain(_roll.calculate(controls.roll, roll), -PID_LIMIT, PID_LIMIT); // inverted roll
    float yawPID = -constrain(_yaw.calculate(0, controls.yaw + yaw), -PID_LIMIT, PID_LIMIT);
    float throttlePID = constrain(_throttle.calculate(150, altitude), -PID_LIMIT, PID_LIMIT);
    if (controls.aux1 < 1500 || altitude == -1)
    {
        throttlePID = (takeoff_voltage - voltage) * 40; // TODO add voltage PID
        throttlePID = -1;                               // disable alt hold for now TODO
        if (throttlePID < 0)
        {
            throttlePID = 0; // disable alt hold
            _throttle.reset();
        }
    }

    // debug print pitch roll yaw
    // print("Error: ");
    // print(roll);
    // print(" roll pid: ");
    // println(rollPID);

    // yawPID = 0; // debug
    // print("altitude: ");
    // print(altitude);
    // print(" throttle: ");
    // println(throttlePID);

#if MODE == PLANE // here motor1,2 -> ailerons and motor3 -> elevator motor4 -> rudder
    outputs.motor1 = controls.throttle;
    outputs.motor2 = constrain(controls.aileron + rollPID, 1000, 2000);
    outputs.motor3 = constrain(controls.aileron + rollPID, 1000, 2000);
    outputs.motor4 = constrain(controls.elevator + pitchPID, 1000, 2000);
    outputs.motor5 = constrain(controls.rudder + yawPID, 1000, 2000);

#elif MODE == QUAD_X // TODO add hexa and octo and other types
    // controls.throttle = controls.throttle + throttlePID;
    outputs.motor1 = constrain(controls.throttle + throttlePID + pitchPID + rollPID + yawPID, 1000, 2000);
    outputs.motor2 = constrain(controls.throttle + throttlePID + pitchPID - rollPID - yawPID, 1000, 2000);
    outputs.motor3 = constrain(controls.throttle + throttlePID - pitchPID - rollPID + yawPID, 1000, 2000);
    outputs.motor4 = constrain(controls.throttle + throttlePID - pitchPID + rollPID - yawPID, 1000, 2000);
#elif MODE == MICRO_QUAD_X
    outputs.motor1 = constrain(controls.throttle + pitchPID + rollPID + yawPID, 0, 65535);
    outputs.motor2 = constrain(controls.throttle + pitchPID - rollPID - yawPID, 0, 65535);
    outputs.motor3 = constrain(controls.throttle - pitchPID - rollPID + yawPID, 0, 65535);
    outputs.motor4 = constrain(controls.throttle - pitchPID + rollPID - yawPID, 0, 65535);
#endif
}

void PIDController::PIDCalculator::setGains(float kp, float ki, float kd)
{
    _kp = kp;
    _ki = ki;
    _kd = kd;
}

PIDController &PIDController::getInstance()
{
    static PIDController instance;
    return instance;
}

void PIDController::resetAll()
{
    _pitch.reset();
    _roll.reset();
    _yaw.reset();
    _throttle.reset();
}

void PIDController::PIDCalculator::reset()
{
    _error = 0.0f;
    _error_sum = 0.0f;
    _last_error = 0.0f;     // last error without the setpoint (input)
    _last_act_error = 0.0f; // last_actual error with the setpoint
}