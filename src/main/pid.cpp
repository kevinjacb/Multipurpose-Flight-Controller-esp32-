#include "pid.h"
#include "globals.h"
#include <Arduino.h>

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

float PIDController::calculate(float setpoint, float input)
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

    return _kp * _error + constrain(_ki * _error_sum, -400, 400) + _kd * error_delta;
}

void PIDController::update(volatile output_t &outputs, volatile state_t &state, volatile control_t &controls, float pitch, float roll, float yaw) // PID Controller
{
    static PIDController _pitch, _roll, _yaw;
    _pitch.setGains(state._Kp, state._Ki, state._Kd);
    _roll.setGains(state._Kp, state._Ki, state._Kd);
    _yaw.setGains(state._Yaw_Kp, state._Yaw_Ki, state._Yaw_Kd);

    float pitchPID = constrain(_pitch.calculate(controls.pitch, pitch), -400, 400);
    float rollPID = -constrain(_roll.calculate(controls.roll, roll), -400, 400); // inverted roll
    float yawPID = -constrain(_yaw.calculate(0, controls.yaw + yaw), -400, 400);

    // yawPID = 0; // debug

    // debug print pitch roll yaw
    // print(" roll pid: ");
    // println(rollPID);

#if defined(BRUSHLESS) && defined(QUAD_X) // TODO add hexa and octo and other types
    outputs.motor1 = constrain(controls.throttle + pitchPID + rollPID + yawPID, 1000, 2000);
    outputs.motor2 = constrain(controls.throttle + pitchPID - rollPID - yawPID, 1000, 2000);
    outputs.motor3 = constrain(controls.throttle - pitchPID - rollPID + yawPID, 1000, 2000);
    outputs.motor4 = constrain(controls.throttle - pitchPID + rollPID - yawPID, 1000, 2000);
#elif defined(QUAD_X)
    outputs.motor1 = constrain(controls.throttle + pitchPID + rollPID + yawPID, 0, 65535);
    outputs.motor2 = constrain(controls.throttle + pitchPID - rollPID - yawPID, 0, 65535);
    outputs.motor3 = constrain(controls.throttle - pitchPID - rollPID + yawPID, 0, 65535);
    outputs.motor4 = constrain(controls.throttle - pitchPID + rollPID - yawPID, 0, 65535);
#endif
}

void PIDController::setGains(float kp, float ki, float kd)
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