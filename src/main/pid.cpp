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
}

PIDController::PIDController()
{

    _kp = pre_Kp;
    _ki = pre_Ki;
    _kd = pre_Kd;
    _dt = pre_dt;
    _error = 0.0f;
    _error_sum = 0.0f;
    _last_error = 0.0f;
}

float PIDController::calculate(float setpoint, float input)
{
    _error = setpoint - input;
    _error_sum += _error * _dt;
    if (millis() - lastCorrection > _dt * 1000)
    {
        error_delta = (_error - _last_error) / _dt;
        _last_error = _error;
        lastCorrection = millis();
    }
    println(_kd * error_delta);
    return _kp * _error + _ki * _error_sum + _kd * error_delta;
}

void PIDController::update(output_t &outputs, state_t state, control_t controls, float pitch, float roll, float yaw) // PID Controller
{
    static PIDController _pitch, _roll, _yaw;
    _pitch.setGains(state._Kp, state._Ki, state._Kd);
    _roll.setGains(state._Kp, state._Ki, state._Kd);
    _yaw.setGains(state._Kp, state._Ki / 5, state._Kd / 5);
    float pitchPID = _pitch.calculate(controls.pitch, pitch);
    float rollPID = -_roll.calculate(controls.roll, roll); // inverted roll
    float yawPID = _yaw.calculate(0, controls.yaw + yaw);
    // yawPID = 0; // debug

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