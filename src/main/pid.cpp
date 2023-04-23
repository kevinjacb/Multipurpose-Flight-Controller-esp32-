#include "pid.h"
#include "globals.h"
#include <Arduino.h>

PIDController::PIDController()
{
    EEPROMHandler &eeprom = EEPROMHandler::getInstance();
    _kp = eeprom.read(KP_ADDR);
    _ki = eeprom.read(KI_ADDR);
    _kd = eeprom.read(KD_ADDR);
    _dt = dt;
    _error = 0.0f;
    _error_sum = 0.0f;
    _last_error = 0.0f;
}

float PIDController::calculate(float setpoint, float input)
{
    _error = setpoint - input;
    _error_sum += _error * _dt;
    float error_delta = (_error - _last_error) / _dt;
    _last_error = _error;
    return _kp * _error + _ki * _error_sum + _kd * error_delta;
}

void PIDController::update(output_t &outputs, control_t controls, float pitch, float roll, float yaw) // PID Controller
{
    static PIDController _pitch, _roll, _yaw;
    float pitchPID = _pitch.calculate(controls.pitch, pitch);
    float rollPID = _roll.calculate(controls.roll, roll);
    float yawPID = _yaw.calculate(controls.yaw, yaw);

#if defined(BRUSHLESS) && defined(QUAD_X) // TODO add hexa and octo and other types
    outputs.motor1 = constrain(controls.throttle + pitchPID + rollPID - yawPID, 1000, 2000);
    outputs.motor2 = constrain(controls.throttle + pitchPID - rollPID + yawPID, 1000, 2000);
    outputs.motor3 = constrain(controls.throttle - pitchPID + rollPID - yawPID, 1000, 2000);
    outputs.motor4 = constrain(controls.throttle - pitchPID - rollPID + yawPID, 1000, 2000);
#elif defined(QUAD_X)
    outputs.motor1 = constrain(controls.throttle + pitchPID + rollPID - yawPID, 1000, 2000);
    outputs.motor2 = constrain(controls.throttle + pitchPID - rollPID + yawPID, 1000, 2000);
    outputs.motor3 = constrain(controls.throttle - pitchPID + rollPID - yawPID, 1000, 2000);
    outputs.motor4 = constrain(controls.throttle - pitchPID - rollPID + yawPID, 1000, 2000);
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