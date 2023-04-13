#include "pid.h"
#include "globals.h"

PIDController::PIDController()
{
    _kp = Kp;
    _ki = Ki;
    _kd = Kd;
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