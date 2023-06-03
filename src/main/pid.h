#ifndef PID_H
#define PID_H

#include "globals.h"

class PIDController
{
public:
    void setGains(float kp, float ki, float kd);
    void update(volatile output_t &outputs, volatile state_t &state, volatile control_t &controls, float pitch, float roll, float yaw);
    static PIDController &getInstance();
    void begin();

private:
    static PIDController _pitch, _roll, _yaw;
    PIDController();
    float calculate(float setpoint, float input);
    float _kp, _ki, _kd;
    float _yaw_kp, _yaw_ki, _yaw_kd;
    float _dt;
    float _error;
    float _error_sum;
    float _last_error, _last_act_error;
    long lastCorrection;
    float error_delta;
};

#endif // PID_H