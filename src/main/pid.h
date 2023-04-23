#ifndef PID_H
#define PID_H

#include "globals.h"

class PIDController
{
public:
    void setGains(float kp, float ki, float kd);
    void update(output_t &outputs, control_t controls, float pitch, float roll, float yaw);
    static PIDController &getInstance();

private:
    PIDController();
    float calculate(float setpoint, float input);
    float _kp;
    float _ki;
    float _kd;
    float _dt;
    float _error;
    float _error_sum;
    float _last_error;
};

#endif // PID_H