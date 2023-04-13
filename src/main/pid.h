#ifndef PID_H
#define PID_H

class PIDController
{
public:
    float calculate(float setpoint, float input);
    void setGains(float kp, float ki, float kd);
    static PIDController &getInstance();

private:
    PIDController();
    float _kp;
    float _ki;
    float _kd;
    float _dt;
    float _error;
    float _error_sum;
    float _last_error;
};

#endif // PID_H