#ifndef PID_H
#define PID_H

#include "globals.h"

class PIDController
{
public:
    void update(volatile output_t &outputs, volatile state_t &state, volatile control_t &controls, float pitch, float roll, float yaw);
    void update(volatile output_t &outputs, volatile state_t &state, volatile control_t &controls, float pitch, float roll, float yaw, float altitude, float takeoff_voltage, float voltage);
    static PIDController &getInstance();
    void begin();
    void resetAll();

private:
    PIDController();
    class PIDCalculator
    {
    private:
        float _dt;
        float _error;
        float _error_sum;
        float _last_error, _last_act_error;
        long lastCorrection;
        float error_delta;

    public:
        float _kp, _ki, _kd;
        float _yaw_kp, _yaw_ki, _yaw_kd;
        PIDCalculator();
        void setGains(float kp, float ki, float kd);
        float calculate(float setpoint, float input);
        void reset();
    };
    PIDCalculator _pitch, _roll, _yaw, _throttle;
};

#endif // PID_H