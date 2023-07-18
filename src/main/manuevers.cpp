#include "manuevers.h"
#include "globals.h"
#include "pid.h"

Manuevers::Manuevers()
{
}

Manuevers &Manuevers::getInstance()
{
    static Manuevers instance;
    return instance;
}

void Manuevers::vertical_hover(control_t controls, output_t &output, state_t &state, float pitch, float roll, float yaw) // make plane hover vertically
{
    const float desired_pitch = 90.0, desired_yaw = 0.0;
    PIDController &pid = PIDController::getInstance();
    controls.pitch += 90.0; // set pitch to 90 degrees
    pid.update(output, state, controls, pitch, roll, yaw);
    // TODO incomplete
}

void Manuevers::land_assist(control_t controls, output_t &output, state_t &state, float pitch, float roll, float yaw) // make plane land
{
    const float desired_pitch = 90.0, desired_yaw = 0.0;
    PIDController &pid = PIDController::getInstance();
    controls.pitch += 20.0; // set pitch to landing angle
    pid.update(output, state, controls, pitch, roll, yaw);
    // TODO incomplete
}