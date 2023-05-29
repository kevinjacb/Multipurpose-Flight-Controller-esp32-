#include "motors.h"
#include <ESP32Servo.h>
#include "globals.h"

Outputs::Outputs()
{
}

void Outputs::begin()
{
#if defined(QUAD_X) // repurpose the unused pins maybe?
    motorPins = new uint8_t[4]{MOTOR1, MOTOR2, MOTOR3, MOTOR4};
#if defined(BRUSHLESS) // TODO add hexa and octo and other types
    for (int i = 0; i < 4; i++)
    {
        motors[i].setPeriodHertz(PWM_FREQ_BRUSHLESS);
        motors[i].attach(motorPins[i], 1000, 2000);
    }
#elif defined(QUAD_X)
    for (int i = 0; i < 4; i++)
    {
        ledcSetup(i, PWM_FREQ_BRUSHED, PWM_RES_BRUSHED);
        ledcAttachPin(motorPins[i], i);
    }
#endif // TODO for other types
#endif
}

void Outputs::setOutputs(volatile output_t &outputs)
{
#if defined(BRUSHLESS) // TODO add hexa and octo and other types
    motors[0].writeMicroseconds(outputs.motor1);
    motors[1].writeMicroseconds(outputs.motor2);
    motors[2].writeMicroseconds(outputs.motor3);
    motors[3].writeMicroseconds(outputs.motor4);

#elif defined(QUAD_X) // TODO add hexa and octo and other types
    ledcWrite(0, outputs.motor1);
    ledcWrite(1, outputs.motor2);
    ledcWrite(2, outputs.motor3);
    ledcWrite(3, outputs.motor4);
#endif
}

Outputs &Outputs::getInstance()
{
    static Outputs instance;
    return instance;
}
