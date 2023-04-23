#include <Arduino.h>
#include "globals.h"

class Outputs
{
private:
    Outputs();
    Outputs(const Outputs &) = delete; // disable copy constructor
    uint8_t *motorPins;
#if defined(BRUSHLESS) && defined(QUAD_X) // TODO add hexa and octo and other types
    Servo motors[4];
#endif

public:
    void setOutputs(output_t outputs);
    static Outputs &getInstance();
};
