#include <Arduino.h>
#include "globals.h"

// perfrom manuevers such as vertical hover on aircraft

class Manuevers
{
private:
    Manuevers();

public:
    Manuevers &getInstance();
    void vertical_hover(control_t controls, output_t &output, state_t &state, float pitch, float roll, float yaw);
    void land_assist(control_t controls, output_t &output, state_t &state, float pitch, float roll, float yaw);
};