#include "rc_reciever.h"
#include "globals.h"

RCReciever::RCReciever()
{
}

RCReciever &RCReciever::getInstance()
{
    static RCReciever instance;
    return instance;
}

void RCReciever::begin()
{
    for (int i = 0; i < NUM_CHANNELS; i++)
        if (i != 2)
            channels[i] = 1500;
        else
            channels[i] = 1000;
    pinMode(USE_RC, INPUT_PULLUP);
    attachInterrupt(USE_RC, ISR_RC, FALLING);
}

void IRAM_ATTR RCReciever::ISR_RC()
{
    unsigned long pulse_width = micros() - last_pulse;
    if (pulse_width > 5000 && pulse_width < 20000)
    {
        current_channel = 0;
        ready = true;
    }
    else if (pulse_width > 950 && pulse_width < 2050 && ready)
    {
        if (current_channel == NUM_CHANNELS)
        {
            current_channel = 0;
            ready = false;
        }
        else
            channels[current_channel++] = pulse_width;
    }
    else
        ready = false;
    last_pulse = micros();
}

void RCReciever::receive(volatile control_t &prevControls, volatile state_t &state)
{
    // Serial.print("RC: ");
    // for (int i = 0; i < NUM_CHANNELS; i++)
    //     Serial.print(channels[i]), Serial.print(" ");
    Serial.println();
    if (ready)
    {

        prevControls.throttle = channels[2];
#if MODE == PLANE
        prevControls.aileron = map(channels[0], 990, 2010, 900, 2100);
        prevControls.elevator = map(channels[1], 990, 2010, 900, 2100);
        prevControls.rudder = map(channels[3], 990, 2010, 900, 2100);
#else
        prevControls.aileron = channels[0];
        prevControls.elevator = channels[1];
        prevControls.rudder = channels[3];
#endif
        prevControls.pitch = map(channels[1], 990, 2010, -PITCH_ROLL_ENDPOINT, PITCH_ROLL_ENDPOINT);
        prevControls.pitch = (state.inverted_pitch) ? -prevControls.pitch : prevControls.pitch;
        prevControls.roll = map(channels[0], 990, 2010, -PITCH_ROLL_ENDPOINT, PITCH_ROLL_ENDPOINT);
        prevControls.roll = (state.inverted_roll) ? -prevControls.roll : prevControls.roll;
        prevControls.yaw = map(channels[3], 990, 2010, -YAW_ENDPOINT, YAW_ENDPOINT);
        prevControls.yaw = (state.inverted_yaw) ? -prevControls.yaw : prevControls.yaw;
        prevControls.aux1 = channels[4];
        prevControls.aux2 = channels[5];
    }
}

volatile unsigned long RCReciever::last_pulse = 0;
bool RCReciever::ready = false;
volatile uint8_t RCReciever::current_channel = 0;
volatile uint16_t RCReciever::channels[NUM_CHANNELS] = {1500, 1500, 1000, 1500, 1000, 1000, 1000, 1000}; // Aileron, Elevator, Throttle, Rudder, Aux1, Aux2
