#include <Arduino.h>
#include "globals.h"

class RCReciever
{
private:
    RCReciever();
    RCReciever(const RCReciever &) = delete;
    static void IRAM_ATTR ISR_RC();

public:
    volatile static uint16_t channels[NUM_CHANNELS]; // Aileron, Elevator, Throttle, Rudder, Aux1, Aux2
    volatile static uint8_t current_channel;
    volatile unsigned static long last_pulse;
    static bool ready;

    static RCReciever &getInstance();
    void begin();
    void receive(volatile control_t &prevControls, volatile state_t &state);
};
