#include "errorNotifier.h"
#include "Arduino.h"
#include "globals.h"

/*
    Sections:
    1. IMU
    2. WiFiBluetooth
    3. EEPROM
    4. BMP

    Error LEDS:
    LED 1: 34 RED
    LED 2: 25 GREEN
    LED 3: 26 BLUE

    Errors:
        LED 2: Steady - No error : Error code 0
        LED 1: Blinking - Error with IMU : Error code 1
        LED 1,2,3: strobe - Initialization : Error code 2
        LED 3: Blinking, LED 2: steady - Waiting for wifi/client connection: Error code 3
        LED 2,3: steady - Error with WiFi/Bluetooth : Error code 4
        LED 1,2,3: steady - Writing to EEPROM: Error code 5
        LED 1: delayed blinking - Error with BMP : Error code 6 // low priority

*/
ErrorNotifier::ErrorNotifier()
{
}

void ErrorNotifier::begin()
{
    for (uint8_t i = 0; i < 3; i++)
    {
        pinMode(Globals::LED_PINS[i], OUTPUT);
    }
}

ErrorNotifier &ErrorNotifier::getInstance()
{
    static ErrorNotifier instance;
    return instance;
}

void ErrorNotifier::notifyError()
{
    switch (Globals::getInstance().getError())
    {

    case 0:
        // No error
        digitalWrite(RED_L, LOW);
        digitalWrite(BLUE_L, LOW);
        digitalWrite(GREEN_L, HIGH);
        break;

    case 1:
        // Error with IMU
        digitalWrite(GREEN_L, LOW);
        digitalWrite(BLUE_L, LOW);
        blink(RED_L, 100);

        break;
    case 2:
        // Initialization
        blink(RED_L, 100);
        blink(GREEN_L, 200);
        blink(BLUE_L, 400);
        break;
    case 3:
        // Waiting for wifi/client connection
        digitalWrite(RED_L, LOW);
        digitalWrite(GREEN_L, HIGH);
        blink(BLUE_L, 100);
        break;

    case 4:
        // Error with WiFi/Bluetooth
        digitalWrite(RED_L, LOW);
        digitalWrite(GREEN_L, HIGH);
        digitalWrite(BLUE_L, HIGH);
        break;

    case 5:
        // Writing to EEPROM
        digitalWrite(RED_L, HIGH);
        digitalWrite(GREEN_L, HIGH);
        digitalWrite(BLUE_L, HIGH);
        break;

    case 6:
        // Error with BMP
        blink(RED_L, 600);
        digitalWrite(GREEN_L, HIGH);
        digitalWrite(BLUE_L, LOW);
        break;

    default:
        break;
    }
}

void ErrorNotifier::blink(uint8_t pin, uint8_t delay)
{
    if (millis() - lastBlinkMillis < delay)
        digitalWrite(pin, HIGH);
    else if (millis() - lastBlinkMillis < delay * 2)
        digitalWrite(pin, LOW);
    else
        lastBlinkMillis = millis();
}