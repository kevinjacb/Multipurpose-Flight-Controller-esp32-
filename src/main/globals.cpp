#include "globals.h"
#include <Arduino.h>

control_t copyControl(volatile control_t &control)
{
    control_t newControl = {control.throttle, control.roll, control.pitch, control.yaw, control.aux1, control.aux2};
    return newControl;
}

Globals::Globals()
{
    error = 0;
    handler = 0;
}

// return singleton instance
Globals &Globals::getInstance()
{
    static Globals instance;
    return instance;
}

// return error code
uint8_t Globals::getError()
{
    std::unique_lock<std::mutex> lock(errorMutex);
    return error;
}

// set error code
void Globals::setError(uint8_t err, uint8_t section)
{
    if (error != 0 && section != handler && handler != 0)
        return;
    std::unique_lock<std::mutex> lock(errorMutex);
    handler = section;
    error = err;
}

EEPROMHandler::EEPROMHandler()
{
}

void EEPROMHandler::begin()
{
    EEPROM.begin(EEPROM_SIZE);
}

EEPROMHandler &EEPROMHandler::getInstance()
{
    static EEPROMHandler instance;
    return instance;
}

void EEPROMHandler::write(uint16_t address, float value)
{
    byte *p = (byte *)(void *)&value;
    Globals::getInstance().setError(5, 3);
    for (uint8_t i = 0; i < sizeof(value); i++)
    {
        if (EEPROM.read(address) != *p)
            EEPROM.write(address++, *p++);
    }
    Globals::getInstance().setError(0, 3);
}

float EEPROMHandler::read(uint16_t address)
{
    float value = 0.0;
    byte *p = (byte *)(void *)&value;
    for (uint8_t i = 0; i < sizeof(value); i++)
        *p++ = EEPROM.read(address++);
    return value;
}