#include "globals.h"
#include <Arduino.h>

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
    if (error != 0 && section != handler)
        return;
    std::unique_lock<std::mutex> lock(errorMutex);
    handler = section;
    error = err;
}