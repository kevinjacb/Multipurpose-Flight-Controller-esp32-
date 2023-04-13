#ifndef ERR_H
#define ERR_H

#include <Arduino.h>

class ErrorNotifier
{
private:
    long lastBlinkMillis = 0;
    ErrorNotifier();
    static ErrorNotifier instance;

    ErrorNotifier(const ErrorNotifier &) = delete;
    void blink(uint8_t pin, uint8_t delay);

public:
    static ErrorNotifier &getInstance();
    void notifyError();
};

#endif ERR_H