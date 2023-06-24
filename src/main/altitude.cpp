#include "altitude.h"
#include <Adafruit_BMP085.h>
#include "globals.h"

// Section: 4

BMPSeries::BMPSeries()
{
    // TODO
}

void BMPSeries::begin()
{
    if (!bmp.begin())
    {
        Globals::getInstance().setError(4, 6); // Error with BMP
        return;
    }
    Globals::getInstance().setError(4, 2); // initialization
    for (int i = 0; i < 50; i++)
    {
        float alt;
        getAlt(alt, true);
        startAltitude += alt;
        delay(40);
    }
    startAltitude /= 50.0;
    Serial.print("Start altitude: ");
    Serial.println(startAltitude);
    Globals::getInstance().setError(4, 0); // No error
}

void BMPSeries::getData(float &temp, float &altitude)
{
    try
    {
        temp = bmp.readTemperature();
        altitude = bmp.readAltitude() - startAltitude;
    }
    catch (const std::exception &e)
    {
        Globals::getInstance().setError(4, 6); // Error with BMP
        temp = -1;
        altitude = -1;
    }
}

void BMPSeries::getAlt(float &altitude, bool init)
{

    try
    {
        altitude = bmp.readAltitude() - ((init) ? 0 : startAltitude);
    }
    catch (const std::exception &e)
    {
        Globals::getInstance().setError(4, 6); // Error with BMP
        altitude = -1;
    }
}

BMPSeries &BMPSeries::getInstance()
{
    static BMPSeries instance;
    return instance;
}

Ultrasonic &Ultrasonic::getInstance()
{
    static Ultrasonic instance;
    return instance;
}

Ultrasonic::Ultrasonic()
{
    // TODO
}

void Ultrasonic::begin()
{
    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);
}

// get the actual height from the ground
void Ultrasonic::getAlt(float &altitude, float pitch, float roll)
{
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);

    long duration = pulseIn(ECHO_PIN, HIGH);
    altitude = duration * 0.034 / 2; // inclined height
    // actual height
    altitude = altitude * cos(pitch * PI / 180) * cos(roll * PI / 180);
}

void Ultrasonic::getAlt(float &altitude)
{
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);

    long duration = pulseIn(ECHO_PIN, HIGH);
    altitude = duration * 0.034 / 2;
}

Battery::Battery()
{
    // TODO
}

Battery &Battery::getInstance()
{
    static Battery instance;
    return instance;
}

void Battery::begin()
{
    // TODO
    // set adc resolution to desired
    maxVoltage = (BATT_CELLS * 4.2) / (BATT_DIV + 1);
    minVoltage = (BATT_CELLS * 3.3) / (BATT_DIV + 1);
}

float Battery::getVoltage()
{

    if (batteryVoltage != -1)
        batteryVoltage = batteryVoltage * 0.92 + (analogRead(BATT) * 5.0 / (ADC_RES * 1.0) - minVoltage) * maxVoltage / (maxVoltage - minVoltage);
    else
        batteryVoltage = (analogRead(BATT) * 5.0 / (ADC_RES * 1.0) - minVoltage) * maxVoltage / (maxVoltage - minVoltage);
    return batteryVoltage;
}

float Battery::getMaximumVoltage()
{
    return maxVoltage;
}