#include <Arduino.h>
#include "globals.h"
#include <Adafruit_BMP085.h>

// BMP180 currently
class BMPSeries
{
private:
  float startAltitude = 0;
  Adafruit_BMP085 bmp;
  BMPSeries();

public:
  static BMPSeries &getInstance();
  void getData(float &temp, float &altitude);
  void getAlt(float &altitude, bool init = false);
  void begin();
};

class Ultrasonic
{
private:
  Ultrasonic();

public:
  static Ultrasonic &getInstance();
  void getAlt(float &altitude);
  void getAlt(float &altitude, float pitch, float roll);
  void begin();
};

class Battery
{
public:
  static Battery &getInstance();
  void begin();
  float getVoltage();
  float getMaximumVoltage();

private:
  float batteryVoltage = -1;
  float minVoltage = -1, maxVoltage = -1;
  Battery();
};