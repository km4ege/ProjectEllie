
#ifndef BMP085_h
#define BMP085_h

#if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
#else
  #include "WProgram.h"
  #include <pins_arduino.h>
#endif
#include "I2C.h"

#define BMP085_ADDRESS 0x77  // I2C address of BMP085
#define OSS   0  // Oversampling Setting

class BMP085{
private:
    float temperature;
    long pressure;


    // Calibration values
    int ac1;
    int ac2;
    int ac3;
    unsigned int ac4;
    unsigned int ac5;
    unsigned int ac6;
    int b1;
    int b2;
    int mb;
    int mc;
    int md;

    // b5 is calculated in getTemperature(...), this variable is also used in getPressure(...)
    // so ...Temperature(...) must be called before ...Pressure(...).
    long b5;

    unsigned int readUncompensatedTemperature();            // Read the uncompensated temperature value
    unsigned long readUncompensatedPressure();              // Read the uncompensated pressure value
public:
    BMP085();
    void setup();
    void readSensor();
    long getPressure();                  // get pressure in units of Pa.
    float getTemperature();              // get temperature in deg C
};


#endif
