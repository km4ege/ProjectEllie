
#ifndef ADXL345_h
#define ADXL345_h

#if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
#else
  #include "WProgram.h"
  #include <pins_arduino.h>
#endif
#include "I2C.h"

//Registers in the ADXL345
#define ADXL345_ADDRESS 0x53
#define ADXL345_POWER_CTL 0x2d
#define ADXL345_DATA_FORMAT 0x31
#define ADXL345_DATAX0 0x32
#define ADXL345_DATAX1 0x33
#define ADXL345_DATAY0 0x34
#define ADXL345_DATAY1 0x35
#define ADXL345_DATAZ0 0x36
#define ADXL345_DATAZ1 0x37

class ADXL345{
private:
    float x;
    float y;
    float z;
    float gains[3];
public:
    ADXL345();
    void setup();
    void readAxis();
    void setRangeSetting(int val);          // Sets the range setting, possible values are: 2, 4, 8, 16
    float getX();
    float getY();
    float getZ();

};


#endif
