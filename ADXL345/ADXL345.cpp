#include "ADXL345.h"


ADXL345::ADXL345() {
    x = 0;
    y = 0;
    z = 0;
    gains[0] = 0.00376390;
    gains[1] = 0.00376009;
    gains[2] = 0.00349265;
}

void ADXL345::setup(){
    I2CWrite(ADXL345_ADDRESS, ADXL345_POWER_CTL, 0x08);        // Turning on the ADXL345
    setRangeSetting(2);
}

void ADXL345::readAxis(){
     x = ((I2CRead(ADXL345_ADDRESS, ADXL345_DATAX1)<<8)  |  I2CRead(ADXL345_ADDRESS, ADXL345_DATAX0)); //combine upper bytes with lower bytes of x
     y = ((I2CRead(ADXL345_ADDRESS, ADXL345_DATAY1)<<8)  |  I2CRead(ADXL345_ADDRESS, ADXL345_DATAY0)); //combine upper bytes with lower bytes of y
     z = ((I2CRead(ADXL345_ADDRESS, ADXL345_DATAZ1)<<8)  |  I2CRead(ADXL345_ADDRESS, ADXL345_DATAZ0)); //combine upper bytes with lower bytes of z
}


void ADXL345::setRangeSetting(int val) {
  byte _s;
  byte _b;

  switch (val) {
  case 2:
    _s = B00000000;
    break;
  case 4:
    _s = B00000001;
    break;
  case 8:
    _s = B00000010;
    break;
  case 16:
    _s = B00000011;
    break;
  default:
    _s = B00000000;
  }
  _b = I2CRead(ADXL345_ADDRESS, ADXL345_DATA_FORMAT);
  I2CWrite(ADXL345_ADDRESS, ADXL345_DATA_FORMAT, _s);
}


float ADXL345::getX(){
    return x;
}

float ADXL345::getY(){
    return y;
}

float ADXL345::getZ(){
    return z;
}
