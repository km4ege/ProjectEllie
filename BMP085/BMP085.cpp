#include "BMP085.h"


BMP085::BMP085() {

  ac1 = 0;
  ac2 = 0;
  ac3 = 0;
  ac4 = 0;
  ac5 = 0;
  ac6 = 0;
  b1 = 0;
  b2 = 0;
  b5 = 0;
  mb = 0;
  mc = 0;
  md = 0;
}

void BMP085::setup(){
  ac1 = I2CReadWord(BMP085_ADDRESS,0xAA);
  ac2 = I2CReadWord(BMP085_ADDRESS,0xAC);
  ac3 = I2CReadWord(BMP085_ADDRESS,0xAE);
  ac4 = I2CReadWord(BMP085_ADDRESS,0xB0);
  ac5 = I2CReadWord(BMP085_ADDRESS,0xB2);
  ac6 = I2CReadWord(BMP085_ADDRESS,0xB4);
  b1 = I2CReadWord(BMP085_ADDRESS,0xB6);
  b2 = I2CReadWord(BMP085_ADDRESS,0xB8);
  mb = I2CReadWord(BMP085_ADDRESS,0xBA);
  mc = I2CReadWord(BMP085_ADDRESS,0xBC);
  md = I2CReadWord(BMP085_ADDRESS,0xBE);
}

unsigned int BMP085::readUncompensatedTemperature(){
  I2CWrite(BMP085_ADDRESS, 0xF4, 0x2E);         // Write 0x2E into Register 0xF4. This requests a temperature reading
  delay(5);                                     // Wait at least 4.5ms
  return I2CReadWord(BMP085_ADDRESS, 0xF6);     // Read two bytes from registers 0xF6 and 0xF7 and return un-calibrated temperature
}

unsigned long BMP085::readUncompensatedPressure(){
  unsigned char msb, lsb, xlsb;

  I2CWrite(BMP085_ADDRESS, 0xF4, 0x34 + (OSS<<6));   // Write 0x34+(OSS<<6) into register 0xF4. Request a pressure reading w/ oversampling setting
  delay(2 + (3<<OSS));                                    // Wait for conversion, delay time dependent on OSS
  msb = I2CRead(BMP085_ADDRESS, 0xF6);                    // Read register 0xF6 (MSB)
  lsb = I2CRead(BMP085_ADDRESS, 0xF7);                     // Read register 0xF7 (LSB)
  xlsb = I2CRead(BMP085_ADDRESS, 0xF8);                    // Read register 0xF8 (XLSB)

  return(((unsigned long) msb << 16) | ((unsigned long) lsb << 8) | (unsigned long) xlsb) >> (8-OSS); //return un-calibrated pressure
}

void BMP085::readSensor(){
  long x1, x2, x3,x4, x5, b3, b6;
  unsigned long b4, b7;

// temperature calculations
  x4 = (((long)(readUncompensatedTemperature()) - (long)ac6)*(long)ac5) >> 15;
  x5 = ((long)mc << 11)/(x4 + md);
  b5 = x4 + x5;
  temperature = ((b5 + 8)>>4)/10;

// pressure calculations
  b6 = b5 - 4000;
  // Calculate B3
  x1 = (b2 * (b6 * b6)>>12)>>11;
  x2 = (ac2 * b6)>>11;
  x3 = x1 + x2;
  b3 = (((((long)ac1)*4 + x3)<<OSS) + 2)>>2;

  // Calculate B4
  x1 = (ac3 * b6)>>13;
  x2 = (b1 * ((b6 * b6)>>12))>>16;
  x3 = ((x1 + x2) + 2)>>2;
  b4 = (ac4 * (unsigned long)(x3 + 32768))>>15;

  b7 = ((unsigned long)(readUncompensatedPressure() - b3) * (50000>>OSS));
  if (b7 < 0x80000000)
    pressure = (b7<<1)/b4;
  else
    pressure = (b7/b4)<<1;

  x1 = (pressure>>8) * (pressure>>8);
  x1 = (x1 * 3038)>>16;
  x2 = (-7357 * pressure)>>16;
  pressure += (x1 + x2 + 3791)>>4;
}


float BMP085::getTemperature(){
  return temperature;
}

long BMP085::getPressure(){
    return pressure;
}
