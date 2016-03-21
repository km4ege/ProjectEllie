#ifndef I2C_h
#define I2C_h

#if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
#else
  #include "WProgram.h"
  #include <pins_arduino.h>
#endif
#include "Wire.h"


//Parameters:
//  char address: The I2C address of the sensor
//  char registerAddress: The address of the register on the sensor that should be written to.
//  char data: The value to be written to the specified register.
void I2CWrite(char address, char registerAddress, char data);

//This function will read the data from a specified register and return the value.
//Parameters:
//  char address: The I2C address of the sensor.
//  char registerAddress: The address of the register on the sensor that should be read
//Return:
//  unsigned char: The value currently residing in the specified register
unsigned char I2CRead(char address, char registerAddress);

//read from two registers
int I2CReadWord(char address, char registerAddress);


#endif
