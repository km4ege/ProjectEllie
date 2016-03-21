#include "I2C.h"
void I2CWrite(char address, char registerAddress, char data)
{
  Wire.beginTransmission(address);                              //Initiate a communication sequence with the desired i2c device
  Wire.write(registerAddress);                                  //Tell the I2C address which register we are writing to
  Wire.write(data);                                             //Send the value to write to the specified register
  Wire.endTransmission();                                       //End the communication sequence
}

unsigned char I2CRead(char address, char registerAddress)
{
  unsigned char data=0;                                       //This variable will hold the contents read from the i2c device.
  Wire.beginTransmission(address);                            //Send the register address to be read.
  Wire.write(registerAddress);                                //Send the Register Address
  Wire.endTransmission();                                     //End the communication sequence.

  Wire.beginTransmission(address);                            //Ask the I2C device for data
  Wire.requestFrom(address, 1);

  if(Wire.available())                                        //Wait for a response from the I2C device
    data = Wire.read();                                       //Save the data sent from the I2C device

  Wire.endTransmission();                                     //End the communication sequence.

  return data;                                                //Return the data read during the operation
}


int I2CReadWord(char address, char registerAddress)
{
  unsigned char msb, lsb;
  msb = I2CRead(address, registerAddress);
  lsb = I2CRead(address, registerAddress+1);
  return (int) msb<<8 | lsb;
}
