
#ifndef ITG3200_h
#define ITG3200_h

#if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
#else
  #include "WProgram.h"
  #include <pins_arduino.h>
#endif
#include "I2C.h"

//Registers in the ITG-3200
#define WHO_AM_I     0x00
#define SMPLRT_DIV   0x15
#define DLPF_FS      0x16
#define itgAddress   0x68
#define GYRO_XOUT_H  0x1D
#define GYRO_XOUT_L  0x1E
#define GYRO_YOUT_H  0x1F
#define GYRO_YOUT_L  0x20
#define GYRO_ZOUT_H  0x21
#define GYRO_ZOUT_L  0x22

//This is a list of settings that can be loaded into the registers.
//DLPF, Full Scale Register Bits
//FS_SEL must be set to 3 for proper operation
//Set DLPF_CFG to 3 for 1kHz Fint and 42 Hz Low Pass Filter
#define DLPF_CFG_0      1<<0
#define DLPF_CFG_1      1<<1
#define DLPF_CFG_2      1<<2
#define DLPF_FS_SEL_0   1<<3
#define DLPF_FS_SEL_1   1<<4

class ITG3200{
private:
    int x;
    int y;
    int z;
public:
    ITG3200();
    void setup();
    void readAxis();
    int getX();
    int getY();
    int getZ();

};


#endif
