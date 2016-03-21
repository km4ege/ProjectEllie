
#include "ITG3200.h"


ITG3200::ITG3200() {
    x = 0;
    y = 0;
    z = 0;
}

void ITG3200::setup(){
//Configure the gyroscope
  I2CWrite(itgAddress, DLPF_FS, (DLPF_FS_SEL_0|DLPF_FS_SEL_1|DLPF_CFG_0));   // Set the gyroscope scale for the outputs to +/-2000 degrees per second
  I2CWrite(itgAddress, SMPLRT_DIV, 9);

}

void ITG3200::readAxis(){
    x = ( I2CRead(itgAddress, GYRO_XOUT_H)<<8 ) |  I2CRead(itgAddress, GYRO_XOUT_L); //combine upper bytes with lower bytes of x
    y = ( I2CRead(itgAddress, GYRO_YOUT_H)<<8 ) |  I2CRead(itgAddress, GYRO_YOUT_L); //combine upper bytes with lower bytes of y
    z = ( I2CRead(itgAddress, GYRO_ZOUT_H)<<8 ) |  I2CRead(itgAddress, GYRO_ZOUT_L); //combine upper bytes with lower bytes of z

}

int ITG3200::getX(){
    return x;
}

int ITG3200::getY(){
    return y;
}

int ITG3200::getZ(){
    return z;
}
