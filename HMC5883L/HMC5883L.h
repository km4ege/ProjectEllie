/*
HMC5883L.h - Header file for the HMC5883L Triple Axis Magnetometer Arduino Library.
Copyright (C) 2011 Love Electronics (loveelectronics.co.uk)

This program is free software: you can redistribute it and/or modify
it under the terms of the version 3 GNU General Public License as
published by the Free Software Foundation.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.

 WARNING: THE HMC5883L IS NOT IDENTICAL TO THE HMC5883!
 Datasheet for HMC5883L:
 http://www51.honeywell.com/aero/common/documents/myaerospacecatalog-documents/Defense_Brochures-documents/HMC5883L_3-Axis_Digital_Compass_IC.pdf

*/

#ifndef HMC5883L_h
#define HMC5883L_h

#if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
#else
  #include "WProgram.h"
  #include <pins_arduino.h>
#endif
#include <inttypes.h>
#include "../Wire/Wire.h"

#define HMC5883L_Address 0x1E
#define ConfigurationRegisterA 0x00
#define ConfigurationRegisterB 0x01
#define ModeRegister 0x02
#define DataRegisterBegin 0x03

#define Measurement_Continuous 0x00
#define Measurement_SingleShot 0x01
#define Measurement_Idle 0x03


class HMC5883L
{
	public:
	  HMC5883L();
	  int setup(int scale, int measurement);
	  void readAxis();
	  int SetMeasurementMode(uint8_t mode);
	  int SetScale(float gauss);
	  int getX();
	  int getRawX();
	  int getY();
	  int getRawY();
	  int getZ();
	  int getRawZ();

	protected:
	  void Write(int address, int byte);
	  uint8_t* Read(int address, int length);

	private:
	  float m_Scale;
	  int x;
	  int y;
	  int z;
	  int rawX;
	  int rawY;
	  int rawZ;
};
#endif
