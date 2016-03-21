#ifndef IR_h
#define IR_h
#include "Arduino.h"
#include "TimerOne.h"

const int MAX = 100;
const int led_pin = 3;
const int MAX_DELAY = 500;

class IR{
  private:
	long startPulse, endPulse;
	int prevReading, currReading, oldOnTime[MAX], oldOffTime[MAX], onTime[MAX], offTime[MAX], onSize, offSize;

    void printSignal();
	static void turnOnLed();
  public:
	IR();
    void receive();
	void send(int *onBuffer, int *offBuffer, int bufferSize);
};

#endif