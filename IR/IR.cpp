#include "IR.h"
IR::IR(){
	startPulse =0;
	endPulse =0;
	prevReading = 1;
	currReading = 1;
	onSize = 0;
	offSize = 0;
	pinMode(led_pin, OUTPUT);		
	pinMode(8, INPUT); 

}

void IR::receive(){

   currReading = !(PINB & 1);                     // read input
   
  if(prevReading  ==0 && currReading == 1){       // rising edge ( beginning of signal )
    startPulse = micros();                        // start time
    if(endPulse!= 0){                             // if this is not the beginning of the signal, store the off time 
      offTime[offSize] = startPulse - endPulse;
      offSize++;
    }
    
  }else if(prevReading  ==1 && currReading == 0){ // falling edge ( end of signal )
     endPulse =  micros();                        // end time
     onTime[onSize] = endPulse - startPulse;
     onSize++;
  }
  
  if(endPulse!=0){                                // if MAX_DELAY ms passes with no signal, print the signal
    long timeDiff = millis() - (endPulse/1000) ; 
		if(timeDiff >= MAX_DELAY){
		   printSignal();
		   onSize = 0;  offSize = 0;  startPulse =0; endPulse =0;
		}
  }
  
  prevReading = currReading; 
}



void IR::printSignal(){
		bool differnetSignal = false;
		for(int i=0; i<onSize; i++){			// check if the previous signal equal the current signal
			if( abs(oldOnTime[i]-onTime[i]) >= 100)
				differnetSignal = true;
			
			if( abs(oldOffTime[i]-offTime[i]) >= 100)
				differnetSignal = true;
	   }

		if(differnetSignal){
		  Serial.print("int on[] = {");
		   for(int i=0; i<onSize; i++){
			   Serial.print(onTime[i]);
			   if(i!=onSize-1)
				 Serial.print(", ");
		  }
		  Serial.println("};");
		  Serial.print("int SIZE = ");
		  Serial.print(onSize);
		  Serial.println(";");
      
		  Serial.print("int off[] = {");
		   for(int i=0; i<offSize; i++){
			   Serial.print(offTime[i]);
			   if(i!=offSize-1)
				 Serial.print(", ");
		  }
		  Serial.println("};");
		  Serial.println("--------------");
		}else{
			 Serial.println("Same Signal");
		}

	  for(int i=0; i<onSize; i++){			 // save last readings
		oldOnTime[i] = onTime[i];
		oldOffTime[i] = offTime[i];
	  }
}



void IR::send(int *onBuffer, int *offBuffer, int bufferSize){
 int i = 0;
 boolean sending = true;
 long timer = micros();

   Timer1.initialize(26);         // 13us period
   Timer1.attachInterrupt(turnOnLed);
   
   while(i< bufferSize){
      if(sending ){  // set infrared to Low
        if(micros()- timer >= onBuffer[i]){
          Timer1.detachInterrupt();
          digitalWrite(led_pin,LOW);
          timer = micros();
          sending = false;
        }
      }else // set infrared to High 
        if((micros()- timer >= offBuffer[i])){
          Timer1.attachInterrupt(turnOnLed);
          timer = micros();
          sending = true;
          i++;
      }
   } // end while
   
    Timer1.detachInterrupt();
    digitalWrite(led_pin,LOW);
}


void IR::turnOnLed()
{
    digitalWrite(led_pin, HIGH);
    delayMicroseconds(2);
    digitalWrite(led_pin, LOW);
}