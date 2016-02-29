void runMotor(int numCycles, bool clockwise, bool motor1) //512 cycles = 1 full revolution
{
  int IN1;
  int IN2;
  int IN3;
  int IN4;
  
  if(motor1)
  {
    IN1 = MOTOR1_IN1;  //motor input pins, defined in main file
    IN2 = MOTOR1_IN2;
    IN3 = MOTOR1_IN3;
    IN4 = MOTOR1_IN4;
  }
  else
  {
    IN1 = MOTOR2_IN1;
    IN2 = MOTOR2_IN2;
    IN3 = MOTOR2_IN3;
    IN4 = MOTOR2_IN4;
  }
    
  int increment; 
  int init;
  if(clockwise)
  {
    increment = -1;
    init = 7;
  }
  else
  {
    increment = 1;
    init = 0; 
  }
    
  
  for(int i = 0; i < numCycles; i++)
  {
    for(int j = 0; j < 8; j++)
    {
       switch(init + (increment * j))
       {
           case 0:
           digitalWrite(IN1, LOW); 
           digitalWrite(IN2, LOW);
           digitalWrite(IN3, LOW);
           digitalWrite(IN4, HIGH);
           delayMicroseconds(1000);  //1000 microseconds, 1 ms
           break;                    //gives angular speed of ~90 deg/s
           
           case 1: 
           digitalWrite(IN1, LOW); 
           digitalWrite(IN2, LOW);
           digitalWrite(IN3, HIGH);
           digitalWrite(IN4, HIGH);
           delayMicroseconds(1000);
           break;
        
           case 2:
           digitalWrite(IN1, LOW); 
           digitalWrite(IN2, LOW);
           digitalWrite(IN3, HIGH);
           digitalWrite(IN4, LOW);
           delayMicroseconds(1000);
           break;
        
           case 3:
           digitalWrite(IN1, LOW); 
           digitalWrite(IN2, HIGH);
           digitalWrite(IN3, HIGH);
           digitalWrite(IN4, LOW);
           delayMicroseconds(1000);
           break;
           
           case 4:
           digitalWrite(IN1, LOW); 
           digitalWrite(IN2, HIGH);
           digitalWrite(IN3, LOW);
           digitalWrite(IN4, LOW);
           delayMicroseconds(1000);
           break;
           
           case 5:
           digitalWrite(IN1, HIGH); 
           digitalWrite(IN2, HIGH);
           digitalWrite(IN3, LOW);
           digitalWrite(IN4, LOW);
           delayMicroseconds(1000);
           
           case 6:
           digitalWrite(IN1, HIGH); 
           digitalWrite(IN2, LOW);
           digitalWrite(IN3, LOW);
           digitalWrite(IN4, LOW);
           delayMicroseconds(1000);
           break;
           
           case 7:
           digitalWrite(IN1, HIGH); 
           digitalWrite(IN2, LOW);
           digitalWrite(IN3, LOW);
           digitalWrite(IN4, HIGH);
           delayMicroseconds(1000);
           break;
       }
    }
  }
  digitalWrite(IN1, LOW); 
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}
