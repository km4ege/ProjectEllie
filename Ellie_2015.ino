//Needed libraries:
#include <SFE_BMP180.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <I2C.h>
#include <Metro.h>
#include <Servo.h> 
#include <TinyGPS.h>
/********************************************BEGIN 9 DEGREES OF FREEDOM FIRMWARE*******************************************************************
 * Razor AHRS Firmware v1.4.1
 * 9 Degree of Measurement Attitude and Heading Reference System
 * for Sparkfun "9DOF Razor IMU" (SEN-10125 and SEN-10736)
 * and "9DOF Sensor Stick" (SEN-10183, 10321 and SEN-10724)
 *
 * Released under GNU GPL (General Public License) v3.0
 * Copyright (C) 2011-2012 Quality & Usability Lab, Deutsche Telekom Laboratories, TU Berlin
 *
 * Infos, updates, bug reports and feedback:
 *     http://dev.qu.tu-berlin.de/projects/sf-razor-9dof-ahrs
 * 
 ***************************************************************************************************************/

/*****************************************************************/
/*********** USER SETUP AREA! Set your options here! *************/
/*****************************************************************/

// HARDWARE OPTIONS
/*****************************************************************/
// Select your hardware here by uncommenting one line!
#define HW__VERSION_CODE 10724 // SparkFun "9DOF Sensor Stick" version "SEN-10724" (HMC5883L magnetometer)

// OUTPUT OPTIONS
/*****************************************************************/
// Set your serial port baud rate used to send out data here!
#define OUTPUT__BAUD_RATE 4800

// Sensor data output interval in milliseconds
// This may not work, if faster than 20ms (=50Hz)
// Code is tuned for 20ms, so better leave it like that
#define OUTPUT__DATA_INTERVAL 100  // in milliseconds
Metro timeMetro = Metro(OUTPUT__DATA_INTERVAL); //Time object; OUTPUT__DATA_INTERVAL defined in Razor AHRS firmware

// SENSOR CALIBRATION
/*****************************************************************/
// How to calibrate? Read the tutorial at http://dev.qu.tu-berlin.de/projects/sf-razor-9dof-ahrs
// Put MIN/MAX and OFFSET readings for your board here!
// Accelerometer
// "accel x,y,z (min/max) = X_MIN/X_MAX  Y_MIN/Y_MAX  Z_MIN/Z_MAX"
#define ACCEL_X_MIN ((float) -266)
#define ACCEL_X_MAX ((float) 275)
#define ACCEL_Y_MIN ((float) -258)
#define ACCEL_Y_MAX ((float) 264)
#define ACCEL_Z_MIN ((float) -295)
#define ACCEL_Z_MAX ((float) 230)

// Magnetometer (standard calibration)
// "magn x,y,z (min/max) = X_MIN/X_MAX  Y_MIN/Y_MAX  Z_MIN/Z_MAX"
#define MAGN_X_MIN ((float) -600)
#define MAGN_X_MAX ((float) 600)
#define MAGN_Y_MIN ((float) -600)
#define MAGN_Y_MAX ((float) 600)
#define MAGN_Z_MIN ((float) -600)
#define MAGN_Z_MAX ((float) 600)

// Gyroscope
// "gyro x,y,z (current/average) = .../OFFSET_X  .../OFFSET_Y  .../OFFSET_Z
#define GYRO_AVERAGE_OFFSET_X ((float) 32.7)
#define GYRO_AVERAGE_OFFSET_Y ((float) -13.72)
#define GYRO_AVERAGE_OFFSET_Z ((float) -11.27)
/*****************************************************************/
/****************** END OF USER SETUP AREA!  *********************/
/*****************************************************************/

// Sensor calibration scale and offset values
#define ACCEL_X_OFFSET ((ACCEL_X_MIN + ACCEL_X_MAX) / 2.0f)
#define ACCEL_Y_OFFSET ((ACCEL_Y_MIN + ACCEL_Y_MAX) / 2.0f)
#define ACCEL_Z_OFFSET ((ACCEL_Z_MIN + ACCEL_Z_MAX) / 2.0f)
#define ACCEL_X_SCALE (GRAVITY / (ACCEL_X_MAX - ACCEL_X_OFFSET))
#define ACCEL_Y_SCALE (GRAVITY / (ACCEL_Y_MAX - ACCEL_Y_OFFSET))
#define ACCEL_Z_SCALE (GRAVITY / (ACCEL_Z_MAX - ACCEL_Z_OFFSET))

#define MAGN_X_OFFSET ((MAGN_X_MIN + MAGN_X_MAX) / 2.0f)
#define MAGN_Y_OFFSET ((MAGN_Y_MIN + MAGN_Y_MAX) / 2.0f)
#define MAGN_Z_OFFSET ((MAGN_Z_MIN + MAGN_Z_MAX) / 2.0f)
#define MAGN_X_SCALE (100.0f / (MAGN_X_MAX - MAGN_X_OFFSET))
#define MAGN_Y_SCALE (100.0f / (MAGN_Y_MAX - MAGN_Y_OFFSET))
#define MAGN_Z_SCALE (100.0f / (MAGN_Z_MAX - MAGN_Z_OFFSET))
// Gain for gyroscope (ITG-3200)
#define GYRO_GAIN (0.06957) // Same gain on all axes
#define GYRO_SCALED_RAD(x) (x * TO_RAD((GYRO_GAIN))) // Calculate the scaled gyro readings in radians per second

// DCM parameters
#define Kp_ROLLPITCH (0.02f)
#define Ki_ROLLPITCH (0.00002f)
#define Kp_YAW (1.2f)
#define Ki_YAW (0.00002f)

// Stuff
#define GRAVITY 256.0f // "1G reference" used for DCM filter and accelerometer calibration
#define TO_RAD(x) (x * 0.01745329252)  // *pi/180
#define TO_DEG(x) (x * 57.2957795131)  // *180/pi

// Sensor variables
float accel[3];  // Actually stores the NEGATED acceleration (equals gravity, if board not moving).
float accel_min[3];
float accel_max[3];

float magnetom[3];
float magnetom_min[3];
float magnetom_max[3];
float magnetom_tmp[3];

float gyro[3];
float gyro_average[3];
int gyro_num_samples = 0;

// DCM variables
float MAG_Heading;
float Accel_Vector[3]= {
  0, 0, 0}; // Store the acceleration in a vector
float Gyro_Vector[3]= {
  0, 0, 0}; // Store the gyros turn rate in a vector
float Omega_Vector[3]= {
  0, 0, 0}; // Corrected Gyro_Vector data
float Omega_P[3]= {
  0, 0, 0}; // Omega Proportional correction
float Omega_I[3]= {
  0, 0, 0}; // Omega Integrator
float Omega[3]= {
  0, 0, 0};
float errorRollPitch[3] = {
  0, 0, 0};
float errorYaw[3] = {
  0, 0, 0};
float DCM_Matrix[3][3] = {
  {
    1, 0, 0      }
  , {
    0, 1, 0      }
  , {
    0, 0, 1      }
};
float Update_Matrix[3][3] = {
  {
    0, 1, 2      }
  , {
    3, 4, 5      }
  , {
    6, 7, 8      }
};
float Temporary_Matrix[3][3] = {
  {
    0, 0, 0      }
  , {
    0, 0, 0      }
  , {
    0, 0, 0      }
};

// Euler angles
float yaw;
float pitch;
float roll;

// DCM timing in the main loop
unsigned long timestamp;
unsigned long timestamp_old;
float G_Dt; // Integration time for DCM algorithm

// More output-state variables
boolean output_stream_on;
boolean output_single_on;
int curr_calibration_sensor = 0;
boolean reset_calibration_session_flag = true;

void read_sensors() {
  Read_Gyro(); // Read gyroscope
  Read_Accel(); // Read accelerometer
  Read_Magn(); // Read magnetometer
}

// Read every sensor and record a time stamp
// Init DCM with unfiltered orientation
// TODO re-init global vars?
void reset_sensor_fusion() {
  float temp1[3];
  float temp2[3];
  float xAxis[] = {
 1.0f, 0.0f, 0.0f      };

  read_sensors(); // read all sensors 
  timestamp = millis(); // update timestamp

  // GET PITCH
  // Using y-z-plane-component/x-component of gravity vector
  pitch = -atan2(accel[0], sqrt(accel[1] * accel[1] + accel[2] * accel[2]));

  // GET ROLL
  // Compensate pitch of gravity vector 
  Vector_Cross_Product(temp1, accel, xAxis);
  Vector_Cross_Product(temp2, xAxis, temp1);
  // Normally using x-z-plane-component/y-component of compensated gravity vector
  // roll = atan2(temp2[1], sqrt(temp2[0] * temp2[0] + temp2[2] * temp2[2]));
  // Since we compensated for pitch, x-z-plane-component equals z-component:
  roll = atan2(temp2[1], temp2[2]);

  // GET YAW
  Compass_Heading();
  yaw = MAG_Heading;

  // Init rotation matrix
  init_rotation_matrix(DCM_Matrix, yaw, pitch, roll);
}

// Apply calibration to raw sensor readings
void compensate_sensor_errors() {
  // Compensate accelerometer error
  accel[0] = (accel[0] - ACCEL_X_OFFSET) * ACCEL_X_SCALE;
  accel[1] = (accel[1] - ACCEL_Y_OFFSET) * ACCEL_Y_SCALE;
  accel[2] = (accel[2] - ACCEL_Z_OFFSET) * ACCEL_Z_SCALE;

  // Compensate magnetometer error
  magnetom[0] = (magnetom[0] - MAGN_X_OFFSET) * MAGN_X_SCALE;
  magnetom[1] = (magnetom[1] - MAGN_Y_OFFSET) * MAGN_Y_SCALE;
  magnetom[2] = (magnetom[2] - MAGN_Z_OFFSET) * MAGN_Z_SCALE;

  // Compensate gyroscope error
  gyro[0] -= GYRO_AVERAGE_OFFSET_X;
  gyro[1] -= GYRO_AVERAGE_OFFSET_Y;
  gyro[2] -= GYRO_AVERAGE_OFFSET_Z;
}
/////////////////////////END 9 DEGREES OF FREEDOM FIRMWARE////////////////////////////////////////////////////////////////

/////////////////////////BEGIN FUNCTIONS FOR SHT15 HUMIDITY SENSOR////////////////////////////////////////////////////////

#define SHT_dataPin 12 //For SHT15 sensor
#define SHT_clockPin 11 //For SHT15 sensor
//SHT1x sht15(SHT_dataPin, SHT_clockPin);


float temperature = 0; //For SHT15 sensor
float humidity = 0; //For SHT15 sensor
unsigned long shtTime = 0; //The last time the sht15 sensor was read


float getTemp(){
  //Return Temperature in Celsius
  SHT_sendCommand(B00000011, SHT_dataPin, SHT_clockPin);
  SHT_waitForResult(SHT_dataPin);

  int val = SHT_getData(SHT_dataPin, SHT_clockPin);
  SHT_skipCrc(SHT_dataPin, SHT_clockPin);
  return (float)val * 0.01 - 40.1; //convert to celsius
}


float getHumidity(){
  //Return  Relative Humidity
  SHT_sendCommand(B00000101, SHT_dataPin, SHT_clockPin);
  SHT_waitForResult(SHT_dataPin);
  int val = SHT_getData(SHT_dataPin, SHT_clockPin);
  SHT_skipCrc(SHT_dataPin, SHT_clockPin);
  return -4.0 + 0.0405 * val + -0.0000028 * val * val; 
}

void SHT_sendCommand(int command, int dataPin, int clockPin){
  // send a command to the SHTx sensor
  // transmission start
  pinMode(dataPin, OUTPUT);
  pinMode(clockPin, OUTPUT);
  digitalWrite(dataPin, HIGH);
  digitalWrite(clockPin, HIGH);
  digitalWrite(dataPin, LOW);
  digitalWrite(clockPin, LOW);
  digitalWrite(clockPin, HIGH);
  digitalWrite(dataPin, HIGH);
  digitalWrite(clockPin, LOW);

  // shift out the command (the 3 MSB are address and must be 000, the last 5 bits are the command)
  shiftOut(dataPin, clockPin, MSBFIRST, command);

  // verify we get the right ACK
  digitalWrite(clockPin, HIGH);
  pinMode(dataPin, INPUT);

//  if (digitalRead(dataPin)) Serial.println("ACK error 0");
  digitalWrite(clockPin, LOW);
//  if (!digitalRead(dataPin)) Serial.println("ACK error 1");
}


void SHT_waitForResult(int dataPin){
  // wait for the SHTx answer
  pinMode(dataPin, INPUT);

  int ack; //acknowledgement

  //need to wait up to 2 seconds for the value
  for (int i = 0; i < 1000; ++i){ //This part has been modified to protect data rate. The humidity value can be read in less then 100ms wait time
    delay(2);                    //HOWEVER the TEMP MUST have a longer wait time (make it 1000 instead of 100) otherwise, expect an error
    ack = digitalRead(dataPin);
    if (ack == LOW) break;
  }

}

int SHT_getData(int dataPin, int clockPin){
  // get data from the SHTx sensor

  // get the MSB (most significant bits)
  pinMode(dataPin, INPUT);
  pinMode(clockPin, OUTPUT);
  byte MSB = shiftIn(dataPin, clockPin, MSBFIRST);

  // send the required ACK
  pinMode(dataPin, OUTPUT);
  digitalWrite(dataPin, HIGH);
  digitalWrite(dataPin, LOW);
  digitalWrite(clockPin, HIGH);
  digitalWrite(clockPin, LOW);

  // get the LSB (less significant bits)
  pinMode(dataPin, INPUT);
  byte LSB = shiftIn(dataPin, clockPin, MSBFIRST);
  return ((MSB << 8) | LSB); //combine bits
}

void SHT_skipCrc(int dataPin, int clockPin){
  // skip CRC data from the SHTx sensor
  pinMode(dataPin, OUTPUT);
  pinMode(clockPin, OUTPUT);
  digitalWrite(dataPin, HIGH);
  digitalWrite(clockPin, HIGH);
  digitalWrite(clockPin, LOW);
}
//////////////////////////////////////////////////////// End sht15 functions ////////////////////////////////////////////////////////////////////

//cutdown stuff:

static const unsigned long BALLOON_CUTDOWN_START_TIME_MS = (14400000); // 14400000 ms = 4 hr  15000;
bool balloon_cutdown_complete = false;
static const unsigned long PAYLOAD_CUTDOWN_START_TIME_MS = (21600000); // 21600000 ms = 6 hr  45000;
bool payload_cutdown_complete = false;

//cutdown step motor pins:
#define MOTOR1_IN1 3
#define MOTOR1_IN2 4
#define MOTOR1_IN3 5
#define MOTOR1_IN4 6

#define MOTOR2_IN1 31
#define MOTOR2_IN2 32
#define MOTOR2_IN3 33
#define MOTOR2_IN4 34

// Pressure object
SFE_BMP180 BMP180;
bool tempReady;
bool pressureReady;
double BMPTemp, BMPPressure;
#define MAX_DELAY 40

//GPS connection pins
//SCHANGE complete
// must update to correct pins for GPS, check the schematic for pin numbers
TinyGPS gps; //GPS object from library

//GPS serial
//SCHANGE complete
// change from sd_serial to gpsSerial
SoftwareSerial gpsSerial(52,53); //RX, TX

// Geiger counter variable
unsigned long geiger_count = 0;
#define GEIGER_COUNTER_PIN (2)
#define GEIGER_COUNTER_INTERRUPT (0)

void geiger_increment(void)
{
  geiger_count++;
}

void turn_output_stream_on()
{
  output_stream_on = true;
}


//Start of program
void setup()
{  
    //setup cutdown step motor pins:
    pinMode(MOTOR1_IN1,OUTPUT);
    pinMode(MOTOR1_IN2,OUTPUT);
    pinMode(MOTOR1_IN3,OUTPUT);
    pinMode(MOTOR1_IN4,OUTPUT);

    digitalWrite(MOTOR1_IN1,LOW);
    digitalWrite(MOTOR1_IN2,LOW);
    digitalWrite(MOTOR1_IN3,LOW);
    digitalWrite(MOTOR1_IN4,LOW);
    
    pinMode(MOTOR2_IN1,OUTPUT);
    pinMode(MOTOR2_IN2,OUTPUT);
    pinMode(MOTOR2_IN3,OUTPUT);
    pinMode(MOTOR2_IN4,OUTPUT);

    digitalWrite(MOTOR2_IN1,LOW);
    digitalWrite(MOTOR2_IN2,LOW);
    digitalWrite(MOTOR2_IN3,LOW);
    digitalWrite(MOTOR2_IN4,LOW);
  
  // Init serial output
  //SCHANGE complete
  // switch Serial and SoftwareSerial
  gpsSerial.begin(OUTPUT__BAUD_RATE);
  Serial.begin(9600);

  // Init sensors
  delay(50);  // Give sensors enough time to start
  I2C_Init(); 
  Accel_Init();
  Magn_Init();
  Gyro_Init(); 
  BMP180.begin();
  tempReady = false;
  pressureReady = false;

  // Read sensors, init DCM algorithm
  delay(20);  // Give sensors enough time to collect data
  reset_sensor_fusion();

  // Enable geiger counter interrupt
  pinMode(GEIGER_COUNTER_PIN, INPUT);
  attachInterrupt(GEIGER_COUNTER_INTERRUPT, geiger_increment, RISING);

  // Init output
  turn_output_stream_on();

}

// Main loop -continuous execution of program-
void loop()
{
  
  // Time to read the sensors again?
  if(timeMetro.check() == 1)
  {
    timestamp_old = timestamp;
    timestamp = millis();
    if (timestamp > timestamp_old)
      G_Dt = (float) (timestamp - timestamp_old) / 1000.0f; // Real time of loop run. We use this on the DCM algorithm (gyro integration time)
    else G_Dt = 0;
    
    // Update sensor readings
    read_sensors();

    // Apply sensor calibration
    compensate_sensor_errors();   
    // Run DCM algorithm
    Compass_Heading(); // Calculate magnetic heading
    Matrix_update();
    Normalize();
    Drift_correction();
    Euler_angles();
    
    //read sht15
    if( timestamp >= shtTime + 3000)
    {
        shtTime = timestamp;
        humidity = getHumidity();
        temperature = getTemp();
    }
    
    if (output_stream_on || output_single_on) 
      output_sd();  
    output_single_on = false;
    
    tempReady = false;
    pressureReady = false;
  }

  if(!tempReady)
  {
    char delayms = BMP180.startTemperature();
    if(delayms <= MAX_DELAY)
    {
      delay(delayms);
      BMP180.getTemperature(BMPTemp);
      tempReady = true;
    }
  }
  if(!pressureReady)
  {
    char delayms = BMP180.startPressure(0);
    if(delayms <= MAX_DELAY)
    {
      delay(delayms);
      BMP180.getPressure(BMPPressure, BMPTemp);
      pressureReady = true;
    }
  }
  
    //Balloon and payload cutdown code
    if(millis() >= BALLOON_CUTDOWN_START_TIME_MS && !balloon_cutdown_complete)
    {
      Serial.println("/RUNNING BALLOON CUTDOWN/");
      runMotor(1280,true,true);
      balloon_cutdown_complete = true;
    }
    if(millis() >= PAYLOAD_CUTDOWN_START_TIME_MS && !payload_cutdown_complete)
    {
      Serial.println("/RUNNING PAYLOAD CUTDOWN/");
      runMotor(1280,true,false);
      payload_cutdown_complete = true;
    }
   

// Simulate behavior of serialEvent() for the SoftwareSerial connection
  softwareSerialEvent();
}

