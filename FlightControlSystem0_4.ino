// We'll use Servo to control our ESCs
#include <Servo.h> 
// Libraries for interaction with IMU
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_BMP085_U.h>
#include <Adafruit_L3GD20_U.h>
#include <Adafruit_10DOF.h>

// PWM pins
#define ESCPin1_TT 3
#define ESCPin2_LT 5
#define ESCPin3_TB 6
#define ESCPin4_RB 9
#define ESCPin5_RT 10
#define ESCPin6_LB 11

// Here we declare the ESCs,... we treat them as servos
Servo ESC_RT;
Servo ESC_LT;
Servo ESC_TT;
Servo ESC_RB;
Servo ESC_LB;
Servo ESC_TB;

int PWMMax = 200;  // maximum PWM value
int PWMMin = 10;   // minimum PWM value that causes the motors to spin
int PWMZero = 0;   // PWM zeroing value used to calibrate the ESC

// Assign a unique ID to the sensors
Adafruit_10DOF                dof   = Adafruit_10DOF();
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);
Adafruit_LSM303_Mag_Unified   mag   = Adafruit_LSM303_Mag_Unified(30302);
Adafruit_BMP085_Unified       bmp   = Adafruit_BMP085_Unified(18001);
Adafruit_L3GD20_Unified       gyro  = Adafruit_L3GD20_Unified(20);

/* Update this with the correct SLP for accurate altitude measurements */
float seaLevelPressure = SENSORS_PRESSURE_SEALEVELHPA;

// Orientation that we get from sensors
sensors_vec_t   orientation;

// Init all the sensors of our IMU
void initSensors()
{
  if(!accel.begin())
  {
    /* There was a problem detecting the LSM303 ... check your connections */
    Serial.println(F("Ooops, no LSM303 detected ... Check your wiring!"));
    while(1);
  }
  if(!mag.begin())
  {
    /* There was a problem detecting the LSM303 ... check your connections */
    Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
    while(1);
  }
  if(!bmp.begin())
  {
    /* There was a problem detecting the BMP180 ... check your connections */
    Serial.println("Ooops, no BMP180 detected ... Check your wiring!");
    while(1);
  }
  if(!gyro.begin())
  {
    /* There was a problem detecting the L3GD20 ... check your connections */
    Serial.println("Ooops, no L3GD20 detected ... Check your wiring!");
    while(1);
  }
}

// Not used in the current system due to problems with accelerometer
/*void getSensorData(){
  sensors_event_t accel_event;
  sensors_event_t mag_event;
  sensors_event_t bmp_event;
  
  // Calculate pitch and roll from the raw accelerometer data
  accel.getEvent(&accel_event);
  if (dof.accelGetOrientation(&accel_event, &orientation))
  {
    // 'orientation' should have valid .roll and .pitch fields
    Serial.print(F("Roll: "));
    Serial.print(orientation.roll);
    Serial.print(F("; "));
    Serial.print(F("Pitch: "));
    Serial.print(orientation.pitch);
    Serial.print(F("; "));
  }
  
  // Calculate the heading using the magnetometer 
  mag.getEvent(&mag_event);
  if (dof.magGetOrientation(SENSOR_AXIS_Z, &mag_event, &orientation))
  {
    // 'orientation' should have valid .heading data now
    Serial.print(F("Heading: "));
    Serial.print(orientation.heading);
    Serial.print(F("; "));
  }
}
*/

// PWMValue_base is the one that we control programatically
int PWMValue_base = 10;

// PWMValue_bottom is the value calculated using collected data using tables
int PWMValue_bottom = 16;

// PWM base values of RightTop, LeftTop, TailTop, RightBottom, LeftBottom, TailBottom
int PWMValue_RT = 10;
int PWMValue_LT = 10;
int PWMValue_TT = 10;
int PWMValue_RB = 16;
int PWMValue_LB = 16;
int PWMValue_TB = 16;

// PWM offset values of RightTop, LeftTop, TailTop, RightBottom, LeftBottom, TailBottom
// based on the gyroscope data
int offset_RT = 0;
int offset_LT = 0;
int offset_TT = 0;
int offset_RB = 0;
int offset_LB = 0;
int offset_TB = 0;

// Enable sending gyroscope data through XBee: Configurable at runtime
boolean sendGyroscopeData = false;

// Enabling dynamic correction based on gyroscope data: Configurable at runtime
boolean enablePitchBalance = false;
boolean enableRollBalance = false;
boolean enableYawBalance = false;

// Which bottom prop gain are we modifying
int bottomProp = 0;

// How much imbalance have been observed by sensor
int pitchImbalanceCycles = 0;
int rollImbalanceCycles = 0;
int yawImbalanceCycles = 0;

// How often do we correct imbalance
int pitchCorrectionPeriod = 10;
int rollCorrectionPeriod = 10;
int yawCorrectionPeriod = 10;

// deviation of yaw that is considered standard
float pitchSensitivity = 0.03;
float rollSensitivity = 0.03;
float yawSensitivity = 0.03;

void getGyroData() {
  sensors_event_t gyro_event;
  gyro.getEvent(&gyro_event);

  // Corrected data according to   
  float gyro_x = gyro_event.gyro.x + 0.03;
  float gyro_y = gyro_event.gyro.y + 0.01;
  float gyro_z = gyro_event.gyro.z + 0.02;
  if (sendGyroscopeData){
    Serial.print(millis());  Serial.print(": Gyroscope: ");
    Serial.print("\tX: "); Serial.print(gyro_x);// Serial.print(" rad/s ");
    Serial.print(" \tY: "); Serial.print(gyro_y);// Serial.print(" rad/s ");
    Serial.print(" \tZ: "); Serial.print(gyro_z);// Serial.print(" rad/s ");
    Serial.print(" \tRT: "); Serial.print(PWMValue_RT + offset_RT);
    Serial.print("\tLT: "); Serial.print(PWMValue_LT + offset_LT);
    Serial.print("\tTT: "); Serial.print(PWMValue_TT + offset_TT);
    Serial.print("\tRB: "); Serial.print(PWMValue_RB + offset_RB);
    Serial.print("\tLB: "); Serial.print(PWMValue_LB + offset_LB);
    Serial.print("\tTB: "); Serial.print(PWMValue_TB + offset_TB);
    Serial.print("\tYP: "); Serial.print(yawCorrectionPeriod);
    Serial.print("\tYS: "); Serial.print(yawSensitivity);
    Serial.println("");
  }

  if (enableRollBalance) {
    if (gyro_x > rollSensitivity){
      rollImbalanceCycles ++;
    }
    else if (gyro_x < -rollSensitivity){
      rollImbalanceCycles --;
    }
  }
  
  if (enablePitchBalance) {
    if (gyro_y > pitchSensitivity){
      pitchImbalanceCycles ++;
    }
    else if (gyro_y < -pitchSensitivity){
      pitchImbalanceCycles --;
    }
  }
  
  if (enableYawBalance) {
    if (gyro_z > yawSensitivity){
      yawImbalanceCycles ++;
    }
    else if (gyro_z < -yawSensitivity){
      yawImbalanceCycles --;
    }
  }

}

void balanceOffset(){
  // Balance yaw if configured
  if (enableYawBalance) {
    if (yawImbalanceCycles > yawCorrectionPeriod){
      yawImbalanceCycles -= yawCorrectionPeriod;
      // Go to the next of 3 props
      bottomProp = (bottomProp + 1) % 3;
      
      // Change offset on needed prop
      if (bottomProp == 0){
        offset_TB ++;
      }
      else if (bottomProp == 1){
        offset_RB ++; 
      }
      else if (bottomProp == 2) {
        offset_LB ++; 
      }
    }
    else if (yawImbalanceCycles < -yawCorrectionPeriod) {
      yawImbalanceCycles += yawCorrectionPeriod;
      // Change offset on needed prop
      if (bottomProp == 0){
        offset_TB --;
      }
      else if (bottomProp == 1){
        offset_RB --; 
      }
      else if (bottomProp == 2) {
        offset_LB --; 
      }

      // Go to previous of 3 props
      bottomProp = (bottomProp + 2) % 3;
    }
  }

  // Enable roll balance if configured
  if (enableRollBalance) {
    // Rolling right
    if (rollImbalanceCycles > rollCorrectionPeriod){
      rollImbalanceCycles -= rollCorrectionPeriod;
      offset_LT--;
      offset_RT++;
    }
    // Rolling left
    else if (rollImbalanceCycles < -rollCorrectionPeriod){
      rollImbalanceCycles += rollCorrectionPeriod;
      offset_LT++;
      offset_RT--;
    }
  }

  // Enable pitch balance if configured
  if (enablePitchBalance) {
    // Pitching forward
    if (pitchImbalanceCycles > pitchCorrectionPeriod){
      pitchImbalanceCycles -= pitchCorrectionPeriod;
      offset_TT--;
    }
    // Pitching backward
    else if (pitchImbalanceCycles < -pitchCorrectionPeriod){
      pitchImbalanceCycles += pitchCorrectionPeriod;
      offset_TT++;
    }
  }

}


void setup(){
  delay(10000);
  
  // Setup the connection to the XBee
  Serial.begin(9600);

  // Connect to the ESCs as if they were servos
  ESC_RT.attach(ESCPin5_RT);
  ESC_LT.attach(ESCPin2_LT);
  ESC_TT.attach(ESCPin1_TT);
  ESC_RB.attach(ESCPin4_RB);
  ESC_LB.attach(ESCPin6_LB);
  ESC_TB.attach(ESCPin3_TB);

  Serial.print( "Initialize ESC... " );
  ESC_RT.write(PWMZero); 
  ESC_LT.write(PWMZero); 
  ESC_TT.write(PWMZero); 
  ESC_RB.write(PWMZero); 
  ESC_LB.write(PWMZero); 
  ESC_TB.write(PWMZero); 

  // Here we zero the ESCs and wait for 10 seconds. You must zero the ESCs to 
  // initialize them. You only need to wait 5 seconds for initialization, but
  // I wait longer, just to get out of the way of the Y6.
  // Also, change the switch in UART position
  for (int count = 10; count >= 1; count--){
    Serial.println(count);
    delay(1000);
  }
  
  // Initializing sensors
  initSensors();
  
  // Now we set the minimum PWM value before starting the test.
  
  Serial.println( "initialization complete... " );
  Serial.println( "Starting test... " );
}

void adjust_bottom_props(){
  // Set top props based on base PWMValue
  PWMValue_RT = PWMValue_base;
  PWMValue_LT = PWMValue_base;
  PWMValue_TT = PWMValue_base;
  
  // Calculate PWMValue on bottom props based on table
  if (PWMValue_base <= 20){
    PWMValue_bottom = PWMValue_base + 6 + (PWMValue_base-10) * 4 / 10;
  }
  else if (PWMValue_base <= 40){
    PWMValue_bottom = PWMValue_base + 10 + (PWMValue_base-10) / 2;
  }
  else if (PWMValue_base <= 60){
    PWMValue_bottom = PWMValue_base + 21;
  }
  else if (PWMValue_base <= 70){
    PWMValue_bottom = PWMValue_base + 22;
  }
  else if (PWMValue_base <= 80){
    PWMValue_bottom = PWMValue_base + 23;
  }
  else if (PWMValue_base <= 85){
    PWMValue_bottom = PWMValue_base + 23 + (PWMValue_base-80) * 3 / 5;
  }
  else if (PWMValue_base <= 100){
    PWMValue_bottom = PWMValue_base + 26 + (PWMValue_base-85) * 2 / 5;
  }
  else {
    PWMValue_bottom = PWMValue_base + 32 + (PWMValue_base-100) * 3 / 5;
  }
  
  PWMValue_RB = PWMValue_bottom;
  PWMValue_LB = PWMValue_bottom;
  PWMValue_TB = PWMValue_bottom;
}

boolean ESC_ON = false;

void abortMission(){
  ESC_ON = false;
}

void startMission(){
  ESC_ON = true;
}

// Increase the base value and adjust bottom props based on table
void ascend(){
  PWMValue_base++;
  adjust_bottom_props();
}

// Decrease the base value and adjust bottom props based on table
void descend(){
  PWMValue_base--;
  adjust_bottom_props();
}

// Log currently applied PWM values
void log_pwm(){
  Serial.print("PWMValue RT = "); 
  Serial.println(PWMValue_RT + offset_RT);
  Serial.print("PWMValue LT = "); 
  Serial.println(PWMValue_LT + offset_LT);
  Serial.print("PWMValue TT = "); 
  Serial.println(PWMValue_TT + offset_TT);
  Serial.print("PWMValue RB = "); 
  Serial.println(PWMValue_RB + offset_RB);
  Serial.print("PWMValue LB = "); 
  Serial.println(PWMValue_LB + offset_LB);
  Serial.print("PWMValue TB = "); 
  Serial.println(PWMValue_TB + offset_TB);
}

void loop() {
  // We are going to use 1 char commands
  if (Serial.available()) {
    char command = Serial.read();
    switch (command){
      case '1': if (PWMValue_RT<PWMMax) PWMValue_RT++; break;
      case '2': if (PWMValue_LT<PWMMax) PWMValue_LT++; break;
      case '3': if (PWMValue_TT<PWMMax) PWMValue_TT++; break;
      case '4': if (PWMValue_RB<PWMMax) PWMValue_RB++; break;
      case '5': if (PWMValue_LB<PWMMax) PWMValue_LB++; break;
      case '6': if (PWMValue_TB<PWMMax) PWMValue_TB++; break;
      case 'q':
      case 'Q': if (PWMValue_RT>PWMMin) PWMValue_RT--; break;
      case 'w':
      case 'W': if (PWMValue_LT>PWMMin) PWMValue_LT--; break;
      case 'e':
      case 'E': if (PWMValue_TT>PWMMin) PWMValue_TT--; break;
      case 'r':
      case 'R': if (PWMValue_RB>PWMMin) PWMValue_RB--; break;
      case 't':
      case 'T': if (PWMValue_LB>PWMMin) PWMValue_LB--; break;
      case 'y':
      case 'Y': if (PWMValue_TB>PWMMin) PWMValue_TB--; break;
      case 'a': 
      case 'A': abortMission(); break;
      case 's': 
      case 'S': startMission(); break;
      case 'u':
      case 'U': ascend();       break;
      case 'd': 
      case 'D': descend();      break;
      case 'l':
      case 'L': log_pwm();      break;
      case 'z':
      case 'Z': sendGyroscopeData = !sendGyroscopeData; break;
      case 'x':
      case 'X': enablePitchBalance = !enablePitchBalance; break;
      case 'c':
      case 'C': enableRollBalance = !enableRollBalance; break;
      case 'v':
      case 'V': enableYawBalance = !enableYawBalance; break;
      case 'b':
      case 'B': if (yawCorrectionPeriod > 1) yawCorrectionPeriod --; break; 
      case 'n':
      case 'N': yawCorrectionPeriod ++; break;
      case 'h':
      case 'H': if (yawSensitivity > 0.01) yawSensitivity -=0.01; break; 
      case 'j':
      case 'J': yawSensitivity +=0.01; break;

      default:                  break;
    }
  }
  if (ESC_ON) {
    // Calculate offset based on gyro data
    balanceOffset();
    
    // Write values plus offset to the ESC
    ESC_RT.write(PWMValue_RT + offset_RT);
    ESC_LT.write(PWMValue_LT + offset_LT);
    ESC_TT.write(PWMValue_TT + offset_TT);
    ESC_RB.write(PWMValue_RB + offset_RB);
    ESC_LB.write(PWMValue_LB + offset_LB);
    ESC_TB.write(PWMValue_TB + offset_TB);
  }
  else {
    // Stop the copter
    ESC_RT.write(PWMZero);
    ESC_LT.write(PWMZero);
    ESC_TT.write(PWMZero);
    ESC_RB.write(PWMZero);
    ESC_LB.write(PWMZero);
    ESC_TB.write(PWMZero);
  }
  getGyroData();
}
