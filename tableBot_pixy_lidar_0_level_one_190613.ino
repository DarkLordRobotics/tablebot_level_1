////////////////////////////////////////////////////////////////////////////////
// tabletbot on Bile chassis
// using PixyCam to scan for object & goal
// using VL53L1X LIDAR to scan for goal(?) & for navigation
// using SparkFun Ardumoto Motor Driver Shield, Arduino Mega2560 (?)
////////////////////////////////////////////////////////////////////////////////
#define code_version "190613_1700"

//RH count encoder ticks leading up to u-turn, off set the count difference
//RH add direction to inc/dec encoders as needed
// equalize encoder counts? drive ticksTurned to zero?
//RH change nav array to three readings, test ranges, delays, etc.
//  try +/- 45 deg?
// set 4x4 ROI


/*
todo:

get lidar working for staying on the table, end of table detect, maybe goal

get pixy working for block & goal

use lidar to detect range to block @ pixy angle??

nav code using Encoders, angle from pixy, range from lidar

upgrade to BNO055? for course, u-turn
*/
/*
//====================================
table bot (bile)

lidar on pan/tilt servos to scan for edges, end of table

setup:
1. pixy cam scan of table at start
2. determine level based on what's detected
3. use LED to show what's found (red, green, blue) for nothing, goal, block

loop:
4. plot course (angle) to block, calc distance (remember both)
5. turn using encoder or compass, drive forward distance
6. confirm scan detects block
7. reverse course change, drive distance (use 2x angle change if past halfway point, 3', or 3x if past 4')
8. turn course change angle, back to straight line at center
9. drive forward to goal
10. detect goal, back up, turn around
11. drive to first edge, victory dance!

// pixy cam scan table, find object & goal, if any
// set strategy
// if no block/goal detected, set level 1
//   tilt down & center lidar to look for table end
//   move forward, detect & return
// if block detected, set level 2
//   turn & nav to block, turn & nav to table center
//   move to end, detect & return
// if block & goal detected, set level 3
//   turn & nav to block, turn & nav to table center
//   move to end, detect goal & return
// set success = 1;
// nav: if block, set angle (turn) and move distance
// then turn back & move distance
// then turn back on centerline course, move to end/goal

nav:
if object, record angle course change to get, also distance traveled.
reverse change after capture & travel same distance, then turn back onto straight course
if distance > half table, return angle change is 2x initial course change.
turn back to straight is same as return angle change.

*/

/* ----------------------------
PIN MAP Mega2560:
0: serial                   // add wireless?
1: serial                   // add wireless?
2: encoderPinB              // INT
3: encoderPinA              // INT
4: 
5: servoPanLidar.attach(5);  
6: servoTiltLidar.attach(6);  
7: DIR_B_PIN                // direction control for motor outputs 3 and 4, motor B, right side motor
8: DIR_A_PIN                // direction control for motor outputs 1 and 2, motor A, left side motor
9: PWM_A_PIN                // PWM control for motor outputs 1 and 2, speed A
10: PWM_B_PIN               // PWM control for motor outputs 3 and 4, speed B
11: xshut0
12: xshut1
13: 

A0: 
A1: 
A2: 
A3: 

I2C:
0x32: LP55231 LED driver
//0x1D & 0x6B (gyro): Pololu MiniIMU-9 v3
//0x52: VL53L0X I2C device address (0x28)
0x52: VL53L1X I2C device address (0x29?)
---------------------------- */

// ------------------------------
// Pixy Cam
#include <SPI.h>  
#include <Pixy.h>

Pixy pixy;

#define X_CENTER        ((PIXY_MAX_X-PIXY_MIN_X)/2)       
#define Y_CENTER        ((PIXY_MAX_Y-PIXY_MIN_Y)/2)

int panError, tiltError;
//int32_t panError, tiltError;

// ------------------------------
//For I2C interface:
#include <Wire.h>

//Adafruit TCA9548A 1-to-8 I2C Multiplexer Breakout
#define TCAADDR 0x70

// SparkFun LED Driver Breakout - LP55231
// I2C device address 0x32
#include <lp55231.h>
Lp55231 ledChip;

// ------------------------------
//RH needed?

// VL53L1X chin laser
//#include <VL53L1X.h>  // Pololu
//VL53L1X distanceSensorL1X;

// VL53L1X I2C device address: 0x52
#include "SparkFun_VL53L1X_Arduino_Library.h"
VL53L1X distanceSensor;

// laser pan & tilt servos
#include <Servo.h> 
Servo servoPanLidar;
Servo servoTiltLidar;

#define h_Lidar 155  // 155 mm down, 160 mm level
#define h_Lidar_floor 866  // h_lidar plus 28" table height

#define numLines   3   // 0 = goal, 1 = nav, 2 = check for block capture
#define arraySize0 41  // 41 for goal; if 1" per pan step, 3' wide table (autoset from scan?)
#define arraySize1 3   // 5? nav
//#define arraySize1 5   // 7? nav

int arraySize;

float valuePanLidar;
//float scanOffsetArray[numLines] = {20.0,30.0,10.0};   // 0 = goal, 1 = nav, 2 = check for block capture
//float panStepSizeArray[numLines] = {1.0,30.00,10.0};  // 0 = goal, 1 = nav, 2 = check for block capture  (array size = 3)
float scanOffsetArray[numLines] = {20.0,40.0,10.0};   // 0 = goal, 1 = nav, 2 = check for block capture
float panStepSizeArray[numLines] = {1.0,40.00,10.0};  // 0 = goal, 1 = nav, 2 = check for block capture  (array size = 3)

//float panStepSizeArray[numLines] = {1.0,30.00,2.20};  // 0 = goal, 1 = nav, 2 = check for block capture  (array size = 5)
//float scanOffsetArray[numLines] = {20.0,16.7,19.6,23.2,27.4,31.6,36.9,43.0,49.0,56.0,60.0,48.0};  // 180827
//float panStepSizeArray[numLines] = {1.0,0.81,0.96,1.13,1.34,1.54,1.80,2.10,2.39,2.73,2.93,2.20};  // 180827

int pan_offset = 5.0;
//int pan_offset = -12.0;

float valueTiltLidar;  // 90 (level) to 140? (down)
float valueTiltLidarArray[numLines] = {90.00,120.00,160.00};  // 0 = goal, 1 = nav, 2 = check for block capture
//float valueTiltLidarArray[numLines] = {90.00,130.00,160.00};  // 0 = goal, 1 = nav, 2 = check for block capture
//float valueTiltLidarArray[numLines] = {90.00,96.10,97.28,98.71,100.46,102.64,105.41,109.07,114.19,122.16,138.18,160.00};  // for scanTable 180809

float valueTiltLidarCenter = 130.00;  // nav, to check for table end
//float valueTiltLidarCenter = 138.18;  // nav, to check for table end
int tilt_offset = 0.0;  // -10??
int tiltLineNum;

// rangeFloor = h_lidar_floor (866) / sin(rad(90 - valueTiltLidar));
int tiltFloorDist = 1627;  // 0 = goal, 1 = nav, 2 = check for block capture
//int tiltFloorDist[numLines] = {0,1627,922};  // 0 = goal, 1 = nav, 2 = check for block capture
//int tiltFloorDist[numLines] = {0,8150,6834,5719,4770,3958,3259,2651,2113,1627,1162,922};

// rangeTable = h_lidar (160) / sin(rad(90 - valueTiltLidar));
int tiltTableDist = 375;  // 0 = goal, 1 = nav, 2 = check for block capture
//int tiltTableDist[numLines] = {0,375,133};  // 0 = goal, 1 = nav, 2 = check for block capture
//int tiltTableDist[numLines] = {0,1670,1630,1625,1620,1615,1604,930,620,375,210,133};

int tiltFloorDetect = 600;  // tiltTableDist + (tiltFloorDist - tiltTableDist)/2
//int tiltFloorDetect = 850;  // tiltTableDist + (tiltFloorDist - tiltTableDist)/2
//int tiltFloorDetect = 1000;  // tiltTableDist + (tiltFloorDist - tiltTableDist)/2

int tiltDelay = 200;
int stepDelay = 40;   // 10 ms ?  (array size = 3)
//int stepDelay = 20;   // 10 ms ?  (array size = 5)
int scanDirection = 0;  // 0 = sweep left to right

// to store mesured range readings in units of mm
int rangeLidarArray[arraySize0];  // make large enough to hold goal range scans

// to look for anomalies in range readings to detect goal:
int rangeGroupGoal[arraySize0];

//RH needed?
// to look for anomalies in range readings to detect block:
int rangeGroupBlock[arraySize1];

int maxRange = 2000;
#define goalDistance 1875  // 8.5 feet

//RH needed??
//int rangeAvg = 0;
//int rangeAvgCenter = 0;

// ------------------------------
// PWM pins MEGA/DUE: 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13
// SparkFun Ardumoto Shield
#define DIR_A_PIN   8  // L298 Input pins for direction // direction A, left side motor
#define DIR_B_PIN   7  // L298 Input pins for direction // direction B, right side motor
#define PWM_A_PIN   9  // L298 Enable pins. Use analogWrite to control speed value // enable & speed A
#define PWM_B_PIN  10  // L298 Enable pins. Use analogWrite to control speed value // enable & speed B

int speedLeft;            // PWM value for speed, range 0 to 255
int speedRight;           // PWM value for speed, range 0 to 255

// Bile (devastator chassis)
//int speedLeftDefault  = 0;  // enc A
//int speedRightDefault = 0;  // enc B

// tablebot speeds:
int speedLeftDefault  = 127;  // enc A
int speedRightDefault = 129;  // enc B
//int speedLeftDefault  = 191;  // enc A
//int speedRightDefault = 188;  // enc B

// floorbot speeds:
//int speedLeftDefault  = 255;  // PWM value for speed, range 0 to 255
//int speedRightDefault = 251;  // PWM value for speed, range 0 to 255

/* ------------------------------
Motors:
Pololu 2284 34:1 metal gear motors
Pololu 2285 47:1 metal gear motors // not used on Bile
6 V with 48 CPR encoder
By counting both the rising and falling edges of both the A and B outputs results in 48 counts per revolution of the motor shaft. 
To compute the counts per revolution of the gearbox output, multiply the gear ratio by 48.
Using just a single edge of one channel results in 12 counts per revolution of the motor shaft
// connector:
Color  Function
Red    motor power (connects to one motor terminal)
Black  motor power (connects to the other motor terminal)
Green  encoder GND
Blue   encoder Vcc (3.5 – 20 V)
Yellow encoder A output
White  encoder B output
*/

//------------------------------ 
//  Digital Pins With Interrupts
//  Mega2560 2, 3, 18, 19, 20, 21
//  On the Arduino Due, doubles have 8-byte (64 bit) precision
//  On the Uno and other ATMEGA based boards, this occupies 4 bytes

#define encoderPinA 3
#define encoderPinB 2
volatile int encoderPosA = 0;
volatile int encoderPosB = 0;
double encoderCountA = 0.0;
double encoderCountB = 0.0;
int encoderCountAmB = 0;          // A minus B for proportional factor (P)
double encoderAccumCountA = 0.0;
double encoderAccumCountB = 0.0;
double encoderAccumCountAmB = 0;  // cumulative A minus B for integral factor (I)
double encoderAccumCountAmBLast = 0;  // placeholder for last value
//int encoderAccumCountAmBOffset = 0;  // amount to track for turns, path planning (A - B)
int ticksTurned = 0;    // neg for left, pos for right
int distanceToTarget = 0;

float DT;
float Kp = 0.3;   // 191 speed
//float Kp = 0.8;   // 191 speed
float Ki = 0.15;   // was 0.1
float Kd = 0.22;  // was 0.2

// calibrate distance vs. encoder ticks (no longer needed)
//#define encoderTicksPerRev 816  // 24 shaft ticks * 34 (only one encoder channel, using both edges)
// 10 feet is 
//20285, 20157, 20248 ticks on left enc
//20134, 20139, 20107 ticks on right enc

#define ticksUturn    1575  // on battery?
//#define ticksUturn    1725  // on USB cable?
//#define ticksUturn     2200 //RH needs to be calibrated
#define ticksMoveBack  400  // 4" ? //RH needs to be calibrated
#define ticksSideAvoid 100  // increase?

// ------------------------------
// misc variables

int strategy  = 0;  // 0 = error, 1 = level 1, 2 = level 2, 3 = level 3
int success   = 0;  // 1 = successful/done

int detectedGoal = 0;
int detectedBlock = 0;
int detectedEdgeLeft = 0;
int detectedEdgeRight = 0;
int detectedEnd = 0;

int tableSide = 0;  // side detected
int tableEnd  = 0;  // to count ends detected

// save angles to block if found
float blockAngle = 90.0;  // pan angle, use int?
int blockDistance = maxRange;   // save distance based on tilt angle & blockDistanceArray, use encoders to nav
int capturedBlock = 0;  // is block between hook 'em horns?

// debug:
#define debug 2  // use to turn on debug code if set to 1, also time readouts if set to 2


////////////////////////////////////////////////////////////////////////////////
void setup() {
////////////////////////////////////////////////////////////////////////////////

  if (debug >= 1) {
    Serial.begin(115200);  // no line ending in terminal
    Serial.println();
    Serial.println("Dark Lord Robotics:"); 
    Serial.println("One Bot to Rule Them All, and in the Darkness, Bind Them..."); 
    Serial.println("Tablebot_LIDAR_pixy");
    Serial.println(code_version);
    Serial.println();
    delay (2000);
  }
  if (debug >= 2) {
    Serial.print("Time setup start: ");
    Serial.println(millis());
    Serial.println();
  }

  Wire.begin();
  //Wire.setClock(400000); //Increase I2C bus speed to 400 kHz

  // SparkFun Ardumoto Shield
  pinMode(DIR_A_PIN,OUTPUT);
  pinMode(DIR_B_PIN,OUTPUT);
  pinMode(PWM_A_PIN,OUTPUT);
  pinMode(PWM_B_PIN,OUTPUT);

  moveStop();

  // initalize speed settings
  speedLeft  = speedLeftDefault;
  speedRight = speedRightDefault;

  // interrupts for encoders
  pinMode(encoderPinA, INPUT_PULLUP);
  pinMode(encoderPinB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoderPinA), doEncoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPinB), doEncoderB, CHANGE);
  // (RISING, FALLING and CHANGE all work with this library)

  // initialize/center Lidar servos
  servoPanLidar.attach(5);  
  servoTiltLidar.attach(6);  
  valuePanLidar  = 90;
  valueTiltLidar = 90;
  servoPanLidar.write(valuePanLidar + pan_offset);
  servoTiltLidar.write(valueTiltLidar + tilt_offset);
  delay(tiltDelay);

  tcaselect(2);  // I2C port 2

  // LED I2C board
  ledChip.Begin();
  ledChip.Enable();

  // LED I2C bulb test
  if (debug >= 1) {
    Serial.println("LED I2C bulb test");
    Serial.println();
  }
  LED_I2C_GREEN_ON();
  LED_I2C_BLUE_ON();
  LED_I2C_RED_ON();
  delay(1000);
  LED_I2C_GREEN_OFF();
  LED_I2C_BLUE_OFF();
  LED_I2C_RED_OFF();

/*
  tcaselect(0);  // I2C port 0

  // initialize Pololu VL53L1X chin laser
  if (distanceSensorL1X.begin() == false)
    Serial.println("Sensor offline!");
  distanceSensorL1X.startMeasurement();  //Write configuration bytes to initiate measurement
  while (distanceSensorL1X.newDataReady() == false)  //Poll for completion of measurement. Takes 40-50 ms.
  delay(5);
  int distance = distanceSensorL1X.getDistance(); //Get the result of the measurement from the sensor
  if (debug >= 1) {
    Serial.print("VL53L1X chin lidar distance(mm): ");
    Serial.println(distance);
    Serial.println();
  }
*/

  tcaselect(1);  // I2C port 1

  // initialize SparkFun VL53L1X
  if (distanceSensor.begin() == false)
    Serial.println("Sensor offline!");
  distanceSensor.writeRegister(0x80, 0x33);  // set 4x4 ROI
  distanceSensor.startMeasurement();  //Write configuration bytes to initiate measurement
  while (distanceSensor.newDataReady() == false)  //Poll for completion of measurement. Takes 40-50 ms.
  delay(5);
  int distance = distanceSensor.getDistance(); //Get the result of the measurement from the sensor
  if (debug >= 1) {
    Serial.print("VL53L1X distance(mm): ");
    Serial.println(distance);
    Serial.println();
  }

  // find goal
  // read pixycam? later....
  // get ready for lidar scan
  tiltLineNum = 0;  // 0 = goal, 1 = nav, 2 = check for block capture
  arraySize = arraySize0;  // 0 = goal, 1 = nav
  // set tilt servo angle
  valueTiltLidar = valueTiltLidarArray[tiltLineNum];
  servoTiltLidar.write(valueTiltLidar + tilt_offset);
  delay(tiltDelay);
  // set pan servo angle
  valuePanLidar = 90 - scanOffsetArray[tiltLineNum];
  servoPanLidar.write(valuePanLidar + pan_offset);
  delay(tiltDelay);

  scanLine();
  findGoalLidar();  // check scan results

  if (debug >= 1) {
    displayScanResultsSerial();
  }

/*
// remove for nav testing:
  // find block, read pixycam
  // find goal, read pixycam? later....
  pixy.init();  // initialize pixycam
  findBlock();
  findGoalPixy();
*/

  if (debug >= 1) {
    displayResultsSummary();
  }



  // get ready for first nav scan
  tiltLineNum = 1;  // 0 = goal, 1 = nav, 2 = check for block capture
  arraySize = arraySize1;  // 0 = goal, 1 = nav
  // set tilt servo angle
  valueTiltLidar = valueTiltLidarArray[tiltLineNum];
  servoTiltLidar.write(valueTiltLidar + tilt_offset);
  delay(tiltDelay);
  // set pan servo angle
  valuePanLidar = 90;
  //valuePanLidar = 90 - scanOffsetArray[tiltLineNum];
  servoPanLidar.write(valuePanLidar + pan_offset);
  delay(tiltDelay);


  // set strategy based on detectedGoal & detectedBlock
  if (detectedGoal == 1 && detectedBlock == 1) {
    strategy = 3;
    distanceToTarget = blockDistance;
    LED_I2C_GREEN_ON();
    LED_I2C_BLUE_ON();
    LED_I2C_RED_ON();
  }
  else if (detectedBlock == 1) {
    strategy = 2;
    distanceToTarget = blockDistance;
    LED_I2C_GREEN_OFF();
    LED_I2C_BLUE_ON();
    LED_I2C_RED_ON();
  }
  else {
    strategy = 1;
    LED_I2C_GREEN_OFF();
    LED_I2C_BLUE_OFF();
    LED_I2C_RED_ON();
  }

  if (debug >= 1) {
    Serial.print("strategy: ");
    Serial.println(strategy);
    Serial.println();
    Serial.println("*** Setup Done ***");
    Serial.println();
    delay(2000);
  }
}  // end setup
 


////////////////////////////////////////////////////////////////////////////////
void loop() {
////////////////////////////////////////////////////////////////////////////////

  if (debug >= 2) {
    Serial.print("*** Time loop start: ");
    Serial.println(millis());
    Serial.println();
  }

  while (success == 1) {
    moveStop();
    victoryDance();
  }

//RH nav test, table edge detection, etc.
  // what is path?
  // move, turn, scan, stop, avoid, move

  // move to block
  // make sure it's captured (Lidar & pixy)
  // turn to center & move
  // turn to goal/end and move (lidar & pixy)
  // detect end of table, turn around
  // move to other end of table


  tiltLineNum = 1;  // 0 = goal, 1 = nav, 2 = check for block capture
  arraySize = arraySize1;  // 0 = goal, 1 = nav

  scanLine();  // for a given tiltNum, use lookup table with Lidar pan values to scan table

  checkLine();  // table edge/end detection based on scanLine();
  displayScanResultsSerial();  // if debug >= 1

  //tableSideAvoid();  // turn to avoid edges if detected (end in pathPlan)

  pathPlanUpdate();  // code to control vehicle motion

  readEncoders();  // read encoder counts & update position
 
  pidUpdate();  // calculate throttle changes based on encoder readings


}  // end void loop



////////////////////////////////////////////////////////////////////////////////
// functions
////////////////////////////////////////////////////////////////////////////////



// -------------------------------------
void scanLine() {
  // for a given tiltNum, use lookup table with values for pan to scan table

  if (debug >= 1) {
    Serial.println("*** scanLine();");
    Serial.print("tiltLineNum: ");
    Serial.print(tiltLineNum);
    //Serial.print("\tvalueTiltLidar: ");
    //Serial.print(valueTiltLidar);
    Serial.print("\tarraySize: ");
    Serial.println(arraySize);
    //Serial.println();
  }
  // set tilt servo angle
  valueTiltLidar = valueTiltLidarArray[tiltLineNum];
  servoTiltLidar.write(valueTiltLidar + tilt_offset);
  //delay(tiltDelay);

  // pan & read each point across the line
  int j;
  for (int i = 0; i < arraySize; i++) {
    if (scanDirection == 0) {
      valuePanLidar  = (i * panStepSizeArray[tiltLineNum]) + (90 - scanOffsetArray[tiltLineNum]);
      j = i;
    }
    else {
      valuePanLidar  = ((arraySize - 1) - i) * panStepSizeArray[tiltLineNum] + (90 - scanOffsetArray[tiltLineNum]);
      j = (arraySize - 1) - i;
    }
    servoPanLidar.write(valuePanLidar + pan_offset);  // set pan servo angle
    //delay(stepDelay);  // or else below if false end detections


    if (tiltLineNum == 0) {
      delay(stepDelay);
    }
    else if (tiltLineNum > 0) {
      if (i == arraySize/2) {
        valueTiltLidar = valueTiltLidarCenter;
      }
      else {
        valueTiltLidar = valueTiltLidarArray[tiltLineNum];
      }
      servoTiltLidar.write(valueTiltLidar + tilt_offset);
      delay(stepDelay);
    }


    //read the LIDAR sensor VL53L1X
    distanceSensor.startMeasurement();  // Write configuration bytes to initiate measurement
    while (distanceSensor.newDataReady() == false)  // Poll for completion of measurement, takes 40-50 ms
    delay(5);
    // use j instead to keep left/right alignment on readings
    rangeLidarArray[j] = distanceSensor.getDistance(); // Get the result of the measurement from the sensor

    if (debug >= 1) {
      Serial.print("Pan: ");
      Serial.print(valuePanLidar);
      Serial.print("\tvalueTiltLidar: ");
      Serial.print(valueTiltLidar);
      Serial.print("\trangeLidar: ");
      Serial.println(rangeLidarArray[j]);
    }
  }
  if (scanDirection == 0) {
    scanDirection = 1;
  }
  else if (scanDirection == 1) {
    scanDirection = 0;
  }
  if (debug == 2) {
    Serial.print("Time scanLine: ");
    Serial.println(millis());
  }
  if (debug >= 1) {
    Serial.println();
  }
}  // scanLine


// -------------------------------------
void checkLine() {
  // check table for sides, end

  if (debug >= 1) {
    Serial.println("*** checkLine();");
  }
  int edgeDetected = 0;
  for (int m = 0; m < arraySize; m++) {
    if (rangeLidarArray[m] >= tiltFloorDetect) {
      edgeDetected = 1;
    }
  }
  if (edgeDetected == 1) {
    moveStop();
    //scanLine();
    scanCenter();
  }

  detectedEdgeLeft = 0;
  detectedEdgeRight = 0;
  detectedEnd = 0;

  for (int m = 0; m < arraySize; m++) {
    rangeGroupBlock[m] = 0;
    if (rangeLidarArray[m] >= tiltFloorDetect) {
      if (m == arraySize/2) {
        detectedEnd = 1;
      }
      if (m < arraySize/2) {
        detectedEdgeLeft = 1;
      }
      if (m > arraySize/2) {
        detectedEdgeRight = 1;
      }
      rangeGroupBlock[m] = -1;  // off table
      moveStop();
    }
  }
  if (debug >= 1) {
    //Serial.println("*** checkLine();");
    Serial.print("detectedEnd: ");
    Serial.println(detectedEnd);
    Serial.print("detectedEdgeLeft: ");
    Serial.println(detectedEdgeLeft);
    Serial.print("detectedEdgeRight: ");
    Serial.println(detectedEdgeRight);
  }
  if (debug == 2) {
    Serial.print("Time checkLine: ");
    Serial.println(millis());
  }
  if (debug >= 1) {
    Serial.println();
  }
}  // end checkLine();


//-----------------------------------------------
void displayScanResultsSerial() {

  if (debug >= 1) {
    Serial.println("*** displayScanResultsSerial();");
    //Serial.print("tiltLineNum: ");
    //Serial.println(tiltLineNum);
    //Serial.print("arraySize: ");
    //Serial.println(arraySize);

    if (tiltLineNum == 0) {
      for (int i = 0; i < arraySize; i++) {
        if (rangeGroupGoal[i] == 0) {
          Serial.print(".");
        }
        else if (rangeGroupGoal[i] == 1) {
          Serial.print("G");
        }
      }
    }
    else if (tiltLineNum > 0) {
      for (int i = 0; i < arraySize; i++) {
        if (rangeGroupBlock[i] == -1) {
          Serial.print(".");
        }
        else if (rangeGroupBlock[i] == 0) {
          Serial.print("T");
        }
      }
    }
    Serial.println();
    //Serial.println();
  }
  if (debug == 2) {
    Serial.print("Time displayScanResultsSerial: ");
    Serial.println(millis());
  }
  if (debug >= 1) {
    Serial.println();
  }
}

// -------------------------------------
void pathPlanUpdate() {
  // set path plan based on strategy & current position
  // blockDistance = rangeLidarArray[m]; ??? //RH calibrate to encoders?

  if (strategy == 1) {   // level 1, move forward, turn around at end, come back
    if ((detectedEnd == 1) || (detectedEdgeLeft == 1 && detectedEdgeRight == 1)) {
      detectedEnd == 1;
      detectedEdgeLeft = 0;
      detectedEdgeRight = 0;
      moveStop();
    //}
    //if (detectedEnd == 1) {
      if (tableEnd == 1) {
        tableEnd = 2;
        success = 1;
        victoryDance();        
      }
      else {
        tableEnd = 1;
        detectedEnd = 0;
        // backup?
        moveBackward();
        if (debug >= 1) {
          Serial.println("moveBackward complete!");
        }
        // U-turn:
        moveEncoders(HIGH,LOW,ticksUturn);  // RH calibrate!
        if (debug >= 1) {
          Serial.println("U-turn complete!");
        }
        // equalize encoder counts? drive ticksTurned to zero?
        equalizeEncoders();
        clearEncoders();   // ?? does this work?
        moveForward();
      }
    }
    else if ((detectedEdgeLeft == 1 && detectedEdgeRight == 0) || (detectedEdgeLeft == 0 && detectedEdgeRight == 1)) {
      tableSideAvoid();
    }
    else {
      moveForward();
    }
  }

  else if (strategy >= 2) {
    distanceToTarget = distanceToTarget - encoderAccumCountA;
    if (distanceToTarget <= 0) {
      moveStop();
      if (capturedBlock == 1) {
        if (ticksTurned < 0) {
          moveEncoders(LOW,HIGH,ticksTurned);  // turn right
        }
        else if (ticksTurned > 0) {
          moveEncoders(HIGH,LOW,ticksTurned);  // turn left
        }
        strategy = 1;
        moveForward();
      }
      else {
        checkLine2();  // check for block capture
        if (capturedBlock == 1) {
          // 2023 ticks is one foot
          if (ticksTurned < 0) {
            moveEncoders(HIGH,LOW,ticksTurned*2);  // turn right
          }
          else if (ticksTurned > 0) {
            moveEncoders(LOW,HIGH,ticksTurned*2);  // turn left
          }
          distanceToTarget = blockDistance;
        }
        else {
//RH check pixy cam?
          distanceToTarget = 200;
          moveForward();
        }
       }
      }
    else {
      moveForward();        
    }
  }

  else if (strategy == 3) {
    distanceToTarget = distanceToTarget - encoderAccumCountA;
    if (distanceToTarget <= 0) {
      moveStop();
      if (capturedBlock == 1) {
        if (ticksTurned < 0) {
          moveEncoders(LOW,HIGH,ticksTurned);  // turn right
        }
        else if (ticksTurned > 0) {
          moveEncoders(HIGH,LOW,ticksTurned);  // turn left
        }
        tiltLineNum = 10;  // 11 hits treads?
        valueTiltLidar = 90;
        servoTiltLidar.write(valueTiltLidar + tilt_offset);
        delay(100);
        moveForward();
      }
      else {
        checkLine2();  // check for block capture
        if (capturedBlock == 1) {
          // 2023 ticks is one foot
          if (ticksTurned < 0) {
            moveEncoders(HIGH,LOW,ticksTurned*2);  // turn right
          }
          else if (ticksTurned > 0) {
            moveEncoders(LOW,HIGH,ticksTurned*2);  // turn left
          }
          distanceToTarget = blockDistance;
        }
        else {
//RH check pixy cam?
          distanceToTarget = 200;
          moveForward();
        }
       }
      }
    else {
      moveForward();        
    }
  }
  if (debug >= 1) {
    Serial.print("*** pathPlanUpdate();");
    Serial.println();
    Serial.print("strategy: ");
    Serial.println(strategy);
    Serial.print("tableEnd: ");
    Serial.println(tableEnd);
    Serial.print("capturedBlock: ");
    Serial.println(capturedBlock);
    Serial.print("distanceToTarget: ");
    Serial.println(distanceToTarget);
    //Serial.println();
  }
  if (debug == 2) {
    Serial.print("Time pathPlanUpdate: ");
    Serial.println(millis());
  }
  if (debug >= 1) {
    Serial.println();
  }
}  // end pathPlanUpdate();


// -------------------------------------
void tableSideAvoid() {

  if (debug >= 1) {
    Serial.println("*** tableSideAvoid();");
    Serial.print("detectedEdgeLeft: ");
    Serial.println(detectedEdgeLeft);
    Serial.print("detectedEdgeRight: ");
    Serial.println(detectedEdgeRight);
    //Serial.println();
  }
  if (detectedEdgeLeft == 1) {  // left edge
    moveStop();
    turnLeftEncoder(LOW, ticksSideAvoid);
    moveStop();
    ticksTurned = ticksTurned + ticksSideAvoid;
    moveForward();
  }
  else if (detectedEdgeRight == 1) {  // right edge
    moveStop();
    turnRightEncoder(LOW, ticksSideAvoid);
    moveStop();
    ticksTurned = ticksTurned - ticksSideAvoid;
    moveForward();
  }
  detectedEdgeLeft = 0;
  detectedEdgeRight = 0;

  if (debug >= 1) {
    Serial.println("tableSideAvoid complete");
    Serial.print("ticksTurned: ");
    Serial.println(ticksTurned);
    //Serial.print("detectedEdgeLeft: ");
    //Serial.println(detectedEdgeLeft);
    //Serial.print("detectedEdgeRight: ");
    //Serial.println(detectedEdgeRight);
  }
  if (debug == 2) {
    Serial.print("Time tableSideAvoid: ");
    Serial.println(millis());
  }
  if (debug >= 1) {
    Serial.println();
  }
}

// -------------------------------------
void scanCenter() {

  if (debug >= 1) {
    Serial.println("*** scanCenter();");
    Serial.print("tiltLineNum: ");
    Serial.print(tiltLineNum);
    //Serial.print("\tvalueTiltLidar: ");
    //Serial.print(valueTiltLidar);
    Serial.print("\tarraySize: ");
    Serial.println(arraySize);
    //Serial.println();
  }

  // set tilt servo angle
  valueTiltLidar = valueTiltLidarCenter;
  servoTiltLidar.write(valueTiltLidar + tilt_offset);
  // set pan servo angle
  valuePanLidar  = ((arraySize/2) * panStepSizeArray[tiltLineNum]) + (90 - scanOffsetArray[tiltLineNum]);
  servoPanLidar.write(valuePanLidar + pan_offset);
  delay(stepDelay);

  //read the LIDAR sensor VL53L1X
  distanceSensor.startMeasurement();  // Write configuration bytes to initiate measurement
  while (distanceSensor.newDataReady() == false)  // Poll for completion of measurement, takes 40-50 ms
    delay(5);
  rangeLidarArray[(arraySize/2)] = distanceSensor.getDistance(); // Get the result of the measurement from the sensor

  if (debug >= 1) {
    Serial.print("valuePanLidar: ");
    Serial.print(valuePanLidar);
    Serial.print("\tvalueTiltLidar: ");
    Serial.print(valueTiltLidar);
    Serial.print("\trangeLidar: ");
    Serial.println(rangeLidarArray[(arraySize/2)]);
    Serial.println();
  }
}


// -------------------------------------
void checkLine2() {
  // check area just in front of bot for block

  if (debug >= 1) {
    Serial.println("*** checkLine2();");
    Serial.println("checking line 11 for block...");
  }
  capturedBlock = 0;

  // set new tilt servo angle
  int tiltLineNumOld = tiltLineNum;
  tiltLineNum = 2;
  valueTiltLidar = valueTiltLidarArray[tiltLineNum];
  servoTiltLidar.write(valueTiltLidar + tilt_offset);
  delay(tiltDelay);

  scanLine();
  // check line scan results
  for (int m = 0; m < arraySize; m++) {
    if (rangeLidarArray[m] <= 20) {
      rangeGroupBlock[m] = 1;
    }
    blockAngle = 90;
    blockDistance = 0;
    capturedBlock = 1;
    LED_I2C_BLUE_ON();   // block detected
  }

  // restore old tilt servo angle
  tiltLineNum = tiltLineNumOld;
  valueTiltLidar = valueTiltLidarArray[tiltLineNum];
  servoTiltLidar.write(valueTiltLidar + tilt_offset);
  delay(tiltDelay);

  if (debug >= 1) {
    Serial.print("capturedBlock: ");
    Serial.println(capturedBlock);
    Serial.println();
  }
}

// -------------------------------------
void findGoalLidar() {
  // initial scan for goal?
  // assumes tiltlinenum = 0, scanline has been run

  if (debug >= 1) {
    Serial.println("*** findGoalLidar();");
    Serial.print("tiltLineNum: ");
    Serial.println(tiltLineNum);
    Serial.print("arraySize: ");
    Serial.println(arraySize);
    Serial.println();
  }
  int j = 0;
  // check for goal (simple)
  if (tiltLineNum == 0) {
    for (int i = 0; i < arraySize; i++) {
      rangeGroupGoal[i] = 0;
      if (rangeLidarArray[i] <= goalDistance) {
        j = j + 1;
        rangeGroupGoal[i] = 1;
      }
      if (j >= 5) {
        detectedGoal = 1;
        LED_I2C_GREEN_ON();
      }
    }
    if (debug >= 1) {
      Serial.print("detectedGoal: ");
      Serial.println(detectedGoal);
      for (int i = 0; i < arraySize; i++) {
        //Serial.println();
        Serial.print("Pan: ");
        Serial.print(i);
        Serial.print("\trange[i]: ");
        Serial.print(rangeLidarArray[i]);
        Serial.print("\trngGrpGoal[i]: ");
        Serial.println(rangeGroupGoal[i]);
      }
      Serial.println();
    }
  }
  if (debug == 2) {
    Serial.print("Time findGoalLidar: ");
    Serial.println(millis());
    Serial.println();
  }
}  // end findGoalLidar();


//-----------------------------------------------
void displayResultsSummary() {

  if (debug >= 1) {
    Serial.println("*** displayResultsSummary();");
    Serial.print("detectedGoal: ");
    Serial.println(detectedGoal);
    Serial.print("detBlock: ");
    Serial.println(detectedBlock);
    Serial.print("blockAngle: ");
    Serial.println(blockAngle);
    Serial.print("blockDistance: ");
    Serial.println(blockDistance);
    Serial.println();
  }
}

// -------------------------------------
void pidUpdate() {
  // calculate throttle changes based on encoder readings

  DT = (Kp*encoderCountAmB + Ki*(encoderAccumCountAmB - ticksTurned));
  //DT = (Kp*encoderCountAmB + Ki*(encoderAccumCountAmB - encoderAccumCountAmBOffset));
  //DT = (Kp*encoderCountAmB + Ki*encoderAccumCountAmB + Kd*(encoderCountAmB - encoderCountAmBlast));
  speedRight = speedRight + DT;

  if (speedRight >= 255) {
    speedRight = 255;
  }
  else if (speedRight <= 0 || speedLeft == 0) {
    speedRight = 0;
  }
  analogWrite(PWM_B_PIN, speedRight);
  
  if (debug >= 1) {
    Serial.println("*** pidUpdate();");
    Serial.print("ticksTurned: ");
    Serial.println(ticksTurned);
    Serial.print("speedLeft (A): ");
    Serial.print(speedLeft);
    Serial.print("   speedRight (B): ");
    Serial.println(speedRight);
    Serial.print("DT: ");
    Serial.println(DT);
    Serial.println();
  }
}


//-----------------------------------------------
void equalizeEncoders() {
  // ticksTurned

  if (debug >= 1) {
    Serial.println("*** equalizeEncoders();");
    Serial.print("ticksTurned: ");
    Serial.println(ticksTurned);
    Serial.print("encoderAccumCountAmBLast: ");
    Serial.println(encoderAccumCountAmBLast);
  }
  //encoderAccumCountAmBLast = encoderAccumCountAmB;
  clearEncoders();
  digitalWrite(DIR_A_PIN, HIGH);
  digitalWrite(DIR_B_PIN, HIGH);

  if (ticksTurned > 0) {
    analogWrite(PWM_A_PIN, speedLeftDefault - 20);
    analogWrite(PWM_B_PIN, speedRightDefault);
    while (ticksTurned > 0) {
      readEncoders();
      ticksTurned = ticksTurned + encoderCountAmB;
      //ticksTurned = ticksTurned + (encoderAccumCountAmB - encoderAccumCountAmBLast);
      if (debug >= 1) {
        Serial.print("ticksTurned: ");
        Serial.println(ticksTurned);
      }
      if (encoderCountB > 400) {
        break;
      }
    }
    moveStop();
  }
  else if (ticksTurned < 0) {
    analogWrite(PWM_A_PIN, speedLeftDefault);
    analogWrite(PWM_B_PIN, speedRightDefault - 20);
    while (ticksTurned < 0) {
      readEncoders();
      ticksTurned = ticksTurned + encoderCountAmB;
      //ticksTurned = ticksTurned + (encoderAccumCountAmB - encoderAccumCountAmBLast);
      if (debug >= 1) {
        Serial.print("ticksTurned: ");
        Serial.println(ticksTurned);
      }
      if (encoderCountA > 400) {
        break;
      }
    }
    moveStop();
  }
  else {
    moveStop();
  }

  if (debug >= 1) {
    Serial.println("equalizeEncoders complete!");
    Serial.print("ticksTurned: ");
    Serial.println(ticksTurned);
    Serial.println();
  }
}

//-----------------------------------------------
void moveEncoders(int DIR_A, int DIR_B, int numticks) {

  int a_done = 0;
  int b_done = 0;
  encoderAccumCountAmBLast = encoderAccumCountAmB;
  clearEncoders();
  digitalWrite(DIR_A_PIN, DIR_A);
  digitalWrite(DIR_B_PIN, DIR_B);
  analogWrite(PWM_A_PIN, speedLeftDefault);
  analogWrite(PWM_B_PIN, speedRightDefault);
  while (a_done == 0 && b_done == 0) {
    readEncoders();
    if (encoderAccumCountA >= numticks) {
      a_done = 1;
    }
    if (encoderAccumCountB >= numticks) {
      b_done = 1;
    }
  }
  moveStop();
  ticksTurned = ticksTurned + (encoderAccumCountAmB - encoderAccumCountAmBLast);
}

//-----------------------------------------------
void turnLeftEncoder(int DIR_B, int numticks) {

  if (debug >= 1) {
    Serial.println("*** turnLeftEncoder();");
  }
  int a_done = 0;
  encoderAccumCountAmBLast = encoderAccumCountAmB;
  clearEncoders();
  digitalWrite(DIR_A_PIN, HIGH);
  digitalWrite(DIR_B_PIN, DIR_B);
  analogWrite(PWM_A_PIN, 0);
  analogWrite(PWM_B_PIN, speedRightDefault);
  while (a_done == 0) {
    readEncoders();
    if (encoderAccumCountB >= numticks) {
      a_done = 1;
    }
  }
  moveStop();
  ticksTurned = ticksTurned + (encoderAccumCountAmB - encoderAccumCountAmBLast);
}

//-----------------------------------------------
void turnRightEncoder(int DIR_A, int numticks) {

  if (debug >= 1) {
    Serial.println("*** turnRightEncoder();");
  }
  int a_done = 0;
  encoderAccumCountAmBLast = encoderAccumCountAmB;
  clearEncoders();
  digitalWrite(DIR_A_PIN, DIR_A);
  digitalWrite(DIR_B_PIN, HIGH);
  analogWrite(PWM_A_PIN, speedLeftDefault);
  analogWrite(PWM_B_PIN, 0);
  while (a_done == 0) {
    readEncoders();
    if (encoderAccumCountA >= numticks) {
      a_done = 1;
    }
  }
  moveStop();
  ticksTurned = ticksTurned + (encoderAccumCountAmB - encoderAccumCountAmBLast);
}

// -------------------------------------
// encoder INT service routines

void doEncoderA() {
  encoderPosA++;
}

void doEncoderB() {
  encoderPosB++;
}

// -------------------------------------
void readEncoders() {
  // read encoder counts & update position

  //RH add direction to inc/dec encoders as needed

  encoderCountA = encoderPosA;
  encoderCountB = encoderPosB;
  if (debug >= 1) {
    Serial.println("*** readEncoders();");
    Serial.print("encoderCountA: ");
    Serial.println(encoderCountA);
    Serial.print("encoderCountB: ");
    Serial.println(encoderCountB);
  }
  encoderCountB = encoderCountB * 994/1000;  // calibration +0.6%, 0.4%
  if (debug >= 1) {
    Serial.print("encoderCountB cal: ");
    Serial.println(encoderCountB);
  }
  encoderPosA = 0;
  encoderPosB = 0;
  encoderCountAmB = encoderCountA - encoderCountB;  // A minus B for course corrections, turns, etc.
  encoderAccumCountA = encoderAccumCountA + encoderCountA;
  encoderAccumCountB = encoderAccumCountB + encoderCountB;
  encoderAccumCountAmB = encoderAccumCountA - encoderAccumCountB;  // A minus B for course corrections, turns, etc.

  if (debug >= 1) {
    Serial.print("encoderCountA: ");
    Serial.println(encoderCountA);
    Serial.print("encoderCountB: ");
    Serial.println(encoderCountB);
    Serial.print("encoderCountAmB: ");
    Serial.println(encoderCountAmB);
    Serial.print("encoderAccumCountA: ");
    Serial.println(encoderAccumCountA);
    Serial.print("encoderAccumCountB: ");
    Serial.println(encoderAccumCountB);
    Serial.print("encoderAccumCountAmB: ");
    Serial.println(encoderAccumCountAmB);
    Serial.println();
  }
  if (debug == 2) {
    Serial.print("Time readEncoders: ");
    Serial.println(millis());
    Serial.println();
  }
}

// -------------------------------------
void clearEncoders() {
  // clear all encoder counts

  if (debug >= 1) {
    Serial.println("*** clearEncoders();");
    Serial.println();
  }
  encoderCountA = 0;
  encoderCountB = 0;
  encoderCountAmB = 0;
  encoderPosA = 0;
  encoderPosB = 0;
  encoderAccumCountA = 0;
  encoderAccumCountB = 0;
  encoderAccumCountAmB = 0;
}

// -------------------------------------
void tcaselect(uint8_t i) {
  //Adafruit TCA9548A 1-to-8 I2C Multiplexer Breakout
  if (i > 7) return;
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();  
}

//-----------------------------------------------
// SparkFun LED Driver Breakout - LP55231

void LED_I2C_RED_ON() {
  tcaselect(2);
  ledChip.SetChannelPWM(6, 127);  // red LED 0
  tcaselect(1);
}
void LED_I2C_BLUE_ON() {
  tcaselect(2);
  ledChip.SetChannelPWM(3, 127);  // blue LED 1
  tcaselect(1);
}
void LED_I2C_GREEN_ON() {
  tcaselect(2);
  ledChip.SetChannelPWM(4, 63);   // green LED 2
  tcaselect(1);
}
void LED_I2C_RED_OFF() {
  tcaselect(2);
  ledChip.SetChannelPWM(6, 0);    // red LED 0
  tcaselect(1);
}
void LED_I2C_BLUE_OFF() {
  tcaselect(2);
  ledChip.SetChannelPWM(3, 0);    // blue LED 1
  tcaselect(1);
}
void LED_I2C_GREEN_OFF() {
  tcaselect(2);
  ledChip.SetChannelPWM(4, 0);    // green LED 2
  tcaselect(1);
}


/*
//-----------------------------------------------
// SparkFun LED Driver Breakout - LP55231 (old)

void LED_I2C_RED_ON() {
  ledChip.SetChannelPWM(6, 127);  // red LED 0
}
void LED_I2C_BLUE_ON() {
  ledChip.SetChannelPWM(3, 127);  // blue LED 1
}
void LED_I2C_GREEN_ON() {
  ledChip.SetChannelPWM(4, 63);   // green LED 2
}
void LED_I2C_RED_OFF() {
  ledChip.SetChannelPWM(6, 0);    // red LED 0
}
void LED_I2C_BLUE_OFF() {
  ledChip.SetChannelPWM(3, 0);    // blue LED 1
}
void LED_I2C_GREEN_OFF() {
  ledChip.SetChannelPWM(4, 0);    // green LED 2
}
*/

//-----------------------------------------------
// move code for Arduino Motor Shield R3

void moveStop() {
  digitalWrite(DIR_A_PIN,LOW);
  digitalWrite(DIR_B_PIN,LOW);
  analogWrite(PWM_A_PIN,0);
  analogWrite(PWM_B_PIN,0);
}

void moveForward() {
  digitalWrite(DIR_A_PIN, HIGH);
  digitalWrite(DIR_B_PIN, HIGH);
  analogWrite(PWM_A_PIN, speedLeft);
  analogWrite(PWM_B_PIN, speedRight);
}

void moveBackward() {
  speedLeft  = speedLeftDefault;
  speedRight = speedRightDefault;
  moveEncoders(LOW,LOW,ticksMoveBack);  // RH calibrate!
  moveStop();
}

//-----------------------------------------------
void victoryDance() {

  if (debug >= 1) {
    Serial.println();
    Serial.println();
    Serial.print("*** victoryDance!! ***");
    Serial.println("*** victoryDance!! ***");
    Serial.print("*** victoryDance!! ***");
    Serial.println("*** victoryDance!! ***");
    Serial.print("*** victoryDance!! ***");
    Serial.println("*** victoryDance!! ***");
    Serial.println();
    Serial.println();
  }

  success = 1;
  moveStop();  // keep this!
  moveBackward();
  //moveEncoders(LOW,LOW,ticksMoveBack);  // RH calibrate!
  //moveStop();
  speedLeftDefault = 200;
  speedRightDefault = 200;

  for (int i = 0; i < 3; i++) {
    moveStop();
    //moveSpinLeft();
    moveEncoders(LOW,HIGH,500);  // RH calibrate!
    LED_I2C_RED_ON();
    LED_I2C_GREEN_ON();
    LED_I2C_BLUE_OFF();
    delay(100);
    moveStop();
    //moveSpinRight();
    moveEncoders(HIGH,LOW,500);  // RH calibrate!
    LED_I2C_RED_OFF();
    LED_I2C_GREEN_OFF();
    LED_I2C_BLUE_ON();
    delay(100);
  }
  moveStop();

  while (true) {
    LED_I2C_RED_ON();
    LED_I2C_GREEN_OFF();
    LED_I2C_BLUE_OFF();
    delay(200);
    LED_I2C_RED_OFF();
    LED_I2C_GREEN_ON();
    LED_I2C_BLUE_OFF();
    delay(200);
    LED_I2C_RED_OFF();
    LED_I2C_GREEN_OFF();
    LED_I2C_BLUE_ON();
    delay(200);
    if (debug >= 1) {
      Serial.println("*** victoryDance!! ***");
    }
  }

}  // victoryDance();



/*
// -------------------------------------
void pidUpdate() {
  // read encoders & calculate PID loop changes

  // RH add something to compensate for left side motor speed, a scaling factor:
  // DT = (Kp*IC + Ki*TC + Kd*(ICn - ICn-1)) * speedLeft/speedLeftDefault;
  // DT = Kp*IC + Ki*TC + Kd*(ICn - ICn-1)
  // IC interval count error:
  //int encoderCountAmB = 0;  // A minus B for proportional factor (P)
  //int encoderCountAmBlast = 0;  // previous A minus B for differential factor (D)
  // TC total count error:
  //double encoderAccumCountAmB = 0;  // cumulative A minus B for integral factor (I)
  
  timeNow = millis();
  timeElapsed = timeNow - timeLast;
  //float DTnew;

  // update once every encoderReadInterval:
  //if (timeElapsed >= 1) {
  if (timeElapsed >= encoderReadInterval) {

    if (debug >= 1) {
      Serial.println("pidUpdate...");
      Serial.print("timeNow: ");
      Serial.print(timeNow);
      Serial.print(" timeLast: ");
      Serial.print(timeLast);
      Serial.print(" timeElapsed: ");
      Serial.println(timeElapsed);
    }
    encoderCountAmBlast = encoderCountAmB;
    readEncoders();  // updates encoderCountAmB, encoderAccumCountAmB
    timeLast = timeNow;

    // DT = Kp*IC + Ki*TC + Kd*(ICn - ICn-1)   // ?
    //speedRight = speedRight + Kp*encoderCountAmB + Ki*encoderAccumCountAmB + Kd*(encoderCountAmB - encoderCountAmBlast);
    //DT = (Kp*encoderCountAmB + Kd*encoderAccumCountAmB + Ki*(encoderCountAmB - encoderCountAmBlast)) * (speedLeft/255) * float(100.0/timeElapsed);

    DT = (Kp*encoderCountAmB + Kd*encoderAccumCountAmB + Ki*(encoderCountAmB - encoderCountAmBlast));
    speedRight = speedRight + DT;

    //DTnew = DT * speedLeft/timeElapsed * 100/255;
    //speedRight = speedRight + DTnew;
    //speedRight = speedRight + DT * speedLeft/timeElapsed * 100/255;
    //speedRight = speedRight + DT * (speedLeft/255) * float(100.0/timeElapsed);

    if (speedRight >= 255) {
      speedRight = 255;
    }
    else if (speedRight <= 0 || speedLeft == 0) {
      speedRight = 0;
    }
    analogWrite(PWM_B_PIN, speedRight);
  }
  if (debug >= 1) {
    Serial.print("encoderCountAmBlast: ");
    Serial.println(encoderCountAmBlast);
    Serial.print("speedLeft (A): ");
    Serial.print(speedLeft);
    Serial.print(" speedRight (B): ");
    Serial.println(speedRight);
    Serial.print("DT: ");
    Serial.print(DT);
    //Serial.print(" DTnew: ");
    //Serial.println(DTnew);
  }
}  end pidUpdate();
*/








// -------------------------------------
void findBlock() {
  // use pixycam to look for block (color?)
  // also turn to face block, measure range? how?

  if (debug >= 1) {
    Serial.println("*** findBlock...");
    Serial.println();
  }
  uint16_t blocks;
  detectedBlock = 0;

  while (detectedBlock == 0) {
    blocks = pixy.getBlocks();
    if (blocks) {
      detectedBlock = 1;
      panError = X_CENTER-pixy.blocks[0].x;
      if (debug >= 1) {
        Serial.println("\n");
        Serial.println("block detected");
        Serial.print("panError: ");
        Serial.println(panError);
      }
    }
  }

  if (debug >= 1) {
    Serial.println();
    Serial.print("detBlock: ");
    Serial.print(detectedBlock);
    //Serial.print("\tticksTurned: ");
    //Serial.print(ticksTurned);
    //Serial.print("\tblockAngle: ");
    //Serial.print(blockAngle);
    //Serial.print("\tblockDistance: ");
    //Serial.println(blockDistance);
  }

}   // findBlock

// -------------------------------------
void readPixyCam() {

  uint16_t blocks;
  blocks = pixy.getBlocks();

  if (blocks) {
    Serial.println("*** ");
    Serial.println("block detected");  // debug only RH
    panError = X_CENTER-pixy.blocks[0].x;
  }
}

/*
// -------------------------------------
void pixysteering() { 

//RH valueSteering ??? servoSteering ???

  uint16_t blocks;
  //int32_t panError, tiltError;
  blocks = pixy.getBlocks();

  if (blocks) {
    Serial.println("\n");
    Serial.println("block detected");  // debug only RH
    panError = X_CENTER-pixy.blocks[0].x;

    // set steering angle based on object direction from pixycam pan angle
    if (panError < -30) {
      valueSteering = 1200;
    }
    else if (panError > 30) {
      valueSteering = 1800;
    }
    else {
      valueSteering = map(panError, -30, 30, 0, 600) + 1200;  // Max-10 steering servo
    }
    servoSteering.writeMicroseconds(valueSteering);

    moveForward();
    delay(100);

    if (debug >= 1) {
      Serial.print("panError: ");
      Serial.println(panError);
      Serial.print("steering: ");
      Serial.println(valueSteering);
    }
  }
  else {   // stop and look around
    moveStop();
    // turn left
    turnLeftEncoder(HIGH, 50);
    moveStop();

    if (debug >= 1) {
      Serial.print("Looking Left...");
    }
    blocks = pixy.getBlocks();

    if (blocks) {
      if (debug >= 1) {
        Serial.println("\n");
        Serial.println("block detected");  // debug only RH
      }
      moveForward();
      delay(100);
    }
    else {
      moveStop();
      // turn right??
      turnRightEncoder(HIGH, 50);
      moveStop();
  
      if (debug >= 1) {
        Serial.print("Looking right...");
      }
      blocks = pixy.getBlocks();

      if (blocks) {
        if (debug >= 1) {
          Serial.println("\n");
          Serial.println("block detected");  // debug only RH
        }
        moveForward();
        delay(100);
      }
    }
  }
}  // end pixysteering
*/
