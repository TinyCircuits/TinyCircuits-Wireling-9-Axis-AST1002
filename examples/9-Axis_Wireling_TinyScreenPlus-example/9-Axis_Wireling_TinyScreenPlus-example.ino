/************************************************************************
 * LSM9SD1 9-Axis Wireling Example:
 * This code has the ability to print out all available values from the 
 * 9 axis sensor, but some are commented out in this Sketch so as not to 
 * over-crowd the Serial Monitor. This program shows the basic methods 
 * on interfacing with this sensor to retrieve value readings. 
 *
 * Hardware by: TinyCircuits
 * Written by: Ben Rose, Lavérena Wienclaw, & Brandon Farmer for TinyCircuits
 *
 * Initiated: 11/20/2017 
 * Updated: 12/06/2019
 ************************************************************************/

#include <Wire.h>            // For I2C connection
#include <Wireling.h>        // For Wireling Interfacing
#include <TinyScreen.h>      // For the TinyScreen+

// For the communication with the LSM9DS1
#include "RTIMUSettings.h"    
#include "RTIMU.h"
#include "RTFusionRTQF.h"

// TinyScreen+ variables
TinyScreen display = TinyScreen(TinyScreenPlus);
int background = TS_16b_Black;

#ifndef ARDUINO_ARCH_SAMD
#include <EEPROM.h>
#endif

RTIMU *imu;                         // the IMU object
RTFusionRTQF fusion;                // the fusion object
RTIMUSettings settings;             // the settings object
int DISPLAY_INTERVAL = 300;         // interval between pose displays

// Global variables to retrieve, store, and schedule readings from the sensor
unsigned long lastDisplay;
unsigned long lastRate;
int sampleCount;
RTVector3 accelData;
RTVector3 gyroData;
RTVector3 compassData;
RTVector3 fusionData; 

// Make Serial Monitor compatible for all TinyCircuits processors
#if defined(ARDUINO_ARCH_AVR)
  #define SerialMonitorInterface Serial
#elif defined(ARDUINO_ARCH_SAMD)
  #define SerialMonitorInterface SerialUSB
#endif


void setup() {
  int errcode;
  SerialMonitorInterface.begin(115200);
  while (!SerialMonitorInterface); // Sensor will not begin printing data until Serial Monitor is opened
  Wire.begin(); // Begin I2C communication

  // Initialize Wireling
  Wireling.begin();
  Wireling.selectPort(0); 

  // TinyScreen appearance variables
  display.begin();
  display.setBrightness(15);
  display.setFlip(true);
  display.setCursor(0, 0);
  display.fontColor(TS_8b_White, TS_8b_Black);
  display.setFont(thinPixel7_10ptFontInfo);
  display.print("9-Axis Test");

  Wireling.selectPort(0); //9-Axis Sensor Port, may differ for you

  imu = RTIMU::createIMU(&settings);        // create the imu object

  SerialMonitorInterface.print("ArduinoIMU begin using device "); SerialMonitorInterface.println(imu->IMUName());
  if ((errcode = imu->IMUInit()) < 0) {
    SerialMonitorInterface.print("Failed to init IMU: "); SerialMonitorInterface.println(errcode);
  }

  // See line 69 of RTIMU.h for more info on compass calibaration 
  if (imu->getCalibrationValid())
    SerialMonitorInterface.println("Using compass calibration");
  else
    SerialMonitorInterface.println("No valid compass calibration data");

  lastDisplay = lastRate = millis();
  sampleCount = 0;

  // Slerp power controls the fusion and can be between 0 and 1
  // 0 means that only gyros are used, 1 means that only accels/compass are used
  // In-between gives the fusion mix.
  fusion.setSlerpPower(0.02);

  // use of sensors in the fusion algorithm can be controlled here
  // change any of these to false to disable that sensor
  fusion.setGyroEnable(true);
  fusion.setAccelEnable(true);
  fusion.setCompassEnable(true);
}

void loop() {
  unsigned long now = millis();
  unsigned long delta;

  if (imu->IMURead()) {     // get the latest data if ready 
    fusion.newIMUData(imu->getGyro(), imu->getAccel(), imu->getCompass(), imu->getTimestamp());
    sampleCount++;
    if ((delta = now - lastRate) >= 1000) {
      SerialMonitorInterface.print("Sample rate: "); SerialMonitorInterface.print(sampleCount);
      if (imu->IMUGyroBiasValid()) {
        SerialMonitorInterface.println(", gyro bias valid");
      }
      else {
        SerialMonitorInterface.println(", calculating gyro bias - don't move IMU!!");
      }

      sampleCount = 0;
      lastRate = now;
    }
    if ((now - lastDisplay) >= DISPLAY_INTERVAL) {
      lastDisplay = now;

      // Get updated readings from sensor and update those values in the 
      // respective RTVector3 object
      accelData = imu->getAccel();
      gyroData = imu->getGyro();
      compassData = imu->getCompass();
      fusionData = fusion.getFusionPose();

      // Acceleration data
      displayScreenAxis("Accel:", accelData.x(), accelData.y(), accelData.z()); 
      displayAxis("Accel:", accelData.x(), accelData.y(), accelData.z());         

// The following data is commented out for easy reading and you can uncomment it all by 
// highlighting it and using "'Ctrl' + '/'" for windows and "'COMMAND' + '/'" for Mac
//      // Gyro data
//      displayScreenAxis("Gyro:", gyroData.x(), gyroData.y(), gyroData.z()); 
//      displayAxis("Gyro:", gyroData.x(), gyroData.y(), gyroData.z());       
//
//      // Compass data
//      displayScreenAxis("Mag:", compassData.x(), compassData.y(), compassData.z());   
//      displayAxis("Mag:", compassData.x(), compassData.y(), compassData.z());    
//
//      // Fused output
//      displayScreenDegrees("Pose:", fusionData.x(), fusionData.y(), fusionData.z());
//      displayDegrees("Pose:", fusionData.x(), fusionData.y(), fusionData.z());      
      SerialMonitorInterface.println();   
    }
  }
}


// Prints out pieces of different radian axis data to Serial Monitor
void displayAxis(const char *label, float x, float y, float z) {
  SerialMonitorInterface.print(label);
  SerialMonitorInterface.print(" x:"); SerialMonitorInterface.print(x);
  SerialMonitorInterface.print(" y:"); SerialMonitorInterface.print(y);
  SerialMonitorInterface.print(" z:"); SerialMonitorInterface.print(z);
}

// Converts axis data from radians to degrees and prints values to Serial Monitor
void displayDegrees(const char *label, float x, float y, float z) {
  SerialMonitorInterface.print(label);
  SerialMonitorInterface.print(" x:"); SerialMonitorInterface.print(x * RTMATH_RAD_TO_DEGREE);
  SerialMonitorInterface.print(" y:"); SerialMonitorInterface.print(y * RTMATH_RAD_TO_DEGREE);
  SerialMonitorInterface.print(" z:"); SerialMonitorInterface.print(z * RTMATH_RAD_TO_DEGREE);
}

// Prints out pieces of different radian axis data to TinyScreen+
void displayScreenAxis(const char *label, float x, float y, float z) {
  // This will make the screen look a little unsteady but is needed in order
  // to clear old values 
  display.clearScreen();

  display.setCursor(0, 0);
  display.fontColor(TS_8b_White, TS_8b_Black);
  display.print("9-Axis Test");
  
  display.setCursor(0, 16);
  display.print(label); 

  display.fontColor(TS_8b_Red, background);
  display.setCursor(0, 28);
  display.print("X = ");
  display.print(x);

  display.fontColor(TS_8b_Green, background);
  display.setCursor(0, 40);
  display.print("Y = ");
  display.print(y);

  display.fontColor(TS_8b_Blue, background);
  display.setCursor(0, 52);
  display.print("Z = ");
  display.print(z);
}

// Converts axis data from radians to degrees and prints values to TinyScreen+
void displayScreenDegrees(const char *label, float x, float y, float z) {
   // This will make the screen look a little unsteady but is needed in order
  // to clear old values 
  display.clearScreen();

  display.setCursor(0, 0);
  display.fontColor(TS_8b_White, TS_8b_Black);
  display.print("9-Axis Test"); 
  
  display.setCursor(0, 16);
  display.print(label);

  display.fontColor(TS_8b_Red, background);
  display.setCursor(0, 28);
  display.print("X = ");
  display.print(x * RTMATH_RAD_TO_DEGREE);

  display.fontColor(TS_8b_Green, background);
  display.setCursor(0, 40);
  display.print("Y = ");
  display.print(y * RTMATH_RAD_TO_DEGREE);

  display.fontColor(TS_8b_Blue, background);
  display.setCursor(0, 52);
  display.print("Z = ");
  display.print(z * RTMATH_RAD_TO_DEGREE);
}
