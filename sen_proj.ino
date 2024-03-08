#include <stdint.h>
#include <Wire.h>
#include "LIDARLite_v4LED.h"
#include <SD.h>
#include <SPI.h>

//IMU libs
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28, &Wire);

LIDARLite_v4LED myLidarLite;

uint16_t distanceR = 0;
uint16_t distanceL = 0;

//Lane splitting variables
unsigned long timeEntered = 0;
bool preLaneSplitMode = false;
bool laneSplitMode = false;

//SD card variables
File dataFile;
unsigned long lastLogTime = 0;
const unsigned long logInterval = 1000;


#define FAST_I2C

void setup() {
  // put your setup code here, to run once:
  // Initialize Arduino serial port (for display of ASCII output to PC)
  Serial.begin(115200);
  if (!SD.begin(4)) { // Assuming the CS pin for the SD card module is connected to pin 4
    Serial.println("SD card initialization failed!");
  }
  dataFile = SD.open("data.txt", FILE_WRITE);
  if (!dataFile) {
    Serial.println("Error opening data file!");
  }
  digitalWrite(SCL, LOW);
  digitalWrite(SDA, LOW);

  // Initialize Arduino I2C (for communication to LidarLite)
  Wire.begin();
  #ifdef FAST_I2C
      #if ARDUINO >= 157
          Wire.setClock(400000UL); // Set I2C frequency to 400kHz (for Arduino Due)
      #else
          TWBR = ((F_CPU / 400000UL) - 16) / 2; // Set I2C frequency to 400kHz
      #endif
  #endif

  digitalWrite(SCL, LOW);
  digitalWrite(SDA, LOW);

  myLidarLite.setI2Caddr(0x55, 1, 0x55);

  // Initialize IMU //////////////////////////////////////////////////////////////////

  Serial.println("Orientation Sensor Raw Data Test");
  Serial.println("");

  /* Initialise the sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    //while(1);
  }
  delay(1000);

  /* Display the current temperature */
  int8_t temp = bno.getTemp();
  Serial.print("Current Temperature: ");
  Serial.print(temp);
  Serial.println(" C");
  Serial.println("");

  bno.setExtCrystalUse(true);

  Serial.println("Calibration status values: 0=uncalibrated, 3=fully calibrated");

  myLidarLite.configure(0, 0x55);
  myLidarLite.configure(0, 0x62);

}

void loop() {
  // put your main code here, to run repeatedly:
  
  // Lidar Sensor ///////////////////////////////////////////////////////////////////////
  distanceContinuousR(&distanceR);
  distanceContinuousL(&distanceL);
  
  Serial.print("distanceR: ");
  Serial.println(distanceR);

  Serial.print("distanceL: ");
  Serial.println(distanceL);

  // IMU Sensor ///////////////////////////////////////////////////////////////////////////

  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

  /* Display the floating point data */
 // Serial.print("X: ");
 //Serial.print(euler.x());
  Serial.print(" Y: ");
  Serial.print(euler.y());
 // Serial.print(" Z: ");
 //Serial.print(euler.z());
  Serial.print("\t\t");
  Serial.println();
  Serial.println();

  Serial.print("pre: ");
  Serial.println(preLaneSplitMode);
  Serial.print("lane split: ");
  Serial.println(laneSplitMode);
  //delay(500);

  unsigned long currentTime = millis();
  if (currentTime - lastLogTime >= logInterval) {
    lastLogTime = currentTime;
    logData(euler.y(), distanceR, distanceL);
  }
  
  // Check for pre-lane splitting condition
  if(!laneSplitMode) {
    if (distanceL < 100 && distanceR < 100) {
      if (!preLaneSplitMode) {
        timeEntered = millis(); // Record the time when the condition is met
        preLaneSplitMode = true;
      } else if (millis() - timeEntered >= 1000 && !laneSplitMode) { // 1 second elapsed
        laneSplitMode = true;
        preLaneSplitMode = false;
        timeEntered = 0;
      }
    } else {
      preLaneSplitMode = false;
      timeEntered = 0;
    }
  }else if(distanceL >= 100 || distanceR >= 100) {
    if(timeEntered == 0) {
      timeEntered = millis();
    }
    else if(laneSplitMode && millis() - timeEntered >= 1000) {
      laneSplitMode = false;
    }
  }

  // lane splitting or lean angle
  if (abs(euler.y()) <= 30) {
    if (laneSplitMode) {
      // Lane splitting mode, set LEDs to green
      laneSplitLED();
    } else {
      setRightLED(distanceR);
      setLeftLED(distanceL);
    }
  } else if (euler.y() < -30) {
    setLeftLED(distanceL);
    rightLEDOFF();
  } else {
    setRightLED(distanceR);
    leftLEDOFF();
  }

}
void laneSplitLED() {
  analogWrite(6,3);
  analogWrite(3,5);
  analogWrite(5,3);
  analogWrite(10,5);
}
void rightLEDOFF() {
  analogWrite(6,0);
  analogWrite(3,0);
}

void leftLEDOFF() {
  analogWrite(5,0);
  analogWrite(10,0);
}

void setRightLED(uint16_t distanceR) {
  if (distanceR>=400 && distanceR<466)
    {
      analogWrite(6,255); /* Right red led at 100% duty cycle */
      analogWrite(3,76); /* Right green led at ~29% duty cycle */
    }
    else if (distanceR>=350 && distanceR<400)
    {
      analogWrite(6,255); /* Right red led at 100% duty cycle */
      analogWrite(3,65); /* Right green led at ~25% duty cycle */
    }
    else if (distanceR>=300 && distanceR<350)
    {
      analogWrite(6,255); /* Right red led at 100% duty cycle */
      analogWrite(3,55); /* Right green led at ~22% duty cycle */
    }
    else if (distanceR>=250 && distanceR<300)
    {
      analogWrite(6,255); /* Right red led at 100% duty cycle */
      analogWrite(3,45); /* Right green led at ~18% duty cycle */
    }
    else if (distanceR>=200 && distanceR<250)
    {
      analogWrite(6,255); /* Right red led at 100% duty cycle */
      analogWrite(3,35); /* Right green led at ~14% duty cycle */
    }
    else if (distanceR>=150 && distanceR<200)
    {
      analogWrite(6,255); /* Right red led at 100% duty cycle */
      analogWrite(3,25); /* Right green led at ~10% duty cycle */
    }
    else if (distanceR>=100 && distanceR<150)
    {
      analogWrite(6,255);
      analogWrite(3,15); 
    }
    else if (distanceR>=0 && distanceR<100)
    {
      analogWrite(6,255); /* Right red led at 100% duty cycle */
      analogWrite(3,0); /* Right green led at 0% duty cycle */
    }
    else
    {
      analogWrite(6,0); /* Right red led at 0% duty cycle */
      analogWrite(3,0); /* Right green led at 0% duty cycle */
    }
}

void setLeftLED(uint16_t distanceL) {
  if (distanceL>=400 && distanceL<466)
    {
      analogWrite(5,255); /* Left red led at 100% duty cycle */
      analogWrite(10,76); /* Left green led at ~29% duty cycle */
    }
    else if (distanceL>=350 && distanceL<400)
    {
      analogWrite(5,255); /* Left red led at 100% duty cycle */
      analogWrite(10,65); /* Left green led at ~25% duty cycle */
    }
    else if (distanceL>=300 && distanceL<350)
    {
      analogWrite(5,255); /* Left red led at 100% duty cycle */
      analogWrite(10,55); /* Left green led at ~22% duty cycle */
    }
    else if (distanceL>=250 && distanceL<300)
    {
      analogWrite(5,255); /* Left red led at 100% duty cycle */
      analogWrite(10,45); /* Left green led at ~18% duty cycle */
    }
    else if (distanceL>=200 && distanceL<250)
    {
      analogWrite(5,255); /* Right red led at 100% duty cycle */
      analogWrite(10,35); /* Right green led at ~14% duty cycle */
    }
    else if (distanceL>=150 && distanceL<200)
    {
      analogWrite(5,255); 
      analogWrite(10,25); 
    }
    else if (distanceL>=100 && distanceL<150)
    {
      analogWrite(5,255);
      analogWrite(10,15); 
    }
    else if (distanceL>=0 && distanceL<100)
    {
      analogWrite(5,255); /* Left red led at 100% duty cycle */
      analogWrite(10,0); /* Left green led at 0% duty cycle */
    }
    else
    {
      analogWrite(5,0); /* Left red led at 0% duty cycle */
      analogWrite(10,0); /* Left green led at 0% duty cycle */
    }
}
uint8_t distanceContinuousR(uint16_t * distance)
{
    uint8_t newDistance = 0;

    // Check on busyFlag to indicate if device is idle
    // (meaning = it finished the previously triggered measurement)
    if (myLidarLite.getBusyFlag() == 0)
    {
        // Trigger the next range measurement
        myLidarLite.takeRange();

        // Read new distance data from device registers
        *distance = myLidarLite.readDistance();

        // Report to calling function that we have new data
        newDistance = 1;
    }

    return newDistance;
}
uint8_t distanceContinuousL(uint16_t * distance)
{
    uint8_t newDistance = 0;

    // Check on busyFlag to indicate if device is idle
    // (meaning = it finished the previously triggered measurement)
    if (myLidarLite.getBusyFlag(0x55) == 0)
    {
        // Trigger the next range measurement
        myLidarLite.takeRange(0x55);

        // Read new distance data from device registers
        *distance = myLidarLite.readDistance(0x55);

        // Report to calling function that we have new data
        newDistance = 1;
    }

    return newDistance;
}


void logData(float eulerY, uint16_t distanceR, uint16_t distanceL) {
  // Write the data to the SD card file
  dataFile.print(eulerY);
  dataFile.print(",");
  dataFile.print(distanceR);
  dataFile.print(",");
  dataFile.println(distanceL);

  // Flush the data to the SD card to ensure it's written
  dataFile.flush();
}
