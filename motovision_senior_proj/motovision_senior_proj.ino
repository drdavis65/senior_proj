#include <stdint.h>
#include <Wire.h>
#include "LIDARLite_v4LED.h"

//IMU libs
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28, &Wire);

LIDARLite_v4LED myLidarLite;

#define FAST_I2C

void setup() {
  // put your setup code here, to run once:
  // Initialize Arduino serial port (for display of ASCII output to PC)
  Serial.begin(115200);

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

   Serial.println("Orientation Sensor Raw Data Test"); Serial.println("");

  /* Initialise the sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
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
}

void loop() {
  // put your main code here, to run repeatedly:
  
  // Lidar Sensor ///////////////////////////////////////////////////////////////////////

  uint16_t distanceR;
  uint16_t distanceL;
  
  distanceContinuousR(&distanceR);
  distanceContinuousL(&distanceL);
  
  Serial.println("distanceR: ");
  Serial.println(distanceR);

  Serial.println("distanceL: ");
  Serial.println(distanceL);

  // IMU Sensor ///////////////////////////////////////////////////////////////////////////

  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

  /* Display the floating point data */
  Serial.print("X: ");
  Serial.print(euler.x());
  Serial.print(" Y: ");
  Serial.print(euler.y());
  Serial.print(" Z: ");
  Serial.print(euler.z());
  Serial.print("\t\t");
  Serial.println();
  Serial.println();
  
  delay(1000);

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