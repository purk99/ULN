#include <Arduino.h>
#include <Arduino_LSM6DS3.h>

#include "IMU.h"

bool imuSetup()
{
  if (!IMU.begin()) {
    return false;
  }

  #ifdef PRINT_DEBUG_BUILD
    Serial.print("Gyroscope sample rate = ");
    Serial.print(IMU.gyroscopeSampleRate());
    Serial.println(" Hz");
    Serial.println();
    Serial.print("Accelerometer sample rate = ");
    Serial.print(IMU.accelerationSampleRate());
    Serial.println(" Hz");
    Serial.println();
  #endif

  return true;
}

void imuLoop()
{
  if (IMU.accelerationAvailable()) {
     IMU.readAcceleration(x,y,z);
  }

  if (IMU.gyroscopeAvailable()) {
    IMU.readGyroscope(gx, gy, gz);
  }

  #ifdef PRINT_DEBUG_BUILD
    Serial.print(x);
    Serial.print('\t');
    Serial.print(y);
    Serial.print('\t');
    Serial.print(z);
    Serial.print('\t');
    Serial.print(gx);
    Serial.print('\t');
    Serial.print(gy);
    Serial.print('\t');
    Serial.println(gz);
  #endif
}
