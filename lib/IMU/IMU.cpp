#include <Arduino.h>

#ifdef TARGET_NANO_OTHER
#include <Arduino_LSM9DS1.h>
//#include <LSM9DS1.h>
#else
#include <Arduino_LSM6DS3.h>
#endif

#include "IMU.h"

float x, y, z;
float gx, gy, gz;

bool imuSetup()
{
  if (!IMU.begin()) {
    return false;
  }

  #ifdef DEBUG
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
