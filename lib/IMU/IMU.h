#ifndef _IMU_H_
#define _IMU_H_

float x, y, z;
float gx, gy, gz;

bool imuSetup();
void imuLoop();

#endif