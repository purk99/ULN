#ifndef _IMU_H_
#define _IMU_H_

bool imuSetup();
void imuLoop();

extern float x, y, z;
extern float gx, gy, gz;

#endif