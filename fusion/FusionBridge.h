#ifndef AHRS_H
#define AHRS_H

#include "./Fusion.h"

#ifdef __cplusplus
extern "C" {
#endif

void initAhrs(int platform, int rotation, float gain);
void updateAhrs(float deltaTime, float accel[3], float gyro[3], float mag[3]);
void setAhrsInterfaceRotation(int rotation);
void zeroAhrs(void);
float getAhrsRoll(void);
float getAhrsPitch(void);
float getAhrsYaw(void);
float getAhrsHeading(void);

#ifdef __cplusplus
}
#endif

#endif
