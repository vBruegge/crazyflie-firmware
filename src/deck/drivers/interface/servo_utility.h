#ifndef SERVO_UTILITY_H
#define SERVO_UTILITY_H

#include "motors.h"

void servoInit(const MotorPerifDef* servoDef);

void servoDeInit(void);

bool servoTest();

void servoSetRatio(uint16_t driveAngle);

void servoEnablePWM(void);

void servoDisablePWM(void);

void servoStop();


#endif