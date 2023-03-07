#ifndef __CONTROL_H
#define __CONTROL_H

#include "sys.h"
#include "motor.h"
#include "oled.h"
#include "Kalman.h"
#include "pid.h"
#include "pidtool.h"
#include "math.h"
#include "mpu6050.h"
#include "key.h"
#include "delay.h"

extern PID_TypeDef Roll_PID,Pitch_PID;

void angle_calibration(void);

void mode_1(void);
void mode_2(void);
void mode_3(void);
void mode_4(void);
void mode_5(void);

uint8_t Set_Length(void);
uint8_t switch_mode(void);

#endif /* __CONTROL_H */
