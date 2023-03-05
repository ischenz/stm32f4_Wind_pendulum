#ifndef __CONTROL_H
#define __CONTROL_H

#include "sys.h"
#include "motor.h"
#include "oled.h"
#include "Kalman.h"

typedef struct{
    float ProportionConstant;
    float IntegralConstant;
	float DerivativeConstant;
    float Err;
    float LastErr;
	float PenultErr;
    float Integral;//»ý·ÖºÍ
    float Target;
	int16_t PID_out;
}PID_TypeDef;

int16_t PID_Calculate(PID_TypeDef *PID,int16_t CurrentValue);
void PID_TimerInit(void);
void Roll_PID_Init(float TargetValue);
void Pitch_PID_Init(float TargetValue);
void mode_1(void);
void mode_2(void);

uint8_t Set_Length(void);
uint8_t switch_mode(void);

#endif /* __CONTROL_H */
