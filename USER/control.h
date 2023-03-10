#ifndef __CONTROL_H
#define __CONTROL_H

#include "sys.h"

void angle_calibration(void);

void mode_1(void);
void mode_2(void);
void mode_3(void);
void mode_4(void);
void mode_5(void);

uint8_t Set_Angle(void);
uint8_t Set_Length(void);
uint8_t switch_mode(void);
void limit_angle(void);


#endif /* __CONTROL_H */
