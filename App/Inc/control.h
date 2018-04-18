#ifndef _CONTROL_H_
#define _CONTROL_H_

#include "include.h"

extern void Moto_Out(int32 moto1_pwm,int32 moto2_pwm);
extern void Moto_Init(void);
extern void control(void);
extern void StepMoto_Init(void);
extern void StepMoto_Control(void);
#endif