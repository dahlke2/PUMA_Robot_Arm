#ifndef _PUMA_ROBOT_ARM_
#define _PUMA_ROBOT_ARM_

#include "DSP28x_Project.h"
#include "isr.h"
#include "init_functions.h"
#include "Example_posspeed.h"
#include "Feedback.h"
#include "Serial_Comm.h"

void potentiometer_compare_speed(void);
void potentiometer_compare_position(void);
void Speed_control(volatile Uint16 *pwm);
Uint16 ceil(Uint16 value, Uint16 max);

extern 	POSSPEED qep_posspeed;

#define Joint_Speed 50
#define rpm_pwm_cal 1

#endif /*PUMA_ROBOT_ARM_H_*/
