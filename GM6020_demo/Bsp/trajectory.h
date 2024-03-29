#ifndef _TRAJECTORY_H
#define _TRAJECTORY_H

#include "stm32f4xx_hal.h"
#include "arm_math.h"

#define stepNum 300

typedef struct Trajectory{	
	int16_t leftStepReceived;
	int16_t rightStepReceived;
	float left_state_angle[stepNum+1];
	float left_state_velocity[stepNum+1];
	float right_state_angle[stepNum+1];
	float right_state_velocity[stepNum+1];
} Trajectory;

uint8_t whichBehind(uint16_t direction, float* angles);
float rapAngle(float angle);

#endif
