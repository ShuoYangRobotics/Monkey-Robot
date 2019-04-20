#include "trajectory.h"

// trajectory read from serial port. The main trajectory
Trajectory traj;

// auxiliary trajectory, prepare robot to move to the starting point of the main trajectory
Trajectory aux_traj;

// get which arm is in the hind, left is 0, right is 1
uint8_t whichBehind(uint16_t direction, float* angles) {
	uint8_t arm;
	if(direction == 0){ // forward
		if(rapAngle(angles[0])+rapAngle(angles[1]) > 0) // left arm behind, because left arm >0 mean in the behind
			arm = 0;
		else // right hand behind
			arm = 1;
	} else { // backward
		if(rapAngle(angles[0])+rapAngle(angles[1]) > 0) // left arm forward
			arm = 1; // right arm behind
		else
			arm = 0;
	}
	return arm;
}

// rap an angle into (-pi,pi]
float rapAngle(float angle) {
	while(angle>PI) {
		angle -= 2*PI;
	}
	while(angle<=-PI){
		angle += 2*PI;
	}
	return angle;
}
