#include <cmath>
#include <iostream>

float Tf = 4.0f; // use 4 seconds to finish this trajectory
const int step = 200;  // how many points in the trajectory
float left_start_state[2];   // 
float left_end_state[2];     // 
float left_state_angle[step+1];
float left_state_velocity[step+1];

float right_start_state[2];   // 
float right_end_state[2];     // 
float right_state_angle[step+1];
float right_state_velocity[step+1];
float motor_angle_rad[2];
float motor_velocity_rads[2];
void init_simple_trajectory(int motor_idx, float Tf, const int step, float tgt_state[], float state_angle[], float state_velocity[])
{
	float curr_angle, curr_velocity;
	float dt = Tf/step;
	if (motor_idx == 0) // left
	{
		curr_angle = motor_angle_rad[0];
		curr_velocity = motor_velocity_rads[0];
	}
	else // right
	{
		curr_angle = motor_angle_rad[1];
		curr_velocity = motor_velocity_rads[1];
	}
	state_angle[0] = curr_angle;
	state_velocity[0] = curr_velocity;
	for (int i = 1; i < step; i++)
	{
		state_angle[i] = state_angle[0] + (float)i/step*(tgt_state[0] - state_angle[0]);
		
		state_velocity[i] = 0.6f*(state_angle[i] - state_angle[i-1])/dt + 0.4f*state_velocity[i-1];
	}
	state_angle[step] = tgt_state[0];
	state_velocity[step] = tgt_state[1];
}	

int main(void)
{
	motor_angle_rad[0] = 4.573;
	motor_angle_rad[1] = 1.590;
    float tgt_state[2] = {M_PI,0};
	init_simple_trajectory(0, Tf, step, tgt_state, left_state_angle, left_state_velocity);
	init_simple_trajectory(1, Tf, step, tgt_state, right_state_angle, right_state_velocity);
    std::cout << "left" <<std::endl;
	for (int i = 1; i < step; i++)
	{
        std::cout << left_state_angle[i] << " ";
    }
    std::cout << std::endl;
    for (int i = 1; i < step; i++)
	{
        std::cout << left_state_velocity[i] << " ";
    }
    std::cout << std::endl;
    std::cout << "right" <<std::endl;
	for (int i = 1; i < step; i++)
	{
        std::cout << right_state_angle[i] << " ";
    }
    std::cout << std::endl;
    for (int i = 1; i < step; i++)
	{
        std::cout << right_state_velocity[i] << " ";
    }
    std::cout << std::endl;
	init_simple_trajectory(1, Tf, step, tgt_state, right_state_angle, right_state_velocity);

    return 0;
}