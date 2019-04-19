#include "protocol.h"

//// Variable
Serial_struct lastSentPack;
extern bool acked;
extern uint16_t packIndex;

extern pid_struct_t motor_angle_pid[2];
extern pid_struct_t motor_velocity_pid[2];
extern pid_struct_t motor_current_pid[2];

//// Functions
Serial_struct unpack(uint8_t* head) {
	Serial_struct data;
	data.flag = 0xAA;
	data.type = *(head+1);
	data.value = (*(head+3)<<8) + *(head+2);
	data.position = (*(head+7)<<24) + (*(head+6)<<16) + (*(head+5)<<8) + *(head+4);
	data.velocity = (*(head+11)<<24) + (*(head+10)<<16) + (*(head+9)<<8) + *(head+8);
	
	// do crc
	uint16_t crc_ccitt_ffff_val = 0xffff;
	uint8_t* ptr = (uint8_t *) &data;
	int i;
	for(i = 0; i<12; i++) { // do crc with the first 12 uint8
		crc_ccitt_ffff_val = update_crc_ccitt(crc_ccitt_ffff_val, *ptr);
		ptr++;
	}
	data.crc = crc_ccitt_ffff_val;
	return data;
}

Serial_struct pack(uint8_t type, uint16_t value, uint32_t position, uint32_t velocity) {
	Serial_struct dataPack;
	dataPack.flag = 0xAA;
	dataPack.type = type;
	dataPack.value = value;
	dataPack.position = position;
	dataPack.velocity = velocity;
		
	// do crc
	uint16_t crc_ccitt_ffff_val = 0xffff;
	uint8_t* ptr = (uint8_t *) &dataPack;
	int i;
	for(i = 0; i<12; i++) { // do crc with the first 12 uint8
		crc_ccitt_ffff_val = update_crc_ccitt(crc_ccitt_ffff_val, *ptr);
		ptr++;
	}
	dataPack.crc = crc_ccitt_ffff_val;
	return dataPack;
}

Serial_struct execute(Serial_struct data, RobotControl* robot_control, Trajectory* traj) {
	Serial_struct ackPack;
	switch(data.type) {
		case 0:
				robot_control -> output_enable = 0;
			break;
		
		case 1:
			switch(data.value) {
				case 0:
					
					break;
				case 1:
					
					break;
				case 2:
			
					break;
				case 3:
					
					break;
				case 4:
					break;
			}
			break;
		case 2: // start receive points
			// set ctrl_mode to 5
			robot_control -> ctrl_mode = 5;
			ackPack = ack(data, data.value, data.position, data.velocity);
			traj -> leftStepReceived = 0;
			traj -> rightStepReceived = 0;
			break;
		
		case 3: // left motor traj
			if(robot_control -> ctrl_mode == 5) { 
				if(traj -> leftStepReceived == data.value) {
					traj -> left_state_angle[traj -> leftStepReceived] = ((float)data.position)/1000.0f;
					traj -> left_state_velocity[traj -> leftStepReceived] = ((float)data.velocity)/1000.0f;
					traj -> leftStepReceived++;
					ackPack = ack(data, traj -> leftStepReceived, data.position, data.velocity);
				} else {
					robot_control -> ctrl_mode = 0; // error, back to idle
					ackPack = err(data, data.value, data.position, data.velocity);
				}
			}
			break;
			
		case 4: // right motor traj
			if(robot_control -> ctrl_mode == 5) {
				if(traj -> rightStepReceived == data.value) {
					traj -> right_state_angle[traj -> rightStepReceived] = ((float)data.position)/1000.0f;
					traj -> right_state_velocity[traj -> rightStepReceived] = ((float)data.velocity)/1000.0f;
					traj -> rightStepReceived++;
					ackPack = ack(data, traj -> rightStepReceived, data.position, data.velocity);
				} else {
					robot_control -> ctrl_mode = 0; // error, back to idle
					ackPack = err(data, traj -> rightStepReceived, data.position, data.velocity);
				}
			}
			break;
			
		case 5:
			if(robot_control -> ctrl_mode == 5 && 
				traj -> leftStepReceived == data.value && 
				traj -> rightStepReceived == data.value) 
			{
				robot_control -> ctrl_mode = 6;
				ackPack = ack(data, traj -> leftStepReceived, data.position, data.velocity);
			} else {
				ackPack = err(data, traj -> leftStepReceived, data.position, data.velocity);
			}
			break;
			
		case 6:  
			if(robot_control -> ctrl_mode == 6) { // in ready to start mode
				robot_control -> output_enable = 1; // enable power
				robot_control -> ctrl_mode = 4; // start trajectory tracking
				
				// only do data.value check once in aux state, do not need to do it here
				
				ackPack = ack(data, traj -> leftStepReceived, data.position, data.velocity);
				LED_RED_TOGGLE();
			} else {
				ackPack = err(data, traj -> leftStepReceived, data.position, data.velocity);
			}
			break;
			
		case 7:
			if(robot_control -> ctrl_mode == 6) { // in ready to start mode
				robot_control -> output_enable = 1; // enable power
				robot_control -> ctrl_mode = 8; // start aux trajectory to prepare
				
				// added 2019-04-16, use data.value to determine swing arm and swing direction
				if (data.value == 0) {
					robot_control -> ctrl_side = 1;
					robot_control -> ctrl_direction = 0;
				} else if (data.value == 1) {
					robot_control -> ctrl_side = -1;
					robot_control -> ctrl_direction = 0;
				} else if (data.value == 10) {
					robot_control -> ctrl_side = 1;
					robot_control -> ctrl_direction = 1;
				} else if (data.value == 11) {
					robot_control -> ctrl_side = -1;
					robot_control -> ctrl_direction = 1;
				}
				
				ackPack = ack(data, traj -> leftStepReceived, data.position, data.velocity);
				LED_RED_TOGGLE();
			} else {
				ackPack = err(data, traj -> leftStepReceived, data.position, data.velocity);
			}
			break;
		
		case 8:
			
			if (data.value == 0) {
				robot_control -> pwm_pulse_left = 1500-320;
			}
			else {
				robot_control -> pwm_pulse_left = 1500;
			}
			break;
		case 9:
			if (data.value == 0) {
				robot_control -> pwm_pulse_right = 1500+320;
			}
			else {
				robot_control -> pwm_pulse_right = 1500;
			}
			break;
			
		case 20: {
			robot_control -> traj_start_delay = data.value;
			ackPack = ack(data, data.value, data.position, data.velocity);
			LED_RED_TOGGLE();
			break;
		}
		case 25: {
			if(data.value == 0) { // tune pos Kp
				if(data.position>0)
					motor_angle_pid[0].kp = data.position;
				if(data.velocity>0)
					motor_angle_pid[1].kp = data.velocity;
			}else if(data.value == 1) { // tune pos Ki
				if(data.position>0)
					motor_angle_pid[0].ki = data.position;
				if(data.velocity>0)
					motor_angle_pid[1].ki = data.velocity;
			}else if(data.value == 2) { // tune pos Kd
				if(data.position>0)
					motor_angle_pid[0].kd = data.position;
				if(data.velocity>0)
					motor_angle_pid[1].kd = data.velocity;
			}else if(data.value == 3) { // tune vel Kp
				if(data.position>0)
					motor_velocity_pid[0].kp = data.position;
				if(data.velocity>0)
					motor_velocity_pid[1].kp = data.velocity;
			}else if(data.value == 4) { // tune vel Ki
				if(data.position>0)
					motor_velocity_pid[0].ki = data.position;
				if(data.velocity>0)
					motor_velocity_pid[1].ki = data.velocity;
			}else if(data.value == 5) { // tune vel Kd
				if(data.position>0)
					motor_velocity_pid[0].kd = data.position;
				if(data.velocity>0)
					motor_velocity_pid[1].kd = data.velocity;
			}else if(data.value == 6) { // tune cur Kp
				if(data.position>0)
					motor_current_pid[0].kp = data.position;
				if(data.velocity>0)
					motor_current_pid[1].kp = data.velocity;
			}else if(data.value == 7) { // tune cur Ki
				if(data.position>0)
					motor_current_pid[0].ki = data.position;
				if(data.velocity>0)
					motor_current_pid[1].ki = data.velocity;
			}else if(data.value == 8) { // tune cur Kd
				if(data.position>0)
					motor_current_pid[0].kd = data.position;
				if(data.velocity>0)
					motor_current_pid[1].kd = data.velocity;
			}
			
			ackPack = ack(data, data.value, data.position, data.velocity);
			LED_RED_TOGGLE();
			break;
		}
		case 27: {
			robot_control -> Tf = ((float)data.value)/1000.0f;
			ackPack = ack(data, data.value, data.position, data.velocity);
			LED_RED_TOGGLE();
			break;
		}
		case 49: {
			robot_control -> debug_print = 4;
			robot_control -> acked = 1;
			break;
		}
		case 51: {
			robot_control -> acked = 1;
			packIndex ++;
			break;
		}
		default:
			break;
	}
	return ackPack;
}

//Serial_struct ack(Serial_struct data) {
//	Serial_struct ackPack;
//	ackPack.flag = 0xAA;
//	ackPack.type = 5; // Ack
//	ackPack.value = data.type;
//	ackPack.position = data.position;
//	ackPack.velocity = data.velocity;
//		
//	// do crc
//	uint16_t crc_ccitt_ffff_val = 0xffff;
//	uint8_t* ptr = (uint8_t *) &ackPack;
//	int i;
//	for(i = 0; i<12; i++) { // do crc with the first 12 uint8
//		crc_ccitt_ffff_val = update_crc_ccitt(crc_ccitt_ffff_val, *ptr);
//		ptr++;
//	}
//	ackPack.crc = crc_ccitt_ffff_val;
//	return ackPack;
//}

Serial_struct ack(Serial_struct data, uint16_t value, uint32_t position, uint32_t velocity) {
	Serial_struct ackPack;
	ackPack.flag = 0xAA;
	ackPack.type = 40; // Ack
	ackPack.value = value;
	ackPack.position = position;
	ackPack.velocity = velocity;
		
	// do crc
	uint16_t crc_ccitt_ffff_val = 0xffff;
	uint8_t* ptr = (uint8_t *) &ackPack;
	int i;
	for(i = 0; i<12; i++) { // do crc with the first 12 uint8
		crc_ccitt_ffff_val = update_crc_ccitt(crc_ccitt_ffff_val, *ptr);
		ptr++;
	}
	ackPack.crc = crc_ccitt_ffff_val;
	return ackPack;
}

Serial_struct err(Serial_struct data, uint16_t value, uint32_t position, uint32_t velocity) {
	Serial_struct ackPack;
	ackPack.flag = 0xAA;
	ackPack.type = 100; // error
	ackPack.value = value;
	ackPack.position = position;
	ackPack.velocity = velocity;
		
	// do crc
	uint16_t crc_ccitt_ffff_val = 0xffff;
	uint8_t* ptr = (uint8_t *) &ackPack;
	int i;
	for(i = 0; i<12; i++) { // do crc with the first 12 uint8
		crc_ccitt_ffff_val = update_crc_ccitt(crc_ccitt_ffff_val, *ptr);
		ptr++;
	}
	ackPack.crc = crc_ccitt_ffff_val;
	return ackPack;
}

void setLastSend(Serial_struct data) {
	lastSentPack = data;
}

