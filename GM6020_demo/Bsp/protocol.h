#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <stdlib.h>
#include "main.h"
#include "trajectory.h"
#include "bsp_led.h"

#define packSize 14

// serial communication protocol
__packed  typedef struct{
  uint8_t flag;            // 1 bytes 0xAA 0x10101010
	uint8_t type;            // 1 bytes
	int16_t value;          // 2 bytes
	int32_t position;       // 4 bytes
	int32_t velocity;       // 4 bytes
  uint16_t crc;						 // 2 bytes
} Serial_struct;
// sizeof(serial_struct)  // 14 bytes

Serial_struct unpack(uint8_t* head);

Serial_struct execute(Serial_struct data, RobotControl* robot_control, Trajectory* traj);

Serial_struct ack(Serial_struct data, uint16_t value, uint32_t position, uint32_t velocity);
Serial_struct err(Serial_struct data, uint16_t value, uint32_t position, uint32_t velocity);

#endif
