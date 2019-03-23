#ifndef _PROTOCOL_H
#define _PROTOCOL_H

#include <stdlib.h>
#include "main.h"

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

void execute(Serial_struct data);

#endif
