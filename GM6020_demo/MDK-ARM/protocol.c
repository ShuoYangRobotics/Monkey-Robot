#include "protocol.h"

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
