# Monkey-Robot

A monkey robot designed for CMU course 24-775

## Get code

```
git clone --recurse-submodules $REPO_URL
```

## File Description

- GM6020_demo 	: The sample code to control GM6020 motor
- Doc			: The folder for all the documents
- MonkeyMatlab	: Simualtion for 3-Link Monkey

## Protocol

```
__packed  typedef struct{
	uint8_t flag;            // 1 bytes 0xAA 0x10101010
	uint8_t type;            // 1 bytes
	uint16_t value;          // 2 bytes
	uint32_t position;       // 4 bytes
	uint32_t velocity;       // 4 bytes
	uint16_t crc;			 // 2 bytes
} Serial_struct;
// sizeof(serial_struct)     // 14 bytes
```

## TODO

 - [x] Read through demo code and control two motor  position through PWM
 - [x] Add DMA Uart
 - [ ] Understand how SPI and CAN works
 - [ ] FK IK function
 - [ ] unpack msg to change state
 - [ ] trajectory send & receive logic

