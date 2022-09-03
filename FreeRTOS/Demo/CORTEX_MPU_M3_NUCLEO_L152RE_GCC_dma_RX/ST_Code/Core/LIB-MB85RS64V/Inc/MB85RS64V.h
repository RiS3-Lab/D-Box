/*
 * MB85RS64V.h
 *
 *  Created on: May 13, 2020
 *      Author: Alejandro Mera
 */

#ifndef THIRD_PARTY_MB85RS64V_LIB_INC_MB85RS64V_H_
#define THIRD_PARTY_MB85RS64V_LIB_INC_MB85RS64V_H_

#include "main.h"
#include "FreeRTOS.h"





/* Fujitsu MB85RS64V FRAM 8kB (64 kbit)
* Command description:
* WREN:  Write Enable Latch. WEL shall be set with the WREN command before writing
*  		 operation (WRSR command and WRITE command)
* WRDI:  The WRDI command resets WEL (Write Enable Latch) . Writing operation
* 		 (WRITE command and WRSRcommand) are not performed when WEL is reset.
* RDSR:  The RDSR command reads status register data. After op-code of RDSR is
* 		 input to SI, 8-cycle clock is input to SCK. The SI value is invalid
* 		 during this time. SO is output synchronously to a falling edge of SCK.
* 		 In theRDSR command, repeated reading of status register is
* 		 enabled by sending SCK continuously before rising of CS.
* WRSR:  The WRSR command writes data to the non volatile memory bit of status
* 		 register. After performing WRSRop-code to a SI pin, 8 bits writing
* 		 data is input. WEL (Write Enable Latch) is not able to be written with
* 		 WRSRcommand. A SI value correspondent to bit 1 is ignored. Bit 0 of the
* 		 status register is fixed to “0” and cannot be written. The SI value
* 		 corresponding to bit 0 is ignored. The WP signal level shall be fixed
* 		 before performing the WRSR command, and do not change the WP signal
* 		 level until the end of command sequence.
* READ:  The READ command reads FRAM memory cell array data. Arbitrary 16 bits
* 		 address and op-code of READare input to SI. The 3-bit upper
* 		 address bit is invalid. Then, 8-cycle  clock is input to SCK.
* 		 SO is output synchronously to the falling edge of SCK. While reading,
* 		 the SI value is invalid. When CS is risen, the READcommand is completed,
* 		 but keeps on reading with automatic address increment which is enabled
* 		 by continuously sending clocks to SCK in unit of 8 cycles before CS
* 		 rising. When it reaches the most significant address, it rolls over
* 		 to the starting address, and reading cycle keeps on infinitely.
* WRITE: The WRITE command writes data to FRAM memory cell array. WRITE op-code,
* 		 arbitrary 16 bits of address and 8 bits of writing data are input to SI.
* 		 The 3-bit upper address bit is invalid. When 8 bits of writing data is
* 		 input, data is written to FRAM memory cell array. Risen CS will terminate
* 		 the WRITE command. However,if you continue sending the writing data for 8
* 		 bits each before CS rising, it is possible to continue writing with
* 		 automatic  address  increment.  When  it  reaches  the  most  significant
* 		 address,  it  rolls  over  to  the  starting address, and writing cycle
* 		 keeps on infinitely.
* RDID:	 The RDID command reads fixed Device ID. After performing RDID op-code to SI,
* 		 32-cycle clock is input to SCK. The SI value is invalid during this time. SO
* 		 is output synchronously to a falling edge of SCK. The output is in order of
* 		 Manufacturer ID (8bit)/Continuation code (8bit)/Product ID
* 		 (1st Byte)/Product ID (2nd Byte). In the RDID command, SO holds the output
* 		 state of the last bit in 32-bit Device ID until CS is risen.
*/

/* OP-Codes */
#define WREN 0x06
#define WRDI 0x04
#define RDSR 0x05
#define WRSR 0x01
#define READ 0x03
#define WRITE 0x02
#define RDID 0x9f
#define DUMMY 0x00


/* SR bits */
#define SR_WPEN_MASK 7
#define SR_WPEN_BIT (1 << SR_WPEN_MASK)
#define SR_BP1_MASK 3
#define SR_BP1_BIT (1 << SR_BP1_MASK)
#define SR_BP0_MASK 2
#define SR_BP0_BIT (1 << SR_BP1_MASK)
#define SR_WEL_MASK 1
#define SR_WEL_BIT (1 << SR_WEL_MASK)

//#define MAX_BUFFER 128

//typedef struct MB85RS64V_t MB85RS64V_t;

// Type definitions
typedef struct
{
	uint32_t *spiH;
	uint8_t ucCSpin;
	uint8_t (*Read_SR)(void);
	uint32_t (*Read_ID)(void);
	void (*Write_EN)(void);
	void (*WriteProtectBlock)(uint8_t block);
	void (*WriteUnProtectBlock)(uint8_t block);
	int16_t (*Write_Data)(uint8_t *data, uint16_t size);
	int16_t (*Read_Data)(uint8_t *data, uint16_t size);

} MB85RS64V_t;

typedef union {
	uint8_t   u8[4];
	uint16_t u16[2];
	uint32_t u32;

} bytesFields ;

/* Function Prototypes */
int8_t MB85R_Init(MB85RS64V_t *mb85H_v, uint32_t *pulPeripheral, uint8_t ucCSpin);
uint8_t MB85R_Read_SR(MB85RS64V_t *mb85H_v);
uint32_t MB85R_Read_ID(MB85RS64V_t *mb85H_v);
void MB85R_Write_EN(MB85RS64V_t *mb85H_v);
void MB85R_W_Protect_Block(MB85RS64V_t *mb85H_v, uint8_t block);
void MB85R_W_UnProtect_Block(MB85RS64V_t *mb85H_v, uint8_t block);
uint32_t MB85R_Write_Data(MB85RS64V_t *mb85H_v, uint16_t address, uint8_t *data, uint8_t ucDataRegion, uint16_t size);
uint32_t MB85R_Read_Data(MB85RS64V_t *mb85H_v, uint16_t address, uint8_t *data, uint8_t ucDataRegion, uint16_t size);

#endif /* THIRD_PARTY_MB85RS64V_LIB_INC_MB85RS64V_H_ */
