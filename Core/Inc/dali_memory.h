/*
 * dali_memory.h
 * This file implements the memory banks of DALI input devices
 *  Created on: Aug 4, 2020
 *      Author: vtran
 */

#ifndef INC_DALI_MEMORY_H_
#define INC_DALI_MEMORY_H_
#include "stdint.h"
// Parameters in memory bank 0
enum memory_bank_0_address
{
	lastByte_addr			= 0x00,
	lastBank_addr			= 0x02,
	GTIN_0_addr,	// address for MSB
	GTIN_1_addr,
	GTIN_2_addr,
	GTIN_3_addr,
	GTIN_4_addr,
	GTIN_5_addr,	// address for LSB
	fw_major_ver_addr,
	fw_minor_ver_addr,
	id_0_addr,		// address for MSB
	id_1_addr,
	id_2_addr,
	id_3_addr,
	id_4_addr,
	id_5_addr,
	id_6_addr,
	id_7_addr,		// address for LSB
	hw_major_ver_addr,
	hw_minor_ver_addr,
	ver_num_101_addr,
	ver_num_102_addr,
	ver_num_103_addr,
	control_device_number_addr,
	control_gear_number_addr,
	device_index_addr
};

// Parameters in memory bank 189
enum memory_bank_189_address
{
	lockByte_addr 			= 0x02,
	parameterLock_addr,
	factoryReset_addr,
	calibrationScale_addr,
	calibrationOffset_addr,
	pidProportionalCoeff_addr,
	pidIntegralCoeff_addr,
	pidDerivativeCoeff_addr,
	calibrateDark,
	calibrateFullScale,
	fullScaleRange_addr 	= 0x15
};
#define MEMORY_NVM_VAR_ADDR			0x0800E800
#define MEMORY_ROM_VAR_ADDR			0x0800EC00
#define MEMORY_BANK_0_ADDR			0x0800F000
#define MEMORY_BANK_189_ADDR		0x0800F400
//#define GTIN						((* (uint8_t*) (MEMORY_BANK_0_ADDR + GTIN_0_addr)) | (* (uint8_t*) (MEMORY_BANK_0_ADDR + GTIN_1_addr))  \
//									| (* (uint8_t*) (MEMORY_BANK_0_ADDR + GTIN_2_addr)) | (* (uint8_t*) (MEMORY_BANK_0_ADDR + GTIN_3_addr)) \
//									| (* (uint8_t*) (MEMORY_BANK_0_ADDR + GTIN_4_addr)) | (* (uint8_t*) (MEMORY_BANK_0_ADDR + GTIN_5_addr)))
#define FW_MAJOR_VER				(* (uint8_t*) (MEMORY_BANK_0_ADDR + fw_major_ver_addr))
#define FW_MINOR_VER				(* (uint8_t*) (MEMORY_BANK_0_ADDR + fw_minor_ver_addr))
//#define ID							((* (uint8_t*) (MEMORY_BANK_0_ADDR + id_0_addr)) | (* (uint8_t*) (MEMORY_BANK_0_ADDR + id_1_addr))  \
//									| (* (uint8_t*) (MEMORY_BANK_0_ADDR + id_2_addr)) | (* (uint8_t*) (MEMORY_BANK_0_ADDR + id_3_addr)) \
//									| (* (uint8_t*) (MEMORY_BANK_0_ADDR + id_4_addr)) | (* (uint8_t*) (MEMORY_BANK_0_ADDR + id_5_addr)) \
//									| (* (uint8_t*) (MEMORY_BANK_0_ADDR + id_5_addr)) | (* (uint8_t*) (MEMORY_BANK_0_ADDR + id_6_addr)) \
//									| (* (uint8_t*) (MEMORY_BANK_0_ADDR + id_7_addr)))
#define HW_MAJOR_VER				(* (uint8_t*) (MEMORY_BANK_0_ADDR + hw_major_ver_addr))
#define HW_MINOR_VER				(* (uint8_t*) (MEMORY_BANK_0_ADDR + hw_minor_ver_addr))
#define VER_NUM_101					(* (uint8_t*) (MEMORY_BANK_0_ADDR + ver_num_101_addr))
#define VER_NUM_102					(* (uint8_t*) (MEMORY_BANK_0_ADDR + ver_num_102_addr))
#define VER_NUM_103					(* (uint8_t*) (MEMORY_BANK_0_ADDR + ver_num_103_addr))
#define CONTROL_DEVICE_NUMBER		(* (uint8_t*) (MEMORY_BANK_0_ADDR + control_device_number_addr))
#define CONTROL_GEAR_NUMBER			(* (uint8_t*) (MEMORY_BANK_0_ADDR + control_gear_number_addr))
#define DEVICE_INDEX				(* (uint8_t*) (MEMORY_BANK_0_ADDR + device_index_addr))

#define fullScaleRange				((* (uint8_t*) (MEMORY_BANK_189_ADDR + fullScaleRange_addr + 1)) << 8) | (* (uint8_t*) (MEMORY_BANK_189_ADDR + fullScaleRange_addr))
#define calibrationScale			(* (uint8_t*) (MEMORY_BANK_189_ADDR + calibrationScale_addr))
#define calibrationOffset			(* (uint8_t*) (MEMORY_BANK_189_ADDR + calibrationOffset_addr))
#define pidProportionalCoeff		(* (uint8_t*) (MEMORY_BANK_189_ADDR + pidProportionalCoeff_addr))
#define pidIntegralCoeff			(* (uint8_t*) (MEMORY_BANK_189_ADDR + pidIntegralCoeff_addr))
#define pidDerivativeCoeff			(* (uint8_t*) (MEMORY_BANK_189_ADDR + pidDerivativeCoeff_addr))
#define factoryReset				(* (uint8_t*) (MEMORY_BANK_189_ADDR + factoryReset_addr))
#define parameterLock				(* (uint8_t*) (MEMORY_BANK_189_ADDR + parameterLock_addr))

typedef struct
{
	uint8_t success;
	uint8_t value;
} memory_read_t;

extern uint8_t darkCalibrate;
extern uint8_t fullScaleCalibrate;
/*
 * Initialize dali memory bank
 */
void dali_memory_init(void);

/*
 * Reset all unlocked memory banks to the reset value
 * Parameters: 	memory_bank_number: identify the memory bank to be reset
 * 				If memory_bank_number == 0, reset all memory banks except bank 0
 */
void dali_memory_reset(uint8_t memory_bank_number);

/*
 * Memory bank reading
 * Parameters: 	memory_bank_number: the memory bank number, usually given by DTR1
 * 				memory_offset: location in the memory bank, usually given by DTR0
 * Return a struct that notifies if the read is succeeded or not, and the memory value if read successfully
*/
memory_read_t dali_memory_read(uint8_t memory_bank_number, uint8_t memory_offset);

/*
 * Memory bank writing
 * Parameters:	memory bank number: the memory bank number, given by DTR1
 * 				memory_offset: location in the memory bank, given by DTR0
 * Return 0 if success write on RAM variable, 1 if there is an error, 2 is okay to do NVM writing
 */
uint8_t dali_memory_write(uint8_t memory_bank_number, uint8_t memory_offset, uint8_t data);

/*
 * NVM related part of Memory bank writing
 * Need to split the write function into 2 functions because the device need to response with an
 * answer in 13ms, so the first function check if the memory bank location is legit and this function
 * does the actual writing
 */
void memory_write(uint8_t memory_bank_number, uint8_t memory_offset, uint8_t data);
/*
 * Erase a page memory
 * Parameters: pointer to the variables array
 */
uint32_t erase_page(uint32_t page_address);

/*
 * Unlock the NVM for writing
 */
void dali_NVM_unlock();
void dali_NVM_lock();
#endif /* INC_DALI_MEMORY_H_ */
