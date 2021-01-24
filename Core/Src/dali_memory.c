/*
 * dali_memory.c
 * This file implements the memory banks of DALI input devices
 *  Created on: Aug 4, 2020
 *      Author: vtran
 */
#include "dali_memory.h"
#include "stm32f0xx_hal.h"


uint8_t const lastByte_memory_bank_189			= 0x16;
uint8_t const indicatorByte						= 0xFF;
uint8_t const lockByte_default					= 0xFF;
uint16_t const fullScaleRange_default 			= 1000;
uint8_t const calibrationScale_default			= 255;
uint8_t const calibrationOffset_default			= 0;
uint8_t const parameterLock_default				= 0xFF;
uint8_t const factoryReset_default				= 0xFF;
uint8_t const pidProportionalCoeff_default		= 0xFF;
uint8_t const pidIntegralCoeff_default			= 0xFF;
uint8_t const pidDerivativeCoeff_default		= 0xFF;

uint32_t memory_bank_addr[256] = {0};
uint8_t lock_byte[256];	// Locked by set to 0x55
// If more memory banks are defined, the dali_memory_init function needs to be modified too
void dali_memory_init(void)
{
	memory_bank_addr[0] = MEMORY_BANK_0_ADDR;
	memory_bank_addr[189] = MEMORY_BANK_189_ADDR;
	lock_byte[189] = 0xFF;
	if((* (uint8_t*) (MEMORY_BANK_0_ADDR)) == 0xFF)
	{
		dali_NVM_unlock();
		// Implement memory bank 0
		(* (uint16_t*) (MEMORY_BANK_0_ADDR)) = 0xFF1A;
		(* (uint16_t*) (MEMORY_BANK_0_ADDR + 0x02)) = 0xBD;		// Last accessible memory bank = 189, MSB GTIN = 0
		(* (uint16_t*) (MEMORY_BANK_0_ADDR + 0x04)) = 0x3CC8;	// GTIN = 0x00 C8 3C 58 86 4A
		(* (uint16_t*) (MEMORY_BANK_0_ADDR + 0x06)) = 0x8658;
		(* (uint16_t*) (MEMORY_BANK_0_ADDR + 0x08)) = 0x4A;		// GTIN LSB = 0x4A; fw major version = 0
		(* (uint16_t*) (MEMORY_BANK_0_ADDR + 0x0A)) = 0x01;		// fw minor version = 0, MSB id = 0
		(* (uint16_t*) (MEMORY_BANK_0_ADDR + 0x12)) = 0;		// LSB id = 0; hw major version = 0
		(* (uint16_t*) (MEMORY_BANK_0_ADDR + 0x14)) = 0x901; 	// version number: 9 (2.1), hardware version minor: 1
		(* (uint16_t*) (MEMORY_BANK_0_ADDR + 0x16)) = 0x9FF;	// 102 version number: 0xFF, 103 version number: 9 (2.1)
		(* (uint16_t*) (MEMORY_BANK_0_ADDR + 0x18)) = 1;		// 1 control device, 0 control gear on bus
		(* (uint16_t*) (MEMORY_BANK_0_ADDR + 0x1A)) = 0xFF00;	// Index number = 0

		// Implement memory bank 1
		(* (uint16_t*) (MEMORY_BANK_189_ADDR)) = 0xFF16;
		(* (uint16_t*) (MEMORY_BANK_189_ADDR + 0x04)) = (calibrationScale_default << 8) | factoryReset_default;
		(* (uint16_t*) (MEMORY_BANK_189_ADDR + 0x06)) = (pidProportionalCoeff_default << 8) | (calibrationOffset_default);
		(* (uint16_t*) (MEMORY_BANK_189_ADDR + 0x14)) = ((fullScaleRange_default & 0xFF) << 8) | 0xFF;
		(* (uint16_t*) (MEMORY_BANK_189_ADDR + 0x16)) = 0xFF00 | (fullScaleRange_default >> 8);
		dali_NVM_lock();
	}
}

uint32_t erase_page(uint32_t page_address)
{

	if(HAL_FLASH_Unlock() != HAL_OK)
	{
		Error_Handler();
	}
	CLEAR_BIT(FLASH->CR, FLASH_CR_PG);
	FLASH_EraseInitTypeDef flash1 = {FLASH_TYPEERASE_PAGES, page_address, 1};
	uint32_t erase_err = 0;
	__disable_irq();
	HAL_FLASHEx_Erase(&flash1, &erase_err);
	__enable_irq();
	SET_BIT(FLASH->CR, FLASH_CR_PG);
	if(HAL_FLASH_Lock() != HAL_OK)
	{
		Error_Handler();
	}
	return erase_err;
}

void dali_NVM_unlock()
{
	if(HAL_FLASH_Unlock() != HAL_OK)
	{
		Error_Handler();
	}
	SET_BIT(FLASH->CR, FLASH_CR_PG);
}
void dali_NVM_lock()
{
	CLEAR_BIT(FLASH->CR, FLASH_CR_PG);
	if(HAL_FLASH_Lock() != HAL_OK)
	{
		Error_Handler();
	}
}
memory_read_t dali_memory_read(uint8_t memory_bank_number, uint8_t memory_offset)
{
	memory_read_t read;
	uint32_t memory_bank_address = memory_bank_addr[memory_bank_number];
	uint32_t read_address = memory_bank_address + memory_offset;
	if(memory_bank_address == 0) // memory bank is not implemented
	{
		read.success = 0;
	}
	else if((memory_offset > *(uint8_t *) memory_bank_address) || (memory_offset == 1)) // memory bank location is not implemented (not accessible)
	{
		read.success = 0;
	}
	else
	{
		if((memory_bank_number != 0) && (memory_offset == lockByte_addr))
		{
			read.value = lock_byte[memory_bank_number];
		}
		else if((memory_bank_number == 189) && (memory_offset == factoryReset_addr))
		{
			read.value = 0xFF;
		}
		else
		{
			read.value = *(uint8_t *)read_address;
		}
		read.success = 1;
	}
	return read;
}

uint8_t dali_memory_write(uint8_t memory_bank_number, uint8_t memory_offset, uint8_t data)
{
	uint32_t memory_bank_address = memory_bank_addr[memory_bank_number];
	uint32_t write_address = memory_bank_address + memory_offset;
	// Write lock byte
	if((memory_bank_number != 0) && (memory_offset == lockByte_addr))
	{
		lock_byte[memory_bank_number] = data;
		return 0;
	}
	if((memory_bank_number == 189))
	{
		if(memory_offset == factoryReset_addr)
		{
			return (data) ? 0 : 2;
		}
		else if(memory_offset == calibrateDark)
		{
			darkCalibrate = 1;
			return 0;
		}
		else if(memory_offset == calibrateFullScale)
		{
			fullScaleCalibrate = 1;
			return 0;
		}
	}
	// Check if the memory bank location is locked or not implemented
	else if((memory_bank_addr[memory_bank_number] == 0) || (lock_byte[memory_bank_number] != 0x55) || (memory_offset > * (uint8_t *) memory_bank_address) || ((memory_bank_number == 189) && (memory_offset != parameterLock_addr) && (parameterLock!= 0)))
	{
		return 1;
	}
	else
	{
		return 2;
	}
}

// Need to split the write function into 2 separate functions so that the device can response with a backframe without waiting for the memory write
void memory_write(uint8_t memory_bank_number, uint8_t memory_offset, uint8_t data)
{
	uint32_t memory_bank_address = memory_bank_addr[memory_bank_number];
	uint32_t write_address = memory_bank_address + memory_offset;
	if((memory_bank_number == 189) && (memory_offset == factoryReset_addr) && (data == 0))
		dali_memory_reset(189);
	else
	{
		// Add up to 4 memory temporary banks if more banks are implemented in one memory page (1 page = 4 banks)
			// Currently only implement 2 memory banks, each stays on separate memory page so we only need 1 storing array
			uint32_t memory_temp_0[64];
			uint32_t memory_page_addr = (memory_bank_address/0x400)*0x400;
			dali_NVM_unlock();
			for (uint8_t i = 0; i < ((* (uint8_t *) memory_page_addr)/4); i++) // Only save implemented locations in the memory bank
			{
				memory_temp_0[i] = * (uint32_t *) (memory_page_addr + i*4);
			}

			// Erase memory page
			uint32_t erase_err = erase_page(memory_page_addr);
			if (erase_err != 0xFFFFFFFF)
			{
				return 1;
			}

			// Modify the temporary memory with the new data
			uint32_t temp;
			temp = memory_temp_0[(write_address - memory_page_addr)/4];

			switch(write_address % 4)
			{
			case 0:
				temp = (temp & 0xFFFFFF00) | data;
				break;
			case 1:
				temp = (temp & 0xFFFF0000) | (data << 8) | (temp & 0xFF);
				break;
			case 2:
				temp = (temp & 0xFF000000) | (data << 16) | (temp & 0xFFFF);
				break;
			case 3:
				temp = (data << 24) | (temp & 0xFFFFFF);
				break;
			}
			memory_temp_0[(write_address - memory_page_addr)/4] = temp;

			// Write back temporary data to flash
			for (uint8_t i = 0; i < ((* (uint8_t *) memory_page_addr)/4); i++)
			{
				__disable_irq();
				HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, memory_page_addr + i*4, memory_temp_0[i]);
				__enable_irq();
			}
			dali_NVM_lock();
	}
}
void dali_memory_reset(uint8_t memory_bank_number)
{
	if ((memory_bank_number == 0) || (memory_bank_number == 189))
	{
		// return if all bytes are locked
		if (lock_byte[189] != 0x55)
			return;
		dali_NVM_unlock();
		// Currently only 1 memory bank is manufacturer-implemented, so reset all is the same as reset 1 bank
		uint32_t erase_err = erase_page(MEMORY_BANK_189_ADDR);

		// Write default values
		uint32_t dataW = (parameterLock_default << 24) | (lockByte_default << 16) | (indicatorByte << 8) | lastByte_memory_bank_189;
		__disable_irq();
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, MEMORY_BANK_189_ADDR, dataW);
		dataW = (pidProportionalCoeff_default << 24) | (calibrationOffset_default << 16) | (calibrationScale_default << 8) | factoryReset_default;
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, MEMORY_BANK_189_ADDR + 4, dataW);
		dataW = (fullScaleRange_default << 16) | (pidDerivativeCoeff_default << 8) | pidIntegralCoeff_default ;
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, MEMORY_BANK_189_ADDR + 8, dataW);
		__enable_irq();
		dali_NVM_lock();
		lock_byte[189] = 0;
	}
}
