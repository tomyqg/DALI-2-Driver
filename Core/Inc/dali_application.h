/*
 * dali_application.h
 * This file implements the application layer of DALI-2 interface
 * For control devices
 *  Created on: Jul 22, 2020
 *      Author: vtran
 */

#ifndef INC_DALI_APPLICATION_H_
#define INC_DALI_APPLICATION_H_
#include "dali_memory.h"

#define SENSOR_FAILURE				0x01
#define MANUFACTURER_ERROR_1		0x10
#define MANUFACTURER_ERROR_2		0x20
#define MANUFACTURER_ERROR_3		0x40
#define MANUFACTURER_ERROR_4		0x80
// Bit mask for device capabilities
#define CONTROLLER_PRESENT			0x01
#define INSTANCE_PRESENT			0x02
#define CONTROLLER_ALWAYS_ACTIVE	0x04
// Bit mask for device status
#define INPUT_DEVICE_ERROR			0x01
#define QUIESCENT_MODE				0x02
#define SHORT_ADDRESS				0x04
#define APP_ACTIVE					0x08
#define APP_CONTROLLER_ERROR		0x10
#define POWER_CYCLE_SEEN			0x20
#define RESET_STATE					0x40
//Device variables in NVM
#define deviceGroups_NVM						(* (uint32_t*) (MEMORY_NVM_VAR_ADDR + 0))	// each bit represents 1 device group
#define randomAddress_NVM						(* (uint32_t*) (MEMORY_NVM_VAR_ADDR + 4))
#define shortAddress_NVM						(* (uint16_t*) (MEMORY_NVM_VAR_ADDR + 8))	// range from 0 to 63
#define operatingMode_NVM						(* (uint16_t*) (MEMORY_NVM_VAR_ADDR + 10))
#define applicationActive_NVM					(* (uint16_t*) (MEMORY_NVM_VAR_ADDR + 12))
#define powerCycleNotification_NVM				(* (uint16_t*) (MEMORY_NVM_VAR_ADDR + 14))
#define eventPriority_NVM						(* (uint16_t*) (MEMORY_NVM_VAR_ADDR + 16))	// range from 2 to 5

// Device variables in ROM
#define numberOfInstances_NVM					(* (uint16_t*) (MEMORY_ROM_VAR_ADDR + 0))
#define applicationControllerPresent_NVM		(* (uint16_t*) (MEMORY_ROM_VAR_ADDR + 2))
#define applicationControllerAlwaysActive_NVM	(* (uint16_t*) (MEMORY_ROM_VAR_ADDR + 4))
#define versionNumber_NVM						(* (uint16_t*) (MEMORY_ROM_VAR_ADDR + 6))	// 2.1
#define extendedVersionNumber_NVM				(* (uint16_t*) (MEMORY_ROM_VAR_ADDR + 8))	// 2.0

//Instance variables in NVM
#define instanceGroup0_NVM						(* (uint16_t*) (MEMORY_NVM_VAR_ADDR + 18))
#define instanceGroup1_NVM						(* (uint16_t*) (MEMORY_NVM_VAR_ADDR + 20))
#define instanceGroup2_NVM						(* (uint16_t*) (MEMORY_NVM_VAR_ADDR + 22))
#define instanceActive_NVM						(* (uint16_t*) (MEMORY_NVM_VAR_ADDR + 24))
#define eventFilter_NVM							(* (uint16_t*) (MEMORY_NVM_VAR_ADDR + 26))
#define	eventScheme_NVM							(* (uint16_t*) (MEMORY_NVM_VAR_ADDR + 28))

// Instance variables in ROM
#define instanceType_NVM						(* (uint16_t*) (MEMORY_ROM_VAR_ADDR + 10))
#define	resolution_NVM							(* (uint16_t*) (MEMORY_ROM_VAR_ADDR + 12))	// should be 10
#define	instanceNumber_NVM						(* (uint16_t*) (MEMORY_ROM_VAR_ADDR + 14))

// Input device-only variables
#define	tReport_NVM								(* (uint16_t*) (MEMORY_NVM_VAR_ADDR + 30))
#define tDeadtime_NVM							(* (uint16_t*) (MEMORY_NVM_VAR_ADDR + 32))
#define hysteresisMin_NVM						(* (uint16_t*) (MEMORY_NVM_VAR_ADDR + 34))
#define hysteresis_NVM							(* (uint16_t*) (MEMORY_NVM_VAR_ADDR + 36))

typedef struct DALICmdFrame
{
	uint8_t opcode_byte;
	uint8_t instance_byte;
	uint8_t address_byte;
	uint8_t not_care;
} DALICmdFrame_t;

typedef struct DALIEventFrame
{
	uint8_t event_info;
	uint8_t event_source_lsb;
	uint8_t event_source_msb;
	uint8_t not_care;
} DALIEventFrame_t;

enum
{
	DISABLED = 0,
	ENABLED,
	WITHDRAWN		// Only for initialisationState
};

enum
{
	FALSE,
	TRUE
};

enum opcode_app_controller
{
	IDENTIFY_DEVICE 							= 0x00,
	RESET_POWER_CYCLE_SEEN 						= 0x01,
	RESET_VARIABLE								= 0x10,		// This is RESET command in DALI-2 but change to RESET_VARIABLE because there is a conflict with STM library
	RESET_MEMORY_BANK 							= 0x11,
	SET_SHORT_ADDRESS 							= 0X14,
	ENABLE_WRITE_MEMORY 						= 0x15,
	ENABLE_APPLICATION_CONTROLLER 				= 0x16,
	DISABLE_APPLICATION_CONTROLLER 				= 0x17,
	SET_OPERATING_MODE 							= 0x18,
	ADD_TO_DEVICE_GROUPS_0_15 					= 0x19,
	ADD_TO_DEVICE_GROUPS_16_31 					= 0x1A,
	REMOVE_FROM_DEVICE_GROUPS_0_15 				= 0x1B,
	REMOVE_FROM_DEVICE_GROUPS_16_31				= 0x1C,
	START_QUIESCENT_MODE 						= 0x1D,
	STOP_QUIESCENT_MODE 						= 0x1E,
	ENABLE_POWER_CYCLE_NOTIFICATION 			= 0x1F,
	DISABLE_POWER_CYCLE_NOTIFICATION 			= 0x20,
	SAVE_PERSISTENT_VARIABLES 					= 0x21,
	QUERY_DEVICE_STATUS 						= 0x30,
	QUERY_APPLICATION_CONTROLLER_ERROR 			= 0x31,
	QUERY_INPUT_DEVICE_ERROR 					= 0x32,
	QUERY_MISSING_SHORT_ADDRESS 				= 0x33,
	QUERY_VERSION_NUMBER 						= 0x34,
	QUERY_NUMBER_OF_INSTANCES 					= 0x35,
	QUERY_CONTENT_DTR0 							= 0x36,
	QUERY_CONTENT_DTR1 							= 0x37,
	QUERY_CONTENT_DTR2 							= 0x38,
	QUERY_RANDOM_ADDRESS_H 						= 0x39,
	QUERY_RANDOM_ADDRESS_M 						= 0x3A,
	QUERY_RANDOM_ADDRESS_L 						= 0x3B,
	READ_MEMORY_LOCATION 						= 0x3C,
	QUERY_APPLICATION_CONTROLLER_ENABLED 		= 0x3D,
	QUERY_OPERATING_MODE 						= 0x3E,
	QUERY_MANUFACTURER_SPECIFIC_MODE 			= 0x3F,
	QUERY_QUIESCENT_MODE 						= 0x40,
	QUERY_DEVICE_GROUPS_0_7 					= 0x41,
	QUERY_DEVICE_GROUPS_8_15 					= 0x42,
	QUERY_DEVICE_GROUPS_16_23 					= 0x43,
	QUERY_DEVICE_GROUPS_24_31 					= 0x44,
	QUERY_POWER_CYCLE_NOTIFICATION 				= 0x45,
	QUERY_DEVICE_CAPABILITIES 					= 0x46,
	QUERY_EXTENDED_VERSION_NUMBER 				= 0x47,
	QUERY_RESET_STATE 							= 0x48,
	QUERY_APPLICATION_CONTROLLER_ALWAYS_ACTIVE 	= 0x49,
	SET_EVENT_PRIORITY 							= 0x61,
	ENABLE_INSTANCE 							= 0x62,
	DISABLE_INSTANCE 							= 0x63,
	SET_PRIMARY_INSTANCE_GROUP 					= 0x64,
	SET_INSTANCE_GROUP_1 						= 0x65,
	SET_INSTANCE_GROUP_2 						= 0x66,
	SET_EVENT_SCHEME 							= 0x67,
	SET_EVENT_FILTER 							= 0x68,
	QUERY_INSTANCE_TYPE 						= 0x80,
	QUERY_RESOLUTION 							= 0x81,
	QUERY_INSTANCE_ERROR 						= 0x82,
	QUERY_INSTANCE_STATUS 						= 0x83,
	QUERY_EVENT_PRIORITY 						= 0x84,
	QUERY_INSTANCE_ENABLED 						= 0x86,
	QUERY_PRIMARY_INSTANCE_GROUP 				= 0x88,
	QUERY_INSTANCE_GROUP_1 						= 0x89,
	QUERY_INSTANCE_GROUP_2 						= 0x8A,
	QUERY_EVENT_SCHEME 							= 0x8B,
	QUERY_INPUT_VALUE 							= 0x8C,
	QUERY_INPUT_VALUE_LATCH 					= 0x8D,
	QUERY_FEATURE_TYPE 							= 0x8E,
	QUERY_NEXT_FEATURE_TYPE 					= 0x8F,
	QUERY_EVENT_FILTER_0_7 						= 0x90,
	QUERY_EVENT_FILTER_8_15 					= 0x91,
	QUERY_EVENT_FILTER_16_23 					= 0x92
};

enum opcode_input_device_added
{
	SET_REPORT_TIMER 		= 0x30,
	SET_HYSTERESIS 			= 0x31,
	SET_DEADTIME_TIMER 		= 0x32,
	SET_HYSTERESIS_MIN 		= 0x33,
	QUERY_HYSTERESIS_MIN 	= 0x3C,
	QUERY_DEADTIME_TIMER 	= 0x3D,
	QUERY_REPORT_TIMER 		= 0x3E,
	QUERY_HYSTERESIS 		= 0x3F
};

enum special_cmd
{
	TERMINATE							= 0,
	INITIALISE							= 0x01,
	RANDOMISE							= 0x02,
	COMPARE								= 0x03,
	WITHDRAW							= 0x04,
	SEARCHADDRH							= 0x05,
	SEARCHADDRM							= 0x06,
	SEARCHADDRL							= 0x07,
	PROGRAM_SHORT_ADDRESS				= 0x08,
	VERIFY_SHORT_ADDRESS				= 0x09,
	QUERY_SHORT_ADDRESS					= 0x0A,
	WRITE_MEMORY_LOCATION				= 0x20,
	WRITE_MEMORY_LOCATION_NO_REPLY		= 0x21,
	SET_DTR0							= 0x30,
	SET_DTR1							= 0x31,
	SET_DTR2							= 0x32,
	SEND_TESTFRAME						= 0x33,
	DIRECT_WRITE_MEMORY					= 0xC5,
	DTR1_DTR0							= 0xC7,
	DTR2_DTR1							= 0xC9
};

extern union Device_cap device_capabilities;
extern union Device_stat device_status;
// Device variables
extern uint32_t 	searchAddress; // range from 0 to 0xFFFFFF
extern uint8_t 		DTR0;
extern uint8_t 		DTR1;
extern uint8_t 		DTR2;
extern uint8_t	 	quiescentMode;
extern uint8_t		writeEnableState;
extern uint8_t		powerCycleSeen;
extern uint8_t		initialisationState;
extern uint8_t 		applicationControllerError;
extern uint8_t		inputDeviceError;
extern uint8_t		resetState;
extern uint8_t		deviceCapabilities;
extern uint8_t		deviceStatus;

// Instance variables
extern uint16_t		inputValue;
extern uint8_t		instanceError;

// Input device only Variables
extern uint8_t		instanceErrorByte;
extern uint32_t		hysteresisBand;
extern uint32_t		hysteresisBandHigh;
extern uint32_t		hysteresisBandLow;
extern uint16_t 	id_time;

extern volatile uint8_t	quiescent_time;
extern volatile uint8_t	initialise_time;
extern volatile uint16_t report_time;
extern volatile uint16_t dead_time;
extern volatile uint16_t powerNoti_time;
extern volatile uint8_t powerNoti_flag;
// Initialize DALI application
void DALI_AppInit();
// Process Rx data
void DALI_ProcessRxData();
/*
 * Generate Event message
 * In this case, the input device only generate INPUT NOTIFICATION event to report illumination level
 * The event is generated when inputValue is outside of the hysteresis band
 * or after a tReport timeout
 * There are other conditions for event generation such as eventFilter, quiescentMode, tDeadtime,
 * instanceActive, instanceError, applicationActive
 * */
void DALI_SendEvent();
// Reset all variables
void DALI_Reset_Variable();
// Reset memory bank
void DALI_Reset_Memory();
// Save variables to NVM
void DALI_Save_Variable();
// Set inputValue
void DALI_Set_inputValue(uint32_t adcVal);
#endif /* INC_DALI_APPLICATION_H_ */
