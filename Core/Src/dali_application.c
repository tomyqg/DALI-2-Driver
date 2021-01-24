/*
 * dali_application.c
 * This file implements the application layer of DALI-2 interface
 * For control devices
 *  Created on: Jul 22, 2020
 *      Author: vtran
 */

#include "dali.h"
#include "dali_application.h"

#define BLANK_8  0xFF
#define BLANK_16 0xFFFF
#define BLANK_32 0xFFFFFFFF

// Device variables
uint32_t 	searchAddress 						= 0xFFFFFF; // range from 0 to 0xFFFFFF
uint8_t 	DTR0 								= 0;
uint8_t 	DTR1 								= 0;
uint8_t 	DTR2 								= 0;
uint8_t 	quiescentMode						= DISABLED;
uint8_t		writeEnableState					= DISABLED;
uint8_t		powerCycleSeen						= TRUE;
uint8_t		initialisationState 				= DISABLED;
uint8_t 	applicationControllerError 			= FALSE;
uint8_t		inputDeviceError					= FALSE;
uint8_t		resetState							= TRUE;
uint32_t 	deviceGroups;
uint32_t	randomAddress;
uint16_t 	shortAddress;
uint16_t 	numberOfInstances;
uint16_t 	operatingMode;
uint16_t 	applicationActive;
uint16_t 	applicationControllerPresent;
uint16_t 	applicationControllerAlwaysActive;
uint16_t 	powerCycleNotification;
uint16_t 	eventPriority;// range from 2 to 5
uint16_t	versionNumber;	// 2.1
uint16_t 	extendedVersionNumber;
uint8_t 	deviceCapabilities					= 0;
uint8_t		deviceStatus						= 0;

// Instance variables
uint16_t	inputValue;	// maximum = (2^(N*8)-1) where N = round(resolution/8)
uint8_t		instanceError		= FALSE;
uint16_t 	instanceGroup0;
uint16_t 	instanceGroup1;
uint16_t 	instanceGroup2;
uint16_t 	instanceActive;
uint16_t	resolution;	// should be 10
uint16_t	eventFilter;
uint16_t	instanceNumber;
uint16_t	instanceType;
uint16_t	eventScheme;

// Input device only variables
uint8_t		instanceErrorByte		= 0;
uint32_t	hysteresisBand			= 0;
uint32_t	hysteresisBandHigh		= 0;
uint32_t	hysteresisBandLow		= 0;
uint16_t	tReport;
uint16_t 	tDeadtime;
uint16_t 	hysteresisMin			= 10;
uint16_t 	hysteresis				= 5;
uint16_t	id_time					= 0;
uint8_t 	isSecondFrame			= 0;
uint8_t		debug=0;

volatile uint8_t quiescent_time;
volatile uint8_t initialise_time;
volatile uint16_t report_time;
volatile uint16_t dead_time;
volatile uint16_t powerNoti_time = 0;
volatile uint8_t powerNoti_flag = 0;

// Private variables
uint32_t 	previousFrame;
uint32_t 	inputValue_latch;
uint8_t  	inputValue_byte;
uint16_t 	illuminance = 0;
uint32_t	fullFrame = 0;
uint8_t		backFrame = 0;
uint16_t 	inputValue_10b = 0;
// Private functions
void DALI_Reset_Variables();
void DALI_Save_Variable();
void DALI_Send_PowerCycleEvent();
void DALI_Check_ResetState();

void DALI_AppInit()
{
	DALIInit();
	dali_memory_init();

	// Set up NVM variables with default value if it is the first time power up
	dali_NVM_unlock();
	if(deviceGroups_NVM == BLANK_32)
	{
		(* (uint16_t*) (MEMORY_NVM_VAR_ADDR + 0)) = 0;
		(* (uint16_t*) (MEMORY_NVM_VAR_ADDR + 2)) = 0;
	}
	if(randomAddress_NVM == BLANK_32)
	{
		(* (uint16_t*) (MEMORY_NVM_VAR_ADDR + 4)) 	= 0xFFFF;
		(* (uint16_t*) (MEMORY_NVM_VAR_ADDR + 6)) 	= 0xFF;
	}
	if(shortAddress_NVM == BLANK_16)
		shortAddress_NVM = BLANK_8;
	if(operatingMode_NVM == BLANK_16)
		operatingMode_NVM = 0;
	if(applicationActive_NVM == BLANK_16)
		applicationActive_NVM = FALSE;
	if(powerCycleNotification_NVM == BLANK_16)
		powerCycleNotification_NVM = DISABLED;
	if(eventPriority_NVM == BLANK_16)
		eventPriority_NVM = 4;

	if(numberOfInstances_NVM == BLANK_16)
		numberOfInstances_NVM = 1;
	if(applicationControllerPresent_NVM == BLANK_16)
		applicationControllerPresent_NVM = FALSE;
	if(applicationControllerAlwaysActive_NVM == BLANK_16)
		applicationControllerAlwaysActive_NVM = FALSE;
	if(versionNumber_NVM == BLANK_16)
		versionNumber_NVM = 9;
	if(extendedVersionNumber_NVM == BLANK_16)
		extendedVersionNumber_NVM = 8;
	if(instanceGroup0_NVM == BLANK_16)
		instanceGroup0_NVM = BLANK_8;
	if(instanceGroup1_NVM == BLANK_16)
		instanceGroup1_NVM = BLANK_8;
	if(instanceGroup2_NVM == BLANK_16)
		instanceGroup2_NVM = BLANK_8;
	if(instanceActive_NVM == BLANK_16)
		instanceActive_NVM = TRUE;
	if(eventFilter_NVM == BLANK_16)
		eventFilter_NVM = 1;
	if(eventScheme_NVM == BLANK_16)
		eventScheme_NVM = 0;

	if(instanceType_NVM == BLANK_16)
		instanceType_NVM = 4;
	if(resolution_NVM == BLANK_16)
		resolution_NVM = 10;
	if(instanceNumber_NVM == BLANK_16)
		instanceNumber_NVM = 1;

	if(tReport_NVM == BLANK_16)
		tReport_NVM = 30;
	if(tDeadtime_NVM == BLANK_16)
		tDeadtime_NVM = 30;
	if(hysteresisMin_NVM == BLANK_16)
		hysteresisMin_NVM = 10;
	if(hysteresis_NVM == BLANK_16)
		hysteresis_NVM = 5;
	dali_NVM_lock();

	// Set power on value
	shortAddress 							= shortAddress_NVM;
	deviceGroups 							= deviceGroups_NVM;
	searchAddress 							= 0xFFFFFF;
	randomAddress 							= randomAddress_NVM;
	numberOfInstances				 		= numberOfInstances_NVM;
	operatingMode 							= operatingMode_NVM;
	applicationActive 						= applicationActive_NVM;
	applicationControllerPresent 			= applicationControllerPresent_NVM;
	applicationControllerAlwaysActive 		= applicationControllerAlwaysActive_NVM;
	powerCycleNotification 					= powerCycleNotification_NVM;
	eventPriority 							= eventPriority_NVM;
	versionNumber 							= versionNumber_NVM;
	instanceGroup0 							= instanceGroup0_NVM;
	instanceGroup1 							= instanceGroup1_NVM;
	instanceGroup2 							= instanceGroup2_NVM;
	instanceActive 							= instanceActive_NVM;
	instanceType 							= instanceType_NVM;
	resolution 								= resolution_NVM;
	instanceNumber 							= instanceNumber_NVM;
	eventFilter 							= eventFilter_NVM;
	eventScheme 							= eventScheme_NVM;
	quiescent_time 							= 0;
	initialise_time 						= 0;
	tReport									= tReport_NVM;
	tDeadtime								= tDeadtime_NVM;
	hysteresisMin							= hysteresisMin_NVM;
	hysteresis								= hysteresis_NVM;
	if(powerCycleNotification == ENABLED)
	{
		powerNoti_time = 1200;
	}
	DALIConfigureMode(applicationActive);
}

void DALI_ProcessRxData()
{
	uint32_t frame;
	DALICmdFrame_t * cmd;
	DALIEventFrame_t * event;
	if(DALIDataAvailable())
	{
		DALIRxData_t msg = DALIReceiveData();
		DALIRxData_t * msg_ptr = &msg;
		switch(debug)
		{
		case 0:
			debug = 1;
			break;
		case 1:
			debug = 2;
			break;
		case 2:
			debug = 3;
			break;
		case 3:
			debug = 4;
			break;
		case 4:
			debug = 5;
			break;
		case 5:
			debug = 6;
			break;
		case 6:
			debug = 0;
			break;
		default:
			break;
		}
		if ((msg_ptr->rxDone == 1) && (msg_ptr->rxError == 0))
		{
#ifdef CONTROLLER
			fullFrame = msg_ptr->frame;
#endif
			if(msg_ptr->frameType == 0)  // Forward frame
			{
				frame = msg_ptr->frame;
				// Check bit 16 to see if this a command frame or event frame
				if((frame & 0x010000) > 0)  // Command frame
				{
					cmd = &frame;
					uint8_t memory_related = 0;

					// Check if this contains legit address
					if (cmd->address_byte < 0x80) // Bit 23 = 0 -> Short addressing
					{
						if (cmd->address_byte != shortAddress*2 + 1) //legit address
						{
							return;
						}
					}
					else if(cmd->address_byte < 0xC0) // Bit 22 = 0 -> Device group addressing
					{
						uint8_t group = (cmd->address_byte >> 1) & 0x1F;
						if ((deviceGroups && (1 << group)) == 0)
						{
							return;
						}
					}
					else if(cmd->address_byte == 0xFD) // Broadcast unaddressed
					{
						if(shortAddress != 0xFF)
						{
							return;
						}
					}
					else if((cmd->address_byte > 0xE0) && (cmd->address_byte < 0xFD)) // Reserved addressing
					{
						return;
					}
					else if(cmd->address_byte == 0xC1)	// Special command
					{
						switch(cmd->instance_byte)
						{
						case TERMINATE:
							if(cmd->opcode_byte == 0)
							{
								initialisationState = DISABLED;
								initialise_time = 0;
							}
							break;
						case INITIALISE:
							if(frame != previousFrame)
							{
								DALIReceiveTwice();
							}
							else
							{
								if(msg_ptr->rxSendTwicePossible == 1)
								{
									if(((cmd->opcode_byte == 0x7F) && (shortAddress == 0xFF)) || (cmd->opcode_byte == 0xFF) || ((cmd->opcode_byte < 64) && (cmd->opcode_byte == shortAddress)))
									{
										initialisationState = ENABLED;
										initialise_time = 15; // 15 min
									}
								}
								isSecondFrame = 1;
							}
							break;
						case RANDOMISE:
							if(frame != previousFrame)
							{
								DALIReceiveTwice();
							}
							else
							{
								if((msg_ptr->rxSendTwicePossible == 1) && (initialisationState != DISABLED) && (cmd->opcode_byte == 0))
								{
									randomAddress = get_timer_count(&htim6)*250;
									DALI_Save_Variable();
									if(randomAddress != 0xFFFFFF)
										resetState = FALSE;
								}
								isSecondFrame = 1;
							}
							break;
						case COMPARE:
							if((initialisationState == ENABLED) && (randomAddress <= searchAddress) && (cmd->opcode_byte == 0))
							{
								DALITxData_t data = {0xFF, 1, 0, 1};
								DALISendData(data);
							}
							break;
						case WITHDRAW:
							if((initialisationState == ENABLED) && (randomAddress == searchAddress) && (cmd->opcode_byte == 0))
							{
								initialisationState = WITHDRAWN;
							}
							break;
						case SEARCHADDRH:
							if(initialisationState != DISABLED)
							{
								searchAddress = ((cmd->opcode_byte & 0xFF) << 16) | (searchAddress & 0xFFFF);
							}
							break;
						case SEARCHADDRM:
							if(initialisationState != DISABLED)
							{
								searchAddress = ((cmd->opcode_byte & 0xFF) << 8) | (searchAddress & 0xFF00FF);
							}
							break;
						case SEARCHADDRL:
							if(initialisationState != DISABLED)
							{
								searchAddress = (cmd->opcode_byte & 0xFF) | (searchAddress & 0xFFFF00);
							}
							break;
						case PROGRAM_SHORT_ADDRESS:
							if((initialisationState != DISABLED) && (randomAddress == searchAddress) && (cmd->opcode_byte < 64))
							{
								shortAddress = cmd->opcode_byte;
								DALI_Save_Variable();
							}
							break;
						case VERIFY_SHORT_ADDRESS:
							if((initialisationState != DISABLED) && (shortAddress == cmd->opcode_byte))
							{
								DALITxData_t data = {0xFF, 1, 0, 1};
								DALISendData(data);
							}
							break;
						case QUERY_SHORT_ADDRESS:
							if((initialisationState != DISABLED) && (randomAddress == searchAddress) && (cmd->opcode_byte == 0))
							{
								DALITxData_t data = {shortAddress, 1, 0, 1};
								DALISendData(data);
							}
							break;
						case WRITE_MEMORY_LOCATION:
							memory_related = 1;
							if(writeEnableState == ENABLED)
							{

								uint8_t error = dali_memory_write(DTR1, DTR0, cmd->opcode_byte);
								if (error != 1)
								{
									DALITxData_t data = {cmd->opcode_byte, 1, 0, 1};
									DALISendData(data);
									HAL_Delay(20);
									if(error == 2)
										memory_write(DTR1, DTR0, cmd->opcode_byte);
								}
								if((DTR0 < 0xFF) && (DTR1 == 189))
									DTR0++;
							}
							break;
						case WRITE_MEMORY_LOCATION_NO_REPLY:
							memory_related = 1;
							if(writeEnableState == ENABLED)
							{
								uint8_t error = dali_memory_write(DTR1, DTR0, cmd->opcode_byte);
								if(error == 2)
									memory_write(DTR1, DTR0, cmd->opcode_byte);
								if((DTR0 < 0xFF) && (DTR1 == 189))
									DTR0++;
							}
							break;
						case SET_DTR0:
							memory_related = 1;
							DTR0 = cmd->opcode_byte;
							break;
						case SET_DTR1:
							memory_related = 1;
							DTR1 = cmd->opcode_byte;
							break;
						case SET_DTR2:
							memory_related = 1;
							DTR2 = cmd->opcode_byte;
							break;
						case SEND_TESTFRAME:
							if ((cmd->opcode_byte > 0x7F) && ((cmd->opcode_byte & 0x07) <= 5) && ((cmd->opcode_byte & 0x07) >= 1)\
									 && (((cmd->opcode_byte & 0x20) == 0) || (applicationControllerPresent == TRUE)))
							{
								uint32_t temp;
								if((cmd->opcode_byte & 0x20) == 0)
								{
									temp = (DTR0 << 16) | (DTR1 << 8) | DTR2;
								}
								else
								{
									temp = (DTR0 << 8) | DTR1;
								}
								uint8_t priority = cmd->opcode_byte & 0x07;
								uint8_t repeat	 = (cmd->opcode_byte >> 3) & 0x03;
								DALITxData_t data = {temp, 0, 0, priority};
								DALISendData(data);
								priority = ((cmd->opcode_byte & 0x40) == 1) ? 1 : 0;
								data.priority = priority;
								while(repeat > 0)
								{
									DALISendData(data);
									repeat--;
								}
							}
							break;
						}
						previousFrame = (isSecondFrame) ? 0 : frame;
						isSecondFrame = 0;
						return;
					}
					else if (cmd->address_byte == DIRECT_WRITE_MEMORY)
					{
						memory_related = 1;
						if(writeEnableState == ENABLED)
						{
							DTR0 = cmd->instance_byte;
							uint8_t error = dali_memory_write(DTR1, DTR0, cmd->opcode_byte);
							if (error != 1)
							{
								DALITxData_t data = {cmd->opcode_byte, 1, 0, 1};
								DALISendData(data);
								HAL_Delay(20);
								if(error == 2)
									memory_write(DTR1, DTR0, cmd->opcode_byte);
							}
							if((DTR0 < 0xFF) && (DTR1 == 189))
								DTR0++;
						}
						previousFrame = frame;
						return;
					}
					else if (cmd->address_byte == DTR1_DTR0)
					{
						memory_related = 1;
						DTR1 = cmd->instance_byte;
						DTR0 = cmd->opcode_byte;
						previousFrame = frame;
						return;
					}
					else if (cmd->address_byte == DTR2_DTR1)
					{
						memory_related = 1;
						DTR2 = cmd->instance_byte;
						DTR1 = cmd->opcode_byte;
						previousFrame = frame;
						return;
					}

					// Check instance byte
					if(cmd->instance_byte == 0xFE) // Device commands
					{
						switch(cmd->opcode_byte)
						{
						case IDENTIFY_DEVICE:
							if(frame != previousFrame)
							{
								DALIReceiveTwice();
							}
							else
							{
								writePin(LED_Pin, 0);
								id_time = 10000;
								isSecondFrame = 1;
							}
							break;
						case RESET_POWER_CYCLE_SEEN:
							if(frame != previousFrame)
							{
								DALIReceiveTwice();
							}
							else
							{
								if(msg_ptr->rxSendTwicePossible == 1)
								{
									powerCycleSeen = FALSE;
								}
								isSecondFrame = 1;
							}
							break;
						case RESET_VARIABLE:
							if(frame != previousFrame)
							{
								DALIReceiveTwice();
							}
							else
							{
								if(msg_ptr->rxSendTwicePossible == 1)
								{
									DALI_Reset_Variables();
								}
								isSecondFrame = 1;
							}
							break;
						case RESET_MEMORY_BANK:
							dali_memory_reset(DTR0);
							break;
						case SET_SHORT_ADDRESS:
							if(frame != previousFrame)
							{
								DALIReceiveTwice();
							}
							else
							{
								if(msg_ptr->rxSendTwicePossible == 1)
								{
									if((DTR0 == 0xFF) || (DTR0 < 0x40))
									{
										shortAddress = DTR0;
										DALI_Save_Variable();
									}
								}
								isSecondFrame = 1;
							}
							break;
						case ENABLE_WRITE_MEMORY:
							if(frame != previousFrame)
							{
								DALIReceiveTwice();
							}
							else
							{
								if(msg_ptr->rxSendTwicePossible == 1)
								{
									writeEnableState = ENABLED;
									memory_related = 1;
								}
								isSecondFrame = 1;
							}
							break;
						case ENABLE_APPLICATION_CONTROLLER:
							if(frame != previousFrame)
							{
								DALIReceiveTwice();
							}
							else
							{
								if(msg_ptr->rxSendTwicePossible == 1)
								{
									if(applicationControllerPresent == TRUE)
									{
										applicationActive = TRUE;
										DALIConfigureMode(applicationActive);
										DALI_Save_Variable();
									}
								}
								isSecondFrame = 1;
							}
							break;
						case DISABLE_APPLICATION_CONTROLLER:
							if(frame != previousFrame)
							{
								DALIReceiveTwice();
							}
							else
							{
								if(msg_ptr->rxSendTwicePossible == 1)
								{
									if((applicationControllerAlwaysActive == FALSE) && (applicationControllerPresent == TRUE))
									{
										applicationActive = FALSE;
										DALIConfigureMode(applicationActive);
										DALI_Save_Variable();
									}
								}
								isSecondFrame = 1;
							}
							break;
						case SET_OPERATING_MODE:
							if(frame != previousFrame)
							{
								DALIReceiveTwice();
							}
							else
							{
								// Ignore because there is only 1 operating mode
//								if(msg_ptr->rxSendTwicePossible == 1)
//								{
//									operatingMode = DTR0;
//									DALI_Save_Variable();
//								}
								isSecondFrame = 1;
							}
							break;
						case ADD_TO_DEVICE_GROUPS_0_15:
							if(frame != previousFrame)
							{
								DALIReceiveTwice();
							}
							else
							{
								if(msg_ptr->rxSendTwicePossible == 1)
								{
									deviceGroups = deviceGroups|(DTR2 << 8)|DTR1;
									DALI_Save_Variable();
									if(deviceGroups != 0)
										resetState = FALSE;
								}
								isSecondFrame = 1;
							}
							break;
						case ADD_TO_DEVICE_GROUPS_16_31:
							if(frame != previousFrame)
							{
								DALIReceiveTwice();
							}
							else
							{
								if(msg_ptr->rxSendTwicePossible == 1)
								{
									deviceGroups = deviceGroups|(DTR2 << 24)|(DTR1 << 16);
									DALI_Save_Variable();
									if(deviceGroups != 0)
										resetState = FALSE;
								}
								isSecondFrame = 1;
							}
							break;
						case REMOVE_FROM_DEVICE_GROUPS_0_15:
							if(frame != previousFrame)
							{
								DALIReceiveTwice();
							}
							else
							{
								if(msg_ptr->rxSendTwicePossible == 1)
								{
									deviceGroups = deviceGroups & (~(DTR2 << 8)) & (~DTR1);
									DALI_Save_Variable();
									if(deviceGroups != 0)
										resetState = FALSE;
								}
								isSecondFrame = 1;
							}
							break;
						case REMOVE_FROM_DEVICE_GROUPS_16_31:
							if(frame != previousFrame)
							{
								DALIReceiveTwice();
							}
							else
							{
								if(msg_ptr->rxSendTwicePossible == 1)
								{
									deviceGroups = deviceGroups & (~(DTR2 << 24)) & (~(DTR1 << 16));
									DALI_Save_Variable();
									if(deviceGroups != 0)
										resetState = FALSE;
								}
								isSecondFrame = 1;
							}
							break;
						case START_QUIESCENT_MODE:
							if(frame != previousFrame)
							{
								DALIReceiveTwice();
							}
							else
							{
								if(msg_ptr->rxSendTwicePossible == 1)
								{
									quiescentMode = ENABLED;
									quiescent_time = 15; // 15 min
								}
								isSecondFrame = 1;
							}
							break;
						case STOP_QUIESCENT_MODE:
							if(frame != previousFrame)
							{
								DALIReceiveTwice();
							}
							else
							{
								if(msg_ptr->rxSendTwicePossible == 1)
								{
									quiescentMode = DISABLED;
									quiescent_time = 0;
								}
								isSecondFrame = 1;
							}
							break;
						case ENABLE_POWER_CYCLE_NOTIFICATION:
							if(frame != previousFrame)
							{
								DALIReceiveTwice();
							}
							else
							{
								if(msg_ptr->rxSendTwicePossible == 1)
								{
									powerCycleNotification = ENABLED;
									DALI_Save_Variable();
								}
								isSecondFrame = 1;
							}
							break;
						case DISABLE_POWER_CYCLE_NOTIFICATION:
							if(frame != previousFrame)
							{
								DALIReceiveTwice();
							}
							else
							{
								if(msg_ptr->rxSendTwicePossible == 1)
								{
									powerCycleNotification = DISABLED;
									DALI_Save_Variable();
								}
								isSecondFrame = 1;
							}
							break;
						case SAVE_PERSISTENT_VARIABLES:
							if(frame != previousFrame)
							{
								DALIReceiveTwice();
							}
							else
							{
								if(msg_ptr->rxSendTwicePossible == 1)
								{
									DALI_Save_Variable();
								}
								isSecondFrame = 1;
							}
							break;
						case QUERY_DEVICE_STATUS:
							if(applicationActive)
							{
								SET_BIT(deviceStatus, APP_ACTIVE);
							}
							else
							{
								CLEAR_BIT(deviceStatus, APP_ACTIVE);
							}

							if(inputDeviceError)
							{
								SET_BIT(deviceStatus, INPUT_DEVICE_ERROR);
							}
							else
							{
								CLEAR_BIT(deviceStatus, INPUT_DEVICE_ERROR);
							}

							if(quiescentMode)
							{
								SET_BIT(deviceStatus, QUIESCENT_MODE);
							}
							else
							{
								CLEAR_BIT(deviceStatus, QUIESCENT_MODE);
							}

							if(shortAddress == 0xFF)
							{
								SET_BIT(deviceStatus, SHORT_ADDRESS);
							}
							else
							{
								CLEAR_BIT(deviceStatus, SHORT_ADDRESS);
							}

							if(applicationControllerError)
							{
								SET_BIT(deviceStatus, APP_CONTROLLER_ERROR);
							}
							else
							{
								CLEAR_BIT(deviceStatus, APP_CONTROLLER_ERROR);
							}

							if(powerCycleSeen)
							{
								SET_BIT(deviceStatus, POWER_CYCLE_SEEN);
							}
							else
							{
								CLEAR_BIT(deviceStatus, POWER_CYCLE_SEEN);
							}

							if(resetState)
							{
								SET_BIT(deviceStatus, RESET_STATE);
							}
							else
							{
								CLEAR_BIT(deviceStatus, RESET_STATE);
							}

							DALITxData_t data = {(uint32_t) deviceStatus, 1, 0, 1};
							DALISendData(data);
							break;
						case QUERY_DEVICE_CAPABILITIES:
						{
							if (applicationControllerAlwaysActive)
							{
								SET_BIT(deviceCapabilities, CONTROLLER_ALWAYS_ACTIVE);
							}
							else
							{
								CLEAR_BIT(deviceCapabilities, CONTROLLER_ALWAYS_ACTIVE);
							}
							if (applicationControllerPresent)
							{
								SET_BIT(deviceCapabilities, CONTROLLER_PRESENT);
							}
							else
							{
								CLEAR_BIT(deviceCapabilities, CONTROLLER_PRESENT);
							}
							if (numberOfInstances > 0)
							{
								SET_BIT(deviceCapabilities, INSTANCE_PRESENT);
							}
							else
							{
								CLEAR_BIT(deviceCapabilities, INSTANCE_PRESENT);
							}
							DALITxData_t data = {(uint32_t) deviceCapabilities, 1, 0, 1};
							DALISendData(data);
						}
							break;
						case QUERY_APPLICATION_CONTROLLER_ERROR:
							break;
						case QUERY_INPUT_DEVICE_ERROR:
							if(instanceError != 0)
							{
								DALITxData_t data = {instanceError, 1, 0, 1};
								DALISendData(data);
							}
							break;
						case QUERY_MISSING_SHORT_ADDRESS:
							if (shortAddress == 0xFF)
							{
								DALITxData_t data = {0xFF, 1, 0, 1};
								DALISendData(data);
							}
							break;
						case QUERY_VERSION_NUMBER:;
							memory_read_t version_number = dali_memory_read(0, 0x17);
							if(version_number.success == 1)
							{
								DALITxData_t data = {version_number.value, 1, 0, 1};
								DALISendData(data);
							}
							break;
						case QUERY_CONTENT_DTR0:
						{
							memory_related = 1;
							DALITxData_t data = {DTR0, 1, 0, 1};
							DALISendData(data);
						}
							break;
						case QUERY_CONTENT_DTR1:
						{
							memory_related = 1;
							DALITxData_t data = {DTR1, 1, 0, 1};
							DALISendData(data);
						}
							break;
						case QUERY_CONTENT_DTR2:
						{
							memory_related = 1;
							DALITxData_t data = {DTR2, 1, 0, 1};
							DALISendData(data);
						}
							break;
						case QUERY_NUMBER_OF_INSTANCES:
						{
							DALITxData_t data = {numberOfInstances, 1, 0, 1};
							DALISendData(data);
						}
							break;
						case QUERY_RANDOM_ADDRESS_H:
						{
							DALITxData_t data = {(randomAddress >> 16) & 0xFF, 1, 0, 1};
							DALISendData(data);
						}
							break;
						case QUERY_RANDOM_ADDRESS_M:
						{
							DALITxData_t data = {(randomAddress >> 8) & 0xFF, 1, 0, 1};
							DALISendData(data);
						}
							break;
						case QUERY_RANDOM_ADDRESS_L:
						{
							DALITxData_t data = {randomAddress & 0xFF, 1, 0, 1};
							DALISendData(data);
						}
							break;
						case READ_MEMORY_LOCATION:;
						{
							memory_read_t read = dali_memory_read(DTR1, DTR0);
							if(read.success == 1)
							{
								DALITxData_t data = {read.value, 1, 0, 1};
								DALISendData(data);
								if(DTR0 < 0xFF)
									DTR0++;
							}
							else
							{
								if((DTR1 == 0) || (DTR1 == 189))
									DTR0++;
							}
						}
							break;
						case QUERY_APPLICATION_CONTROLLER_ENABLED:
							if(applicationActive)
							{
								DALITxData_t data = {0xFF, 1, 0, 1};
								DALISendData(data);
							}
							break;
						case QUERY_OPERATING_MODE:
						{
							DALITxData_t data = {operatingMode, 1, 0, 1};
							DALISendData(data);
						}
							break;
						case QUERY_MANUFACTURER_SPECIFIC_MODE:
							if(operatingMode > 0x80)
							{
								DALITxData_t data = {0xFF, 1, 0, 1};
								DALISendData(data);
							}
							break;
						case QUERY_QUIESCENT_MODE:
							if(quiescentMode)
							{
								DALITxData_t data = {0xFF, 1, 0, 1};
								DALISendData(data);
							}
							break;
						case QUERY_DEVICE_GROUPS_0_7:
						{
							DALITxData_t data = {deviceGroups & 0xFF, 1, 0, 1};
							DALISendData(data);
						}
							break;
						case QUERY_DEVICE_GROUPS_8_15:
						{
							DALITxData_t data = {(deviceGroups >> 8) & 0xFF, 1, 0, 1};
							DALISendData(data);
						}
							break;
						case QUERY_DEVICE_GROUPS_16_23:
						{
							DALITxData_t data = {(deviceGroups >> 16) & 0xFF, 1, 0, 1};
							DALISendData(data);
						}
							break;
						case QUERY_DEVICE_GROUPS_24_31:
						{
							DALITxData_t data = {(deviceGroups >> 24) & 0xFF, 1, 0, 1};
							DALISendData(data);
						}
							break;
						case QUERY_POWER_CYCLE_NOTIFICATION:
							if(powerCycleNotification)
							{
								DALITxData_t data = {0xFF, 1, 0, 1};
								DALISendData(data);
							}
							break;
						case QUERY_EXTENDED_VERSION_NUMBER:
							if(DTR0 == 4)
							{
								DALITxData_t data = {extendedVersionNumber, 1, 0, 1};
								DALISendData(data);
							}
							break;
						case QUERY_RESET_STATE:
							DALI_Check_ResetState();
							if(resetState)
							{
								DALITxData_t data = {0xFF, 1, 0, 1};
								DALISendData(data);
							}
							break;
						case QUERY_APPLICATION_CONTROLLER_ALWAYS_ACTIVE:
							if(applicationControllerAlwaysActive)
							{
								DALITxData_t data = {0xFF, 1, 0, 1};
								DALISendData(data);
							}
							break;
						case SET_EVENT_PRIORITY:
							if(frame != previousFrame)
							{
								DALIReceiveTwice();
							}
							else
							{
								if(msg_ptr->rxSendTwicePossible == 1)
								{
									if((DTR0 > 1) && (DTR0 < 6))
									{
										eventPriority = DTR0;
										DALI_Save_Variable();
										if(eventPriority != 4)
											resetState = FALSE;
									}
								}
								isSecondFrame = 1;
							}
							break;
						case QUERY_EVENT_PRIORITY:
						{
							DALITxData_t data = {eventPriority, 1, 0, 1};
							DALISendData(data);
						}
							break;
						case QUERY_FEATURE_TYPE:
							break;
						case QUERY_NEXT_FEATURE_TYPE:
							break;
						default:
							// Discard message
							break;
						}
					}
					else if((cmd->instance_byte == 0xFF) || (cmd->instance_byte == instanceNumber) || (cmd->instance_byte == 0xC0 + instanceType) 	\
							|| ((instanceGroup0 < 0xFF) && (cmd->instance_byte == instanceGroup0 + 0x80))			\
							|| ((instanceGroup1 < 0xFF) && (cmd->instance_byte == instanceGroup2 + 0x80))			\
							|| ((instanceGroup2 < 0xFF) && (cmd->instance_byte == instanceGroup1 + 0x80)))			\
					// Instance command
					{
						switch(cmd->opcode_byte)
						{
						case ENABLE_INSTANCE:
							if(frame != previousFrame)
							{
								DALIReceiveTwice();
							}
							else
							{
								if(msg_ptr->rxSendTwicePossible == 1)
								{
									instanceActive = TRUE;
									DALI_Save_Variable();
								}
								isSecondFrame = 1;
							}
							break;
						case DISABLE_INSTANCE:
							if(frame != previousFrame)
							{
								DALIReceiveTwice();
							}
							else
							{
								if(msg_ptr->rxSendTwicePossible == 1)
								{
									instanceActive = FALSE;
									DALI_Save_Variable();
								}
								isSecondFrame = 1;
							}
							break;
						case SET_PRIMARY_INSTANCE_GROUP:
							if(frame != previousFrame)
							{
								DALIReceiveTwice();
							}
							else
							{
								if(msg_ptr->rxSendTwicePossible == 1)
								{
									if((DTR0 < 32) || (DTR0 == 0xFF))
									{
										instanceGroup0 = DTR0;
										DALI_Save_Variable();
										if(instanceGroup0 != 0xFF)
											resetState = FALSE;
									}

								}
								isSecondFrame = 1;
							}
							break;
						case SET_INSTANCE_GROUP_1:
							if(frame != previousFrame)
							{
								DALIReceiveTwice();
							}
							else
							{
								if(msg_ptr->rxSendTwicePossible == 1)
								{
									if((DTR0 < 32) || (DTR0 == 0xFF))
									{
										instanceGroup1 = DTR0;
										DALI_Save_Variable();
										if(instanceGroup1 != 0xFF)
											resetState = FALSE;
									}
								}
								isSecondFrame = 1;
							}
							break;
						case SET_INSTANCE_GROUP_2:
							if(frame != previousFrame)
							{
								DALIReceiveTwice();
							}
							else
							{
								if(msg_ptr->rxSendTwicePossible == 1)
								{
									if((DTR0 < 32) || (DTR0 == 0xFF))
									{
										instanceGroup2 = DTR0;
										DALI_Save_Variable();
										if(instanceGroup2 != 0xFF)
											resetState = FALSE;
									}
								}
								isSecondFrame = 1;
							}
							break;
						case SET_EVENT_PRIORITY:
							if(frame != previousFrame)
							{
								DALIReceiveTwice();
							}
							else
							{
								if(msg_ptr->rxSendTwicePossible == 1)
								{
									if((DTR0 > 1) && (DTR0 < 6))
									{
										eventPriority = DTR0;
										DALI_Save_Variable();
										if(eventPriority != 4)
											resetState = FALSE;
									}
								}
								isSecondFrame = 1;
							}
							break;
						case SET_EVENT_SCHEME:
							if(frame != previousFrame)
							{
								DALIReceiveTwice();
							}
							else
							{
								if(msg_ptr->rxSendTwicePossible == 1)
								{
									if(DTR0 < 5)
									{
										eventScheme = DTR0;
										DALI_Save_Variable();
										if(eventScheme != 0)
											resetState = FALSE;
									}

								}
								isSecondFrame = 1;
							}
							break;
						case SET_EVENT_FILTER:
							if(frame != previousFrame)
							{
								DALIReceiveTwice();
							}
							else
							{
								if(msg_ptr->rxSendTwicePossible == 1)
								{
									if(applicationActive)
									{
										eventFilter = (DTR2 << 16) | (DTR1 << 8) | DTR0;
										DALI_Save_Variable();
										if(eventFilter != 0xFFFFFF)
											resetState = FALSE;
									}
									else
									{
										if(DTR0 < 2)
										{
											eventFilter = DTR0;
											DALI_Save_Variable();
										}
										if(eventFilter != 1)
											resetState = FALSE;
									}
								}
								isSecondFrame = 1;
							}
							break;
						case QUERY_INSTANCE_TYPE:
						{
							DALITxData_t data = {instanceType, 1, 0, 1};
							DALISendData(data);
						}
							break;
						case QUERY_RESOLUTION:
						{
							DALITxData_t data = {resolution, 1, 0, 1};
							DALISendData(data);
						}
							break;
						case QUERY_INSTANCE_STATUS:;
						{
							uint8_t temp = (instanceError << 7) | (instanceActive << 6);
							DALITxData_t data = {temp, 1, 0, 1};
							DALISendData(data);
						}
							break;
						case QUERY_INSTANCE_ENABLED:
							if(instanceActive == TRUE)
							{
								DALITxData_t data = {0xFF, 1, 0, 1};
								DALISendData(data);
							}
							break;
						case QUERY_PRIMARY_INSTANCE_GROUP:;
						{
							DALITxData_t data = {instanceGroup0, 1, 0, 1};
							DALISendData(data);
						}
							break;
						case QUERY_INSTANCE_GROUP_1:;
						{
							DALITxData_t data = {instanceGroup1, 1, 0, 1};
							DALISendData(data);
						}
							break;
						case QUERY_INSTANCE_GROUP_2:;
						{
							DALITxData_t data = {instanceGroup2, 1, 0, 1};
							DALISendData(data);
						}
							break;
						case QUERY_EVENT_SCHEME:;
						{
							DALITxData_t data = {eventScheme, 1, 0, 1};
							DALISendData(data);
						}
							break;
						case QUERY_INPUT_VALUE:
						{
							inputValue_latch = inputValue;
							inputValue_byte = (resolution + 7)/8 - 1;
							DALITxData_t data = {(inputValue_latch >> (inputValue_byte*8)) & 0xFF, 1, 0, 1};
							DALISendData(data);
						}
							break;
						case QUERY_INPUT_VALUE_LATCH:
							if(inputValue_byte != 0)
							{
								inputValue_byte--;
								DALITxData_t data = {(inputValue_latch >> (inputValue_byte*8)) & 0xFF, 1, 0, 1};
								DALISendData(data);
							}
							break;
						case QUERY_EVENT_PRIORITY:;
						{
							DALITxData_t data = {eventPriority, 1, 0, 1};
							DALISendData(data);
						}
							break;
						case QUERY_FEATURE_TYPE:
							break;
						case QUERY_NEXT_FEATURE_TYPE:
							break;
						case QUERY_EVENT_FILTER_0_7:;
						{
							DALITxData_t data = {eventFilter & 0xFF, 1, 0, 1};
							DALISendData(data);
						}
							break;
						case QUERY_EVENT_FILTER_8_15:;
						{
							DALITxData_t data = {(eventFilter >> 8) & 0xFF, 1, 0, 1};
							DALISendData(data);
						}
							break;
						case QUERY_EVENT_FILTER_16_23:
						{
							DALITxData_t data = {(eventFilter >> 16) & 0xFF, 1, 0, 1};
							DALISendData(data);
						}
							break;
						case SET_REPORT_TIMER:
							if(frame != previousFrame)
							{
								DALIReceiveTwice();
							}
							else
							{
								if(msg_ptr->rxSendTwicePossible == 1)
								{
									tReport = DTR0;
									if(tReport != 30)
										resetState = FALSE;
								}
								isSecondFrame = 1;
							}
							break;
						case SET_HYSTERESIS:
							if(frame != previousFrame)
							{
								DALIReceiveTwice();
							}
							else
							{
								if(msg_ptr->rxSendTwicePossible == 1)
								{
									if(DTR0 <= 25)
									{
										hysteresis = DTR0;
										if(hysteresis != 5)
											resetState = FALSE;
									}
								}
								isSecondFrame = 1;
							}
							break;
						case SET_DEADTIME_TIMER:
							if(frame != previousFrame)
							{
								DALIReceiveTwice();
							}
							else
							{
								if(msg_ptr->rxSendTwicePossible == 1)
								{
									tDeadtime = DTR0;
									if(tDeadtime != 30)
										resetState = FALSE;
								}
								isSecondFrame = 1;
							}
							break;
						case SET_HYSTERESIS_MIN:
							if(frame != previousFrame)
							{
								DALIReceiveTwice();
							}
							else
							{
								if(msg_ptr->rxSendTwicePossible == 1)
								{
									hysteresisMin = DTR0;
									if(hysteresisMin != 10)
										resetState = FALSE;
								}
								isSecondFrame = 1;
							}
							break;
						case QUERY_DEADTIME_TIMER:
						{
							DALITxData_t data = {tDeadtime, 1, 0, 1};
							DALISendData(data);
						}
							break;
						case QUERY_INSTANCE_ERROR:
						{
							if(instanceError != 0)
							{
								DALITxData_t data = {instanceError, 1, 0, 1};
								DALISendData(data);
							}
						}
							break;
						case QUERY_REPORT_TIMER:
						{
							DALITxData_t data = {tReport, 1, 0, 1};
							DALISendData(data);
						}
							break;
						case QUERY_HYSTERESIS:
						{
							DALITxData_t data = {hysteresis, 1, 0, 1};
							DALISendData(data);
						}
							break;
						case QUERY_HYSTERESIS_MIN:
						{
							DALITxData_t data = {hysteresisMin, 1, 0, 1};
							DALISendData(data);
						}
							break;
						}
					}
					if(memory_related == 0)
					{
						writeEnableState = DISABLED;
					}
				}
				else // Event frame
				{
				}
			}
			else // Backward frame
			{
				backFrame = (msg_ptr->frame) & 0xFF;
			}
		}
		previousFrame = (isSecondFrame) ? 0 : frame;
		isSecondFrame = 0;
		return;
	}
}

void DALI_SendEvent()
{
	if((applicationActive == FALSE) && (quiescentMode == FALSE) && (dead_time == 0) && (eventFilter % 2 == 1) && (instanceActive == TRUE) && (instanceError == FALSE))
	{
		if ((((eventScheme == 1) || (eventScheme == 2)) && (shortAddress == 0xFF)) \
				|| ((eventScheme == 3) && (deviceGroups == 0)) || ((eventScheme == 4) && (instanceGroup0 == 0xFF)))
		{
			eventScheme = 0;
			DALI_Save_Variable();
		}

		uint32_t frame;
		switch(eventScheme)
		{
		case 0:
			frame = 0x800000 | ((instanceType << 17) & 0x3E0000) | 0x8000 | ((instanceNumber << 10) & 0x7C00) | ((inputValue >> 6) & 0x3FF);
			break;
		case 1:
			frame = ((shortAddress << 17) & 0x7E0000) | ((instanceType << 10) & 0x7C00) | ((inputValue >> 6) & 0x3FF);
			break;
		case 2:
			frame = ((shortAddress << 17) & 0x7E0000) | 0x8000 | ((instanceNumber << 10) & 0x7C00) | ((inputValue >> 6) & 0x3FF);
			break;
		case 3:;
			uint32_t temp = deviceGroups;
			uint8_t count = 1;
			// Find the lowest device group number of membership of the containing device
			while(temp % 2 == 0)
			{
				temp = temp >> 1;
				count++;
			}
			frame = 0x800000 | ((count << 17) & 0x3E0000) | ((instanceType << 10) & 0x7C00) | ((inputValue >> 6) & 0x3FF);
			break;
		case 4:
			frame = 0xC00000 | ((instanceGroup0 << 17) & 0x3E0000) | ((instanceType << 10) & 0x7C00) | ((inputValue >> 6) & 0x3FF);
			break;
		}

		if((inputValue > hysteresisBandHigh) || (inputValue < hysteresisBandLow))
		{
			DALITxData_t data = {frame, 0, 0, 4};
			DALISendData(data);
			hysteresisBand = (hysteresisMin > (hysteresis * inputValue / 100)) ? hysteresisMin : (hysteresis * inputValue / 100);

			if(inputValue > hysteresisBandHigh)
			{
				hysteresisBandHigh = inputValue;
				hysteresisBandLow = (inputValue > hysteresisBand)? (inputValue - hysteresisBand) : 0;
			}
			else
			{
				hysteresisBandLow = inputValue;
				hysteresisBandHigh = inputValue + hysteresisBand;
			}
			report_time = tReport*1000;
			dead_time = tDeadtime*50;
		}
		else if((report_time == 0) && (tReport != 0))
		{
			DALITxData_t data = {frame, 0, 0, 4};
			DALISendData(data);
			report_time = tReport*1000;
			dead_time = tDeadtime*50;
		}
	}
}

void DALI_Reset_Variables()
{
	deviceGroups 		= 0;
	searchAddress 		= 0xFFFFFF;
	randomAddress		= 0xFFFFFF;
	quiescentMode		= DISABLED;
	writeEnableState 	= DISABLED;
	powerCycleSeen		= FALSE;
	resetState			= TRUE;
	instanceGroup0		= 0xFF;
	instanceGroup1		= 0xFF;
	instanceGroup2		= 0xFF;
	eventPriority		= 4;
	eventScheme			= 0;
	if(applicationActive)
	{
		eventFilter = 0xFFFF;
	}
	else
	{
		eventFilter	= 1;
		tReport		= 30;
		tDeadtime 	= 30;
		hysteresis 	= 5;
		if (resolution <= 6)
			hysteresisMin = 0;
		else if(resolution == 7)
			hysteresisMin = 1;
		else if(resolution == 8)
			hysteresisMin = 2;
		else if(resolution == 9)
			hysteresisMin = 5;
		else if(resolution == 10)
			hysteresisMin = 10;
		else if(resolution == 11)
			hysteresisMin = 20;
		else if(resolution == 12)
			hysteresisMin = 40;
		else if(resolution == 13)
			hysteresisMin = 81;
		else if(resolution == 14)
			hysteresisMin = 163;
		else if(resolution >= 15)
			hysteresisMin = 255;
	}
	DALI_Save_Variable();
}

void DALI_Save_Variable()
{
	erase_page(MEMORY_NVM_VAR_ADDR);
	dali_NVM_unlock();
	shortAddress_NVM = shortAddress;
	(* (uint16_t*) (MEMORY_NVM_VAR_ADDR + 0)) = deviceGroups & 0xFFFF;
	(* (uint16_t*) (MEMORY_NVM_VAR_ADDR + 2)) = (deviceGroups >> 16) & 0xFFFF;
	(* (uint16_t*) (MEMORY_NVM_VAR_ADDR + 4)) = randomAddress & 0xFFFF;
	(* (uint16_t*) (MEMORY_NVM_VAR_ADDR + 6)) = (randomAddress >> 16) & 0xFFFF;
	operatingMode_NVM = operatingMode;
	applicationActive_NVM = applicationActive;
	powerCycleNotification_NVM = powerCycleNotification;
	eventPriority_NVM = eventPriority;
	instanceGroup0_NVM = instanceGroup0;
	instanceGroup1_NVM = instanceGroup1;
	instanceGroup2_NVM = instanceGroup2;
	instanceActive_NVM = instanceActive;
	eventFilter_NVM = eventFilter;
	eventScheme_NVM = eventScheme;
	tReport_NVM = tReport;
	tDeadtime_NVM = tDeadtime;
	hysteresisMin_NVM = hysteresisMin;
	hysteresis_NVM = hysteresis;
	dali_NVM_lock();
}

void DALI_Set_inputValue(uint32_t adcVal)
{
	/*
	 * fullScaleRange is a pre-defined value from manufacturer.
	 * At calibration stage, calibrationOffset is recorded at 0 illuminance
	 * and 16*calibrationScale is recorded at fullScaleRange illuminance
	 * The inputValue is converted so that it equals 1000 at fullScaleRange illuminace (i.e when adcVal = 16*calibrationScale)
	 * and it equals 0 at 0 illuminance (i.e when adcVal = calibrationOffset)
	 */
	inputValue_10b = 1000*(adcVal - calibrationOffset)/(16*calibrationScale - calibrationOffset);	// Convert from 12-bit resolution to 0-1000 scale
	inputValue = ((inputValue_10b << 10) & 0xFC00) | (inputValue_10b & 0x3FF); // MSB-aligned, unused bits conatain a repeating pattern of MSB of the result
}

 void DALI_Send_PowerCycleEvent()
 {
	 uint32_t frame = 0xFEE000;
	 if (deviceGroups > 0)
	 {
		 frame = frame | (1 << 12);
		 uint32_t temp = deviceGroups;
		 uint8_t count = 0;
		 // Find the lowest device group number of membership of the containing device
		 while(temp % 2 == 0)
		 {
			 temp = temp >> 1;
			 count++;
		 }
		 frame = frame | ((count << 7) & 0xF80);
	 }
	 if (shortAddress != 0xFF)
	 {
		 frame = frame | (1 << 6);
		 frame = frame | (shortAddress & 0x1F);
	 }
	 DALITxData_t data = {frame, 0, 0, 3};
	 DALISendData(data);
 }

 void DALI_Check_ResetState()
 {
	 if(resetState != TRUE)
	 {
		 if((deviceGroups != 0) || (searchAddress != 0xFFFFFF) || (instanceGroup0 != 0xFF) || (instanceGroup1 != 0xFF) || (instanceGroup2 != 0xFF) || \
				 (eventFilter != 1) || (eventScheme != 0) || (eventPriority != 4) || (tReport != 30) || (tDeadtime != 30) || (hysteresisMin != 10) || (hysteresis != 5))
		 {
			 return;
		 }
		 resetState = TRUE;
	 }
 }
