/*
 * dali.h
 * This file provides code for the configuration of the DALI-2 protocol
 * physical and link layers for application controller or input device
 *  Created on: Jul 13, 2020
 *      Author: vtran
 */

#ifndef INC_DALI_H_
#define INC_DALI_H_

#include "tim.h"
#include "gpio.h"
#include "main.h"

typedef enum
{
	IDLE,						// DALI state idle, ready to send new data
	SEND_DATA,					// Sending DALI data/command
	RECEIVE_DATA,				// Receiving data from the DALI bus
	RECEIVE_DATA_EXTRA_TE,		// If the last RX'd bit is 1, we need to wait an extra TE for a full 6TE stop condition (we don't want to mistake the 2nd half of bit 1 as stop condition)
	WAIT_FOR_SECOND_FORFRAME,	// Forward frame RX'd in this state is considered send-twice forward frame
	WAIT_FOR_BACKFRAME,			// Waiting for TX/RX backward frame (if any)
	WAIT_TO_SEND_BACKFRAME,
	WAIT_AFTER_RX_BACKFRAME,	// Minimum delay after having received data until we can send again // unused
	BREAK,						// Break the transmission after there is a collision
	PRE_IDLE					// Collision avoidance state, basically just wait for settling time
}dali_state_t;

typedef enum
{
	NO_ERROR = 0,
	BIT_TIMING_ERROR,
	FRAME_TIMING_ERROR,
	FRAME_SIZE_ERROR
}DALIRxError_t;

// The data buffer contains the forward frame, the backward frame and the delay
// between these two. This set of flags give the validity of each of these data
// as well as other information such as weather this is the device that sent the
// forward frame or it's a packet sniffed from the bus.
typedef union
{
	uint8_t flagsByte;
	struct
	{
        unsigned int forwardFrameValid                   : 1;   // Forward frame valid
        unsigned int backwardFrameValid                  : 1;   // Backward frame valid
        unsigned int backwardFrameDelayValid             : 1;   // Delay between forward and backward frames valid
        unsigned int txThisDevice                        : 1;   // This device transmitted the forward frame
        unsigned int txError                             : 1;   // Error on transmission
        unsigned int rxTimingError                       : 1;   // Reception of data at illegal time
        unsigned int rxError                             : 1;   // Error on reception
        unsigned int txType								 : 1;	// Tx forward frame type; 0 for 16-bit, 1 for 24-bit
	};
}data_flags_t;

typedef struct DALITxData
{
	uint32_t frame;
	uint8_t frameType;		// 0 for forward frame, 1 for backward frame
	uint8_t sendTwice;
	uint8_t priority;
} DALITxData_t;

extern struct TXFlags
{
	unsigned int txDone					: 1;
	unsigned int txError				: 1;
} txFlags;

typedef struct DALIRxData
{
	uint32_t frame;
	uint8_t frameLen;
	unsigned int frameType				: 1; 	// 0 for forward frame, 1 for backward frame
	unsigned int rxDone					: 1;
	DALIRxError_t rxError				: 2;
	unsigned int rxSendTwicePossible	: 1;
} DALIRxData_t;

#ifdef CONTROLLER
extern volatile uint8_t power_up_timer;
extern volatile uint8_t power_up_100ms;
extern volatile uint16_t TE;
extern volatile uint8_t collisionDetect;
extern volatile uint8_t collisionDetectTimer;
extern volatile uint8_t collisionDetectCount;
extern volatile uint16_t collisionDetect_start;
extern volatile uint16_t collisionDetect_stop;
#endif
/**********************Public function definitions*****************************/

// Initialize DALI stack.
void DALIInit(void);

// Configure device mode.
// Input: 0 for application controller, 1 for input device
void DALIConfigureMode(uint8_t mode);

// This needs to be called from the ISR unconditionally. It checks and clears
// the Tick timer, Te timer and External interrupt flags.
void DALITimerIntHandler(void);
void DALIRxIntHandler(void);

// Transmit command on the DALI bus. The machine will also wait for any reply
// after the transmission. The function returns 0 when successful and 1 to signal
// an error (machine busy).
uint8_t DALISendData(DALITxData_t data);

// Returns true if there's any data available in the input data buffer
uint8_t DALIDataAvailable(void);

// The data in the input data buffer is composed of forward data, backward data,
// timing data and flags. This function moves the read pointer to the fresh data
// and returns the forward frame. It is required that out of the DALIReceiveData
// functions this be called first.
DALIRxData_t DALIReceiveData(void);

// Function that returns the flags associated to the data being read. These
// give the status of this data transaction. The user must first call
// DALIReceiveDataForward.
uint8_t DALIReceiveDataFlags(void);

//Check if cable is connected. This function is run in SysTick ISR. Cable is considered
//disconnected after 20ms DALI line is low
void DALICheckCable(void);

// Notify the DALI driver when the received forward frame requires a send-twice frame
void DALIReceiveTwice();
// DEBUG: Function that returns the flags of the DALI protocol state machine
uint16_t DALIReadFlags(void);

// DEBUG: Function that returns the state of the DALI protocol state machine
uint8_t DALIReadState(void);

#endif /* INC_DALI_H_ */
