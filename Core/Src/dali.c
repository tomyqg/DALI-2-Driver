/*
 * dali.c
 * This file provides code for the configuration of the DALI-2 protocol
 * physical and link layers for application controller or input device
 * Refer to 3342_015 for state diagram
 *  Created on: Jul 13, 2020
 *      Author: vtran
 */

#include "dali.h"
#include "stm32f0xx_hal.h"
#include "stdlib.h"
#include "time.h"

#define DALI_HI							0
#define DALI_LO                         1

/********************** TRANSMITTING TIME DEFINTIONS ***************************/
// The number of timer counts that make up a 416.6us delay. The timer is configured
// to run at 8MHz, thus the count is 8*416.6 = 3333

#define TE                              3333
// Time for collision detection and collision recovery
#define TE_TX_MIN						2854	// 356.7 us
#define TE_TX_MAX						3814	// 476.7 us
#define TE2_TX_MIN						5787	// 723.3 us
#define TE2_TX_MAX						7546	// 943.3 us
#define TE_BREAK						10400	// 1.3 ms
#define TE_RECOVERY						34400	// 4.3 ms
//The number of timer counts for settling time between forward frame and backward frame
#define TE_TX_WAIT_BF_MAX				64000	// 10.5 ms - 6TE (3 stop bits)
#define TE_TX_WAIT_BF					40000	// A random number between the max and min value
#define TE_TX_WAIT_BF_MIN				24000	// 5.5 ms - 6TE
//The number of timer counts for settling time between any frame and forward frame
#define TE_TX_WAIT_FF1_MIN				88000	// 13.5 ms - 6TE
#define TE_TX_WAIT_FF1					92200	// Random between min and max
#define TE_TX_WAIT_FF1_MAX				97600	// 14.7 ms - 6TE
#define TE_TX_WAIT_FF2_MIN				99200	// 14.9 ms - 6TE
#define TE_TX_WAIT_FF2					102300
#define TE_TX_WAIT_FF2_MAX				108800	// 16.1 ms - 6TE
#define TE_TX_WAIT_FF3_MIN				110400	// 16.3 ms - 6TE
#define TE_TX_WAIT_FF3					117000
#define TE_TX_WAIT_FF3_MAX				121600	// 17.7 ms - 6TE
#define TE_TX_WAIT_FF4_MIN				123200	// 17.9 ms - 6TE
#define TE_TX_WAIT_FF4					128200
#define TE_TX_WAIT_FF4_MAX				134400	// 19.3 ms - 6TE
#define TE_TX_WAIT_FF5_MIN				136000	// 19.5 ms - 6TE
#define TE_TX_WAIT_FF5					141000
#define TE_TX_WAIT_FF5_MAX				148800	// 21.1 ms - 6TE
#define TE_TX_WAIT_FF_MAX				580000	// 75 ms - 6TE

// During transmission on the DALI bus the firmware does collision detection. In
// doing so, it expects to see on the RX pin the value that is set on the TX pin.
// However, the bus takes a while to respond (depending on the driver strength,
// bus capacitance and power supply). Also during the time that we're driving the
// line we don't expect to see any transitions.
// The implementation is that we define a 150us window during which we expect a
// transition if one was generated. If no such transition occurs, or transitions
// occur outside this window, a collision is signaled.
#define TE_TRANSITION_VALID_MAX         	1200

/*********************** RECEIVING TIME DEFINTIONS ****************************/
// The max time the bus line can go without transition during DALI frame reception
// should be max time for 2 half bits (unless the stop bits).
// If the external interrupt is triggered during this 4 Te period, the timer
// register is compared to these 4 values and if it falls within [TE_RX_MIN, TE_RX_MAX]
// a single Te is considered to have elapsed, whereas if it falls within
// [TE2_RX_MIN, TE2_RX_MAX], 2 Te is considered to have elapsed. Otherwise, a reception
// error is signaled.
#define TE_RX_MIN						2366	//2666	// 333.25 us  - correct value is 2666 but minus 300 to compensate for the difference between up and down transition due to RC filter
#define TE_RX_MAX						4300	//4000	// 500 us - add 300 for - should still in grey area
#define TE2_RX_MIN						5132	//5332	// 666.5 us - same as TE_RX_MIN
#define TE2_RX_MAX						8200	//8000	// 1000 us - same as TE_RX_MAX
#define TE_STOP_MIN						19200	// 2400 us

#define TE_RX_BF_MAX					87200	// 13.4 ms - 6TE
#define TE_RX_SEND_TWICE_FF				800000	// 100 ms
#define TE_RX_SEND_TWICE_FF_MAX			820000	// 105 ms - 6TE

// DALI bus decoded data is put in a circular buffer such that the application
// can pick it up at its own pace. This is the size of this buffer.
#define RX_QUEUE_SIZE                   20
#define TX_QUEUE_SIZE					20
// DALI protocol state machine flags. Most of these get reset at the start of each
// frame and are updated as the machine advances. At the end of the frame (or
// frames, since the machine doesn't return to ST_IDLE between a forward frame
// and its associated backward frame) it digests these flags into fewer bits
// which it records along with the received/transmitted data.
// Most of these flags are used to communicate information between states.
typedef union
{
	uint16_t flags_all;
	struct
	{
		unsigned int txError                        : 1;    // Transmission error
		unsigned int txDone                         : 1;    // Transmission done
		DALIRxError_t rxError                       : 2;    // Reception error
		unsigned int rxDone                         : 1;    // Reception done
		unsigned int rxFrameType					: 1;    // Rx frame type; 0 for forward frame, 1 for backward frame
		unsigned int cableConnected                 : 1;    // DALI cable connected to the board
		unsigned int rxSendTwicePossible	        : 1;    // Started receiving frame from WAIT_TO_SEND_BACKFRAME or WAIT_FOR_SECOND_FORFRAME
		unsigned int deviceMode						: 1;	// DeviceMode; 1 for application controller (16-bit frame), 0 for input device (24-bit frame)
		unsigned int txFrameType					: 1;	// Tx frame type; 0 for forward frame, 1 for backward frame
		unsigned int sendTwiceFrame					: 1;	// 1 is for frame that is required to send twice
		unsigned int receiveTwiceFrame              : 1;	// 1 is for frame that is required to receive twice
		dali_state_t rxFromState					: 4;	// Started receiving from this state
	};
}state_flags_t;

dali_state_t daliState;
state_flags_t DALIFlags;
struct TXFlags txFlags = {1, 0};
// Variables used by the state machine
volatile uint32_t txPacket;
uint8_t txPriority;
uint32_t txPacket_temp;
volatile uint32_t rxPacket;
volatile uint8_t halfBitNumber;
volatile uint8_t prevBit;
volatile uint32_t rxFrame;
volatile uint8_t rxPacketLen;
volatile uint32_t rxPacketTime;
volatile uint16_t time_int2[60];
volatile uint16_t time_int[60];
volatile uint16_t time_int3[60];
// Data circular buffer where the DALI stack fills in received data. The
// application can then grab the data from here
struct DALIRxData rxData[RX_QUEUE_SIZE];
struct DALITxData txData[TX_QUEUE_SIZE];
uint16_t backwardFrameDelay[RX_QUEUE_SIZE];
data_flags_t flagsData[RX_QUEUE_SIZE];
volatile uint8_t rxDataR, rxDataW;
volatile uint8_t txDataR, txDataW; // (txDataW: next empty space in the queue, txDataR: next element will be processed)
volatile uint8_t priorityState = 1;
volatile uint32_t overlapTime = 0;

/***********************Local function definitions*****************************/

// Clear most flags such that the new frame gets a fresh start. The cable
// connected and configuration flags remain unchanged
void DALIClearFlags(void);

// Once the machine is ready receiving data, it will use this function to append
// the data to the queue. The application can check if this queue is empty and
// if there is data ready can then retrieve it.
void DALIAppendToQueue(void);
void DALIProcessSendData(struct DALITxData txdata);

/*************************Function implementations*****************************/
void DALIInit(void)
{
	//Initialize the bus to idle state
	writePin(TX_Pin, DALI_HI);

	daliState = IDLE;
	DALIFlags.flags_all = 0;
	DALIConfigureMode(1);
	rxDataR = 0;
	rxDataW = 0;
	srand(time(0));
}

void DALIConfigureMode(uint8_t mode)
{
	DALIFlags.deviceMode = mode;
}

void DALICheckCable(void)
{
	static uint8_t cableDisconnectCounter = 0;
	if (readPin(RX_Pin) == DALI_LO)
	{
		if (cableDisconnectCounter > 0)
		{
			cableDisconnectCounter -= 1;
		}
	}
	else
	{
		cableDisconnectCounter = 20;
	}

	if (cableDisconnectCounter == 0)
	{
		DALIFlags.cableConnected = 0;
	}
	else
	{
		DALIFlags.cableConnected = 1;
	}
}

void DALITimerIntHandler(void)
{
	static uint32_t TE_random;
	uint32_t TE_adjust;
	switch(daliState)
	{
	case SEND_DATA:
		TE_adjust = TE - overlapTime;
		overlapTime = 0;
		switch(halfBitNumber)
		{
		case 1:
			// Start bit, reaches the end of 1st half, start 2nd half
			set_timer_reload_val(TE, &htim2);
			writePin(TX_Pin, DALI_HI);
			break;
		case 2:
			// Data bit,  start of 1st half
			set_timer_reload_val(TE_adjust, &htim2);
			if ((txPacket & 0x800000) != 0)
			{
				// Send a '1' bit, which starts with a DALI_LO
				writePin(TX_Pin, DALI_LO);
			}
			else
			{
				// Send a '0' bit, which starts with a DALI_HI
				writePin(TX_Pin, DALI_HI);
			}
			prevBit = 1;
			break;
		case 50:
			// STOP BIT 1, 1st half
			set_timer_reload_val(TE_adjust, &htim2);
			writePin(TX_Pin, DALI_HI);
			break;
		case 51:
		case 52:
		case 53:
		case 54:
		case 55:
			// STOP BIT 1, 2nd half
			// STOP BIT 2, 3
			set_timer_reload_val(TE_adjust, &htim2);
			writePin(TX_Pin, DALI_HI);
			// No transitions should occur, we should stay put
			break;
		case 56:
			// Transmission finalized OK. Start waiting
			// for a potential backframe. Keep timer running
			if(DALIFlags.txFrameType == 1) // backward frame sent
			{
				daliState = PRE_IDLE;
				set_timer_reload_val(TE_TX_WAIT_FF1, &htim2);
			}
			else
			{
				set_timer_reload_val(TE_RX_BF_MAX, &htim2);
				rxPacketTime = 0;
				daliState = WAIT_FOR_BACKFRAME;
			}
			if(DALIFlags.sendTwiceFrame == 0)
			{
				DALIFlags.txDone = 1;
				DALIAppendToQueue();
			}
			break;
		default:
			// This case handles all the in-between (data) bits, with a
			// distinction between 1st half (even halfBitNumber) and 2nd
			// half (odd halfBitNumber) of a bit.
			set_timer_reload_val(TE_adjust, &htim2);
			switch(halfBitNumber & 0x01)
			{
			case 0:
				// halfBitNumber is even, we are in the 1st half of the bit.
				switch (txPacket & 0xC00000)
				{
				case 0x000000:
					// Last bit sent 0, we need to send 0
					writePin(TX_Pin, DALI_HI);
					break;
				case 0x400000:
					// Last bit sent 0, we need to send 1
					writePin(TX_Pin, DALI_LO);
					break;
				case 0x800000:
					// Last bit sent 1, we need to send 0
					writePin(TX_Pin, DALI_HI);
					break;
				case 0xC00000:
					// Last bit sent 1, we need to send 1
					writePin(TX_Pin, DALI_LO);
					break;
				}
				prevBit = (txPacket & 0x800000) ? 1 : 0;
				txPacket <<= 1;
				break;
			case 1:
				// halfBitNumber is odd, 2nd half of the bit.
				switch(txPacket & 0x800000)
				{
				case 0:
					// We're currently sending a '0'
					writePin(TX_Pin, DALI_LO);
					break;
				case 0x800000:
					// We're currently sending a '1'
					writePin(TX_Pin, DALI_HI);
					break;
				}
				break;
			}
			break;
		}
		// Move on in state SEND_DATA, by incrementing the halfbit we're transmitting

		halfBitNumber++;
		if((DALIFlags.txFrameType == 1) && (halfBitNumber == 18))
		{
			// If sending backward frame, skip to stop bit at half bit 18
			halfBitNumber = 50;
		}
		else if((DALIFlags.deviceMode == 1) && (DALIFlags.txFrameType == 0) && (halfBitNumber == 34))
		{
			// If sending 16-bit forward frame, skip to stop bit at half bit 34
			halfBitNumber = 50;
		}
		time_int[halfBitNumber] = get_timer_count(&htim2);
		break;
	case WAIT_FOR_BACKFRAME:
		// Time-out occurred signal the end of the period in which new frame is interpreted as backward frame
		if(DALIFlags.sendTwiceFrame == 1)
		{
	        daliState = SEND_DATA;
	        DALIFlags.sendTwiceFrame = 0;
	        txPacket = txPacket_temp;
	        halfBitNumber = 1;
	        set_timer_reload_val(TE, &htim2);
	        writePin(TX_Pin, DALI_LO);
		}
		else
		{
			set_timer_reload_val(TE_TX_WAIT_FF1 - TE_RX_BF_MAX, &htim2);
			daliState = PRE_IDLE;
		}
		break;
	case WAIT_AFTER_RX_BACKFRAME:
		daliState = IDLE;
		disable_timer_int(&htim2);
		break;
	case BREAK:
		writePin(TX_Pin, DALI_HI);
		uint8_t wait = 30;
		while (wait--);	// Add a dummy line to make sure the bus line is released before checking it
		if(readPin(RX_Pin) == DALI_LO)
		{
			set_timer_reload_val(TE_TX_WAIT_FF1, &htim2);
		}
		else
		{
			// Set the recovery time randomly between the min and max range
			// to avoid collisions
			TE_random = TE_RECOVERY - 1400 + (rand() % 2800); // Between TE_RECOVERY - 1400 and TE_RECOVERY + 1400
			set_timer_reload_val(TE_random, &htim2);
		}
		DALIAppendToQueue();
		daliState = PRE_IDLE;
		break;
	case PRE_IDLE:
		if(txDataR != txDataW) // if there is data to send
		{
			if(txData[txDataR].priority <= priorityState)
			{
				DALIProcessSendData(txData[txDataR]);
				txDataR = (txDataR + 1) % TX_QUEUE_SIZE;
				return;
			}
		}
		switch(priorityState)
		{
		case 1:
			set_timer_reload_val(TE_TX_WAIT_FF2 - TE_TX_WAIT_FF1, &htim2);
			priorityState++;
			break;
		case 2:
			set_timer_reload_val(TE_TX_WAIT_FF3 - TE_TX_WAIT_FF2, &htim2);
			priorityState++;
			break;
		case 3:
			set_timer_reload_val(TE_TX_WAIT_FF4 - TE_TX_WAIT_FF3, &htim2);
			priorityState++;
			break;
		case 4:
			set_timer_reload_val(TE_TX_WAIT_FF5 - TE_TX_WAIT_FF4, &htim2);
			priorityState++;
			break;
		case 5:
			disable_timer_int(&htim2);
			daliState = IDLE;
			priorityState = 1;
			break;
		}
		break;
	case RECEIVE_DATA:
		/*
		    	// Check first if we're at a final bit (got 8 or 24 bits)
		    	// and the last bit is a 1 or a 0. We need to add an extra delay
		    	// of 1 Te if this is the final bit of a packet and the last bit was
		    	// a 1, just to make sure that we exit RECEIVE_DATA after the last
		    	// stop bit has been fully received.
		    	// Otherwise, if this is not the last bit of a frame, we need to
		    	// signal an error.
		 */
		if (((rxPacketLen == 8) || (rxPacketLen == 24)) && ((rxPacket & 0x01) != 0x00))
		{
			/*
		    		// The last bit we've received is very likely the last one in the
		    		// transmission, and it is a '1'. Thus, we need to add a 1 Te
		    		// delay.
			 */
			daliState = RECEIVE_DATA_EXTRA_TE;
			set_timer_reload_val(TE, &htim2);
		}
		else
		{
			// We've finished receiving some data.
			if (rxPacketLen == 8)
			{
				/*
		    			// What we've just got is a backward frame, as it only has
		    			// 8 bits. If we started receiving it out of the blue (from
		    			// IDLE) this means the backframe came too late after its
		    			// forward frame, so we need to set the rxTimingError bit.
				 */
				DALIFlags.rxFrameType = 1;
				rxFrame = rxPacket;

				if (DALIFlags.rxFromState != WAIT_FOR_BACKFRAME)
				{
					DALIFlags.rxError = FRAME_TIMING_ERROR;
				}

				/*
				 * The halfBitNumber internal state gives the position within
				 * the last received bit. This is important since we need to
				 * know the current state of the DALI_RX line. If the line did
				 * not return to DALI_HI then there is an error.
				 */
				if ((halfBitNumber == 2) && (halfBitNumber == 4))
				{
					DALIFlags.rxError = BIT_TIMING_ERROR;
				}
				set_timer_reload_val(TE_TX_WAIT_FF1, &htim2);
				DALIFlags.rxDone = 1;
				DALIAppendToQueue();
				daliState = PRE_IDLE;
			}
			else if (rxPacketLen == 24)
			{
				/*
		    			// What we've just got is a forward frame, as it has 24 bits.
		    			// If we started receiving it while waiting for a backframe
		    			// (from WAIT_FOR_BACKFRAME) this means that a master did
		    			// not respect the minimum time between frames. Thus we need
		    			// to set the rxTimingError bit.
				 */
				DALIFlags.rxFrameType = 0;
				rxFrame = rxPacket;

				if (DALIFlags.rxFromState == WAIT_FOR_BACKFRAME)
				{
					DALIFlags.rxError = FRAME_TIMING_ERROR;
				}

				if ((halfBitNumber == 2) && (halfBitNumber == 4))
				{
					DALIFlags.rxError = BIT_TIMING_ERROR;
				}

				DALIFlags.rxDone = 1;
				// Wait to send a backward frame if needed
				set_timer_reload_val(TE_TX_WAIT_BF, &htim2);
				DALIAppendToQueue();
				daliState = WAIT_TO_SEND_BACKFRAME;
			}
			else
			{
				// We've received a number of bits that don't make up a whole
				// frame and we need to signal an error.
				DALIFlags.rxError = FRAME_SIZE_ERROR;
				DALIFlags.rxDone = 1;
				set_timer_reload_val(TE_TX_WAIT_FF1, &htim2);
				DALIAppendToQueue();
				daliState = PRE_IDLE;
			}
		}
		break;
	case RECEIVE_DATA_EXTRA_TE:
		/*
		 * If we didn't have any transition during this last wait
		 * (which is safe to assume as such a transition would have stopped
		 * the transmission and signaled an error already) we can check if
		 * the received frame is valid.
		 */
		if (rxPacketLen == 8)
		{
			// Backward frame, same checks as during RECEIVE_DATA
			DALIFlags.rxFrameType = 1;
			rxFrame = rxPacket;

			if (DALIFlags.rxFromState != WAIT_FOR_BACKFRAME)
			{
				DALIFlags.rxError = FRAME_TIMING_ERROR;
			}

			if ((halfBitNumber == 2) && (halfBitNumber == 4))
			{
				DALIFlags.rxError = BIT_TIMING_ERROR;
			}


			set_timer_reload_val(TE_TX_WAIT_FF1, &htim2);
			DALIFlags.rxDone = 1;
			DALIAppendToQueue();
			daliState = PRE_IDLE;
		}
		else if (rxPacketLen == 24)
		{
			// Forward frame, same checks as during RECEIVE_DATA
			DALIFlags.rxFrameType = 0;
			rxFrame = rxPacket;
			if (DALIFlags.rxFromState == WAIT_FOR_BACKFRAME)
			{
				DALIFlags.rxError = FRAME_TIMING_ERROR;
			}

			if ((halfBitNumber == 2) && (halfBitNumber == 4))
			{
				DALIFlags.rxError = BIT_TIMING_ERROR;
			}

			DALIFlags.rxDone = 1;
			set_timer_reload_val(TE_TX_WAIT_BF, &htim2);
			rxPacketTime = 0;
			DALIAppendToQueue();
			daliState = WAIT_TO_SEND_BACKFRAME;
		}
		else
		{
			// Error condition, same checks as during RECEIVE_DATA
			DALIFlags.rxError = FRAME_SIZE_ERROR;
			DALIFlags.rxDone = 1;
			set_timer_reload_val(TE_TX_WAIT_FF1, &htim2);
			DALIAppendToQueue();
			daliState = PRE_IDLE;
		}
		break;
	case WAIT_TO_SEND_BACKFRAME:
		// If the received forward frame requires send-twice frame,
		// wait for the second frame, else go to idle to send a
		// backward frame if needed
		if(DALIFlags.receiveTwiceFrame == 1)
		{
			set_timer_reload_val(TE_RX_SEND_TWICE_FF - TE_TX_WAIT_BF, &htim2);
			daliState = WAIT_FOR_SECOND_FORFRAME;
			DALIFlags.receiveTwiceFrame = 0;
		}
		else
		{
			if((txDataR != txDataW) && (txData[txDataR].frameType == 1)) // if there is a backward frame to send
			{
				DALIProcessSendData(txData[txDataR]);
				txDataR = (txDataR + 1) % TX_QUEUE_SIZE;
				return;
			}
			set_timer_reload_val(TE_TX_WAIT_FF1 - TE_TX_WAIT_BF, &htim2);
			daliState = PRE_IDLE;
		}
		break;
	case WAIT_FOR_SECOND_FORFRAME:
		// Time out while waiting for a second frame -> error
		rxFrame = 0;
		DALIFlags.rxError = FRAME_TIMING_ERROR;
		DALIFlags.rxDone = 1;
		set_timer_reload_val(TE_RX_SEND_TWICE_FF - TE_TX_WAIT_BF, &htim2);
		// Add an empty message with rxError
		DALIAppendToQueue();
		daliState = PRE_IDLE;
		break;
	default:
		disable_timer_int(&htim2);
		break;
	}
}

void DALIRxIntHandler(void)
{
	static uint32_t tim2_value;
	static uint16_t tim3_value;
	static uint16_t prev_halfbit;
	// A time-out of TE_STOP_MIN is set every time a transition is detection
	// Time-out means we either received a stop condition or an error
	switch(daliState)
	{
	case IDLE:
	case WAIT_AFTER_RX_BACKFRAME:
	case PRE_IDLE:
		//Make sure we're not driving the line
		writePin(TX_Pin, DALI_HI);
		if(readPin(RX_Pin) == DALI_HI)
		{
			// Rising-edge detected. This should happen only when the cable has
			// just been connected. Thus do nothing
#ifdef CONTROLLER
			power_up_timer = 100;
#endif
		}
		else
		{
			// Either there is data on the bus or the cable has been disconnected
			// Assume the former. Set time-out of 4TE, after which we'll signal
			// an error if no transition occurs.
			rxPacket = 0;
			rxPacketLen = 0;
			halfBitNumber = 0;
			DALIFlags.rxDone = 0;
			DALIFlags.rxError = 0;
			DALIFlags.rxFrameType = 0;
			DALIFlags.rxSendTwicePossible = 0;
			DALIFlags.rxFromState = daliState;
			reset_timer(&htim2);
			set_timer_reload_val(TE_STOP_MIN, &htim2);
			enable_timer_int(&htim2);	// In case the timer generate an interrupt during this ISR
			daliState = RECEIVE_DATA;
		}
		break;
	case SEND_DATA:
		/** Collision detection: If the timing is in the valid area and
		 * the grey area, do nothing. Otherwise signal an error and
		 * go to BREAK state*/

		if(halfBitNumber == 1)	// 1st half of start bit -> not care
		{
			reset_timer(&htim3);
//			prev_halfbit = halfBitNumber;
		}
		else
		{
			tim3_value = get_timer_count(&htim3);
			tim2_value = get_timer_count(&htim2);
			reset_timer(&htim3);
			time_int2[halfBitNumber] = tim3_value;
			time_int3[halfBitNumber] = tim2_value;

#ifndef CONTROLLER
			if ((tim3_value >= TE_TX_MIN) && (tim3_value <= TE_TX_MAX))// && (readPin(RX_Pin) == DALI_LO) && (halfBitNumber < 51))
			{
				// A falling edge when the bit transition from 1 to 0 is a break condition
				// Either early in bit 0 or late in bit 1 -> if halfBitNumber is even, check next bit, else check previous bit
				if((halfBitNumber == 2) && ((txPacket & 0x800000) == 0) && (readPin(RX_Pin) == DALI_LO))
				{
					DALIFlags.txError = 1;
					DALIFlags.txDone = 0;
					daliState = BREAK;
					reset_timer(&htim2);
					set_timer_reload_val(TE_BREAK, &htim2);
					enable_timer_int(&htim2);	// In case the timer generate an interrupt during this ISR
					DALIAppendToQueue();
					// Re-insert the frame to be resent
					txDataR = (txDataR == 0) ? (TX_QUEUE_SIZE - 1) : (txDataR - 1);
					writePin(TX_Pin, DALI_LO);
					return;
				}
				else if(((((halfBitNumber % 2) == 0) && ((txPacket & 0xC00000) == 0x800000)) || ((halfBitNumber % 2) && prevBit && ((txPacket & 0x800000) == 0))) && (readPin(RX_Pin) == DALI_LO))
				{
					DALIFlags.txError = 1;
					DALIFlags.txDone = 0;
					daliState = BREAK;
					reset_timer(&htim2);
					set_timer_reload_val(TE_BREAK, &htim2);
					enable_timer_int(&htim2);	// In case the timer generate an interrupt during this ISR
					DALIAppendToQueue();
					// Re-insert the frame to be resent
					txDataR = (txDataR == 0) ? (TX_QUEUE_SIZE - 1) : (txDataR - 1);
					writePin(TX_Pin, DALI_LO);
					return;
				}
				else
				{
					return;
				}
			}
			else if ((tim3_value >= TE2_TX_MIN) && (tim3_value <= TE2_TX_MAX))// && ((halfBitNumber - prev_halfbit) == 2))
			{
				// 2TE is false only if it's falling edge and prevBit = 1 or it's rising edge and prevBit = 0
				// If the 2-bit duration is too short, adjust the TE timer accordingly
				if((readPin(RX_Pin) == DALI_LO) && (prevBit == 1))
				{
					if(((halfBitNumber % 2) == 1) && (tim3_value < (TE + TE_TX_MIN)))
					{
						overlapTime = 2*TE - tim3_value;
					}
					return;
				}
				else if((readPin(RX_Pin) == DALI_HI) && (prevBit == 0))
				{
					if(((halfBitNumber % 2) == 0) && (tim3_value > (TE + TE_TX_MAX)))
					{
						set_timer_count((tim2_value - (tim3_value - 2*TE)), &htim2);
					}
					return;
				}
				else
				{
					DALIFlags.txError = 1;
					DALIFlags.txDone = 0;
					daliState = BREAK;
					reset_timer(&htim2);
					set_timer_reload_val(TE_BREAK, &htim2);
					enable_timer_int(&htim2);	// In case the timer generate an interrupt during this ISR
					DALIAppendToQueue();
					// Re-insert the frame to be resent
					txDataR = (txDataR == 0) ? (TX_QUEUE_SIZE - 1) : (txDataR - 1);
					writePin(TX_Pin, DALI_LO);
					return;
				}
			}
			else
			{
				DALIFlags.txError = 1;
				DALIFlags.txDone = 0;
				daliState = BREAK;
				reset_timer(&htim2);
				set_timer_reload_val(TE_BREAK, &htim2);
				enable_timer_int(&htim2);	// In case the timer generate an interrupt during this ISR
				DALIAppendToQueue();
				// Re-insert the frame to be resent
				txDataR = (txDataR == 0) ? (TX_QUEUE_SIZE - 1) : (txDataR - 1);
				writePin(TX_Pin, DALI_LO);
				return;
			}
#endif
//			prev_halfbit = halfBitNumber;
		}
		break;
	case WAIT_FOR_BACKFRAME:
		// Transition during this state signals the start of a backward frame,
		// but it could be that this is a new forward frame being sent by a
		// Control Device that doesn't respect the inter-frame timing.
		// The DALIFlags.rxFromWaitForBackframe bit will tell later states
		// that the current frame started being received at this point. If
		// it turns out this is a backward frame, this is a valid frame, whereas
		// if it turns out to be a forward frame, this will flag an error.

		rxPacket = 0;
		rxPacketLen = 0;
		halfBitNumber = 0;
		DALIFlags.rxDone = 0;
		DALIFlags.rxError = 0;
		DALIFlags.rxFrameType = 0;		// Assume forward frame although it should be backframe, will re-check after receive the whole frame
		DALIFlags.rxSendTwicePossible = 0;
		DALIFlags.rxFromState = daliState;
		reset_timer(&htim2);
		set_timer_reload_val(TE_STOP_MIN, &htim2);
		daliState = RECEIVE_DATA;
		// Timer may have overflowed while we were servicing this
		// interrupt
		enable_timer_int(&htim2);
		break;
	case WAIT_TO_SEND_BACKFRAME:
		// Transition during this state signals a frame that could be a send-twice frame
		rxPacket = 0;
		rxPacketLen = 0;
		halfBitNumber = 0;
		DALIFlags.rxDone = 0;
		DALIFlags.rxError = 0;
		DALIFlags.rxFrameType = 0;
		DALIFlags.rxFromState = daliState;
		DALIFlags.rxSendTwicePossible = 1; // This flag will tell later state to check if this frame is identical to the previous frame
		reset_timer(&htim2);
		set_timer_reload_val(TE_STOP_MIN, &htim2);
		daliState = RECEIVE_DATA;
		// Timer may have overflowed while we were servicing this
		// interrupt
		enable_timer_int(&htim2);
		break;
	case WAIT_FOR_SECOND_FORFRAME:
		// Transition during this state signals a frame that could be a send-twice frame
		rxPacket = 0;
		rxPacketLen = 0;
		halfBitNumber = 0;
		DALIFlags.rxDone = 0;
		DALIFlags.rxError = 0;
		DALIFlags.rxFrameType = 0;
		DALIFlags.rxFromState = daliState;
		DALIFlags.rxSendTwicePossible = 1; // This flag will tell later state to check if this frame is identical to the previous frame
		reset_timer(&htim2);
		set_timer_reload_val(TE_STOP_MIN, &htim2);
		daliState = RECEIVE_DATA;
		// Timer may have overflowed while we were servicing this
		// interrupt
		enable_timer_int(&htim2);
		break;
	case RECEIVE_DATA:
		tim2_value = get_timer_count(&htim2);
		reset_timer(&htim2);
		set_timer_reload_val(TE_STOP_MIN, &htim2);
		enable_timer_int(&htim2);	// In case the timer generate an interrupt during this ISR
		switch (halfBitNumber)
		{
		/* There are 5 cases:
		 * 0: transition in the middle of start bit
		 * 1: transition after a rising at the end of the bit
		 * 	  --> 1 TE means received 1st half of bit 0, next state is case 4
		 * 	      2 TE means error
		 * 2: transition after a falling at the end of the bit
		 *    --> 1 TE means received 1st half of bit 1, next state is case 3
		 *        2 TE means error
		 * 3: transition after a rising at the middle of the bit
		 *    --> 1 TE means received 2nd half of bit 1, next state is case 2
		 *    	  2 TE means received 2nd half of bit 1 and 1st half of bit 0, next state is case 4
		 * 4: transition after a falling at the middle of the bit
		 * 	  --> 1 TE means received 2nd half of bit 0, next state is case 1
		 * 	  	  2 TE means received 2nd half of bit 0 and 1st half of bit 1, next state is case 3
		 * We save the bit after received the first half*/

		case 0:
			if ((tim2_value >= TE_RX_MIN) && (tim2_value <= TE_RX_MAX))
			{
				// This is a rising at the middle of start bit so next is case 3
				halfBitNumber = 3;
#ifdef CONTROLLER
// The commented code below is for collision detection test for truncated idle phase
//				if(collisionDetectCount == 0)
//				{
//					reset_timer(&htim6);
//					set_timer_reload_val(collisionDetect_start, &htim6);
//					collisionDetectCount = 1;
//					collisionDetectTimer = 1;
//					writePin(LED_Pin, 0);
//					asm("nop");
//				}
#endif
			}
			else
			{
				// Signal reception error if the bit timing isn't
				// respected.
				DALIFlags.rxError = BIT_TIMING_ERROR;
			}
			break;
		case 1:
			if ((tim2_value >= TE_RX_MIN) && (tim2_value <= TE_RX_MAX))
			{
				rxPacket <<= 1;
				rxPacketLen++;
				halfBitNumber = 4;
			}
			else
			{
				// Signal reception error if the bit timing isn't
				// respected.
				DALIFlags.rxError = BIT_TIMING_ERROR;
			}
			break;
		case 2:
			if ((tim2_value >= TE_RX_MIN) && (tim2_value <= TE_RX_MAX))
			{
				rxPacket <<= 1;
				rxPacket |= 1;
				rxPacketLen++;
				halfBitNumber = 3;
			}
			else
			{
				// Signal reception error if the bit timing isn't
				// respected.
				DALIFlags.rxError = BIT_TIMING_ERROR;
			}
			break;
		case 3:
			if ((tim2_value >= TE_RX_MIN) && (tim2_value <= TE_RX_MAX))
			{
				halfBitNumber = 2;
			}
			else if ((tim2_value >= TE2_RX_MIN) && (tim2_value <= TE2_RX_MAX))
			{
				rxPacket <<= 1;
				rxPacketLen++;
				halfBitNumber = 4;
			}
			else
			{
				// Signal reception error if the bit timing isn't
				// valid.
				DALIFlags.rxError = BIT_TIMING_ERROR;
			}
			break;
		case 4:
			if ((tim2_value >= TE_RX_MIN) && (tim2_value <= TE_RX_MAX))
			{
				halfBitNumber = 1;
			}
			else if ((tim2_value >= TE2_RX_MIN) && (tim2_value <= TE2_RX_MAX))
			{
				// Middle of a '1' bit
				rxPacket <<= 1;
				rxPacket |= 1;
				rxPacketLen++;
				halfBitNumber = 3;
			}
			else
			{
				// Signal reception error if the bit timing isn't
				// respected.
				DALIFlags.rxError = BIT_TIMING_ERROR;
			}
			break;
		}
		break;
	case RECEIVE_DATA_EXTRA_TE:
		//Transition during the last half of the 3rd stop bit, which is not supposed to happen.
		DALIFlags.rxError = BIT_TIMING_ERROR;
		break;
	default:
		break;
	}
}

/*********DALI Data transmission and reception*********/
uint8_t DALISendData(struct DALITxData data)
{
	// Check if tx queue is full, if full ignore new data
	if((txDataW + 1) % TX_QUEUE_SIZE != txDataR) // not full
	{
		txData[txDataW] = data;
		txDataW = (txDataW + 1) % TX_QUEUE_SIZE;
		if(daliState == IDLE)
		{
			DALIProcessSendData(txData[txDataR]);
			txDataR = (txDataR + 1) % TX_QUEUE_SIZE;
		}
		return 0;
	}
	else // full
	{
		if(daliState == IDLE)
		{
			DALIProcessSendData(txData[txDataR]);
			txDataR = (txDataR + 1) % TX_QUEUE_SIZE;
		}
		return 1;
	}
}
void DALIProcessSendData(struct DALITxData txdata)
{
	DALIFlags.sendTwiceFrame = txdata.sendTwice;
	DALIFlags.txFrameType = txdata.frameType;
	DALIFlags.txError = 0;
	DALIFlags.txDone = 0;
	if(DALIFlags.txFrameType == 1) // 8 bit frame
	{
		txPacket = txdata.frame << 16;
	}
	else
	{
		if (DALIFlags.deviceMode == 0) // 24 bit frame
		{
			txPacket = txdata.frame;
		}
		else
		{
			txPacket = txdata.frame << 8; // 16 bit frame
		}
	}
	if(DALIFlags.sendTwiceFrame == 1)
	{
		// Save the packet in case new data come in before sending the secode send-twice frame
		txPacket_temp = txPacket;
	}
	halfBitNumber = 1;
	daliState = SEND_DATA;
	writePin(TX_Pin, DALI_LO);
	reset_timer(&htim2);
	set_timer_reload_val(TE, &htim2);
	enable_timer_int(&htim2);

}

uint8_t DALIDataAvailable(void)
{
    return rxDataR != rxDataW;
}

struct DALIRxData DALIReceiveData(void)
{
    rxDataR = (rxDataR + 1) % RX_QUEUE_SIZE;
    return rxData[rxDataR-1];
}

uint8_t DALIReceiveDataFlags(void)
{
    return flagsData[rxDataR].flagsByte;
}

uint16_t DALIReadFlags(void)
{
    return DALIFlags.flags_all;
}

uint8_t DALIReadState(void)
{
    return daliState;
}

void DALIClearFlags(void)
{
    uint8_t a;
    a = DALIFlags.cableConnected;
    DALIFlags.flags_all = 0;
    DALIFlags.cableConnected = a;
}

void DALIAppendToQueue(void)
{
    // When the state machine finishes receiving data or encounters an error it
    // uses this function to submit the data to the buffer such that the
    // application can grab it. The flags that are presented to the application
    // are a digested version of the state machine flags at the moment this
    // function is called.

    // Make sure we don't start overwriting old data that hasn't yet been read by
    // the application.
	if((daliState == SEND_DATA) || (daliState == BREAK))
	{
		txFlags.txDone = DALIFlags.txDone;
		txFlags.txError = DALIFlags.txError;
	}
	else if((daliState == RECEIVE_DATA) || (daliState == RECEIVE_DATA_EXTRA_TE) || (daliState == WAIT_FOR_SECOND_FORFRAME) ||(daliState == BREAK))
	{
	    if ((rxDataW + 1) % RX_QUEUE_SIZE != rxDataR)
	    {
	        // Fill data in
	        rxData[rxDataW].frame 				= rxFrame;
	        rxData[rxDataW].frameLen 			= rxPacketLen;
	        rxData[rxDataW].frameType			= DALIFlags.rxFrameType;
	        rxData[rxDataW].rxDone				= DALIFlags.rxDone;
	        rxData[rxDataW].rxError				= DALIFlags.rxError;
	        rxData[rxDataW].rxSendTwicePossible = DALIFlags.rxSendTwicePossible;

	        // Advance writing head on the circular buffer
	        rxDataW = (rxDataW + 1) % RX_QUEUE_SIZE;
	    }
	}
    // Clear machine flags such that the next frame will not inherit garbage.
    DALIClearFlags();
}

void DALIReceiveTwice()
{
	DALIFlags.receiveTwiceFrame = 1;
}

