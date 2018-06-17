/**
 * \file
 *
 * \brief Decode and encode dali transmission bits
 *
 * Copyright (c) 2016 Atmel Corporation. All rights reserved.
 *
 * \asf_license_start
 *
 * \page License
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. The name of Atmel may not be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * 4. This software may only be redistributed and used in connection with an
 *    Atmel microcontroller product.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \asf_license_stop
 *
 */
#include "dali_top.h"

#define BIT_START 0xFF
#define BIT_0 0x01
#define BIT_1 0x02
#define BIT_2 0x03
#define BIT_3 0x04
#define BIT_4 0x05
#define BIT_5 0x06
#define BIT_6 0x07
#define BIT_7 0x08
#define BIT_8 0x09
#define BIT_9 0x0A
#define BIT_10 0x0B
#define BIT_11 0x0C
#define BIT_12 0x0D
#define BIT_13 0x0E
#define BIT_14 0x0F
#define BIT_15 0x10
#define BIT_STOP1 0x11
#define BIT_STOP2 0x12

/*One TE is 1/2.4KHz = 416uS*/
#define MIN_TE_CNT (TE_CNT - 4) // tolerant low limit
#define MAX_TE_CNT (TE_CNT + 4) // tolerant high limit
#define MIN_2TE_CNT (2 * MIN_TE_CNT)
#define MAX_2TE_CNT (2 * MAX_TE_CNT)
#define STOP_CNT (4 * TE_CNT)

/*DALI input pin level*/
volatile bool pin_level;
/*Level keep time*/
volatile uint8_t level_time;
/*DALI bits receiving status */
volatile uint8_t status_receive;
/*Received two bytes*/
volatile uint8_t dali_rec_addr, dali_rec_data;
/*DALI bits sending status*/
volatile uint8_t send_status;
/*The sending byte*/
volatile uint8_t dali_send_data;

void dali_bit_init(void)
{
	/* Налаштування DALI виходу */
	DALI_TX_PORT.DIRSET = 0x01 << DALI_TX_PIN; // Встановлення піна  на вихід (лог."1")
	DALI_TX_PORT.OUTSET = 0x01 << DALI_TX_PIN; //  Встановлення високого (лог."1") рівня піна 
	/* Налаштування DALI  входу */
	DALI_RX_PORT.DIRCLR = 0x01 << DALI_RX_PIN; // Встановлення піна  на вхід лог."0"
	/*Встановлення піну RX джерелом переривання по спадаючому й наростаючому краю*/
	DALI_RX_PORT.DALI_RX_PIN_CTRL = 0x01; // Встановлення піну джерелом переривання
}

/**
 * \brief External pin interrupt handler
 *
 * This is the handler for external pin interrupt
 */
ISR(DALI_RX_VECT)
{
	static uint8_t bit_index;
	uint8_t        dali_bit_rx;

	DALI_RX_PORT.INTFLAGS = 0x01 << DALI_RX_PIN;
	TCB0.CNT              = 0;
	pin_level             = DALI_RX_PORT.IN & (0x01 << DALI_RX_PIN);
	if (status_receive == 0) {
		if (pin_level == LOW) {
			/* DALI bus falling edge indicates start bit */
			TCB0.INTFLAGS  = 0x01; // clear the exiting flag
			TCB0.INTCTRL   = 0x01; // Enable TCB timing
			bit_index      = 0;
			status_receive = BIT_START;
			dali_rec_addr  = 0;
			dali_rec_data  = 0;
		}
	} else if (status_receive == BIT_START) {
		/* DALI pin must be high after the second INT0 edge */
		if ((level_time >= MIN_TE_CNT) && (level_time <= MAX_TE_CNT)) {
			/* Get the start bit and get ready for 16 bit data. */
			status_receive = BIT_0;
			bit_index += 1;
		} else {
			/* Start bit error */
			status_receive = 0;
		}
	} else if (status_receive < BIT_STOP1) {
		if (level_time >= MIN_2TE_CNT) {
			if (bit_index & 0x01) {
				/* Long level (2xTe) is detected */
				bit_index += 2;

			} else {
				// Error as it is front half of the bit
				status_receive = 0;
			}
		} else if ((level_time >= MIN_TE_CNT) && (level_time <= MAX_TE_CNT)) {
			/* Short level (1xTe) is detected */
			bit_index += 1;
		} else {
			status_receive = 0;
		}

		if (bit_index >= 34) {
			/* If the last Te is low (dali bit 0), a rising edge is
			 * detected before stop bit */
			status_receive = BIT_STOP1;
		}

		/* Decode dali bit at every second Te bit */
		if (bit_index & 0x01) {
			/* Shift out the lowest bit to get dali bit */
			dali_bit_rx = bit_index >> 1;
			if (dali_bit_rx <= BIT_7) {
				/* Get the address byte */
				dali_rec_addr <<= 1;
				if (pin_level) {
					dali_rec_addr |= 0x01;
				}
			} else {
				/* Get the data byte */
				dali_rec_data <<= 1;
				if (pin_level) {
					dali_rec_data |= 0x01;
					if (dali_bit_rx == BIT_15) {
						/* If the last Te is high (dali bit 1), a Te
						 * high period is added before stop bit */
						status_receive = BIT_STOP2;
					}
				}
			}
		}
	} else {
		status_receive = 0;
	}
	level_time = 0;
}

ISR(TCB0_INT_vect)
{
	static uint8_t dali_bit_index;
	static uint8_t bit_length;
	uint8_t        dali_bit_tx;
	uint8_t        bit_length_temp;

	/*Clear interrupt flag firstly*/
	TCB0.INTFLAGS = 0x01;
	send_status   = dali_get_sent_status_from_stack();
	/* Dali decoding */
	if (status_receive) {
		level_time++;
		if (status_receive == BIT_STOP2) {
			if (pin_level == HIGH) {
				if (level_time > (STOP_CNT + TE_CNT)) {
					/* Stop bit */
					status_receive = 0;
					/* Receive complete */
					dali_set_addr_to_stack(dali_rec_addr);
					dali_set_data_to_stack(dali_rec_data);
					dali_set_received_flag_to_stack(true);
				}
			} else {
				/* Stop bit error if dali pin low */
				status_receive = 0;
			}
		} else if (status_receive == BIT_STOP1) {
			if (pin_level == HIGH) {
				if (level_time > STOP_CNT) {
					/* Stop bit */
					status_receive = 0;
					/* Receive complete */
					dali_set_addr_to_stack(dali_rec_addr);
					dali_set_data_to_stack(dali_rec_data);
					dali_set_received_flag_to_stack(true);
				}
			} else {
				/* Stop bit error if dali pin low */
				status_receive = 0;
			}
		} else {
			if (level_time >= MAX_2TE_CNT) {
				/* Error */
				status_receive = 0;
			}
		}
	} else if (send_status) { /* Dali encoding */
		dali_send_data = dali_get_sent_data_from_stack();
		/* Send dali backward frame */
		dali_bit_tx     = dali_bit_index;
		bit_length_temp = bit_length;
		bit_length_temp++;
		if (send_status != BIT_STOP1) {
			/* Backward data byte */
			if (dali_bit_tx == 0) {
				/* Start bit */
				if (bit_length_temp == 1) {
					DALI_TX_PORT.OUTCLR = 0x01 << DALI_TX_PIN; // Low level
				} else if (bit_length_temp == (TE_CNT + 1)) {
					DALI_TX_PORT.OUTSET = 0x01 << DALI_TX_PIN; // High level
				}
			} else if (dali_send_data & (1 << (BIT_7 - dali_bit_tx))) {
				/* DALI bit value 1*/
				if (bit_length_temp == 1) {
					DALI_TX_PORT.OUTCLR = 0x01 << DALI_TX_PIN; // Low level
				} else if (bit_length_temp == (TE_CNT + 1)) {
					DALI_TX_PORT.OUTSET = 0x01 << DALI_TX_PIN; // High level
				}
			} else {
				/* DALI bit value 0 */
				if (bit_length_temp == 1) {
					DALI_TX_PORT.OUTSET = 0x01 << DALI_TX_PIN; // High level
				} else if (bit_length_temp == (TE_CNT + 1)) {
					DALI_TX_PORT.OUTCLR = 0x01 << DALI_TX_PIN; // Low level
				}
			}

			if (bit_length_temp == (2 * TE_CNT)) {
				dali_bit_tx++;
				if (dali_bit_tx > BIT_7) {
					send_status = BIT_STOP1;
					dali_bit_tx = 0;
				}
				bit_length_temp = 0;
			}
		} else {
			/* Stop bit */
			if (bit_length_temp < (4 * TE_CNT)) {
				DALI_TX_PORT.OUTSET = 0x01 << DALI_TX_PIN;  // High level
			} else if (bit_length_temp >= (4.5 * TE_CNT)) { // Receive frame 1.9ms after backward frame
				                                            /* Send complete */
				send_status     = SENDING_STOP;
				bit_length_temp = 0;
			}
		}
		dali_set_sent_status_to_stack(send_status);
		bit_length     = bit_length_temp;
		dali_bit_index = dali_bit_tx;
	} else {
		/* When no encoding/decoding, stop this timer */
		TCB0.INTCTRL = 0x00;
	}
}
