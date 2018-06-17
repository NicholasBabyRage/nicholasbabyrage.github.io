/**
 * \file
 *
 * \brief Bod interrupt is used to save power before power off
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
#include "dali_hal.h"

/*Initialize BOD function*/
void dali_bod_init(void)
{
	/* In FUSE setting, for 5V system, BOD enable and level 4.0V
	   For 3.3V, BOD enable and level 2.6V
	*/

	/*Sampled mode in active and sleep, set in FUSE*/
	// BOD_CTRLA = BOD_ACTIVE0_bm|BOD_SLEEP1_bm;
	/*VLM threshold 15% above BOD threshold, level is 3.94*1.15=4.53V */
	BOD_VLMCTRLA = BOD_VLMLVL0_bm;
	/*Interrupt Enable*/
	BOD_INTCTRL = BOD_VLMIE_bm;
}

/* BOD ISR before power off */
ISR(BOD_VLM_vect)
{
	/*Clear interrupt flag*/
	BOD_INTFLAGS = 1;
	/*Power off all leds to save power*/
	dali_hal_update_pwm_output(0);
	PORTA_OUTCLR = 0x08;
}
