/**
 * \file
 *
 * \brief HAL(Hardware abstract layer)
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

void dali_hal_enable_forward_disable_backward(void)
{
	/* Enable dali data input detection */
	DALI_RX_PORT.DALI_RX_PIN_CTRL = 0x01;
	/* Disable dali data input timer */
	TCB0.INTCTRL = 0x00;
}

void dali_hal_disable_forward_enable_backward(void)
{
	/* Disable dali data input detection */
	DALI_RX_PORT.DALI_RX_PIN_CTRL = 0x00;
	/* Enable dali data output timer */
	TCB0.INTFLAGS = 0x01; // clear the exiting flag
	TCB0.INTCTRL  = 0x01;
}

bool dali_hal_get_dali_input_level(void)
{
	return (DALI_RX_PORT.IN & (0x01 << DALI_RX_PIN));
}

void dali_hal_update_pwm_output(uint16_t pwm_value)
{
	/* Update CC value in TCD */
	TCD0.CMPBSET = pwm_value ^ (uint16_t)DALI_PWM_TC_TOP;// встановлення величини порівняння
	TCD0.CTRLE   = 1; // Забезпечує подвійну буферизацію
}

uint32_t dali_hal_get_seed0_value(void)
{
	/* TCC0 count register as seed */
	return TCA0.SINGLE.CNT;
}

uint32_t dali_hal_get_seed1_value(void)
{
	/* TC6 count register as seed */
	return RTC.CNT;
}

void dali_hal_save_persistent_variables(void)
{
	/* Execute update immediately */
	dali_eeprom_update_tick();
}

void dali_hal_identify_device(void)
{
	// identify here
}
