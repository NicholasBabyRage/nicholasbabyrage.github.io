/**
 * \file
 *
 * \brief Functions that used in application layer
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
#ifndef DALI_TOP_H //ƒиректива дл€ захисту в≥д багатократного включенн€. якщо dali_top.h вже була включена
#define DALI_TOP_H  // то цей файл не буде об'€вл€тис€ знову.

#if defined(__GNUC__)  // якщо використовуЇтьс€ комп≥л€тор GNU Compiler Colection
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#elif defined(__ICCAVR__) // якщо використовуЇтьс€ комп≥л€тор ICCAVR
#define __ISR(x) _Pragma(#x)
#define ISR(vect) __ISR(vector = vect) __interrupt void handler_##vect(void)
#endif

#if defined(__GNUC__)
#define EEMEM __attribute__((section(".eeprom")))
#elif defined(__ICCAVR__)
#define EEMEM __eeprom
#define PROGMEM __flash
#endif

#if defined(__ICCAVR__)
#define cli() asm("cli")
#define sei() asm("sei")
#endif

#include "compiler.h"
#include "dali_bit.h"
#include "dali_tc.h"
#include "dali_eeprom.h"
#include "dali_bod.h"

#define SYSTEM_5V // else 3.3V system
/*Select CLK_MAIN by MCU FUSE and configure with clock prescaler*/
#ifdef SYSTEM_5V
#define CLK_CPU 16000000UL
#else
#define CLK_CPU 8000000UL
#endif

/*Update flag*/
#define VAR_UPDATE (1 << 0)   // dali variable
#define BANK0_UPDATE (1 << 1) // dali bank0
#define BANK1_UPDATE (1 << 2) // dali bank1

/**
 *  \brief Set the DALI bytes (address and data) to stack after decoding.
 */
void dali_set_addr_to_stack(uint8_t address);
void dali_set_data_to_stack(uint8_t data);

/**
 *  \brief Set the DALI bytes received flag to stack after decoding.
 */
void dali_set_received_flag_to_stack(bool flag);

/**
 *  \brief Set current DALI byte sent status to stack when encoding.
 */
void dali_set_sent_status_to_stack(uint8_t status);

/**
 *  \brief Get the DALI byte sent status from stack to start encoding.
 */
uint8_t dali_get_sent_status_from_stack(void);

/**
 *  \brief Get the DALI sent byte from stack when encoding.
 */
uint8_t dali_get_sent_data_from_stack(void);

/**
 *  \brief Get the EEPROM update flag address from stack.
 */
uint8_t *dali_get_update_flag_addr_from_stack(void);

/**
 *  \brief Get the data address and size from stack for EEPROM write.
 */
uint8_t *dali_get_data_addr_from_stack(void);
uint8_t  dali_get_data_size_from_stack(void);

/**
 *  \brief Set the control gear failure status to stack.
 */
void dali_set_gear_failure_status_to_stack(bool failure_status);

/**
 *  \brief Set the lamp failure status to stack.
 */
void dali_set_lamp_failure_status_to_stack(bool failure_status);

/**
 *  \brief Set the dali bus bits receive state to stack.
 */
uint8_t dali_set_bus_receive_state_to_stack(void);

/**
 *  \brief process dali fading
 */
void dali_command_fade_ms_tick(void);

/**
 *  \brief Initialize dali frame state.
 */
void dali_frame_init(void);

/**
 *  \brief Process the dali frame transmission
 */
void dali_process_frame_state(void);

/**
 *  \brief Ms tick used by dali frame
 */
void dali_frame_ms_tick(void);

/**
 *  \brief Process state after power on.
 */
void dali_process_power_on(void);

/**
 *  \brief Process the interface failure state.
 */
void dali_process_interface_failure(void);

/**
 *  \brief Process the identify device state.
 */
void dali_process_identify_device(void);

#endif /* DALI_TOP_H */
