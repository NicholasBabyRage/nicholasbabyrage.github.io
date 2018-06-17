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

#ifndef __HARDWARE_ABSTRACT_LAYER__H__
#define __HARDWARE_ABSTRACT_LAYER__H__

/**
 * \brief Enable the forward frame decode,
 *  disable backward encode
 */
void dali_hal_enable_forward_disable_backward(void);

/**
 * \brief Disable the forward frame decode,
 *  enable backward encode
 */
void dali_hal_disable_forward_enable_backward(void);

/**
 * \brief Get the dali input pin level
 */
bool dali_hal_get_dali_input_level(void);

/**
 * \brief Update TC PWM output to lamp
 */
void dali_hal_update_pwm_output(uint16_t pwm_value);

/**
 * \brief Get the seed value
 */
uint32_t dali_hal_get_seed0_value(void);
uint32_t dali_hal_get_seed1_value(void);

/**
 * \brief Save persistent variables in page banks
 */
void dali_hal_save_persistent_variables(void);

/**
 * \brief Execute the identification by flashing, sound or other visual or audible means
 */
void dali_hal_identify_device(void);

#endif /* __HARDWARE_ABSTRACT_LAYER__H__ */
