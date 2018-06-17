/**
 * \file
 *
 * \brief Process dali eeprom
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
#ifndef DALI_EEPROM_H
#define DALI_EEPROM_H

/**
 * \brief load dali data from EEPROM memory
 *
 * Read and check EEPROM page of data from EEPROM memory.
 *
 * \param[in]  ram_addr  Destination ram address to store
 * \param[in] size       The dali data size
 *
 * \return Status code indicating the status of the operation.
 *
 * \retval true    If the data was successfully read
 * \retval false   If the EEPROM is not initialized
 */
bool dali_load_from_eeprom(uint8_t *ram_addr, uint8_t size);

/**
 * \brief Update dali data to EEPROM memory
 *
 * Program dali data to the EEPROM memory space.
 *
 * \param[in] ram_addr   Source ram address to load from
 * \param[in] size       The dali data size
 * \param[in] update_flag   Address of update flag which indicates update needs or not
 */
void dali_update_to_eeprom(uint8_t *ram_addr, uint8_t size, uint8_t *update_flag);

void dali_eeprom_update_tick(void);

#endif /* DALI_EEPROM_H */
