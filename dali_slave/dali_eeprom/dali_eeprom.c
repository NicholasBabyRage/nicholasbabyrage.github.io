/**
 * \file
 *
 * \brief Process dali slave emulator EEPROM
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

/*The front three pages are used to store variable, DALI BANK0 and BANK1 data */
/*The fourth page acts as backup page when programming eeprom*/
#define EEPROM_BACKUP (EEPROM_START + 3 * EEPROM_PAGE_SIZE)
/*The fifth page is used for variable.target_level to lower NVM wear (Non Volatile Memory) */
#define EEPROM_TARGET_LEVEL (EEPROM_BACKUP + EEPROM_PAGE_SIZE)
/*Indicates backup page is empty with all inactive 0xFF*/
#define BACKUP_PAGE_EMPTY 1
/*Data page index 0~2*/
uint8_t page_index = 0;
#define MAX_DATA_PAGE 2 // Page index 0,1,2 which set according to dali data length in sram

enum { UPDATE_IDLE, UPDATE_INIT, CHECK_AND_UPDATE_TARGET, UPDATE_BACKUP };
uint8_t update_state = UPDATE_IDLE;
/*One buffer to copy data to destination page and backup*/
uint8_t eeprom_page_buf[EEPROM_PAGE_SIZE];

/*Read out the target level from the fifth eeprom page*/
uint8_t dali_read_target_level_from_eeprom(void);
/*Write the target level to the fifth eeprom page*/
void dali_write_target_level_to_eeprom(uint8_t level);

bool dali_load_from_eeprom(uint8_t *ram_addr, uint8_t size)
{
	uint8_t  i, j, eeprom_flag = 0, cnt = 0;
	uint8_t *eeprom_addr, *backup_addr;

	/*Check backup eeprom content firstly*/
	backup_addr = (uint8_t *)EEPROM_BACKUP;
	for (i = 0; i < EEPROM_PAGE_SIZE; i++) {
		if (*(backup_addr + i) == 0xFF) {
			cnt++;
		} else {
			break;
		}
	}
	if (cnt == EEPROM_PAGE_SIZE) {
		/*All 0xFF, no data*/
		eeprom_flag = BACKUP_PAGE_EMPTY;
	}

	/*Check front three eeprom pages*/
	eeprom_addr = (uint8_t *)EEPROM_START;
	for (j = 0; j < 3; j++) { // 3 pages
		cnt = 0;
		for (i = 0; i < EEPROM_PAGE_SIZE; i++) {
			if (*(eeprom_addr + i) == 0xFF) {
				cnt++;
			} else {
				break;
			}
		}
		if (cnt == EEPROM_PAGE_SIZE) {
			/*All 0xFF, no data*/
			if (eeprom_flag == BACKUP_PAGE_EMPTY) {
				/*If current and backup pages are all 0xFF, not initialized*/
				return false;
			} else {
				/*Need to copy backup page*/
				for (i = 0; i < EEPROM_PAGE_SIZE; i++) {
					*(eeprom_addr + i) = *(backup_addr + i);
				}
				/*Write eeprom*/
				while (NVMCTRL.STATUS & 0x02)  
					;
				cli();
				CCP = 0x9D;
				/*Don't need to erase page, only program*/
				NVMCTRL.CTRLA = 0x01;
				sei();
				/*Wait until eeprom programming done*/
				while (NVMCTRL.STATUS & 0x02)
					;
				break;
			}
		}
		eeprom_addr += EEPROM_PAGE_SIZE;
	}

	/*Target level stores in address ram_addr*/
	*ram_addr = dali_read_target_level_from_eeprom();
	/*Restore data from eeprom to sram*/
	eeprom_addr = (uint8_t *)EEPROM_START;
	for (i = 0; i < size - 1; i++) {
		/*Store to ram address from address 1 as 0 is for target level */
		*(ram_addr + 1 + i) = *(eeprom_addr + i);
	}

	return true;
}

void dali_update_to_eeprom(uint8_t *ram_addr, uint8_t size, uint8_t *update_flag)
{
	uint8_t  i;
	uint8_t *eeprom_addr;
	uint8_t *data_addr;
	uint8_t  update_size;
	bool     page_need_update;

	if (*update_flag != 0) {
		*update_flag = 0;
		/* restart updating all pages from first page */
		update_state = UPDATE_INIT;
	}

	switch (update_state) {
	case UPDATE_INIT:
		/* Initialization */
		if (*ram_addr != dali_read_target_level_from_eeprom()) {
			dali_write_target_level_to_eeprom(*ram_addr);
		}
		update_state = CHECK_AND_UPDATE_TARGET;
		page_index   = 0;
		break;

	case CHECK_AND_UPDATE_TARGET:
		page_need_update = false;
		eeprom_addr      = (uint8_t *)(EEPROM_START + page_index * EEPROM_PAGE_SIZE);
		/*Start from ram_addr + 1 as ram_addr stores target level*/
		data_addr = ram_addr + 1 + page_index * EEPROM_PAGE_SIZE;
		if (page_index >= MAX_DATA_PAGE) {
			update_size = size - (MAX_DATA_PAGE * EEPROM_PAGE_SIZE); // The last may less one page
			page_index  = 0;
		} else {
			update_size = EEPROM_PAGE_SIZE;
			page_index++;
		}
		/*copy from eeprom to buffer*/
		for (i = 0; i < update_size; i++) {
			*(eeprom_page_buf + i) = *(eeprom_addr + i);
		}
		/*Check if update needed or not*/
		for (i = 0; i < update_size; i++) {
			if (*(eeprom_page_buf + i) != *(data_addr + i)) {
				*(eeprom_page_buf + i) = *(data_addr + i);
				page_need_update       = true;
			}
		}

		/*If there need to update sram to eeprom*/
		if (page_need_update == true) {
			/* copy the whole page once*/
			for (i = 0; i < EEPROM_PAGE_SIZE; i++) {
				*(eeprom_addr + i) = *(eeprom_page_buf + i);
			}

			while (NVMCTRL.STATUS & 0x02)
				;
			cli();
			CCP           = 0x9D;
			NVMCTRL.CTRLA = 0x03;
			sei();
			update_state = UPDATE_BACKUP;
		} else {
			if (page_index == 0) { // all pages checked
				update_state = UPDATE_IDLE;
			}
		}
		break;

	case UPDATE_BACKUP:
		eeprom_addr = (uint8_t *)EEPROM_BACKUP;
		/* Need to copy the whole page */
		for (i = 0; i < EEPROM_PAGE_SIZE; i++) {
			*(eeprom_addr + i) = *(eeprom_page_buf + i);
		}
		/*Program backup page*/
		while (NVMCTRL.STATUS & 0x02)
			;
		cli();
		CCP           = 0x9D;
		NVMCTRL.CTRLA = 0x03;
		sei();

		if (page_index == 0) { // all pages checked
			update_state = UPDATE_IDLE;
		} else {
			update_state = CHECK_AND_UPDATE_TARGET;
		}

		break;

	case UPDATE_IDLE:
	// all pages checked
	default:
		break;
	}
}

uint8_t dali_read_target_level_from_eeprom(void)
{
	uint8_t  i;
	uint8_t *eeprom_addr;

	eeprom_addr = (uint8_t *)EEPROM_TARGET_LEVEL;
	for (i = 0; i < EEPROM_PAGE_SIZE; i++) {
		if (*(eeprom_addr + i) == 0xFF) {
			if (i == 0) {
				/*Uninitialized state, shouldn't goto here*/
				return 0xFF;
			} else {
				/*Get the last non 0xff value*/
				return (*(eeprom_addr + i - 1));
			}
		}
	}
	/*When all are non 0xff, get the last one in page*/
	return (*(eeprom_addr + EEPROM_PAGE_SIZE - 1));
}

void dali_write_target_level_to_eeprom(uint8_t level)
{
	uint8_t  i, j;
	uint8_t *eeprom_addr;

	eeprom_addr = (uint8_t *)EEPROM_TARGET_LEVEL;
	for (i = 0; i < EEPROM_PAGE_SIZE; i++) {
		if (*(eeprom_addr + i) == 0xFF) {
			/*Update the data into eeprom page buffer*/
			*(eeprom_addr + i) = level;
			/*Write page*/
			while (NVMCTRL.STATUS & 0x02)
				;
			cli();
			CCP           = 0x9D;
			NVMCTRL.CTRLA = 0x01;
			sei();
			break;
		} else if (i == EEPROM_PAGE_SIZE - 1) {
			/* When all page are valid data, then erase and re-write */
			*eeprom_addr = level;
			for (j = 1; j < EEPROM_PAGE_SIZE; j++) {
				/*Update the other bytes in eeprom page buffer to 0xff */
				*(eeprom_addr + j) = 0xFF;
			}
			/*Erases and write page*/
			while (NVMCTRL.STATUS & 0x02)
				;
			cli();
			CCP           = 0x9D;
			NVMCTRL.CTRLA = 0x03;
			sei();
		}
	}
}

void dali_eeprom_update_tick(void)
{
	dali_update_to_eeprom(
	    dali_get_data_addr_from_stack(), dali_get_data_size_from_stack(), dali_get_update_flag_addr_from_stack());
}
