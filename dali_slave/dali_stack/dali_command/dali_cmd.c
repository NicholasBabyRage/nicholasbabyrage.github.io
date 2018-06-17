/**
 * \file
 *
 * \brief Process commands dali slave received
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
#include "dali_frame.h"
#include "dali_cmd.h"
#include "adc_window.h"


#define W200MS (200 * ONE_MS_TICK)
#define W500MS (500 * ONE_MS_TICK)
#define W600MS (600 * ONE_MS_TICK) // Must active after 660mS, 60mS ahead
#define W10S ((uint32_t)10 * (uint32_t)1000 * ONE_MS_TICK)
#define W15MIN ((uint32_t)15 * (uint32_t)60 * (uint32_t)1000 * ONE_MS_TICK)

#define FAIL_TIME W500MS
/* For power level */
#define POWER_OFF 0x00
#define POWER_MASK 0xFF
#define POWER_MIN 0x01
#define POWER_MAX 0xFE

#define ADDR_MASK 0xFF

#define LOGARITHMIC 0x00
#define LINEAR 0x01

#define LIGHT_SOURCE 6 // LED type

/* Values for int_fade_direction */
enum {
	FADE_IDLE,
	FADE_STOP,
	FADE_TIME_UP,
	FADE_TIME_DOWN,
	FADE_TIME_DOWN_OFF,
	FADE_RATE_UP,
	FADE_RATE_DOWN,
	FADE_DIRECT,
	FADE_DIRECT_IDENTIFICATION,
	FADE_PENDING,
	FADE_BETWEEN_MINLEVEL_AND_OFF
};

bool write_memory_enabled = false;

/* Bank index */
#define BANK0 0x00
#define BANK1 0x01

/* Following two banks is located in page 1 */
/* Memory map of memory bank0 */

#define BANK0_LAST_MEMORY_LOCATION 0x1A
#define NUMBER_OF_LAST_BANK 0x01
#define GTIN0 0xFF
#define GTIN1 0xFF
#define GTIN2 0xFF
#define GTIN3 0xFF
#define GTIN4 0xFF
#define GTIN5 0xFF
#define FIRMWARE_MAJOR_VER 0x00
#define FIRMWARE_MINOR_VER 0x01
#define ID_NUMBER0 0xFF
#define ID_NUMBER1 0xFF
#define ID_NUMBER2 0xFF
#define ID_NUMBER3 0xFF
#define ID_NUMBER4 0xFF
#define ID_NUMBER5 0xFF
#define ID_NUMBER6 0xFF
#define ID_NUMBER7 0xFF
#define HARDWARE_MAJOR_VER 0x00
#define HARDWARE_MINOR_VER 0x01
#define NUM_OF_101_VER 0x08 // 2.0. x(bits 7-2).y(bits 1-0)
#define NUM_OF_102_VER 0x08
#define NUM_OF_103_VER 0x08
#define CONTROL_DEVICE_NUM 0x01
#define CONTROL_GEAR_NUM 0x01
#define CONTROL_GEAR_INDEX 0x00

/* Memory map of memory bank1 */
#define NOT_LOCK 0x55

#define BANK1_LAST_MEMORY_LOCATION 0x10
#define INDICATOR 0x0E
#define BANK1_LOCK_BYTE 0xFF
#define OEM_GTIN0 0xFF
#define OEM_GTIN1 0xFF
#define OEM_GTIN2 0xFF
#define OEM_GTIN3 0xFF
#define OEM_GTIN4 0xFF
#define OEM_GTIN5 0xFF
#define OEM_ID_NUMBER0 0xFF
#define OEM_ID_NUMBER1 0xFF
#define OEM_ID_NUMBER2 0xFF
#define OEM_ID_NUMBER3 0xFF
#define OEM_ID_NUMBER4 0xFF
#define OEM_ID_NUMBER5 0xFF
#define OEM_ID_NUMBER6 0xFF
#define OEM_ID_NUMBER7 0xFF

struct uint24 search_address;

struct dali_data dali_ram;

/* Define the bits of features */
#define SHORT_CIRCUIT_CAN_BE_QUERIED BIT0
#define OPEN_CIRCUIT_CAN_BE_QUERIED BIT1
#define LOAD_DECREASE_CAN_BE_QUERIED BIT2
#define LOAD_INCREASE_CAN_BE_QUERIED BIT3
#define CURRENT_PROTECTOR_CAN_BE_QUERIED BIT4
#define THERMAL_SHUT_DOWN_CAN_BE_QUERIED BIT5
#define LIGHT_LEVEL_REDUCTION BIT6
#define PHYSICAL_SELECTION_SUPPORTED BIT7

/* Define bit4 of LED module operating mode */
#define NON_LOGARITHMIC_DIMMING_CURVE_ACTIVE BIT4
uint8_t led_module_operating_mode = 0;

/* Used during fade - the step size added to dali.long_level for each INT.
 *  Equal to (flash_inv_fadetime_val) times (change in dali value) */
uint32_t fade_step;

/* Using signed notation on this one, as we are testing for "less than 0". */
uint8_t current_fade_time;

#define W700MS ((uint32_t)700 * ONE_MS_TICK)
#define W1S ((uint32_t)1 * (uint32_t)1000 * ONE_MS_TICK)
/* The fade time between off and minlevel state */
uint32_t fade_time_off_and_minlevel;

uint32_t w15min_waiting_time = 0;
/* Used for identify device */
uint32_t w10s_identify_time = 0;
bool     identify_flag      = false;

bool     interface_fail_flag = false;
uint16_t interface_fail_time = 0;

uint8_t fade_direction = FADE_IDLE;
/* Data Transfer Register used by dali configuration commands. */
uint8_t dtr0 = 0x00;
uint8_t dtr1 = 0x00;
uint8_t dtr2 = 0x00;

uint16_t power_on_time = W600MS;
bool     power_on_flag = true;
/* Used by commands 260-261 */
uint8_t withdraw = 0;
uint8_t optional_features;

bool    device_enabled = false;
bool    dapc_sequence  = false;
uint8_t dapc_sequence_time;
bool    need_fade = false;
uint8_t status_information;
/* EEPROM page update flag */
uint8_t update_flag;

/* dali.level and dali.long_level are the same values, writing to one also
 * updates the other. */
union daliunion {
	/* dali.long_level is used during fade - allows smoother fading between
	 * fixed dali levels. */
	uint32_t long_level;
	struct {
		/* Lower 8 */
		uint8_t byte0;
		uint8_t byte1;
		uint8_t byte2;
		/* Upper 8 bits dali.level is the actual dali level, and gets stored
		 * in EEPROM as well. The remainder is the fractional value. */
		uint8_t level;
	};
} dali;

union uint16_union {
	uint16_t value;
	struct {
		uint8_t lsb;
		uint8_t msb;
	};
};

/* Last active level */
uint8_t last_active_level;

/* Multiplier, range 100ms - 1 min */
uint16_t extended_fade_time_mul[] = {0, 1, 10, 100, 6000};
/* define additional LED module */
#define DEVICE_TYPE LED_MODULES
#define EXTENDED_VER_NUM 1
#define MAX_FAST_FADE_TIME 27

uint8_t current_fast_fade_time;
uint8_t failure_status = 0;
#define SHORT_CIRCUIT BIT0
#define OPEN_CIRCUIT BIT1
#define LOAD_DECREASE BIT2
#define LOAD_INCREASE BIT3
#define CURRENT_PROTECTOR_ACTIVE BIT4
#define THERMAL_SHUT_DOWN BIT5
#define THREMAL_OVERLOAD BIT6
#define REFERRENCE_MEASUREMENT_FAILED BIT7

/* Manufactuers mode */
#define MANUFACTURER_ACCESS_MODE 0x80

/* Command flag of Recall Max/Min Level during identification process */
uint8_t recall_level_flag = 0;
#define MIN_LEVEL_FLAG 0
#define MAX_LEVEL_FLAG 1

/* 12-bit PWM values calculated according to dali standard. The 4 high bits are
 * stuffed 2 in a byte {#1#0, #3#2,....,#255#254} */
PROGMEM uint8_t const flash_high_pwm_val[]
    = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
       0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
       0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
       0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
       0x10, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22,
       0x22, 0x32, 0x33, 0x33, 0x33, 0x33, 0x33, 0x44, 0x44, 0x44, 0x44, 0x55, 0x55, 0x55, 0x65, 0x66, 0x66, 0x77, 0x77,
       0x87, 0x88, 0x98, 0x99, 0xA9, 0xAA, 0xBA, 0xBB, 0xCC, 0xDC, 0xDD, 0xEE, 0xFF, 0xFF};

PROGMEM uint8_t const flash_low_pwm_val[]
    = {0x00, 0x04, 0x04, 0x04, 0x04, 0x05, 0x05, 0x05, 0x05, 0x05, 0x05, 0x05, 0x06, 0x06, 0x06, 0x06, 0x06, 0x06, 0x07,
       0x07, 0x07, 0x07, 0x07, 0x07, 0x08, 0x08, 0x08, 0x08, 0x09, 0x09, 0x09, 0x09, 0x0A, 0x0A, 0x0A, 0x0A, 0x0B, 0x0B,
       0x0B, 0x0C, 0x0C, 0x0C, 0x0D, 0x0D, 0x0D, 0x0E, 0x0E, 0x0E, 0x0F, 0x0F, 0x10, 0x10, 0x10, 0x11, 0x11, 0x12, 0x12,
       0x13, 0x13, 0x14, 0x15, 0x15, 0x16, 0x16, 0x17, 0x18, 0x18, 0x19, 0x1A, 0x1A, 0x1B, 0x1C, 0x1C, 0x1D, 0x1E, 0x1F,
       0x20, 0x21, 0x22, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27, 0x29, 0x2A, 0x2B, 0x2C, 0x2D, 0x2F, 0x30, 0x31, 0x32, 0x34,
       0x35, 0x37, 0x38, 0x3A, 0x3B, 0x3D, 0x3F, 0x41, 0x42, 0x44, 0x46, 0x48, 0x4A, 0x4C, 0x4E, 0x50, 0x53, 0x55, 0x57,
       0x5A, 0x5C, 0x5F, 0x61, 0x64, 0x67, 0x6A, 0x6C, 0x6F, 0x73, 0x76, 0x79, 0x7C, 0x80, 0x83, 0x87, 0x8B, 0x8E, 0x92,
       0x96, 0x9B, 0x9F, 0xA3, 0xA8, 0xAC, 0xB1, 0xB6, 0xBB, 0xC0, 0xC6, 0xCB, 0xD1, 0xD7, 0xDD, 0xE3, 0xE9, 0xEF, 0xF6,
       0xFD, 0x04, 0x0B, 0x12, 0x1A, 0x22, 0x2A, 0x32, 0x3B, 0x43, 0x4C, 0x55, 0x5F, 0x69, 0x72, 0x7D, 0x87, 0x92, 0x9D,
       0xA9, 0xB4, 0xC1, 0xCD, 0xDA, 0xE7, 0xF4, 0x02, 0x10, 0x1F, 0x2E, 0x3D, 0x4D, 0x5E, 0x6E, 0x80, 0x91, 0xA4, 0xB6,
       0xC9, 0xDD, 0xF1, 0x06, 0x1C, 0x32, 0x48, 0x60, 0x78, 0x90, 0xA9, 0xC3, 0xDE, 0xF9, 0x16, 0x33, 0x50, 0x6F, 0x8E,
       0xAF, 0xD0, 0xF2, 0x15, 0x39, 0x5E, 0x84, 0xAB, 0xD3, 0xFC, 0x27, 0x52, 0x7F, 0xAD, 0xDD, 0x0D, 0x3F, 0x73, 0xA7,
       0xDE, 0x15, 0x4F, 0x89, 0xC6, 0x04, 0x44, 0x86, 0xC9, 0x0E, 0x56, 0x9F, 0xEA, 0x37, 0x87, 0xD9, 0x2D, 0x83, 0xDB,
       0x37, 0x94, 0xF4, 0x57, 0xBD, 0x25, 0x91, 0xFF, 0xFF};

/*
 * Note that the first element in both tables is 0, but really should be
 * "infinity", as it means "no fade".
 * When used, this data is loaded into the 32-bit value "dali.long_level",
 * where the upper byte is the actual dali value and the lower 24 the fraction.
 * Fade rate is in other words used as is (after shift), while inverse fade time
 * is a "change per step" and must be multiplied by the number of dali steps in
 * the fade.
 */
/* Fade rate data is shifted 17 bits up in the table, which means it is shifted
 * 7 bits more when loaded into "dali.long_level". */
PROGMEM uint16_t const flash_fade_rate_val[] = {0x0000,
                                                0xB731,
                                                0x8189,
                                                0x5B99,
                                                0x40C5,
                                                0x2DCC,
                                                0x2062,
                                                0x16E6,
                                                0x1031,
                                                0x0B73,
                                                0x0819,
                                                0x05BA,
                                                0x040C,
                                                0x02DD,
                                                0x0206,
                                                0x016E};

/* Inverse fade time data is shifted up 25 bits, which means it multiplies with
 *  the change in steps then shift 1 bit low when loaded into the
 * "dali.long_level". */
PROGMEM uint16_t const flash_inv_fadetime_val[] = {0x0000,
                                                   0xB964,
                                                   0x8312,
                                                   0x5CB2,
                                                   0x4189,
                                                   0x2E59,
                                                   0x20C5,
                                                   0x172B,
                                                   0x1062,
                                                   0x0B96,
                                                   0x0831,
                                                   0x05CB,
                                                   0x0419,
                                                   0x02E5,
                                                   0x020C,
                                                   0x0173};

bool     dali_cmd_check_reset_state(void);
uint16_t dali_cmd_read_pwm_value(uint8_t dali_value);
void dali_cmd_set_pwm_output(uint8_t dali_value);
uint16_t dali_cmd_process_common(dali_slave_service_info_t *slave_info, uint8_t repeat_flag);
uint16_t dali_cmd_process_extended_application(dali_slave_service_info_t *slave_info, uint8_t repeat_flag);
void dali_reset_memory_bank1(void);

void dali_cmd_init(void)
{
	uint8_t i;
	/*Read out and check if var in eeprom initialized*/
	if (dali_load_from_eeprom(&dali_ram.variable.target_level, sizeof(dali_ram)) == false) {
		/* Restore default value */
		dali_ram.variable.target_level                  = POWER_MAX;
		dali_ram.variable.power_on_level                = POWER_MAX;
		dali_ram.variable.system_failure_level          = POWER_MAX;
		dali_ram.variable.minimum_level                 = POWER_MIN;
		dali_ram.variable.maximum_level                 = POWER_MAX;

		dali_ram.variable.fade_rate                     = 0x07;
		dali_ram.variable.fade_time                     = 0x00;
		dali_ram.variable.extended_fade_time_base       = 0;
		dali_ram.variable.extended_fade_time_multiplier = 0;
		dali_ram.variable.short_address                 = ADDR_MASK;
		dali_ram.variable.operating_mode                = 0;
		dali_ram.variable.group_0_7                     = 0x00;
		dali_ram.variable.group_8_15                    = 0x00;
		dali_ram.variable.version_number                = NUM_OF_102_VER;
		dali_ram.variable.physical_minimum_level        = POWER_MIN;
		dali_ram.variable.min_fast_fade_time            = 0x01;
		dali_ram.variable.fast_fade_time                = 0x00;
		dali_ram.variable.gear_type                     = 0x00;
		dali_ram.variable.possible_operation_modes      = 0x00;
		dali_ram.variable.features                      = 0x00;
		dali_ram.variable.failure_status_operation_mode = REFERRENCE_MEASUREMENT_FAILED;
		dali_ram.variable.dimming_curve                 = LOGARITHMIC;
		dali_ram.variable.random_address.high           = 0xFF;
		dali_ram.variable.random_address.mid            = 0xFF;
		dali_ram.variable.random_address.low            = 0xFF;
		for (i = 0; i < 16; i++) {
			dali_ram.variable.scene[i] = POWER_MASK;
		}
		update_flag = VAR_UPDATE;

		/*Check if bank0 in eeprom initialized*/
		/* Restore default value. Persistent memory bank 0 */
		dali_ram.memory.bank0[0] = BANK0_LAST_MEMORY_LOCATION;
		// 0x01 is reserved
		dali_ram.memory.bank0[2]  = NUMBER_OF_LAST_BANK;
		dali_ram.memory.bank0[3]  = GTIN0;
		dali_ram.memory.bank0[4]  = GTIN1;
		dali_ram.memory.bank0[5]  = GTIN2;
		dali_ram.memory.bank0[6]  = GTIN3;
		dali_ram.memory.bank0[7]  = GTIN4;
		dali_ram.memory.bank0[8]  = GTIN5;
		dali_ram.memory.bank0[9]  = FIRMWARE_MAJOR_VER;
		dali_ram.memory.bank0[10] = FIRMWARE_MINOR_VER;
		dali_ram.memory.bank0[11] = ID_NUMBER0;
		dali_ram.memory.bank0[12] = ID_NUMBER1;
		dali_ram.memory.bank0[13] = ID_NUMBER2;
		dali_ram.memory.bank0[14] = ID_NUMBER3;
		dali_ram.memory.bank0[15] = ID_NUMBER4;
		dali_ram.memory.bank0[16] = ID_NUMBER5;
		dali_ram.memory.bank0[17] = ID_NUMBER6;
		dali_ram.memory.bank0[18] = ID_NUMBER7;
		dali_ram.memory.bank0[19] = HARDWARE_MAJOR_VER;
		dali_ram.memory.bank0[20] = HARDWARE_MINOR_VER;
		dali_ram.memory.bank0[21] = NUM_OF_101_VER;
		dali_ram.memory.bank0[22] = NUM_OF_102_VER;
		dali_ram.memory.bank0[23] = NUM_OF_103_VER;
		dali_ram.memory.bank0[24] = CONTROL_DEVICE_NUM;
		dali_ram.memory.bank0[25] = CONTROL_GEAR_NUM;
		dali_ram.memory.bank0[26] = CONTROL_GEAR_INDEX;

		/* Persistent memory bank 1 */
		dali_reset_memory_bank1();
		update_flag |= BANK0_UPDATE | BANK1_UPDATE;
	}

	/* Bank 1 lock byte in RAM, reset to lock state */
	dali_ram.memory.bank1[2] = BANK1_LOCK_BYTE;
	search_address.high      = 0xFF;
	search_address.mid       = 0xFF;
	search_address.low       = 0xFF;
	status_information       = RESET_STATE | POWER_CYCLE_SEEN;
	if (dali_ram.variable.short_address == ADDR_MASK) {
		status_information |= MISSING_SHORT_ADDR;
	} else {
		status_information &= ~MISSING_SHORT_ADDR;
	}

	optional_features      = dali_ram.variable.features;
	last_active_level      = POWER_MAX;
	current_fast_fade_time = dali_ram.variable.fast_fade_time;
	/* Update bit from persistent memory */
	failure_status            = dali_ram.variable.failure_status_operation_mode & REFERRENCE_MEASUREMENT_FAILED;
	led_module_operating_mode = dali_ram.variable.failure_status_operation_mode & NON_LOGARITHMIC_DIMMING_CURVE_ACTIVE;
}

void dali_cmd_direct_arc_power_control(uint8_t level)
{
	uint8_t level_change = 0;
	;
	uint8_t  current_fade = 0;
	uint32_t step         = 0;

dali_ram.variable.maximum_level = ADC_0_result_processing();

	device_enabled       = false;
	write_memory_enabled = false;
	if (level == POWER_MASK) {
		/* Clear in case power on process */
		power_on_flag = false;
		/* MASK meaning Stop Fading */
		fade_direction = FADE_STOP;
	} else {
		power_on_flag = false;
		status_information &= ~(LIMIT_ERROR + POWER_CYCLE_SEEN);
		if (level == 0) {
			if (dali.level == dali_ram.variable.minimum_level) {
				/* Fade from minlevel to off */
				fade_direction = FADE_BETWEEN_MINLEVEL_AND_OFF;
			} else {
				/* Fade lamp down to minimum level and turn it off. */
				fade_direction = FADE_TIME_DOWN_OFF;
				level_change   = (dali.level - dali_ram.variable.minimum_level);
			}

		} else {
			/* If targetLevel is not 0x00, lastActiveLevel shall be set to targetLevel*/
			last_active_level = level; // Set when fade starts to pass section 12.6.10 of standard 102
			if (level < dali_ram.variable.minimum_level) {
				level = dali_ram.variable.minimum_level;
				update_flag |= VAR_UPDATE;
				status_information |= LIMIT_ERROR;
			} else if (level > dali_ram.variable.maximum_level) {
				level = dali_ram.variable.maximum_level;
				status_information |= LIMIT_ERROR;
			}

			/* testing on the actual level here, not the EEPROM
			 * value */
			if (level > dali.level) {
				if (level == dali_ram.variable.minimum_level) {
					/* Fade from off to minlevel */
					fade_direction = FADE_BETWEEN_MINLEVEL_AND_OFF;
				} else {
					if (dali.level < dali_ram.variable.minimum_level) {
						dali.level = dali_ram.variable.minimum_level;
					}
					fade_direction = FADE_TIME_UP;
					level_change   = (level - dali.level);
				}

			} else if (level < dali.level) {
				fade_direction = FADE_TIME_DOWN;
				level_change   = (dali.level - level);
			} else {
				/* already at right level. */
				fade_direction = FADE_STOP;
				level_change   = 0x00;
			}
		}

		if (dapc_sequence == true) {
			if (dali.level == 0) {
				// If the actual arc power level is zero, the new level shall be adopted without fading.
				dali.level     = level;
				fade_direction = FADE_DIRECT;
			} else {
				/* fade rate 1 as dimming speed */
				current_fade       = 1;
				dapc_sequence_time = W200MS;
			}
		} else {
			current_fade = dali_ram.variable.fade_time;
		}

		/* Set new target level */
		dali_ram.variable.target_level = level;
		update_flag |= VAR_UPDATE;

		if (current_fade == 0) {
			/* Fast fade mode */
			if (current_fast_fade_time == 0) {
				/* Check multiplication factor */
				if ((dali_ram.variable.extended_fade_time_multiplier) == 0) {
					dali.level     = level;
					fade_direction = FADE_DIRECT;
				} else {
					/* Formula 2^24 / (0.1 * fade frequency * extended_fade_time), 2^24 corresponding to
					   flash_inv_fadetime_val shift,
					    +1 as there is one bit right shift later */
					step = ((uint32_t)10 << (24 + 1))
					       / ((uint32_t)extended_fade_time_mul[dali_ram.variable.extended_fade_time_multiplier]
					          * ((uint32_t)dali_ram.variable.extended_fade_time_base + 1)
					          * DALI_PWM_CLOCK_HZ);
					need_fade = true;
					status_information |= FADE_RUNNING;
				}

			} else {
				/* Formula 2^24 / (0.025 * fade frequency * current_fast_fade_time), 2^24 corresponding to
				   flash_inv_fadetime_val shift,
				    +1 as there is one bit right shift later */
				step      = ((uint32_t)40 << (24 + 1)) / (current_fast_fade_time * DALI_PWM_CLOCK_HZ);
				need_fade = true;
				status_information |= FADE_RUNNING;
			}
		} else {
#if defined(__GNUC__)
			step = pgm_read_word(&flash_inv_fadetime_val[current_fade]);
#elif defined(__ICCAVR__)
			step = flash_inv_fadetime_val[current_fade];
#endif
			need_fade = true;
			status_information |= FADE_RUNNING;
		}

		if (fade_direction == FADE_BETWEEN_MINLEVEL_AND_OFF) {
			/* Calculate the real fade time */
			if (current_fade & 0x01) {
				fade_time_off_and_minlevel = W700MS << ((current_fade - 1) >> 1);
			} else {
				fade_time_off_and_minlevel = W1S << ((current_fade >> 1) - 1);
			}
		}

		/* Better watch the math here, so we don't get truncated
		 * intermediate results... */
		fade_step = step;
		fade_step = fade_step * level_change;
		/* Shift 1 bits low due to the shift in the stored values for
		 * flash_inv_fadetime_val */
		fade_step = fade_step >> 1;
		if (!(status_information & LAMP_FAILURE)) {
			status_information |= LAMP_ON;
		}
	}
}

uint16_t dali_cmd_process_normal(dali_slave_service_info_t *slave_info, uint8_t repeat_flag)
{
	if (slave_info->rec_data >= REFERENCE_SYSTEM_POWER) {
		return dali_cmd_process_extended_application(slave_info, repeat_flag);
	} else {
		return dali_cmd_process_common(slave_info, repeat_flag);
	}
}

uint16_t dali_cmd_process_common(dali_slave_service_info_t *slave_info, uint8_t repeat_flag)
{
	uint16_t ret;
	uint8_t  current_fade;
	uint8_t  temp;
	uint8_t  i;
	uint8_t  low;
	bool     light_flag = false; // used by command OFF, UP, DOWN, STEP UP, STEP DOWN,
	                             // RECALL MAX LEVEL, RECALL MIN LEVEL, GO TO LAST ACTIVE LEVEL,
	                             // STEP DOWN AND OFF, ON AND STEP UP, GO TO SCENE (sceneNumber)

	if ((slave_info->rec_data >= RESET) && (slave_info->rec_data <= ENABLE_WRITE_MEMORY)) {
		if (repeat_flag == false) {
			return RECEIVE_FIRST;
		}
	}

	ret = ANSWER_NO;

dali_ram.variable.maximum_level = ADC_0_result_processing();

	switch (slave_info->rec_data) {
	/* Extinguish the lamp without fading */
	case OFF:
		dali_ram.variable.target_level = 0;
		update_flag |= VAR_UPDATE;
		status_information &= ~(FADE_RUNNING + LIMIT_ERROR);
		dali.long_level = 0;
		fade_direction  = FADE_DIRECT;
		light_flag      = true;
		break;

	/* Dim up 200 ms using the selected fade rate if lamp is on */
	case UP:
		if (status_information & LAMP_ON) {
			/* Change to actualLevel if actualLevel is lower than maxLevel */
			if (dali.level < dali_ram.variable.maximum_level) {
				fade_direction    = FADE_RATE_UP;
				current_fade_time = W200MS;
				current_fade      = dali_ram.variable.fade_rate;
#if defined(__GNUC__)
				fade_step = pgm_read_word(&flash_fade_rate_val[current_fade]);
#elif defined(__ICCAVR__)
				fade_step = flash_fade_rate_val[current_fade];
#endif
				/* Shift 7 bits due to the shift in the stored values
				 * for flash_fade_rate_val */
				fade_step = fade_step << 7;
				need_fade = true;
				status_information |= FADE_RUNNING;
			} else {
				status_information |= LIMIT_ERROR;
				/* For fading, then stop */
				fade_direction = FADE_STOP;
			}
		} else {
			/* There shall be no change to actualLevel if actualLevel is at 0x00 */
			fade_direction = FADE_STOP;
		}

		light_flag = true;
		break;

	/* Dim down 200 ms using the selected fade rate if lamp is on */
	case DOWN:
		if (status_information & LAMP_ON) {
			/* Change to actualLevel if actualLevel is higher than minLevel */
			if (dali.level > dali_ram.variable.minimum_level) {
				fade_direction    = FADE_RATE_DOWN;
				current_fade_time = W200MS;
				current_fade      = dali_ram.variable.fade_rate;
#if defined(__GNUC__)
				fade_step = pgm_read_word(&flash_fade_rate_val[current_fade]);
#elif defined(__ICCAVR__)
				fade_step = flash_fade_rate_val[current_fade];
#endif
				/* Shift 7 bits due to the shift in the stored values
				 * for flash_fade_rate_val */
				fade_step = fade_step << 7;
				need_fade = true;
				status_information |= FADE_RUNNING;
			} else {
				status_information |= LIMIT_ERROR;
				/* For fading, then stop */
				fade_direction = FADE_STOP;
			}
		} else {
			/* There shall be no change to actualLevel if actualLevel is at 0x00 */
			fade_direction = FADE_STOP;
		}

		light_flag = true;
		break;

	/* Set the actual arc power level one step higher without fading if lamp
	 * is on */
	case STEP_UP:
		if (status_information & LAMP_ON) {
			if (dali.level < dali_ram.variable.maximum_level) {
				dali.level                     = dali.level + 1;
				dali_ram.variable.target_level = dali.level;
				update_flag |= VAR_UPDATE;
				status_information &= ~LIMIT_ERROR;
			} else {
				status_information |= LIMIT_ERROR;
			}

			fade_direction = FADE_DIRECT;
		}

		light_flag = true;
		break;

	/* Set the actual arc power level one step lower without fading if lamp
	 * is on */
	case STEP_DOWN:
		if (status_information & LAMP_ON) {
			if (dali.level > dali_ram.variable.minimum_level) {
				dali.level                     = dali.level - 1;
				dali_ram.variable.target_level = dali.level;
				update_flag |= VAR_UPDATE;
				status_information &= ~LIMIT_ERROR;
			} else {
				status_information |= LIMIT_ERROR;
			}

			fade_direction = FADE_DIRECT;
		}

		light_flag = true;
		break;

	/* Set the actual arc power level to the maximum value. Turn on if off */
	case RECALL_MAX_LEVEL:
		status_information &= ~LIMIT_ERROR;
		dali.level                     = dali_ram.variable.maximum_level;
		dali_ram.variable.target_level = dali.level;
		update_flag |= VAR_UPDATE;
		if (w15min_waiting_time > 0) {
			/* InitialisatoinState is not disable, go into identification procedure */
			fade_direction    = FADE_DIRECT_IDENTIFICATION;
			recall_level_flag = MAX_LEVEL_FLAG;
		} else {
			fade_direction = FADE_DIRECT;
		}
		light_flag = true;
		break;

	/* Set the actual arc power level to the minimum value. Turn on if off */
	case RECALL_MIN_LEVEL:
		status_information &= ~LIMIT_ERROR;
		dali.level                     = dali_ram.variable.minimum_level;
		dali_ram.variable.target_level = dali.level;
		update_flag |= VAR_UPDATE;
		if (w15min_waiting_time > 0) {
			/* InitialisatoinState is not disable, go into identification procedure */
			fade_direction    = FADE_DIRECT_IDENTIFICATION;
			recall_level_flag = MIN_LEVEL_FLAG;
		} else {
			fade_direction = FADE_DIRECT;
		}
		light_flag = true;
		break;

	/* Set the actual arc power level one step lower without fading. Turn
	 * off if already at min level */
	case STEP_DOWN_AND_OFF:
		if (dali.level <= dali_ram.variable.minimum_level) {
			dali_ram.variable.target_level = 0;
			update_flag |= VAR_UPDATE;
			dali.long_level = 0;
		} else {
			dali.level                     = dali.level - 1;
			dali_ram.variable.target_level = dali.level;
			update_flag |= VAR_UPDATE;
		}

		status_information &= ~LIMIT_ERROR;
		fade_direction = FADE_DIRECT;
		light_flag     = true;
		break;

	/* Set the actual arc power level one step higher without fading. Set to
	 * min level if off */
	case ON_AND_STEP_UP:
		if (dali.level == 0) {
			/* Set to minimum */
			dali.level                     = dali_ram.variable.minimum_level;
			dali_ram.variable.target_level = dali.level;
			update_flag |= VAR_UPDATE;
			status_information &= ~LIMIT_ERROR;
		} else if (dali.level < dali_ram.variable.maximum_level) {
			/* Increase level without fading */
			dali.level                     = dali.level + 1;
			dali_ram.variable.target_level = dali.level;
			update_flag |= VAR_UPDATE;
			status_information &= ~LIMIT_ERROR;
		} else {
			status_information |= LIMIT_ERROR;
		}

		fade_direction = FADE_DIRECT;
		light_flag     = true;
		break;

	/* Mark the start of a direct arc power control (DAPC) sequence */
	case ENABLE_DAPC_SEQUENCE:
		dapc_sequence      = true;
		dapc_sequence_time = W200MS;
		break;

	/* Last active level means not 'OFF'(inactive) level */
	case GO_TO_LAST_ACTIVE_LEVEL:
		if (dali.level != last_active_level) {
			/* power_on_flag is cleared inside this function */
			dali_cmd_direct_arc_power_control(last_active_level);
		}
		light_flag = true;
		break;

	/* Set the actual arc power level to the value stored for scene x
	 * using the actual fade time. */
	case GO_TO_SCENE0:
	case GO_TO_SCENE1:
	case GO_TO_SCENE2:
	case GO_TO_SCENE3:
	case GO_TO_SCENE4:
	case GO_TO_SCENE5:
	case GO_TO_SCENE6:
	case GO_TO_SCENE7:
	case GO_TO_SCENE8:
	case GO_TO_SCENE9:
	case GO_TO_SCENE10:
	case GO_TO_SCENE11:
	case GO_TO_SCENE12:
	case GO_TO_SCENE13:
	case GO_TO_SCENE14:
	case GO_TO_SCENE15:
		temp = dali_ram.variable.scene[slave_info->rec_data & 0x0F];
		/* Only if sceneX not equal MASK, targetLevel shall be affected. */
		if (temp != POWER_MASK) {
			power_on_flag = false;
			dali_cmd_direct_arc_power_control(temp);
		}
		status_information &= ~POWER_CYCLE_SEEN;
		break;

	/* Reset the parameters to default settings */
	case RESET:
		dali.long_level                                 = 0xfe000000;
		last_active_level                               = POWER_MAX;
		dali_ram.variable.target_level                  = POWER_MAX;
		dali_ram.variable.power_on_level                = POWER_MAX;
		dali_ram.variable.system_failure_level          = POWER_MAX;
		dali_ram.variable.minimum_level                 = dali_ram.variable.physical_minimum_level;
		dali_ram.variable.maximum_level                 = POWER_MAX;
		dali_ram.variable.fade_rate                     = 0x07;
		dali_ram.variable.fade_time                     = 0x00;
		dali_ram.variable.extended_fade_time_base       = 0;
		dali_ram.variable.extended_fade_time_multiplier = 0;
		dali_ram.variable.random_address.high           = 0xFF;
		dali_ram.variable.random_address.mid            = 0xFF;
		dali_ram.variable.random_address.low            = 0xFF;

		dali_ram.variable.group_0_7  = 0x00;
		dali_ram.variable.group_8_15 = 0x00;
		for (i = 0; i < 16; i++) {
			dali_ram.variable.scene[i] = POWER_MASK;
		}
		current_fast_fade_time           = 0x00;
		dali_ram.variable.fast_fade_time = 0x00;
		dali_ram.variable.dimming_curve  = LOGARITHMIC;
		update_flag |= VAR_UPDATE;

		search_address.high = 0xFF;
		search_address.mid  = 0xFF;
		search_address.low  = 0xFF;

		led_module_operating_mode &= ~NON_LOGARITHMIC_DIMMING_CURVE_ACTIVE;                       // Standard IEC-207
		dali_ram.variable.failure_status_operation_mode &= ~NON_LOGARITHMIC_DIMMING_CURVE_ACTIVE; // Standard IEC-207
		status_information &= ~(POWER_CYCLE_SEEN + FADE_RUNNING + LIMIT_ERROR + LAMP_FAILURE);
		status_information |= RESET_STATE;
		failure_status &= ~CURRENT_PROTECTOR_ACTIVE;

		fade_direction = FADE_DIRECT;
		break;

	/* Store the current light level in the dtr0 */
	case STORE_ACTUAL_LEVEL_IN_THE_DTR:
		dtr0 = dali.level;
		break;

	/* Update eeprom to save variables */
	case SAVE_PERSISTENT_VARIABLES:
		dali_hal_save_persistent_variables();
		/* After processing is completed, the light output shall be at the level
		   as expected before the reception of this command */
		if (status_information & FADE_RUNNING) {
			fade_direction = FADE_DIRECT;
		}
		break;

	/* Set operation mode, manufacture can process special operation with this */
	case SET_OPERATING_MODE:
		if ((dtr0 > 0) && (dtr0 < 0x80)) {
			break;
		}
		dali_ram.variable.operating_mode = dtr0;
		break;

	/* Reset memory banks indicated via dtr0 */
	case RESET_MEMORY_BANK:
		if (dtr0 == 0) {
			/* All unlocked memory banks except bank0 */
			if (dali_ram.memory.bank1[2] == NOT_LOCK) { // Only bank1, add others when needed
				dali_reset_memory_bank1();
				update_flag |= BANK1_UPDATE;
			}
		} else {
			if ((dtr0 == 1) && (dali_ram.memory.bank1[2] == NOT_LOCK)) {
				dali_reset_memory_bank1();
				update_flag |= BANK1_UPDATE;
			}
		}
		break;

	case IDENTIFY_DEVICE:
		identify_flag      = true;
		w10s_identify_time = W10S;
		/* When identification is active, the light output may be at any level */
		if (status_information & FADE_RUNNING) {
			fade_direction = FADE_DIRECT;
		}
		break;

	/* Store the value in the dtr0 as the maximum level */
	case SET_MAX_LEVEL_WITH_DTR0:
		if ((dtr0 > dali_ram.variable.minimum_level)&&(dtr0 <= dali_ram.variable.maximum_level)) {
			dali_ram.variable.maximum_level = dtr0;
		} else {
			/* Max can not be set to be below min */
			dali_ram.variable.maximum_level = dali_ram.variable.minimum_level;
		}

		temp = dali_ram.variable.maximum_level;
		if (dali.level > temp) {
			dali.level                     = temp;
			dali_ram.variable.target_level = temp;
			fade_direction                 = FADE_DIRECT;
			status_information |= LIMIT_ERROR;
		} else {
			/* Changing the max level shall stop any running fade */
			if (status_information & FADE_RUNNING) {
				fade_direction = FADE_DIRECT;
			}
			status_information &= ~LIMIT_ERROR;
		}
		update_flag |= VAR_UPDATE;
		break;

	/* Store the value in the dtr0 as the minimum level */
	case SET_MIN_LEVEL_WITH_DTR0:
		if (dtr0 < dali_ram.variable.physical_minimum_level) {
			dtr0 = dali_ram.variable.physical_minimum_level;
		} else if (dtr0 > dali_ram.variable.maximum_level) {
			dtr0 = dali_ram.variable.maximum_level;
		}
		dali_ram.variable.minimum_level = dtr0;

		if ((dali.level < dtr0) && (dali.level != POWER_OFF)) {
			dali.level                     = dtr0;
			dali_ram.variable.target_level = dtr0;
			fade_direction                 = FADE_DIRECT;
			status_information |= LIMIT_ERROR;
		} else {
			/* Changing the min level shall stop any running fade */
			if (status_information & FADE_RUNNING) {
				fade_direction = FADE_DIRECT;
			}
			status_information &= ~LIMIT_ERROR;
		}

		update_flag |= VAR_UPDATE;
		break;

	/* Store the value in the dtr0 as the system failure level */
	case SET_SYSTEM_FAILURE_LEVEL_WITH_DTR0:
		dali_ram.variable.system_failure_level = dtr0;
		update_flag |= VAR_UPDATE;
		break;

	/* Store the value in the dtr0 as the power on level */
	case SET_POWER_ON_LEVEL_WITH_DTR0:
		dali_ram.variable.power_on_level = dtr0;
		update_flag |= VAR_UPDATE;
		break;

	/* Store the value in the dtr0 as the fade time */
	case SET_FADE_TIME_WITH_DTR0:
		/* Valid range for fade_time is 0-15 */
		if (dtr0 > 0x0F) {
			dtr0 = 0x0F;
		}

		dali_ram.variable.fade_time = dtr0;
		update_flag |= VAR_UPDATE;
		break;

	/* Store the value in the dtr0 as the fade rate */
	case SET_FADE_RATE_WITH_DTR0:
		/* Valid range for fade_rate is 1-15, */
		if (dtr0 > 0x0F) {
			dtr0 = 0x0F;
		} else if (dtr0 == 0x00) { // for LED system 1 is OK.
			dtr0 = 1;
		}

		dali_ram.variable.fade_rate = dtr0;
		update_flag |= VAR_UPDATE;
		break;

	/* Set extended fade time in dtr0 */
	case SET_EXTENDED_FADE_TIME_WITH_DTR0:
		if (dtr0 > 0x4F) { // 0b01001111
			dali_ram.variable.extended_fade_time_base       = 0;
			dali_ram.variable.extended_fade_time_multiplier = 0;
		} else {
			dali_ram.variable.extended_fade_time_base       = dtr0 & 0x0F;        // 0b00001111
			dali_ram.variable.extended_fade_time_multiplier = (dtr0 & 0x70) >> 4; // 0b01110000
		}
		break;

	/* The value in the Data Transfer Register shall be stored as a new
	 * level for the scene XXXX */
	case SET_SCENE0_WITH_DTR0:
	case SET_SCENE1_WITH_DTR0:
	case SET_SCENE2_WITH_DTR0:
	case SET_SCENE3_WITH_DTR0:
	case SET_SCENE4_WITH_DTR0:
	case SET_SCENE5_WITH_DTR0:
	case SET_SCENE6_WITH_DTR0:
	case SET_SCENE7_WITH_DTR0:
	case SET_SCENE8_WITH_DTR0:
	case SET_SCENE9_WITH_DTR0:
	case SET_SCENE10_WITH_DTR0:
	case SET_SCENE11_WITH_DTR0:
	case SET_SCENE12_WITH_DTR0:
	case SET_SCENE13_WITH_DTR0:
	case SET_SCENE14_WITH_DTR0:
	case SET_SCENE15_WITH_DTR0:
		dali_ram.variable.scene[slave_info->rec_data & 0x0F] = dtr0;
		update_flag |= VAR_UPDATE;
		break;

	/* Removing the control gear from scene XXXX means storing
	 * 0b11111111 ('MASK' or 'DON NOT CHANGE') in scene register XXXX */
	case REMOVE_FROM_SCENE0:
	case REMOVE_FROM_SCENE1:
	case REMOVE_FROM_SCENE2:
	case REMOVE_FROM_SCENE3:
	case REMOVE_FROM_SCENE4:
	case REMOVE_FROM_SCENE5:
	case REMOVE_FROM_SCENE6:
	case REMOVE_FROM_SCENE7:
	case REMOVE_FROM_SCENE8:
	case REMOVE_FROM_SCENE9:
	case REMOVE_FROM_SCENE10:
	case REMOVE_FROM_SCENE11:
	case REMOVE_FROM_SCENE12:
	case REMOVE_FROM_SCENE13:
	case REMOVE_FROM_SCENE14:
	case REMOVE_FROM_SCENE15:
		dali_ram.variable.scene[slave_info->rec_data & 0x0F] = POWER_MASK;
		update_flag |= VAR_UPDATE;
		break;

	/* The control gear shall be added to group XXXX */
	case ADD_TO_GROUP0:
	case ADD_TO_GROUP1:
	case ADD_TO_GROUP2:
	case ADD_TO_GROUP3:
	case ADD_TO_GROUP4:
	case ADD_TO_GROUP5:
	case ADD_TO_GROUP6:
	case ADD_TO_GROUP7:
		low                         = slave_info->rec_data & 0x07;
		temp                        = dali_ram.variable.group_0_7;
		temp                        = temp | (1 << low);
		dali_ram.variable.group_0_7 = temp;
		update_flag |= VAR_UPDATE;
		break;

	case ADD_TO_GROUP8:
	case ADD_TO_GROUP9:
	case ADD_TO_GROUP10:
	case ADD_TO_GROUP11:
	case ADD_TO_GROUP12:
	case ADD_TO_GROUP13:
	case ADD_TO_GROUP14:
	case ADD_TO_GROUP15:
		low                          = slave_info->rec_data & 0x07;
		temp                         = dali_ram.variable.group_8_15;
		temp                         = temp | (1 << low);
		dali_ram.variable.group_8_15 = temp;
		update_flag |= VAR_UPDATE;
		break;

	/* The control gear shall be removed from group XXXX */
	case REMOVE_FROM_GROUP0:
	case REMOVE_FROM_GROUP1:
	case REMOVE_FROM_GROUP2:
	case REMOVE_FROM_GROUP3:
	case REMOVE_FROM_GROUP4:
	case REMOVE_FROM_GROUP5:
	case REMOVE_FROM_GROUP6:
	case REMOVE_FROM_GROUP7:
		low                         = slave_info->rec_data & 0x07;
		temp                        = dali_ram.variable.group_0_7;
		temp                        = temp & ~(1 << low);
		dali_ram.variable.group_0_7 = temp;
		update_flag |= VAR_UPDATE;
		break;

	case REMOVE_FROM_GROUP8:
	case REMOVE_FROM_GROUP9:
	case REMOVE_FROM_GROUP10:
	case REMOVE_FROM_GROUP11:
	case REMOVE_FROM_GROUP12:
	case REMOVE_FROM_GROUP13:
	case REMOVE_FROM_GROUP14:
	case REMOVE_FROM_GROUP15:
		low                          = slave_info->rec_data & 0x07;
		temp                         = dali_ram.variable.group_8_15;
		temp                         = temp & ~(1 << low);
		dali_ram.variable.group_8_15 = temp;
		update_flag |= VAR_UPDATE;
		break;

	/* Save the value in the dtr0 as new short address */
	case SET_SHORT_ADDRESS_WITH_DTR0:
		if (dtr0 == ADDR_MASK) {
			dali_ram.variable.short_address = ADDR_MASK;
			update_flag |= VAR_UPDATE;
			status_information |= MISSING_SHORT_ADDR;
		} else if ((dtr0 & 0x81) == 0x01) {
			/* dtr0 = 0AAA AAA1, need to shift down a bit to get
			 * 00AAAAAA in short_address */
			dali_ram.variable.short_address = (dtr0 >> 1);
			update_flag |= VAR_UPDATE;
			status_information &= ~MISSING_SHORT_ADDR;
		}
		break;

	/* Enable subsequent writes to memory */
	case ENABLE_WRITE_MEMORY:
		write_memory_enabled = true;
		break;

	/* Queries related to status information */
	case QUERY_STATUS:
		if (true == dali_cmd_check_reset_state()) {
			status_information |= RESET_STATE;
		} else {
			status_information &= ~RESET_STATE;
		}

		ret = status_information;
		break;

	case QUERY_CONTROL_GEAR:
		/* There is a control gear */
		ret = ANSWER_YES;
		break;

	case QUERY_LAMP_FAILURE:
		/* bit 1	Lamp failure; 0 = OK */
		if ((status_information & LAMP_FAILURE) != 0x00) {
			ret = ANSWER_YES;
		}
		break;

	case QUERY_LAMP_POWER_ON:
		/* bit 2	Lamp arc power on; 0 = OFF */
		if ((status_information & LAMP_ON) != 0x00) {
			ret = ANSWER_YES;
		}
		break;

	case QUERY_LIMIT_ERROR:
		/* bit 3	Query: Limit Error; 0 = Last requested arc power
		 * level is between MIN and MAX LEVEL or OFF */
		if ((status_information & LIMIT_ERROR) != 0x00) {
			ret = ANSWER_YES;
		}
		break;

	case QUERY_RESET_STATE:
		if (true == dali_cmd_check_reset_state()) {
			status_information |= RESET_STATE;
			ret = ANSWER_YES;
		} else {
			status_information &= ~RESET_STATE;
		}
		break;

	case QUERY_MISSING_SHORT_ADDRESS:
		if ((status_information & MISSING_SHORT_ADDR) != 0x00) {
			ret = ANSWER_YES;
		}
		break;

	case QUERY_VERSION_NUMBER:
		ret = dali_ram.variable.version_number;
		break;

	case QUERY_CONTENT_DTR:
		ret = dtr0;
		break;

	case QUERY_DEVICE_TYPE:
		ret = DEVICE_TYPE;
		break;

	case QUERY_PHYSICAL_MINIMUM_LEVEL:
		ret = dali_ram.variable.physical_minimum_level;
		break;

	case QUERY_POWER_FAILURE:
		/* bit 7 Query: "POWER FAILURE"? "0" = "No"; "RESET" or an arc
		 * power control command has been received since last power-on. */
		if ((status_information & POWER_CYCLE_SEEN) != 0x00) {
			ret = ANSWER_YES;
		}
		break;

	case QUERY_CONTENT_DTR1:
		ret = dtr1;
		break;

	case QUERY_CONTENT_DTR2:
		ret = dtr2;
		break;

	case QUERY_OPERATING_MODE:
		ret = dali_ram.variable.operating_mode;
		break;

	case QUERY_LIGHT_SOURCE_TYPE:
		ret = LIGHT_SOURCE;
		break;

	/* Queries related to arc power parameter settings */
	case QUERY_ACTUAL_LEVEL:
		if ((status_information & LAMP_FAILURE) != 0x00) {
			/* Lamp failure, answer MASK */
			ret = POWER_MASK;
		} else {
			ret = dali.byte2 < 0x80 ? dali.level : dali.level + 1;
		}
		break;

	case QUERY_MAX_LEVEL:
		ret = dali_ram.variable.maximum_level;
		break;

	case QUERY_MIN_LEVEL:
		ret = dali_ram.variable.minimum_level;
		break;

	case QUERY_POWER_ON_LEVEL:
		ret = dali_ram.variable.power_on_level;
		break;

	case QUERY_SYSTEM_FAILURE_LEVEL:
		ret = dali_ram.variable.system_failure_level;
		break;

	case QUERY_FADE_TIME_RATE:
		ret = (dali_ram.variable.fade_time << 4) | dali_ram.variable.fade_rate;
		break;

	case QUERY_MANUFACTURER_SPECIFIC_MODE:
		if (dali_ram.variable.operating_mode >= 0x80) {
			ret = ANSWER_YES;
		}
		break;

	case QUERY_NEXT_DEVICE_TYPE:
		// If more than one devices, add support here
		break;

	case QUERY_EXTENDED_FADE_TIME:
		ret = dali_ram.variable.extended_fade_time_base | (dali_ram.variable.extended_fade_time_multiplier << 4);
		break;

	case QUERY_CONTROL_GEAR_FAILURE:
		if (status_information & CONTROL_GEAR_FAILURE) {
			ret = ANSWER_YES;
		}
		break;

	case QUERY_SCENE_LEVEL0:
	case QUERY_SCENE_LEVEL1:
	case QUERY_SCENE_LEVEL2:
	case QUERY_SCENE_LEVEL3:
	case QUERY_SCENE_LEVEL4:
	case QUERY_SCENE_LEVEL5:
	case QUERY_SCENE_LEVEL6:
	case QUERY_SCENE_LEVEL7:
	case QUERY_SCENE_LEVEL8:
	case QUERY_SCENE_LEVEL9:
	case QUERY_SCENE_LEVEL10:
	case QUERY_SCENE_LEVEL11:
	case QUERY_SCENE_LEVEL12:
	case QUERY_SCENE_LEVEL13:
	case QUERY_SCENE_LEVEL14:
	case QUERY_SCENE_LEVEL15:
		ret = dali_ram.variable.scene[slave_info->rec_data & 0x0F];
		break;

	case QUERY_GROUPS_0_7:
		ret = dali_ram.variable.group_0_7;
		break;

	case QUERY_GROUPS_8_15:
		ret = dali_ram.variable.group_8_15;
		break;

	case QUERY_RANDOM_ADDRESS_H:
		ret = dali_ram.variable.random_address.high;
		break;

	case QUERY_RANDOM_ADDRESS_M:
		ret = dali_ram.variable.random_address.mid;
		break;

	case QUERY_RANDOM_ADDRESS_L:
		ret = dali_ram.variable.random_address.low;
		break;

	case READ_MEMORY_LOCATION:
		/* select bank */
		if (dtr1 == BANK0) {
			if (dtr0 <= BANK0_LAST_MEMORY_LOCATION) {
				if (dtr0 != 1) { // Location 1 is reserved - not implement
					ret = dali_ram.memory.bank0[dtr0];
				}
				dtr0++;
			} else if (dtr0 < 0xFF) {
				dtr0++;
			}
		} else if (dtr1 == BANK1) {
			if (dtr0 <= BANK1_LAST_MEMORY_LOCATION) {
				ret = dali_ram.memory.bank1[dtr0++];
			} else if (dtr0 < 0xFF) {
				dtr0++;
			}
		}
		break;

	default:
		break;
	}

	device_enabled = false;
	if (identify_flag == true) {
		if ((slave_info->rec_data != RECALL_MIN_LEVEL) && (slave_info->rec_data != RECALL_MAX_LEVEL)
		    && (slave_info->rec_data != IDENTIFY_DEVICE)) {
			identify_flag      = false;
			w10s_identify_time = 0;
		}
	}

	if (write_memory_enabled == true) {
		if ((slave_info->rec_data != QUERY_CONTENT_DTR) && (slave_info->rec_data != QUERY_CONTENT_DTR1)
		    && (slave_info->rec_data != QUERY_CONTENT_DTR2)
		    && (slave_info->rec_data != ENABLE_WRITE_MEMORY)) {
			write_memory_enabled = false;
		}
	}

	if (light_flag == true) {
		status_information &= ~POWER_CYCLE_SEEN;
		dapc_sequence = false;
		power_on_flag = false;
	}

	return ret;
}

uint16_t dali_cmd_process_extended_application(dali_slave_service_info_t *slave_info, uint8_t repeat_flag)
{
	uint16_t ret;
	uint8_t  temp;

	if ((slave_info->rec_data >= REFERENCE_SYSTEM_POWER) && (slave_info->rec_data <= STORE_DTR_AS_FAST_FADE_TIME)) {
		if (repeat_flag == false) {
			return RECEIVE_FIRST;
		}
	}

	ret = ANSWER_NO;
	if (device_enabled == false) {
		return ret;
	}
	write_memory_enabled = false;

	switch (slave_info->rec_data) {
	case REFERENCE_SYSTEM_POWER:
	case ENABLE_CURRENT_PROTECTOR:
	case DISABLE_CURRENT_PROTECTOR:
		break;

	case SELECT_DIMMING_CURVE:
		if (dtr0 <= LINEAR) {
			dali_ram.variable.dimming_curve = dtr0;
			temp                            = failure_status & REFERRENCE_MEASUREMENT_FAILED;
			if (dtr0 == LOGARITHMIC) {
				led_module_operating_mode &= ~NON_LOGARITHMIC_DIMMING_CURVE_ACTIVE;
				dali_ram.variable.failure_status_operation_mode = temp;
			} else {
				led_module_operating_mode |= NON_LOGARITHMIC_DIMMING_CURVE_ACTIVE;
				dali_ram.variable.failure_status_operation_mode = temp | NON_LOGARITHMIC_DIMMING_CURVE_ACTIVE;
			}
			update_flag |= VAR_UPDATE;
		}
		break;

	case STORE_DTR_AS_FAST_FADE_TIME:
		if (dtr0 == 0) {
			current_fast_fade_time = 0;
		} else if (dtr0 < dali_ram.variable.min_fast_fade_time) {
			current_fast_fade_time = dali_ram.variable.min_fast_fade_time;
		} else if (dtr0 <= MAX_FAST_FADE_TIME) {
			current_fast_fade_time = dtr0;
		} else {
			current_fast_fade_time = MAX_FAST_FADE_TIME;
		}

		dali_ram.variable.fast_fade_time = current_fast_fade_time;
		update_flag |= VAR_UPDATE;
		break;

	case QUERY_GEAR_TYPE:
		ret = dali_ram.variable.gear_type;
		break;

	case QUERY_DIMMING_CURVE:
		ret = dali_ram.variable.dimming_curve;
		break;

	case QUERY_POSSIBLE_OPERATING_MODES:
		ret = dali_ram.variable.possible_operation_modes;
		break;

	case QUERY_FEATURES:
		ret = optional_features;
		break;

	case QUERY_FAILURE_STATUS:
		ret = failure_status;
		break;

	case QUERY_SHORT_CIRCUIT:
	case QUERY_OPEN_CIRCUIT:
	case QUERY_LOAD_DECREASE:
	case QUERY_LOAD_INCREASE:
	case QUERY_CURRENT_PROTECTOR_ACTIVE:
	case QUERY_THERMAL_SHUT_DOWN:
	case QUERY_THERMAL_OVERLOAD:
	case QUERY_REFERENCE_RUNNING:
	case QUERY_REFERENCE_MEASUREMENT_FAILED:
	case QUERY_CURRENT_PROTECTOR_ENABLED:
		break;

	case QUERY_LED_MODULE_OPERATING_MODE:
		ret = led_module_operating_mode;
		break;

	case QUERY_FAST_FADE_TIME:
		ret = current_fast_fade_time;
		break;

	case QUERY_MIN_FAST_FADE_TIME:
		ret = dali_ram.variable.min_fast_fade_time;
		break;

	case QUERY_EXTENDED_VERSION_NUMBER:
		ret = EXTENDED_VER_NUM;
		break;

	default:
		break;
	}

	device_enabled = false;
	return ret;
}

uint16_t dali_cmd_process_special(dali_slave_service_info_t *slave_info, uint8_t repeat_flag)
{
	uint16_t ret;
	uint8_t  temp_short_addr;
	uint8_t  temp              = 0;
	uint8_t  enable_device_cmd = 0;
	uint32_t temp_random_addr, temp_search_addr;
	bool     reply_flag = false;

	ret = ANSWER_NO;

	if ((slave_info->rec_addr >= RANDOMISE) && (slave_info->rec_addr <= QUERY_SHORT_ADDRESS)) {
		if (w15min_waiting_time == 0) {
			return ret;
		}
	}
	if (identify_flag == true) {
		if (slave_info->rec_addr != INITIALISE) {
			identify_flag      = false;
			w10s_identify_time = 0;
		}
	}

	if (withdraw == 1) {                // Update the withdraw state
		if (w15min_waiting_time == 0) { // timeout or terminated
			withdraw = 0;
		}
	}

	temp_random_addr = ((uint32_t)dali_ram.variable.random_address.high << 16)
	                   + ((uint32_t)dali_ram.variable.random_address.mid << 8)
	                   + (uint32_t)dali_ram.variable.random_address.low;

	temp_search_addr
	    = ((uint32_t)search_address.high << 16) + ((uint32_t)search_address.mid << 8) + (uint32_t)search_address.low;
	/* Short address is often transferred in the form 0AAA AAA1, here change
	 * this to  the form 00AA AAAA */
	temp_short_addr = ((slave_info->rec_data & ADDR_SHORT_A) >> 1);

	switch (slave_info->rec_addr) {
	case TERMINATE:
		/* abort the initialise time period */
		w15min_waiting_time = 0;
		break;

	case DATA_TRANSFER_REGISTER:
		dtr0 = slave_info->rec_data;
		break;

	case INITIALISE:
		if (repeat_flag == false) {
			return RECEIVE_FIRST;
		}

		if (slave_info->rec_data == 0) {
			temp = 1;
		} else if (slave_info->rec_data == ADDR_MASK) {
			if (dali_ram.variable.short_address == ADDR_MASK) {
				temp = 1;
			}
		} else if ((slave_info->rec_data & 0x81) == 0x01) {
			/* check short address */
			if (temp_short_addr == dali_ram.variable.short_address) {
				temp = 1;
			}
		}

		if (temp) {
			w15min_waiting_time = W15MIN;
		} else {
			w15min_waiting_time = 0;
		}
		break;

	case RANDOMISE:
		if (repeat_flag == false) {
			return RECEIVE_FIRST;
		}

		/* Use the current timer value as seed for the random generator */
		srand(dali_hal_get_seed0_value());
		dali_ram.variable.random_address.high = rand();
		dali_ram.variable.random_address.mid  = rand();
		srand(dali_hal_get_seed1_value());
		dali_ram.variable.random_address.low = rand();
		update_flag |= VAR_UPDATE;
		break;

	case COMPARE:
		if (withdraw == 0) {
			if (temp_random_addr <= temp_search_addr) {
				ret = ANSWER_YES;
			}
		}
		break;

	case WITHDRAW:
		/* Set withdraw flag */
		if (temp_random_addr == temp_search_addr) {
			withdraw = 1;
		}
		break;

	case PING:
		/* Ignore here , indicate presence of master application controllers */
		break;

	case SEARCHADDRH:
		search_address.high = slave_info->rec_data;
		break;

	case SEARCHADDRM:
		search_address.mid = slave_info->rec_data;
		break;

	case SEARCHADDRL:
		search_address.low = slave_info->rec_data;
		break;

	case PROGRAM_SHORT_ADDRESS:
		if (slave_info->rec_data == ADDR_MASK) {
			/* Short address 0xFF means short address deleted */
			dali_ram.variable.short_address = ADDR_MASK;
			update_flag |= VAR_UPDATE;
			status_information |= MISSING_SHORT_ADDR;
		} else if (temp_random_addr == temp_search_addr) {
			/* Address selected */
			if ((slave_info->rec_data & 0x01) == 0x01) {
				/* Stored as 00AA AAAA in EEPROM */
				dali_ram.variable.short_address = temp_short_addr;
				update_flag |= VAR_UPDATE;
				status_information &= ~MISSING_SHORT_ADDR;
			}
		}
		break;

	case VERIFY_SHORT_ADDRESS:
		/* Data must be 0AAAAAA1b format */
		if ((slave_info->rec_data & 0x81) == 0x01) {
			if (dali_ram.variable.short_address == temp_short_addr) {
				ret = ANSWER_YES;
			}
		}
		break;

	case QUERY_SHORT_ADDRESS:
		if (temp_random_addr == temp_search_addr) {
			ret = (dali_ram.variable.short_address << 1) | 0x01;
		}
		break;

	case ENABLE_DEVICE_TYPE:
		if (slave_info->rec_data == DEVICE_TYPE) {
			device_enabled = true;
		}

		enable_device_cmd = 1;
		break;

	case DATA_TRANSFER_REGISTER_1:
		dtr1 = slave_info->rec_data;
		break;

	case DATA_TRANSFER_REGISTER_2:
		dtr2 = slave_info->rec_data;
		break;

	case WRITE_MEMORY_LOCATION:
		reply_flag = true;
	case WRITE_MEMORY_LOCATION_NO_REPLY:
		if (write_memory_enabled == false) {
			break;
		}

		temp = 0;
		if (dtr1 == BANK1) {
			if (dtr0 < 0x02) {
				/* read-only memory */
				temp = 1;
			} else if (dtr0 == 0x02) {
				/* read/write memory, lock byte location */
				temp = 2;
			} else if ((dtr0 > 0x02) && (dtr0 <= 0x10)) {
				/* lockable memory */
				if (dali_ram.memory.bank1[2] == NOT_LOCK) {
					temp = 2;
				}
			}

			/* Only inside bank1 range update the pointer dtr0 */
			if (temp > 0) {
				/* write memory */
				if (temp == 2) {
					dali_ram.memory.bank1[dtr0] = slave_info->rec_data;
					if (reply_flag) {
						ret = slave_info->rec_data;
					}

					update_flag |= BANK1_UPDATE;
				}
			}
			if (dtr0 < 0xFF) { // increment when lower to 0xff
				dtr0++;
			}
		}
		break;

	default:
		break;
	}

	if (enable_device_cmd == 0) {
		device_enabled = false;
	}

	if (write_memory_enabled == true) {
		if ((slave_info->rec_addr != DATA_TRANSFER_REGISTER) && (slave_info->rec_addr != DATA_TRANSFER_REGISTER_1)
		    && (slave_info->rec_addr != DATA_TRANSFER_REGISTER_2)
		    && (slave_info->rec_addr != WRITE_MEMORY_LOCATION)
		    && (slave_info->rec_addr != WRITE_MEMORY_LOCATION_NO_REPLY)) {
			write_memory_enabled = false;
		}
	}

	return ret;
}

/**
 *  \brief check if it is reset state.
 *
 *  \returns true means reset state, false not
 */
bool dali_cmd_check_reset_state(void)
{
	uint8_t i;

	if (dali_ram.variable.power_on_level != POWER_MAX) {
		return false;
	}

	if (dali_ram.variable.system_failure_level != POWER_MAX) {
		return false;
	}

	if (dali_ram.variable.minimum_level != dali_ram.variable.physical_minimum_level) {
		return false;
	}

	if (dali_ram.variable.maximum_level != POWER_MAX) {
		return false;
	}

	if (dali_ram.variable.fade_rate != 0x07) {
		return false;
	}

	if (dali_ram.variable.fade_time != 0x00) {
		return false;
	}

	if (dali_ram.variable.extended_fade_time_base != 0x00) {
		return false;
	}

	if (dali_ram.variable.extended_fade_time_multiplier != 0x00) {
		return false;
	}

	if (dali_ram.variable.random_address.high != 0xFF) {
		return false;
	}

	if (dali_ram.variable.random_address.mid != 0xFF) {
		return false;
	}

	if (dali_ram.variable.random_address.low != 0xFF) {
		return false;
	}

	if (dali_ram.variable.group_0_7 != 0x00) {
		return false;
	}

	if (dali_ram.variable.group_8_15 != 0x00) {
		return false;
	}

	for (i = 0; i < 16; i++) {
		if (dali_ram.variable.scene[i] != POWER_MASK) {
			return false;
		}
	}

	if (led_module_operating_mode & NON_LOGARITHMIC_DIMMING_CURVE_ACTIVE) {
		return false;
	}

	if (dali_ram.variable.dimming_curve != 0x00) {
		return false;
	}

	if (dali_ram.variable.fast_fade_time != 0x00) {
		return false;
	}
	return true;
}

/**
 *  \brief Write dali data directly to PWM output, no fading.
 *
 *  \param dali_value is the power level to be set.
 */
void dali_cmd_set_pwm_output(uint8_t dali_value)

{
	union uint16_union temp_val;

	if (dali_ram.variable.dimming_curve == LOGARITHMIC) {
/* Upper 4 of the 12 bits PWM value, stored two 4-bit values per
 * byte */
#if defined(__GNUC__)
		temp_val.msb = pgm_read_byte(&flash_high_pwm_val[dali_value >> 1]);
#elif defined(__ICCAVR__)
		temp_val.msb      = flash_high_pwm_val[dali_value >> 1];
#endif
		if ((dali_value & 0x01) == 0x00) {
			/* Even number, data is stored as lower 4 bit */
			temp_val.msb = temp_val.msb & 0x0F;
		} else {
			/* Odd number, data is stored as upper 4 bit */
			temp_val.msb = (temp_val.msb & 0xF0) >> 4;
		}
#if defined(__GNUC__)
		temp_val.lsb = pgm_read_byte(&flash_low_pwm_val[dali_value]);
#elif defined(__ICCAVR__)
		temp_val.lsb      = flash_low_pwm_val[dali_value];
#endif
	} else {
		/* Linear curve, n*4095/254 */
		temp_val.value = ((uint32_t)dali_value * DALI_PWM_TC_TOP) / 254;
	}
	dali_hal_update_pwm_output(temp_val.value);
}

/**
 *  \brief Calculate which PWM output corresponds to the given dali value.
 *
 *  \param dali_value is the power level to be set.
 *
 *  \return corresponding PWM value
 */
uint16_t dali_cmd_read_pwm_value(uint8_t dali_value)
{
	union uint16_union temp_val;

	if (dali_ram.variable.dimming_curve == LOGARITHMIC) {
/* Upper 4 of the 12 bits PWM value, stored two 4-bit values per
 * byte */
#if defined(__GNUC__)
		temp_val.msb = pgm_read_byte(&flash_high_pwm_val[dali_value >> 1]);
#elif defined(__ICCAVR__)
		temp_val.msb      = flash_high_pwm_val[dali_value >> 1];
#endif
		if ((dali_value & 0x01) == 0x00) {
			/* Even number, data is stored as lower 4 bit */
			temp_val.msb = temp_val.msb & 0x0F;
		} else {
			/* Odd number, data is stored as upper 4 bit */
			temp_val.msb = (temp_val.msb & 0xF0) >> 4;
		}
#if defined(__GNUC__)
		temp_val.lsb = pgm_read_byte(&flash_low_pwm_val[dali_value]);
#elif defined(__ICCAVR__)
		temp_val.lsb      = flash_low_pwm_val[dali_value];
#endif
	} else {
		/* Linear curve, n*4095/254 */
		temp_val.value = ((uint32_t)dali_value * DALI_PWM_TC_TOP) / 254;
	}

	return temp_val.value;
}

void dali_process_power_on(void)
{	
	if (power_on_flag == true) {
		if (power_on_time == 0) {
			/* Execute power on action */
			power_on_flag = false;
			if (interface_fail_time == (uint16_t)(FAIL_TIME + 1)) {
				/* Interface failure is exiting */
				if (dali_ram.variable.system_failure_level != POWER_MASK) {
					/*If failure level not MASK, keep failure level active, exit power on action */
					return;
				}
			}
			if (dali_ram.variable.power_on_level == POWER_MASK) {
				dali.level = dali_ram.variable.target_level;
			} else {
				if ((dali_ram.variable.power_on_level < dali_ram.variable.minimum_level)
				    && (dali_ram.variable.power_on_level != POWER_OFF)) {
					dali.level = dali_ram.variable.minimum_level;
					if (dali_ram.variable.power_on_level < POWER_MIN) {
						status_information |= LIMIT_ERROR; // Target power on level less PHM
					}
				} else if (dali_ram.variable.power_on_level > dali_ram.variable.maximum_level) {
					dali.level = dali_ram.variable.maximum_level;
					// status_information |= LIMIT_ERROR; // Target power on level modified
				} else {
					dali.level = dali_ram.variable.power_on_level;
				}
				dali_ram.variable.target_level = dali.level;
				update_flag |= VAR_UPDATE;
			}

			fade_direction = FADE_DIRECT;
		}
	}
}

void dali_process_interface_failure(void)
{
	dali_ram.variable.maximum_level = ADC_0_result_processing();
	
	uint8_t temp;
	if (interface_fail_flag) {
		temp = dali_ram.variable.system_failure_level;
		if (temp != POWER_MASK) {
			if ((temp < dali_ram.variable.minimum_level) && (temp != POWER_OFF)) {
				temp = dali_ram.variable.minimum_level;
			} else if (temp > dali_ram.variable.maximum_level) {
				temp = dali_ram.variable.maximum_level;
			}

			dali.level                     = temp;
			dali_ram.variable.target_level = dali.level;
			update_flag |= VAR_UPDATE;
			fade_direction = FADE_DIRECT;
		}
	}
}

void dali_process_identify_device(void)
{
	if (identify_flag == true) {
		dali_hal_identify_device();
		/* Time out */
		if (w10s_identify_time == 0) {
			identify_flag = false;
		}
	}
}

/**
 *  \brief Mainly realize the power fading function here
 */
void dali_command_fade_ms_tick(void)
{
	uint8_t  temp_level, temp0, temp1;
	uint16_t temp_val0, temp_val1;
	uint32_t temp_fade_step;
	uint32_t temp_long_level;

	/* Using this a lot here, use local parameter to save code space */
	temp_fade_step  = fade_step;
	temp_level      = dali.level;
	temp_long_level = dali.long_level;

	if (power_on_flag == true) {
		if (power_on_time > 0) {
			power_on_time--;
		}
	}

	if (current_fade_time > 0) {
		current_fade_time--;
	}

	if (w15min_waiting_time > 0) {
		w15min_waiting_time--;
	}

	if (w10s_identify_time > 0) {
		w10s_identify_time--;
	}

	if (dapc_sequence == true) {
		if (dapc_sequence_time > 0) {
			dapc_sequence_time--;
		} else {
			dapc_sequence                  = false;
			dali_ram.variable.target_level = temp_level;
			update_flag |= VAR_UPDATE;
			fade_direction = FADE_STOP;
		}
	}

dali_ram.variable.maximum_level = ADC_0_result_processing();

	if (dali_hal_get_dali_input_level() == 0) {
		if (interface_fail_time == (uint16_t)FAIL_TIME) {
			interface_fail_flag = true;
			interface_fail_time++;
		} else if (interface_fail_time < (uint16_t)FAIL_TIME) {
			interface_fail_time++;
		}
	} else {
		interface_fail_time = 0;
	}

	switch (fade_direction) {
	case FADE_IDLE:
		break;

	case FADE_STOP:
		need_fade = false;
		status_information &= ~FADE_RUNNING;
		fade_direction = FADE_IDLE;
		if (temp_level != 0) {
			last_active_level = temp_level;
		}
		break;

	/* Fade using int_fadetime. Actually running this as a fade rate
	 * (inv_int_fadetime_val), not directly timed. */
	case FADE_TIME_UP:
		/* Rounding errors may then cause slight changes in fade time,
		 * but the target level will still be the correct one. */
		if (temp_level < dali_ram.variable.target_level) {
			/* Increase level */
			if ((0xFE000000 - temp_long_level) <= temp_fade_step) {
				/* max level 254 */
				temp_long_level = 0xFE000000;
			} else {
				temp_long_level = temp_long_level + temp_fade_step;
			}
		} else {
			/* Turn off fading when done. */
			fade_direction = FADE_STOP;
		}
		break;

	/* Fade using int_fadetime.This fades down, but does not turn off the
	 * lamp. It will stop at minimum level. */
	case FADE_TIME_DOWN:
		if (temp_level >= dali_ram.variable.target_level) {
			/* decrease level */
			if (temp_long_level <= temp_fade_step) {
				temp_long_level = 0;
				/* Turn off as level is 0 now */
				fade_direction = FADE_STOP;
			} else {
				temp_long_level = temp_long_level - temp_fade_step;
			}
		} else {
			temp_long_level = (uint32_t)(dali_ram.variable.target_level) << 24;
			/* Turn off fading when done. */
			fade_direction = FADE_STOP;
		}
		break;

	/* Fade using int_fadetime.This fades down, and will turn off the lamp
	 * if the level gets that low. */
	case FADE_TIME_DOWN_OFF:
		if (temp_level >= dali_ram.variable.minimum_level) {
			/* decrease level */
			if (temp_long_level <= temp_fade_step) {
				temp_long_level = 0;
			} else {
				if (fade_step == 0) { // Here check fade_step
					/* If these is no need to step, turn off the lamp directly*/
					status_information &= ~LAMP_ON;
					temp_long_level = 0;
					fade_direction  = FADE_STOP;
				} else {
					temp_long_level = temp_long_level - fade_step;
				}
			}
		} else {
			/* Once we have reached minimum level, turn off the lamp */
			status_information &= ~LAMP_ON;
			temp_long_level = 0;
			fade_direction  = FADE_STOP;
		}
		break;

	/* Fade using fade_rate (flash_fade_rate_val). */
	case FADE_RATE_UP:
		if (temp_level >= dali_ram.variable.maximum_level) {
			/* Fade terminates until fade expires */
			if (current_fade_time == 0) {
				temp_level                     = dali_ram.variable.maximum_level;
				temp_long_level                = (uint32_t)temp_level << 24;
				dali_ram.variable.target_level = temp_level;
				update_flag |= VAR_UPDATE;
				status_information |= LIMIT_ERROR;
				/* Turn off fading when done. */
				fade_direction = FADE_STOP;
			}
		} else {
			temp_long_level = temp_long_level + temp_fade_step;
			if (current_fade_time == 0) {
				/* Done fading */
				status_information &= ~LIMIT_ERROR;
				dali_ram.variable.target_level = temp_level;
				update_flag |= VAR_UPDATE;
				/* Turn off fading when done. */
				fade_direction = FADE_STOP;
			}
		}
		break;

	/* Fade using fade_rate (flash_fade_rate_val). Does not turn off
	 * the lamp. */
	case FADE_RATE_DOWN:
		if (temp_level < dali_ram.variable.minimum_level) {
			/* Fade terminates until fade expires */
			if (current_fade_time == 0) {
				temp_level                     = dali_ram.variable.minimum_level;
				temp_long_level                = (uint32_t)temp_level << 24;
				dali_ram.variable.target_level = temp_level;
				update_flag |= VAR_UPDATE;
				status_information |= LIMIT_ERROR;
				/* Turn off fading when done. */
				fade_direction = FADE_STOP;
			}
		} else {
			temp_long_level = temp_long_level - temp_fade_step;
			if (current_fade_time == 0) {
				/* Done fading */
				status_information &= ~LIMIT_ERROR;
				dali_ram.variable.target_level = temp_level;
				update_flag |= VAR_UPDATE;
				/* Turn off fading when done. */
				fade_direction = FADE_STOP;
			}
		}
		break;

	/* Identification fade in initialisation state */
	case FADE_DIRECT_IDENTIFICATION:
		temp_long_level = (uint32_t)temp_level << 24;
		/* Use physical max/min level temporarily */
		if (recall_level_flag == MIN_LEVEL_FLAG) {
			temp_level = POWER_MIN;
		} else {
			temp_level = POWER_MAX;
		}
		dali_cmd_set_pwm_output(temp_level);
		status_information &= ~FADE_RUNNING;
		fade_direction = FADE_IDLE;
		if (temp_level != 0) {
			if (interface_fail_flag == false) {
				last_active_level = temp_level;
			}
			if (!(status_information & LAMP_FAILURE)) {
				status_information |= LAMP_ON;
			}
		} else {
			status_information &= ~LAMP_ON;
		}
		/* Clear interface fail flag after fade if there is one */
		interface_fail_flag = false;
		break;
	/* Output directly */
	case FADE_DIRECT:
		dali_cmd_set_pwm_output(temp_level);
		temp_long_level = (uint32_t)temp_level << 24;
		status_information &= ~FADE_RUNNING;
		fade_direction = FADE_IDLE;
		if (temp_level != 0) {
			if (interface_fail_flag == false) {
				last_active_level = temp_level;
			}
			if (!(status_information & LAMP_FAILURE)) {
				status_information |= LAMP_ON;
			}
		} else {
			status_information &= ~LAMP_ON;
		}
		/* Clear interface fail flag after fade if there is one */
		interface_fail_flag = false;
		break;

	case FADE_BETWEEN_MINLEVEL_AND_OFF:
		if (fade_time_off_and_minlevel > 0) {
			fade_time_off_and_minlevel--;
		} else {
			dali.level     = dali_ram.variable.target_level;
			fade_direction = FADE_DIRECT;
		}
		break;

	default:
		break;
	}

	if (need_fade == true) {
		/* Current dali level */
		temp0 = temp_long_level >> 24;
		/* First 8 bit of the dali level fraction */
		temp1     = temp_long_level >> 16;
		temp_val0 = dali_cmd_read_pwm_value(temp0);
		/* Linear interpolation between two dali steps. Difference
		 * between two steps is maximum 110 in PWM output. */
		temp_val1 = ((dali_cmd_read_pwm_value(temp0 + 1) - dali_cmd_read_pwm_value(temp0)) * temp1) >> 8;

		dali_hal_update_pwm_output(temp_val0 + temp_val1);
	}

	dali.long_level = temp_long_level;
}

void dali_reset_memory_bank1(void)
{
	dali_ram.memory.bank1[0]  = BANK1_LAST_MEMORY_LOCATION;
	dali_ram.memory.bank1[1]  = INDICATOR;
	dali_ram.memory.bank1[2]  = BANK1_LOCK_BYTE;
	dali_ram.memory.bank1[3]  = OEM_GTIN0;
	dali_ram.memory.bank1[4]  = OEM_GTIN1;
	dali_ram.memory.bank1[5]  = OEM_GTIN2;
	dali_ram.memory.bank1[6]  = OEM_GTIN3;
	dali_ram.memory.bank1[7]  = OEM_GTIN4;
	dali_ram.memory.bank1[8]  = OEM_GTIN5;
	dali_ram.memory.bank1[9]  = OEM_ID_NUMBER0;
	dali_ram.memory.bank1[10] = OEM_ID_NUMBER1;
	dali_ram.memory.bank1[11] = OEM_ID_NUMBER2;
	dali_ram.memory.bank1[12] = OEM_ID_NUMBER3;
	dali_ram.memory.bank1[13] = OEM_ID_NUMBER4;
	dali_ram.memory.bank1[14] = OEM_ID_NUMBER5;
	dali_ram.memory.bank1[15] = OEM_ID_NUMBER6;
	dali_ram.memory.bank1[16] = OEM_ID_NUMBER7;
}
