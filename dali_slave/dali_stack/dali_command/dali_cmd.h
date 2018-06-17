/**
 * \file
 *
 * \brief Process dali slave commands process
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

#ifndef COMMANDS_H
#define COMMANDS_H

#define BIT0 (1 << 0)
#define BIT1 (1 << 1)
#define BIT2 (1 << 2)
#define BIT3 (1 << 3)
#define BIT4 (1 << 4)
#define BIT5 (1 << 5)
#define BIT6 (1 << 6)
#define BIT7 (1 << 7)

#define LED_MODULES 6

/* indirect ARC power control */
#define OFF 0
#define UP 1
#define DOWN 2
#define STEP_UP 3
#define STEP_DOWN 4
#define RECALL_MAX_LEVEL 5
#define RECALL_MIN_LEVEL 6
#define STEP_DOWN_AND_OFF 7
#define ON_AND_STEP_UP 8
#define ENABLE_DAPC_SEQUENCE 9
#define GO_TO_LAST_ACTIVE_LEVEL 10
/* 11-15 reserved */
#define GO_TO_SCENE0 16
#define GO_TO_SCENE1 17
#define GO_TO_SCENE2 18
#define GO_TO_SCENE3 19
#define GO_TO_SCENE4 20
#define GO_TO_SCENE5 21
#define GO_TO_SCENE6 22
#define GO_TO_SCENE7 23
#define GO_TO_SCENE8 24
#define GO_TO_SCENE9 25
#define GO_TO_SCENE10 26
#define GO_TO_SCENE11 27
#define GO_TO_SCENE12 28
#define GO_TO_SCENE13 29
#define GO_TO_SCENE14 30
#define GO_TO_SCENE15 31

/* Configuration commands */
#define RESET 32
#define STORE_ACTUAL_LEVEL_IN_THE_DTR 33
#define SAVE_PERSISTENT_VARIABLES 34
#define SET_OPERATING_MODE 35
#define RESET_MEMORY_BANK 36
#define IDENTIFY_DEVICE 37
/* 38-41 reserved */
#define SET_MAX_LEVEL_WITH_DTR0 42
#define SET_MIN_LEVEL_WITH_DTR0 43
#define SET_SYSTEM_FAILURE_LEVEL_WITH_DTR0 44
#define SET_POWER_ON_LEVEL_WITH_DTR0 45
#define SET_FADE_TIME_WITH_DTR0 46
#define SET_FADE_RATE_WITH_DTR0 47
#define SET_EXTENDED_FADE_TIME_WITH_DTR0 48
/* 49-63 reserved */
#define SET_SCENE0_WITH_DTR0 64
#define SET_SCENE1_WITH_DTR0 65
#define SET_SCENE2_WITH_DTR0 66
#define SET_SCENE3_WITH_DTR0 67
#define SET_SCENE4_WITH_DTR0 68
#define SET_SCENE5_WITH_DTR0 69
#define SET_SCENE6_WITH_DTR0 70
#define SET_SCENE7_WITH_DTR0 71
#define SET_SCENE8_WITH_DTR0 72
#define SET_SCENE9_WITH_DTR0 73
#define SET_SCENE10_WITH_DTR0 74
#define SET_SCENE11_WITH_DTR0 75
#define SET_SCENE12_WITH_DTR0 76
#define SET_SCENE13_WITH_DTR0 77
#define SET_SCENE14_WITH_DTR0 78
#define SET_SCENE15_WITH_DTR0 79

#define REMOVE_FROM_SCENE0 80
#define REMOVE_FROM_SCENE1 81
#define REMOVE_FROM_SCENE2 82
#define REMOVE_FROM_SCENE3 83
#define REMOVE_FROM_SCENE4 84
#define REMOVE_FROM_SCENE5 85
#define REMOVE_FROM_SCENE6 86
#define REMOVE_FROM_SCENE7 87
#define REMOVE_FROM_SCENE8 88
#define REMOVE_FROM_SCENE9 89
#define REMOVE_FROM_SCENE10 90
#define REMOVE_FROM_SCENE11 91
#define REMOVE_FROM_SCENE12 92
#define REMOVE_FROM_SCENE13 93
#define REMOVE_FROM_SCENE14 94
#define REMOVE_FROM_SCENE15 95

#define ADD_TO_GROUP0 96
#define ADD_TO_GROUP1 97
#define ADD_TO_GROUP2 98
#define ADD_TO_GROUP3 99
#define ADD_TO_GROUP4 100
#define ADD_TO_GROUP5 101
#define ADD_TO_GROUP6 102
#define ADD_TO_GROUP7 103
#define ADD_TO_GROUP8 104
#define ADD_TO_GROUP9 105
#define ADD_TO_GROUP10 106
#define ADD_TO_GROUP11 107
#define ADD_TO_GROUP12 108
#define ADD_TO_GROUP13 109
#define ADD_TO_GROUP14 110
#define ADD_TO_GROUP15 111

#define REMOVE_FROM_GROUP0 112
#define REMOVE_FROM_GROUP1 113
#define REMOVE_FROM_GROUP2 114
#define REMOVE_FROM_GROUP3 115
#define REMOVE_FROM_GROUP4 116
#define REMOVE_FROM_GROUP5 117
#define REMOVE_FROM_GROUP6 118
#define REMOVE_FROM_GROUP7 119
#define REMOVE_FROM_GROUP8 120
#define REMOVE_FROM_GROUP9 121
#define REMOVE_FROM_GROUP10 122
#define REMOVE_FROM_GROUP11 123
#define REMOVE_FROM_GROUP12 124
#define REMOVE_FROM_GROUP13 125
#define REMOVE_FROM_GROUP14 126
#define REMOVE_FROM_GROUP15 127

#define SET_SHORT_ADDRESS_WITH_DTR0 128
#define ENABLE_WRITE_MEMORY 129
/* 130-143 reserved */

/* Query commands */
#define QUERY_STATUS 144
#define QUERY_CONTROL_GEAR 145
#define QUERY_LAMP_FAILURE 146
#define QUERY_LAMP_POWER_ON 147
#define QUERY_LIMIT_ERROR 148
#define QUERY_RESET_STATE 149
#define QUERY_MISSING_SHORT_ADDRESS 150
#define QUERY_VERSION_NUMBER 151
#define QUERY_CONTENT_DTR 152
#define QUERY_DEVICE_TYPE 153
#define QUERY_PHYSICAL_MINIMUM_LEVEL 154
#define QUERY_POWER_FAILURE 155
#define QUERY_CONTENT_DTR1 156
#define QUERY_CONTENT_DTR2 157
#define QUERY_OPERATING_MODE 158
#define QUERY_LIGHT_SOURCE_TYPE 159
/* Queries related to arc power parameter settings */
#define QUERY_ACTUAL_LEVEL 160
#define QUERY_MAX_LEVEL 161
#define QUERY_MIN_LEVEL 162
#define QUERY_POWER_ON_LEVEL 163
#define QUERY_SYSTEM_FAILURE_LEVEL 164
#define QUERY_FADE_TIME_RATE 165
#define QUERY_MANUFACTURER_SPECIFIC_MODE 166
#define QUERY_NEXT_DEVICE_TYPE 167
#define QUERY_EXTENDED_FADE_TIME 168
#define QUERY_CONTROL_GEAR_FAILURE 170
/* 171-175 reserved */
/* Queries related to system parameter settings */
#define QUERY_SCENE_LEVEL0 176
#define QUERY_SCENE_LEVEL1 177
#define QUERY_SCENE_LEVEL2 178
#define QUERY_SCENE_LEVEL3 179
#define QUERY_SCENE_LEVEL4 180
#define QUERY_SCENE_LEVEL5 181
#define QUERY_SCENE_LEVEL6 182
#define QUERY_SCENE_LEVEL7 183
#define QUERY_SCENE_LEVEL8 184
#define QUERY_SCENE_LEVEL9 185
#define QUERY_SCENE_LEVEL10 186
#define QUERY_SCENE_LEVEL11 187
#define QUERY_SCENE_LEVEL12 188
#define QUERY_SCENE_LEVEL13 189
#define QUERY_SCENE_LEVEL14 190
#define QUERY_SCENE_LEVEL15 191

#define QUERY_GROUPS_0_7 192
#define QUERY_GROUPS_8_15 193
#define QUERY_RANDOM_ADDRESS_H 194
#define QUERY_RANDOM_ADDRESS_M 195
#define QUERY_RANDOM_ADDRESS_L 196
#define READ_MEMORY_LOCATION 197
/* 198-223 reserved */
/************ application extended commands ************/
#define REFERENCE_SYSTEM_POWER 224
#define ENABLE_CURRENT_PROTECTOR 225
#define DISABLE_CURRENT_PROTECTOR 226
#define SELECT_DIMMING_CURVE 227
#define STORE_DTR_AS_FAST_FADE_TIME 228
/* 229-236 reserved */
#define QUERY_GEAR_TYPE 237
#define QUERY_DIMMING_CURVE 238
#define QUERY_POSSIBLE_OPERATING_MODES 239
#define QUERY_FEATURES 240
#define QUERY_FAILURE_STATUS 241
#define QUERY_SHORT_CIRCUIT 242
#define QUERY_OPEN_CIRCUIT 243
#define QUERY_LOAD_DECREASE 244
#define QUERY_LOAD_INCREASE 245
#define QUERY_CURRENT_PROTECTOR_ACTIVE 246
#define QUERY_THERMAL_SHUT_DOWN 247
#define QUERY_THERMAL_OVERLOAD 248
#define QUERY_REFERENCE_RUNNING 249
#define QUERY_REFERENCE_MEASUREMENT_FAILED 250
#define QUERY_CURRENT_PROTECTOR_ENABLED 251
#define QUERY_LED_MODULE_OPERATING_MODE 252
#define QUERY_FAST_FADE_TIME 253
#define QUERY_MIN_FAST_FADE_TIME 254
#define QUERY_EXTENDED_VERSION_NUMBER 255
/*******************************************************/

/****************** Special Commands *******************/
#define TERMINATE 0xA1
#define DATA_TRANSFER_REGISTER 0xA3
#define INITIALISE 0xA5
#define RANDOMISE 0xA7
#define COMPARE 0xA9
#define WITHDRAW 0xAB
#define PING 0xAD
#define SEARCHADDRH 0xB1
#define SEARCHADDRM 0xB3
#define SEARCHADDRL 0xB5
#define PROGRAM_SHORT_ADDRESS 0xB7
#define VERIFY_SHORT_ADDRESS 0xB9
#define QUERY_SHORT_ADDRESS 0xBB
#define ENABLE_DEVICE_TYPE 0xC1
#define DATA_TRANSFER_REGISTER_1 0xC3
#define DATA_TRANSFER_REGISTER_2 0xC5
#define WRITE_MEMORY_LOCATION 0xC7
#define WRITE_MEMORY_LOCATION_NO_REPLY 0xC9
/*******************************************************/

/* Define a three-bytes structure */
struct uint24 {
	uint8_t low;
	uint8_t mid;
	uint8_t high;
};

/* Control gear status */
// uint8_t status_information;
#define CONTROL_GEAR_FAILURE BIT0
#define LAMP_FAILURE BIT1
#define LAMP_ON BIT2
#define LIMIT_ERROR BIT3
#define FADE_RUNNING BIT4
#define RESET_STATE BIT5
#define MISSING_SHORT_ADDR BIT6
#define POWER_CYCLE_SEEN BIT7

// 41 bytes
struct var {
	/* The target level also as last light level of the lamp */
	uint8_t target_level;

	/* The light level to use when powering up the slave */
	uint8_t power_on_level;

	/* The light level to use if the communication fails */
	uint8_t system_failure_level;

	/* The minimum allowed light level */
	uint8_t minimum_level;

	/* The maximum allowed light level */
	uint8_t maximum_level;

	/* The speed to change the light level during a certain time interval */
	uint8_t fade_rate;

	/* The time between the change of light levels */
	uint8_t fade_time;

	/* The time base of extended fade */
	uint8_t extended_fade_time_base;

	/* The multiplier of extended fade */
	uint8_t extended_fade_time_multiplier;

	/* The individual address of a unit */
	uint8_t short_address;

	/* Random address of address allocation */
	struct uint24 random_address;

	/* Control gear operating mode */
	uint8_t operating_mode;

	uint8_t scene[16];
	/* dali commands splits the group in 2 bytes, so it is easier to address these
	    * separately. */
	uint8_t group_0_7;
	uint8_t group_8_15;

	/* The currently implemented dali protocol version number */
	uint8_t version_number;

	/* The smallest allowed physical arc power level */
	uint8_t physical_minimum_level;

	uint8_t min_fast_fade_time;
	uint8_t fast_fade_time;
	uint8_t gear_type;
	uint8_t possible_operation_modes;
	uint8_t features;

	/* Bit7 of failure status and bit4 of operation mode */
	uint8_t failure_status_operation_mode;

	uint8_t dimming_curve;
};

#define BANK0_SIZE 27
#define BANK1_SIZE 17
// 44 bytes
struct mem {
	uint8_t bank0[BANK0_SIZE];
	uint8_t bank1[BANK1_SIZE];
};
// 85 bytes
struct dali_data {
	struct var variable;
	struct mem memory;
};

/**
 *  \brief Initialize the variables used.
 */
void dali_cmd_init(void);

/**
 *  \brief Process direct arc power control command.
 *
 *  \param level The target power level to set.
 */
void dali_cmd_direct_arc_power_control(uint8_t level);

/**
 *  \brief Process the dali common commands 1-255.
 *
 *  \param slave_info pointer to the struct of dali slave information.
 *  \param repeat_flag indicate repetition of configuration command.
 *         0: first received, 1: second received.
 *
 *  \retval RECEIVE_FIRST: first received the configuration command
 *          ANSWER_NO: not need to answer
 *          others: queried data or need to answer YES
 *
 *  \note Divide into two functions can save program space and be meaningful
 */
uint16_t dali_cmd_process_normal(dali_slave_service_info_t *slave_info, uint8_t repeat_flag);

/**
 *  \brief Process the dali special commands 256-349.
 *
 *  \param slave_info pointer to the struct of dali slave information.
 *  \param repeat_flag indicate repetition of configuration command.
 *         0: first received, 1: second received.
 *
 *  \retval RECEIVE_FIRST: first received the configuration command
 *          ANSWER_NO: not need to answer
 *          others: queried data or need to answer YES
 */
uint16_t dali_cmd_process_special(dali_slave_service_info_t *slave_info, uint8_t repeat_flag);

#endif /* COMMANDS_H */
