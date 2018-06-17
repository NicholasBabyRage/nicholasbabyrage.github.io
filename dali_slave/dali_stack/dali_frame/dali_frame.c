/**
 * \file
 *
 * \brief Process dali slave transmission frame
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

#define SPECIAL_CMD_LOW TERMINATE
#define SPECIAL_CMD_HIGH WRITE_MEMORY_LOCATION_NO_REPLY

/* Address type */
#define NOT_ME 0x00
#define GROUP_ADDR 0x01
#define BROADCAST 0x02
#define SHORT_ADDR 0x03
#define SPECIAL_CMD 0x04
#define BROADCAST_UNADDRESSED 0x05

/* Time used in frame */
#define BEFORE_FORWARD_FRAMES_TIME (9 * ONE_MS_TICK)
#define BEFORE_BACKWARD_FRAME_TIME (5 * ONE_MS_TICK)
#define REPETITION_115MS (115 * ONE_MS_TICK)

/* Frame transmission state */
enum {
	IDLE,
	FORWARD_FRAME,
	FORWARD_BACKWARD_DELAY,
	FORWARD_IDLE_DELAY,
	FORWARD_REPEAT_FRAME,
	FORWARD_REPEAT_DELAY,
	BACKWARD_FRAME_ANSWER,
	BACKWARD_IDLE_DELAY
};

extern struct dali_data dali_ram;
extern uint8_t          status_information;
extern uint8_t          update_flag;

uint8_t                   data_backup;
uint8_t                   frame_state;
bool                      active_state = false;
dali_slave_service_info_t service_info;

uint8_t dali_frame_check_addr(dali_slave_service_info_t *slave_info);
uint8_t dali_frame_process_data(dali_slave_service_info_t *slave_info, uint8_t addr_type, bool repeat_flag);

void dali_frame_init(void)
{
	dali_cmd_init();
	frame_state = IDLE;
}

void dali_process_frame_state(void)
{
	uint8_t addr_type;
	switch (frame_state) {
	case IDLE:
		frame_state                = FORWARD_FRAME;
		service_info.get_new_frame = false;
		break;

	case FORWARD_FRAME:
		if (service_info.get_new_frame == true) {
			service_info.frame_time    = 0;
			service_info.get_new_frame = false;
			addr_type                  = dali_frame_check_addr(&service_info);
			if (addr_type != NOT_ME) {
				frame_state  = dali_frame_process_data(&service_info, addr_type, false);
				active_state = false;
			}
		}
		break;

	case FORWARD_REPEAT_FRAME:
		if ((service_info.frame_time < REPETITION_115MS) && (service_info.frame_time >= BEFORE_FORWARD_FRAMES_TIME)) {
			if (service_info.get_new_frame == true) {
				service_info.get_new_frame = false;
				addr_type                  = dali_frame_check_addr(&service_info);
				if (addr_type != NOT_ME) {
					if (data_backup == service_info.rec_data) {
						dali_frame_process_data(&service_info, addr_type, true);
						frame_state = IDLE;
					} else {
						/* if received diffrent command, then execute that one */
						service_info.frame_time = 0;
						frame_state             = dali_frame_process_data(&service_info, addr_type, false);
						active_state            = false;
					}
				} else {
					/* the two forward frame are different, then exit send-twice frame*/
					frame_state = IDLE;
				}
			} else {
				/* Before receive a whole frame, check bus active state */
				if (0 == dali_set_bus_receive_state_to_stack()) {
					/* Does active state occur in between two forward frames */
					if (active_state == true) {
						/* Check new frame again before exit as bus inactive now */
						if (service_info.get_new_frame != true) {
							/* active state occurs but it's not a whole frame */
							frame_state = IDLE;
						}
						active_state = false;
					}
				} else {
					active_state = true;
				}
			}
		} else if (service_info.frame_time >= REPETITION_115MS) {
			frame_state = IDLE;
		}
		break;

	case FORWARD_REPEAT_DELAY:
		if (service_info.frame_time >= REPETITION_115MS) {
			frame_state = IDLE;
		}
		break;

	case FORWARD_BACKWARD_DELAY:
		if (service_info.frame_time >= BEFORE_BACKWARD_FRAME_TIME) {
			/* start backward frame */
			service_info.send_status = SENDING_START;
			dali_hal_disable_forward_enable_backward();
			frame_state = BACKWARD_FRAME_ANSWER;
		}
		break;

	case BACKWARD_FRAME_ANSWER:
		if (service_info.send_status == SENDING_STOP) {
			dali_hal_enable_forward_disable_backward();
			service_info.frame_time = 0;
			frame_state             = BACKWARD_IDLE_DELAY;
		}
		break;

	case FORWARD_IDLE_DELAY:
	case BACKWARD_IDLE_DELAY:
		if (service_info.frame_time >= BEFORE_FORWARD_FRAMES_TIME) {
			frame_state = IDLE;
		}
		break;

	default:
		break;
	}
}

/**
 *  \brief Check the dali address type.
 *
 *  \param slave_info pointer to the struct of dali slave information.
 *
 *  \returns the address type
 *
 */
uint8_t dali_frame_check_addr(dali_slave_service_info_t *slave_info)
{
	uint8_t temp;
	/* group or broadcast address */
	if (slave_info->rec_addr & ADDR_Y) {
		if ((slave_info->rec_addr & ADDR_NON_SHORT) == 0) {
			/* group address, 0-15 */
			temp = (slave_info->rec_addr & ADDR_GROUP_A) >> 1;
			if (temp < 8) {
				if (dali_ram.variable.group_0_7 & (1 << temp)) {
					return GROUP_ADDR;
				}
			} else {
				if (dali_ram.variable.group_8_15 & (1 << (temp - 8))) {
					return GROUP_ADDR;
				}
			}
		} else if ((slave_info->rec_addr & ADDR_BROADCAST) == ADDR_BROADCAST) {
			return BROADCAST;
		} else if ((slave_info->rec_addr & ADDR_BROADCAST) == ADDR_BROADCAST_UNADDRESSED) {
			if (dali_ram.variable.short_address == ADDR_UNADDRESSED) {
				return BROADCAST_UNADDRESSED;
			}
		} else if ((slave_info->rec_addr >= SPECIAL_CMD_LOW) && (slave_info->rec_addr <= SPECIAL_CMD_HIGH)) {
			return SPECIAL_CMD;
		}
	} else if ((slave_info->rec_addr & ADDR_SHORT_A) == (dali_ram.variable.short_address << 1)) {
		return SHORT_ADDR;
	}
	return NOT_ME;
}

/**
 *  \brief Process the dali address and command. Frame_state should be changed
 *   according to the command.
 *
 *  \param slave_info pointer to the struct of dali slave information.
 *  \param addr_type indicate the address type.
 *  \param repeat_flag indicate repetition of configuration command.
 *   false: first received, true: second received.
 *
 *  \returns the frame state
 *
 */
uint8_t dali_frame_process_data(dali_slave_service_info_t *slave_info, uint8_t addr_type, bool repeat_flag)
{
	uint8_t  ret = FORWARD_IDLE_DELAY;
	uint16_t response;

	if (slave_info->rec_addr & ADDR_S) {
		if (addr_type == SPECIAL_CMD) {
			response = dali_cmd_process_special(slave_info, repeat_flag);
		} else {
			response = dali_cmd_process_normal(slave_info, repeat_flag);
		}

		if (response == RECEIVE_FIRST) {
			data_backup = slave_info->rec_data;
			ret         = FORWARD_REPEAT_FRAME;
		} else if (response != ANSWER_NO) {
			slave_info->send_data = (uint8_t)response;
			ret                   = FORWARD_BACKWARD_DELAY;
		}

	} else if (addr_type != SPECIAL_CMD) {
		dali_cmd_direct_arc_power_control(slave_info->rec_data);
	}
	return ret;
}

void dali_frame_ms_tick(void)
{
	if (service_info.frame_time < 0xFF) {
		service_info.frame_time++;
	}
}

void dali_set_addr_to_stack(uint8_t address)
{
	service_info.rec_addr = address;
}

void dali_set_data_to_stack(uint8_t data)
{
	service_info.rec_data = data;
}

uint8_t dali_get_sent_status_from_stack(void)
{
	return service_info.send_status;
}

void dali_set_sent_status_to_stack(uint8_t status)
{
	service_info.send_status = status;
}

void dali_set_received_flag_to_stack(bool flag)
{
	service_info.get_new_frame = flag;
}

uint8_t dali_get_sent_data_from_stack(void)
{
	return service_info.send_data;
}

uint8_t *dali_get_data_addr_from_stack(void)
{
	return (&dali_ram.variable.target_level);
}

uint8_t dali_get_data_size_from_stack(void)
{
	return (sizeof(dali_ram));
}

uint8_t *dali_get_update_flag_addr_from_stack(void)
{
	return (&update_flag);
}

void dali_set_gear_failure_status_to_stack(bool failure_status)
{
	if (failure_status) {
		status_information |= CONTROL_GEAR_FAILURE;
	} else {
		status_information &= ~CONTROL_GEAR_FAILURE;
	}
}

void dali_set_lamp_failure_status_to_stack(bool failure_status)
{
	if (failure_status) { // Set lamp failure
		status_information |= LAMP_FAILURE;
		status_information &= ~LAMP_ON;
	} else { /* Should periodically check for
	         targetLevel changes from 0x00 to another value */
		if (dali_ram.variable.target_level != 0) {
			status_information |= LAMP_ON;
			status_information &= ~LAMP_FAILURE;
		}
	}
}

uint8_t dali_set_bus_receive_state_to_stack(void)
{
	return status_receive;
}
