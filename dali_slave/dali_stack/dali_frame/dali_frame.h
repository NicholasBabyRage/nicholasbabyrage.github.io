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
#ifndef DALI_FRAME_H
#define DALI_FRAME_H

/* Refer to standard, address byte 'YAAA AAAS' */
#define ADDR_Y 0x80
#define ADDR_NON_SHORT 0x60
#define ADDR_SHORT_A 0x7E
#define ADDR_GROUP_A 0x1E
#define ADDR_S 0x01
#define ADDR_BROADCAST 0xFE
#define ADDR_BROADCAST_UNADDRESSED 0xFC
#define ADDR_UNADDRESSED 0xFF

#define ANSWER_NO 0xFF00
#define ANSWER_YES 0x00FF
#define RECEIVE_FIRST 0xF000

typedef struct {
	uint8_t       rec_addr;
	uint8_t       rec_data;
	uint8_t       send_data;
	volatile bool get_new_frame;
	uint8_t       frame_time;
	uint8_t       send_status;
} dali_slave_service_info_t;

#endif /* DALI_FRAME_H */
