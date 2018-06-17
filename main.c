/**
 * \file
 *
 * \brief DALI Slave Demo based on ATtiny1617
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

/**
 * \mainpage
 *
 * \section intro Introduction
 *
 * This application demonstrates the DALI slave function corresponding to DALI standard 2.0.  
 * The slave communicates with DALI master and executes the commands. The commands include
 * power control, configuration, query and special commands. For details please refer to 
 * IEC62386 standards. 
 *
 * Note: Some configurations need to be set in MCU fuse. Please refer to software user guide AVR42793.
 *
 *       For 5V system, BOD fuse enable and set level 4.0V. For 3.3V, BOD fuse enable and level 2.6V.
 *
 *       Per datasheet chapter "Electrical Characteristics", the Maximum Frequency vs. VDD is linear
 *       between 1.8V < VDD < 2.7V and 2.7V < VDD < 4.5V. So For 3.3V system, system clock is configured
 *       to 8MHz divided from prescaler.
 *
 *
 * \section contactinfo Contact Information
 * For further information, visit
 * <A href="http://www.atmel.com/">Atmel</A>.\n
 */
#include <atmel_start.h>
#include "dali_top.h"
#include "adc_window.h"

/*one mS tick flag*/
volatile bool ms_tick_flag;

/* ����� 5 ������� � ������. 50ms ��� ��������� ���� ������� */
#define UPDATE_PERIOD 50
/* ˳������� ��� ��������� EEPROM */
uint16_t ms_count = UPDATE_PERIOD;

// ����������� �������
void system_mcu_init(void);  
void dali_init(void);
void dali_process_ms_tick(void);

/*
 * ��������� DALI SLAVE ���������.
 */
int main(void)
{
	system_mcu_init(); // ������������ ������ ��������/�������.
	dali_init();       // ����������� DALI �����, ���������� �����,BOD �'���
	ADC_0_initialization(); // ����������� ���
	sei();             // ��������� �������� �����������

	while (1) {
		dali_process_power_on(); //������������ ����� ��� ����������� ���� ����� ���� �������� "slave"
		dali_process_frame_state(); // ������� ��������� ����� � ���������� ��������� ������
		dali_process_interface_failure(); //�������� ���� ���� DALI. ��� ����������� ����� interface_fail_flag ����� �������������� � system_failure_level 
		dali_process_identify_device(); // ������� �� �������������. �������� ������, ������� � �.�.
		dali_process_ms_tick(); // ������� ������������ � ��������� ���� ��������� �� �������
	}
}

/*
 * ����������� MCU �������
 */
void system_mcu_init(void)
{
	CCP = 0xD8;	//��������� ������������� I/O �����
#ifdef SYSTEM_5V // ��������� � dali_top.h
	CLKCTRL.MCLKCTRLB = 0; //�������� ������ �������
#else
	CLKCTRL.MCLKCTRLB = 1; // �������� ������ ������� �� 2
#endif
}

/**
 * ����������� ������, �� ���������������� DALI
 */
void dali_init(void)
{
	dali_bit_init(); //������������ ����� DALI
	dali_tc_init();  //������������ �������/����������
	dali_frame_init(); // �������� DALI ����� � RAM
	dali_bod_init(); //������������ FUSE ��� BOD � ��������� �� ������� ��������
}

/**
 * �� ms ���, �� ���������������� DALI
 */
void dali_process_ms_tick(void)
{
	if (ms_tick_flag == true) {  //����������� ������������� �������
		ms_tick_flag = false;
		
		dali_frame_ms_tick();   // frame_time++
		dali_command_fade_ms_tick();	// ������ ���� PWM ������� �� �����������
		ms_count--;
		if (ms_count == 0) {
			dali_eeprom_update_tick(); // ��������� ����� DALI � EEPROM
			ms_count = UPDATE_PERIOD;
		}
	}
}

