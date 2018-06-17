/**
 * \file
 *
 * \brief Configure dali slave TC
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

extern volatile bool ms_tick_flag;

void dali_tc_init(void)
{
	/* TCA ��������������� �� 1mS �� ��� ������������ ������� */
	TCA0.SINGLE.PER = SYS_TICK_TC_TOP;
	/*���������� ����� ������*/
	TCA0.SINGLE.CTRLB = 0x00;
	/*����������� �� ������������*/
	TCA0.SINGLE.INTCTRL = 0x01;
	/*�������� TCA0*/
	TCA0.SINGLE.CTRLA = 0x01;

	/* TCB ��������������� ��� ���������/����������� DALI */
	/*����� ����������� �����������*/
	TCB0.CCMP  = DALI_BIT_TC_TOP;
	TCB0.CTRLB = 0; 
	/*�������� TCB*/
	TCB0.CTRLA = 0x01;
	/*��� ������� �������� �����������*/
	TCB0.INTCTRL = 0;

	/* TCD ��������������� ��� �������� ������ LED */
	/*���� �� �������� DALI_PWM_TC_TOP */
	TCD0.CTRLB   = 0x00;//One Ramp �����
	TCD0.CMPBCLR = DALI_PWM_TC_TOP;
	/*� ������ ������� LED ��������� */
	TCD0.CMPBSET = DALI_PWM_TC_TOP;
	/*��������� ��������� ; ĳ����� ������� - 1*/
	TCD0.CTRLA = 0x61; 

	/* RTC ��������� OSCULP32K ��������������� ��� ���������� ��������� DALI ������ */
	/* OSCULP32K �� ������� 32KHz*/
	// RTC_CLKCTRL = 0;
	RTC.PER = 0xFFFF;
	/*�������� RTC*/
	RTC.CTRLA = 0x01;
}

/*����������� TCA ms*/
ISR(TCA0_OVF_vect)
{
	/*�������� OVF ����*/
	TCA0.SINGLE.INTFLAGS = 0x01;
	ms_tick_flag         = true;
}
