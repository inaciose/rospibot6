/**
 * @file hardware++.h
 *
 */
/* Copyright (C) 2019 by Arjan van Vught mailto:info@raspberrypi-dmx.nl
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of thnDmxDataDirecte Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:

 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.

 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include <stdint.h>
#include <stddef.h>
#include <time.h>
#include <assert.h>

#include "hardware.h"

#include "c/hardware.h"
#include "c/sys_time.h"

#include "h3.h"
#include "h3_watchdog.h"
#include "h3_board.h"

#include "arm/synchronize.h"

#if defined(ORANGE_PI)
#elif defined(ORANGE_PI_ONE)
#else
 #error Platform not supported
#endif

static const char s_SocName[2][4] __attribute__((aligned(4))) = { "H2+", "H3\0" };
static const uint8_t s_SocNameLenghth[2] = { 3, 2 };

static const char s_CpuName[] __attribute__((aligned(4))) = "Cortex-A7";
#define CPU_NAME_LENGHTH (sizeof(s_CpuName) / sizeof(s_CpuName[0]) - 1)

const char s_Machine[] __attribute__((aligned(4))) = "arm";
#define MACHINE_LENGTH (sizeof(s_Machine)/sizeof(s_Machine[0]) - 1)

const char s_SysName[] __attribute__((aligned(4))) = "Baremetal";
#define SYSNAME_LENGTH (sizeof(s_SysName)/sizeof(s_SysName[0]) - 1)

Hardware *Hardware::s_pThis = 0;

Hardware::Hardware(void): m_bIsWatchdog(false) {
	s_pThis = this;
}

Hardware::~Hardware(void) {
}

const char* Hardware::GetMachine(uint8_t& nLength) {
	nLength = MACHINE_LENGTH;
	return s_Machine;
}

const char* Hardware::GetSysName(uint8_t& nLength) {
	nLength = SYSNAME_LENGTH;
	return s_SysName;
}

const char* Hardware::GetBoardName(uint8_t& nLength) {
	nLength = sizeof(H3_BOARD_NAME)  - 1;
	return H3_BOARD_NAME;
}

const char* Hardware::GetCpuName(uint8_t& nLength) {
	nLength = CPU_NAME_LENGHTH;
	return s_CpuName;
}

const char* Hardware::GetSocName(uint8_t& nLength) {
#if defined(ORANGE_PI)
	nLength = s_SocNameLenghth[0];
	return s_SocName[0];
#else
	nLength = s_SocNameLenghth[1];
	return s_SocName[1];
#endif
}

bool Hardware::SetTime(const struct THardwareTime& pTime) {
	struct hardware_time tm_hw;

	tm_hw.year = pTime.tm_year;
	tm_hw.month = pTime.tm_mon;
	tm_hw.day = pTime.tm_mday;
	tm_hw.hour = pTime.tm_hour;
	tm_hw.minute = pTime.tm_min;
	tm_hw.second = pTime.tm_sec;

	hardware_rtc_set(&tm_hw);

	return true;
}

void Hardware::GetTime(struct THardwareTime* pTime) {
	time_t ltime;
	struct tm *local_time;

	ltime = time(0);
    local_time = localtime(&ltime);

    pTime->tm_year = local_time->tm_year;
    pTime->tm_mon = local_time->tm_mon ;
    pTime->tm_mday = local_time->tm_mday;
    //
    pTime->tm_hour = local_time->tm_hour;
    pTime->tm_min = local_time->tm_min;
    pTime->tm_sec = local_time->tm_sec;
}

bool Hardware::Reboot(void) {
	hardware_led_set(1);

	h3_watchdog_enable();

	invalidate_instruction_cache();
	flush_branch_target_cache();
	flush_prefetch_buffer();
	clean_data_cache();
	invalidate_data_cache();

	for (;;)
		;

	__builtin_unreachable ();

	return true;
}
