/*
	Copyright 2019 Benjamin Vedder	benjamin@vedder.se

	This file is part of the VESC firmware.

	The VESC firmware is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    The VESC firmware is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
    */

#ifndef TIMER_H_
#define TIMER_H_

#include <stdint.h>

// Settings
#define TIMER_HZ					1.4e7

typedef enum {
	pid_cc = 1,
	encoder_cc,
	default_cc3,
	default_cc4
}delay_us_cc_t;

void timer_init(void);
uint32_t timer_time_now(void);
float timer_seconds_elapsed_since(uint32_t time);
void timer_sleep(float seconds);
void osDelayUs(uint32_t _uiTimeOut, delay_us_cc_t _CC);
#endif /* TIMER_H_ */
