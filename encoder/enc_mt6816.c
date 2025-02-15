/*
	Copyright 2016 - 2022 Benjamin Vedder	benjamin@vedder.se
	Copyright 2022 Marcos Chaparro	mchaparro@powerdesigns.ca
	Copyright 2022 Jakub Tomczak

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

#include "enc_mt6816.h"
#include "mc_interface.h"
#include "utils_math.h"
#include "spi_bb.h"
#include "timer.h"
#include <math.h>
#include <string.h>

#define MT6816_NO_MAGNET_ERROR_MASK		0x0002


void enc_mt6816_routine(MT6816_config_t *cfg) {
	
	float timestep = timer_seconds_elapsed_since(cfg->state.last_update_time);
	if (timestep > 1.0f) {
		timestep = 1.0f;
	}
	cfg->state.last_update_time = timer_time_now();
	
	uint16_t pc;
	uint16_t pos;
	uint16_t reg_data_03;
	uint16_t reg_data_04;
	uint16_t reg_addr_03 = 0x8300;
	uint16_t reg_addr_04 = 0x8400;

    // CS
		cs_down;
		HAL_SPI_TransmitReceive(&hspi1, (uint8_t*)&reg_addr_03, (uint8_t*)&reg_data_03, 1, 10);
		cs_up;
		cs_down;
		HAL_SPI_TransmitReceive(&hspi1, (uint8_t*)&reg_addr_04, (uint8_t*)&reg_data_04, 1, 10);
		cs_up;
//		cs_down;
//		HAL_SPI_TransmitReceive(&hspi1, (uint8_t*)&tx[2], (uint8_t*)&rx[2], 1, 10);
//		cs_up;
		

	pos = (reg_data_03 << 8) | reg_data_04;
	cfg->state.spi_val = pos;

	// check pc
	pc = pos;
	pc ^= pc >> 8;
	pc ^= pc >> 4;
	pc ^= pc >> 2;
	pc ^= pc >> 1;
		
	if ((~pc) & 1) {
		if (pos & MT6816_NO_MAGNET_ERROR_MASK) {
			++cfg->state.encoder_no_magnet_error_cnt;
			UTILS_LP_FAST(cfg->state.encoder_no_magnet_error_rate, 1.0f, timestep);
		} else {
			pos = pos >> 2;
			cfg->state.last_enc_angle = ((float) pos * 360.0f) / 16384.0f;
			UTILS_LP_FAST(cfg->state.spi_error_rate, 0.0f, timestep);
			UTILS_LP_FAST(cfg->state.encoder_no_magnet_error_rate, 0.0f, timestep);
		}
	} else {
		++cfg->state.spi_error_cnt;
		UTILS_LP_FAST(cfg->state.spi_error_rate, 1.0f, timestep);
	}
}
