/*
	Copyright 2016 Benjamin Vedder	benjamin@vedder.se

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

#include "comm_usb.h"
#include "packet.h"
#include "commands.h"
#include "usbd_cdc_if.h"

// Private variables
#define SERIAL_RX_BUFFER_SIZE		2048
static uint8_t serial_rx_buffer[SERIAL_RX_BUFFER_SIZE];
static int serial_rx_read_pos = 0;
static int serial_rx_write_pos = 0;
//static THD_WORKING_AREA(serial_read_thread_wa, 256);
//static THD_WORKING_AREA(serial_process_thread_wa, 2048);

static osThreadId_t process_id;
static const osThreadAttr_t processTask_attributes = {
  .name = "USB_process",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 16
};

osThreadId_t usb_read_id;
const osThreadAttr_t readTask_attributes = {
  .name = "USB_read",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};

static osMutexId_t send_mutex;
static const osMutexAttr_t send_mutex_attributes = {
  .name = "send_mutex"
};

extern uint8_t UserRxBufferFS[APP_RX_DATA_SIZE];
static volatile unsigned int write_timeout_cnt = 0;
static volatile bool was_timeout = false;
static PACKET_STATE_t packet_state;

// Private functions
static void process_packet(unsigned char *data, unsigned int len);
static void send_packet_raw(unsigned char *buffer, unsigned int len);
//StartDefaultTask(void *argument)
static void serial_read_thread(void *argument) {
	(void)argument;

	uint8_t buffer[2];
	int had_data = 0;

	for(;;) {
		osThreadFlagsWait(0xffff,osFlagsWaitAny|osFlagsNoClear,osWaitForever);
		int len = osThreadFlagsGet();
		osThreadFlagsClear(len);
		
		for (int i = 0;i < len;i++) {
			serial_rx_buffer[serial_rx_write_pos++] = UserRxBufferFS[i];

			if (serial_rx_write_pos == SERIAL_RX_BUFFER_SIZE) {
				serial_rx_write_pos = 0;
			}

			had_data = 1;
		}

		if (had_data) {
			CDC_Transmit_FS(&serial_rx_buffer[serial_rx_write_pos],len);
			//osThreadFlagsSet(process_tp,0x01);
			had_data = 0;
		}
	}
}

static  void serial_process_thread(void *argument) {
	(void)argument;

	for(;;) {
		osThreadFlagsWait(0xFFFFFFFF,osFlagsWaitAny,osWaitForever);

		while (serial_rx_read_pos != serial_rx_write_pos) {
			packet_process_byte(serial_rx_buffer[serial_rx_read_pos++], &packet_state);

			if (serial_rx_read_pos == SERIAL_RX_BUFFER_SIZE) {
				serial_rx_read_pos = 0;
			}
		}
	}
}

static void process_packet(unsigned char *data, unsigned int len) {
	commands_process_packet(data, len, comm_usb_send_packet);
}

static void send_packet_raw(unsigned char *buffer, unsigned int len) {
	CDC_Transmit_FS((uint8_t *)buffer, len);
}

void comm_usb_init(void) {
	
//	packet_init(send_packet_raw, process_packet, &packet_state);

//	send_mutex = osMutexNew(&send_mutex_attributes);
	
	// Threads
//	process_tp = osThreadNew(serial_process_thread, NULL, &processTask_attributes);
	usb_read_id = osThreadNew(serial_read_thread, NULL, &readTask_attributes);
}

void comm_usb_send_packet(unsigned char *data, unsigned int len) {
	osMutexAcquire(send_mutex,osWaitForever);
	packet_send_packet(data, len, &packet_state);
	osMutexRelease(send_mutex);
}

unsigned int comm_usb_get_write_timeout_cnt(void) {
	return write_timeout_cnt;
}