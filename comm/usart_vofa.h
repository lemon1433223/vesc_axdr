#ifndef __USART_VOFA_H
#define __USART_VOFA_H
#include "usart.h"
#define byte0(dw_temp)     (*(char*)(&dw_temp))
#define byte1(dw_temp)     (*((char*)(&dw_temp) + 1))
#define byte2(dw_temp)     (*((char*)(&dw_temp) + 2))
#define byte3(dw_temp)     (*((char*)(&dw_temp) + 3))
#define RXBUFFERSIZE  256   
#define TXBUFFERSIZE 1024

#define BIG_LITTLE_SWAP32(x)        ( (((*(long int *)&x) & 0xff000000) >> 24) | \
                                      (((*(long int *)&x) & 0x00ff0000) >> 8) | \
                                      (((*(long int *)&x) & 0x0000ff00) << 8) | \
                                      (((*(long int *)&x) & 0x000000ff) << 24) )

extern uint8_t RxBuffer[RXBUFFERSIZE];  
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void vofa_demo(void);
void vofa_start(void);
void vofa_sendframetail(void);
void vofa_send_data(uint8_t num, float data);
void vofa_receive(void);
void vofa_command(void);
uint16_t usb_printf(const char *format, ...);																			
#endif
