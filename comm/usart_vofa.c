#include "usart_vofa.h"
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include "usbd_cdc_if.h"
#include "usart.h"

extern DMA_HandleTypeDef hdma_usart3_tx;
extern DMA_HandleTypeDef hdma_usart3_rx;

extern uint8_t UserRxBufferFS[APP_RX_DATA_SIZE];
extern uint8_t UserTxBufferFS[APP_TX_DATA_SIZE];

uint8_t RxBuffer[RXBUFFERSIZE];  
uint8_t TxBuffer[TXBUFFERSIZE];
uint16_t USART_RX_LEN = 0;//串口接收数据长度
uint16_t USART_TX_CNT;//串口待发送字节数
void vofa_transmit(uint8_t* buf, uint16_t len);
void vofa_send_data(uint8_t num, float data);
void vofa_sendframetail(void);

float hex_to_float(uint8_t *hex) {
    // 将小端在前的十六进制数据直接解释为浮点数

	union {
		uint8_t hex[4];
		float f;
	} data;
	
//	data.hex[0] = *(hex +3);
//	data.hex[1] = *(hex +2);
//	data.hex[2] = *(hex +1);
//	data.hex[3] = *hex;
	
	data.hex[0] = *hex;
	data.hex[1] = *(hex +1);
	data.hex[2] = *(hex +2);
	data.hex[3] = *(hex +3);
	return data.f;
}

int fputc(int ch, FILE *f)
{
	HAL_UART_Transmit(&huart3, (uint8_t *)&ch, 1, 0x02);
 return ch;
}

void vofa_receive()
{

	uint32_t idle_flag = 0;   
	uint32_t rx_count;
	
	idle_flag =__HAL_UART_GET_FLAG(&huart3,UART_FLAG_IDLE); //获取IDLE标志位
	if((idle_flag != RESET))//idle标志被置位
	{
		__HAL_UART_CLEAR_IDLEFLAG(&huart3);//清除标志位
		HAL_UART_DMAStop(&huart3); //  停止DMA传输
		rx_count  =  __HAL_DMA_GET_COUNTER(&hdma_usart3_rx);// 获取DMA中未传输的数据个数   
		USART_RX_LEN =  RXBUFFERSIZE - rx_count; //总计数减去未传输的数据个数，得到已经接收的数据个数
		vofa_command();//处理接收到的数据
	}

}

/**
***********************************************************************
* @brief:      vofa_transmit(uint8_t* buf, uint16_t len)
* @param:		   void
* @retval:     void
* @details:    修改通信工具，USART或者USB
***********************************************************************
**/
void vofa_transmit(uint8_t* buf, uint16_t len)
{
//	HAL_UART_Transmit_DMA(&huart3, (uint8_t *)buf, len);
	//HAL_UART_Transmit(&huart3, (uint8_t *)buf, len,10);
	CDC_Transmit_FS((uint8_t *)buf, len);
}
/**
***********************************************************************
* @brief:      vofa_send_data(float data)
* @param[in]:  num: 数据编号 data: 数据 
* @retval:     void
* @details:    将浮点数据拆分成单字节
***********************************************************************
**/
void vofa_send_data(uint8_t num, float data) 
{
	TxBuffer[USART_TX_CNT++] = byte0(data);
	TxBuffer[USART_TX_CNT++] = byte1(data);
	TxBuffer[USART_TX_CNT++] = byte2(data);
	TxBuffer[USART_TX_CNT++] = byte3(data);
}
/**
***********************************************************************
* @brief      vofa_sendframetail(void)
* @param      NULL 
* @retval     void
* @details:   给数据包发送帧尾
***********************************************************************
**/
void vofa_sendframetail(void) 
{
	TxBuffer[USART_TX_CNT++] = 0x00;
	TxBuffer[USART_TX_CNT++] = 0x00;
	TxBuffer[USART_TX_CNT++] = 0x80;
	TxBuffer[USART_TX_CNT++] = 0x7f;
	
	/* 将数据和帧尾打包发送 */
	vofa_transmit((uint8_t *)TxBuffer, USART_TX_CNT);
	USART_TX_CNT = 0;// 每次发送完帧尾都需要清零
}

//void USB_Status_Init(void)
//{
//    GPIO_InitTypeDef GPIO_InitStruct = {0};
// 
//    /* GPIO Ports Clock Enable */
//    __HAL_RCC_GPIOA_CLK_ENABLE();
// 
//    /*Configure GPIO pin Output Level */
//    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11 | GPIO_PIN_12, GPIO_PIN_RESET);
// 
//    /*Configure GPIO pin : W25Q256_CS_Pin */
//    GPIO_InitStruct.Pin = GPIO_PIN_11 | GPIO_PIN_12;
//    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
//    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
// 
//    //假如不行的话，下面的延时加长即可。
//    HAL_Delay(10);
//}
void vofa_command()
{

	if(USART_RX_LEN!=0)
	 {
		if (RxBuffer[USART_RX_LEN - 1] == 0x0A) //判断帧尾
		{
				switch (RxBuffer[0]) {
					case 0x4F:
						if (RxBuffer[USART_RX_LEN - 2] == 0x30) {
							//Motor_stop();
						} else {
							//Motor_start();
						}
						break;
					case 0x73://设置转速
						if (USART_RX_LEN == 6) {
//							foc.spd_ref = hex_to_float(&RxBuffer[1]);
						}
						break;
					case 0x74://设置滑膜参数l
//						observer_smo.smo_l = hex_to_float(&RxBuffer[1]);
						break;
					case 0x75://设置滑膜参数m
//						observer_smo.smo_m = hex_to_float(&RxBuffer[1]);
						break;
					default:
							// Handle default case if needed
							break;
				}
		}
		
		memset(RxBuffer,0x00,USART_RX_LEN);
		USART_RX_LEN = 0;
		//HAL_UART_Receive_DMA(&huart3,RxBuffer,RXBUFFERSIZE);//重新打开DMA接收
 }
}



 
 
uint16_t usb_printf(const char *format, ...)
{
    va_list args;
    uint32_t length;
 
    va_start(args, format);
    length = vsnprintf((char *)UserTxBufferFS, APP_TX_DATA_SIZE, (char *)format, args);
    va_end(args);
    CDC_Transmit_FS(UserTxBufferFS, length);
	return length;
}
