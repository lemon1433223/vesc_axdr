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
uint16_t USART_RX_LEN = 0;//���ڽ������ݳ���
uint16_t USART_TX_CNT;//���ڴ������ֽ���
void vofa_transmit(uint8_t* buf, uint16_t len);
void vofa_send_data(uint8_t num, float data);
void vofa_sendframetail(void);

float hex_to_float(uint8_t *hex) {
    // ��С����ǰ��ʮ����������ֱ�ӽ���Ϊ������

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
	
	idle_flag =__HAL_UART_GET_FLAG(&huart3,UART_FLAG_IDLE); //��ȡIDLE��־λ
	if((idle_flag != RESET))//idle��־����λ
	{
		__HAL_UART_CLEAR_IDLEFLAG(&huart3);//�����־λ
		HAL_UART_DMAStop(&huart3); //  ֹͣDMA����
		rx_count  =  __HAL_DMA_GET_COUNTER(&hdma_usart3_rx);// ��ȡDMA��δ��������ݸ���   
		USART_RX_LEN =  RXBUFFERSIZE - rx_count; //�ܼ�����ȥδ��������ݸ������õ��Ѿ����յ����ݸ���
		vofa_command();//������յ�������
	}

}

/**
***********************************************************************
* @brief:      vofa_transmit(uint8_t* buf, uint16_t len)
* @param:		   void
* @retval:     void
* @details:    �޸�ͨ�Ź��ߣ�USART����USB
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
* @param[in]:  num: ���ݱ�� data: ���� 
* @retval:     void
* @details:    ���������ݲ�ֳɵ��ֽ�
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
* @details:   �����ݰ�����֡β
***********************************************************************
**/
void vofa_sendframetail(void) 
{
	TxBuffer[USART_TX_CNT++] = 0x00;
	TxBuffer[USART_TX_CNT++] = 0x00;
	TxBuffer[USART_TX_CNT++] = 0x80;
	TxBuffer[USART_TX_CNT++] = 0x7f;
	
	/* �����ݺ�֡β������� */
	vofa_transmit((uint8_t *)TxBuffer, USART_TX_CNT);
	USART_TX_CNT = 0;// ÿ�η�����֡β����Ҫ����
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
//    //���粻�еĻ����������ʱ�ӳ����ɡ�
//    HAL_Delay(10);
//}
void vofa_command()
{

	if(USART_RX_LEN!=0)
	 {
		if (RxBuffer[USART_RX_LEN - 1] == 0x0A) //�ж�֡β
		{
				switch (RxBuffer[0]) {
					case 0x4F:
						if (RxBuffer[USART_RX_LEN - 2] == 0x30) {
							//Motor_stop();
						} else {
							//Motor_start();
						}
						break;
					case 0x73://����ת��
						if (USART_RX_LEN == 6) {
//							foc.spd_ref = hex_to_float(&RxBuffer[1]);
						}
						break;
					case 0x74://���û�Ĥ����l
//						observer_smo.smo_l = hex_to_float(&RxBuffer[1]);
						break;
					case 0x75://���û�Ĥ����m
//						observer_smo.smo_m = hex_to_float(&RxBuffer[1]);
						break;
					default:
							// Handle default case if needed
							break;
				}
		}
		
		memset(RxBuffer,0x00,USART_RX_LEN);
		USART_RX_LEN = 0;
		//HAL_UART_Receive_DMA(&huart3,RxBuffer,RXBUFFERSIZE);//���´�DMA����
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
