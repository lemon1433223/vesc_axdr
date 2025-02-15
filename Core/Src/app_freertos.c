/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : app_freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "app_freertos.h"
#include "mcpwm_foc.h"
#include "usart_vofa.h"
#include "enc_mt6816.h"
#include "utils_math.h"
#include "timer.h"
#include "stdio.h"
#include "task.h"
#include "conf_general.h"
#include "mc_interface.h"
#include "terminal.h"
#include "commands.h"
#include "comm_usb.h"
#include "mempools.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
uint8_t pcWriteBuffer[512];
osThreadId_t osTaskManagerID;
const osThreadAttr_t osTaskManager_attributes = {
  .name = "osTaskManager",
  .priority = (osPriority_t) osPriorityNormal + 3,
  .stack_size = 128 * 4
};

osThreadId_t printTaskInfoID;
const osThreadAttr_t task_info_attributes = {
  .name = "printfTaskInfo",
  .priority = (osPriority_t) osPriorityNormal + 1,
  .stack_size = 128 * 4
};

osThreadId_t confgeneralTaskId;
const osThreadAttr_t confgeneralTask_attributes = {
  .name = "confgeneralTask",
  .priority = (osPriority_t) osPriorityNormal + 2,
  .stack_size = 256 * 4
};

uint16_t adc_input = 0;

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void printTaskInfo(osThreadId_t taskId);
void confgeneralTask(void *argument);
void managerTask(void *argument);

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* Hook prototypes */
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName);

/* USER CODE BEGIN 4 */
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName)
{
   /* Run time stack overflow checking is performed if
   configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2. This hook function is
   called if a stack overflow is detected. */
}
/* USER CODE END 4 */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
//	mempools_init();
	conf_general_init();
	mc_interface_init();
	osTaskManagerID  	= osThreadNew(managerTask, NULL, &osTaskManager_attributes);
	confgeneralTaskId = osThreadNew(confgeneralTask, NULL, &confgeneralTask_attributes);
	printTaskInfoID 	= osThreadNew(printTaskInfo, NULL, &task_info_attributes);
//	commands_init();
//	comm_usb_init();
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* init code for USB_Device */
  MX_USB_Device_Init();
  /* USER CODE BEGIN StartDefaultTask */


  /* Infinite loop */

//	osThreadFlagsWait(0x01,osFlagsWaitAny,osWaitForever);
  for(;;)
  {
//		osThreadFlagsWait(0x01,osFlagsWaitAny,osWaitForever);
		UTILS_LP_FAST(adc_input,ADC_Value[11],0.3f);
		
		if(m_motor_1.m_state == MC_STATE_RUNNING)
		{
			float duty = (4070.0f - adc_input) / 4070.0f;//* m_motor_1.m_conf->foc_encoder_ratio
			mcpwm_foc_set_duty(duty);
		}
//	float speed_set = (4070 - ADC_Value[11]) / 4070.0f * 8000.0f;
//	mcpwm_foc_set_pid_speed(speed_set);
//	timer_update((motor_all_state_t*)&m_motor_1,0.001f);
//	foc_run_pid_control_speed(1,0.001f,(motor_all_state_t*)&m_motor_1);
		
		float speed = RADPS2RPM_f(m_motor_1.m_pll_speed);
		
		vofa_send_data(0,TIM1->CCR1);
		vofa_send_data(1,TIM1->CCR2);
		vofa_send_data(2,TIM1->CCR3);
		vofa_send_data(3,TIM1->CNT);
		vofa_send_data(4,m_motor_1.m_motor_state.phase);
		vofa_send_data(5,m_motor_1.m_phase_now_encoder);
		vofa_send_data(6,m_motor_1.m_motor_state.id_target);
		vofa_send_data(7,m_motor_1.m_motor_state.id);
		vofa_send_data(8,m_motor_1.m_motor_state.iq_target);
		vofa_send_data(9,m_motor_1.m_motor_state.iq);
		vofa_send_data(10,m_motor_1.m_motor_state.vd);
		vofa_send_data(11,m_motor_1.m_motor_state.vq);
		vofa_send_data(12,speed);
//		vofa_send_data(13,m_motor_1.m_speed_pid_set_rpm);
//		vofa_send_data(14,m_motor_1.m_min_rpm_hyst_timer);
//		vofa_send_data(15,m_motor_1.m_phase_now_observer_override);
//		vofa_send_data(16,m_motor_1.m_phase_now_observer);
//		vofa_send_data(17,m_motor_1.m_motor_state.v_alpha);
//		vofa_send_data(18,m_motor_1.m_motor_state.v_beta);
//		vofa_send_data(19,m_motor_1.m_motor_state.va);
//		vofa_send_data(20,m_motor_1.m_motor_state.vb);
//		vofa_send_data(21,m_motor_1.m_motor_state.vc);		
//		vofa_send_data(22,m_motor_1.m_observer_state.x1);
//		vofa_send_data(23,m_motor_1.m_observer_state.x2);
//		vofa_send_data(24,m_motor_1.m_currents_adc[0]);
//		vofa_send_data(25,m_motor_1.m_currents_adc[1]);
//		vofa_send_data(26,m_motor_1.m_currents_adc[2]);
//		vofa_send_data(27,ADC_curr_norm_value[0]);
//		vofa_send_data(28,ADC_curr_norm_value[1]);
//		vofa_send_data(29,ADC_curr_norm_value[2]);
//		vofa_send_data(30,ADC_Value[5]);
//		vofa_send_data(31,ADC_Value[6]);
//		vofa_send_data(32,ADC_Value[7]);
//		vofa_send_data(33,m_motor_1.m_motor_state.i_alpha);
//		vofa_send_data(34,m_motor_1.m_motor_state.i_beta);
		
//		osKernelLock();
		vofa_sendframetail();
//		osKernelUnlock();
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void printTaskInfo(void *argument)
{
/*
	X:running
	B:blockde
	R:ready
	D:deleted
	S:supended	
*/
	for(;;){
		osThreadFlagsWait(0x01,osFlagsWaitAny,osWaitForever);
		vTaskList((char *)&pcWriteBuffer); 
		usb_printf("Name   State    Priority   Size   Order\n");
		usb_printf("%s",pcWriteBuffer);
		osDelay(10);
		usb_printf("total free heap size:%d\n",xPortGetFreeHeapSize());
		osDelay(10);
	}

}

void managerTask(void *argument)
{

//  MX_USB_Device_Init();

	
	for(;;){
		osThreadFlagsWait(0x0f,osFlagsWaitAny|osFlagsNoClear,osWaitForever);
		uint32_t flag = osThreadFlagsGet();
		osThreadFlagsClear(flag);
		if(flag == 0x01)
		{
			osThreadResume(defaultTaskHandle);					
		}else if(flag == 0x02)
		{
			osThreadSuspend(defaultTaskHandle);
		}
	}

}

void confgeneralTask(void *argument)
{
	for(;;)
	{
		osThreadFlagsWait(0x01,osFlagsWaitAny,osWaitForever);
		conf_general_detect_apply_all_foc(10.0f,true,false);
		osDelay(10);
	}
}
/* USER CODE END Application */

