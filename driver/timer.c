#include "timer.h"
#include "hw.h"
#include "stm32g4xx_hal.h"
#include "cmsis_os.h"

/*
	定义用于硬件定时器的TIM， 可以使 TIM2 - TIM5
*/
//#define USE_TIM2
//#define USE_TIM3
//#define USE_TIM4
#define USE_TIM5

#ifdef USE_TIM2
	#define TIM_HARD					TIM2
	#define	RCC_TIM_HARD_CLK_ENABLE()	__HAL_RCC_TIM2_CLK_ENABLE()
	#define TIM_HARD_IRQn				TIM2_IRQn
	#define TIM_HARD_IRQHandler			TIM2_IRQHandler
#endif

#ifdef USE_TIM3
	#define TIM_HARD					TIM3
	#define	RCC_TIM_HARD_CLK_ENABLE()	__HAL_RCC_TIM3_CLK_ENABLE()	
	#define TIM_HARD_IRQn				TIM3_IRQn
	#define TIM_HARD_IRQHandler			TIM3_IRQHandler
#endif

#ifdef USE_TIM4
	#define TIM_HARD					TIM4
	#define	RCC_TIM_HARD_CLK_ENABLE()	__HAL_RCC_TIM4_CLK_ENABLE()
	#define TIM_HARD_IRQn				TIM4_IRQn
	#define TIM_HARD_IRQHandler			TIM4_IRQHandler
#endif

#ifdef USE_TIM5
	#define TIM_HARD					TIM5
	#define	RCC_TIM_HARD_CLK_ENABLE()	__HAL_RCC_TIM5_CLK_ENABLE()
	#define TIM_HARD_IRQn				TIM5_IRQn
	#define TIM_HARD_IRQHandler			TIM5_IRQHandler
#endif

void  AppTask1CallBack    (void);
void  AppTask2CallBack    (void);
void  AppTask3CallBack    (void);
void  AppTask4CallBack		(void);

typedef struct
{
	uint8_t CC;                /* 定时器通道，1-4 */
	uint32_t uiTimeOut;        /* 延迟时间设置 */
	void (*_pCallBack)(void);  /* 定时器中断回调 */
	char *pName;               /* 信号量名 */
	osSemaphoreId_t SemaphoreId;    /* 信号量句柄 */
}DELAYUS_T;

DELAYUS_T  g_tDelayUS[] = 
{
	{1, 0, AppTask1CallBack, "AppTask1",0},
	{2, 0, AppTask2CallBack, "AppTask2",0},
	{3, 0, AppTask3CallBack, "AppTask3",0},
	{4, 0, AppTask4CallBack, "AppTask4",0},	
};

const osSemaphoreAttr_t Semaphore1_attributes = {
  .name = "Semaphore1"
};
const osSemaphoreAttr_t Semaphore2_attributes = {
  .name = "Semaphore2"
};
const osSemaphoreAttr_t Semaphore3_attributes = {
  .name = "Semaphore3"
};
const osSemaphoreAttr_t Semaphore4_attributes = {
  .name = "Semaphore4"
};

void timer_init(void) {
	uint16_t PrescalerValue = (uint16_t) ((SYSTEM_CORE_CLOCK) / TIMER_HZ) - 1;

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
	TIM_HandleTypeDef TimHandle = {0};
	
	RCC_TIM_HARD_CLK_ENABLE();
	
  TimHandle.Instance = TIM_HARD;
  TimHandle.Init.Prescaler = PrescalerValue;
  TimHandle.Init.CounterMode = TIM_COUNTERMODE_UP;
  TimHandle.Init.Period = 0xFFFFFFFF;
  TimHandle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  TimHandle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	HAL_TIM_Base_Init(&TimHandle);

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	HAL_TIM_ConfigClockSource(&TimHandle, &sClockSourceConfig);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&TimHandle, &sMasterConfig);

		/* 配置定时器中断，给CC捕获比较中断使用 */
	{
		HAL_NVIC_SetPriority(TIM_HARD_IRQn, 10, 0);
		HAL_NVIC_EnableIRQ(TIM_HARD_IRQn);	
	}
	
	HAL_TIM_Base_Start(&TimHandle);

	g_tDelayUS[0].SemaphoreId = osSemaphoreNew(1,0,&Semaphore1_attributes);
	g_tDelayUS[1].SemaphoreId = osSemaphoreNew(1,0,&Semaphore2_attributes);
	g_tDelayUS[2].SemaphoreId = osSemaphoreNew(1,0,&Semaphore3_attributes);
	g_tDelayUS[3].SemaphoreId = osSemaphoreNew(1,0,&Semaphore4_attributes);
}

uint32_t timer_time_now(void) {
	return TIM5->CNT;
}

float timer_seconds_elapsed_since(uint32_t time) {
	uint32_t diff = TIM5->CNT - time;
	return (float)diff / (float)TIMER_HZ;
}

/**
 * Blocking sleep based on timer.
 *
 * @param seconds
 * Seconds to sleep.
 */
void timer_sleep(float seconds) {
	uint32_t start_t = TIM5->CNT;

	for (;;) {
		if (timer_seconds_elapsed_since(start_t) >= seconds) {
			return;
		}
	}
}


/* 保存 TIM定时中断到后执行的回调函数指针 */
static void (*s_TIM_CallBack1)(void);
static void (*s_TIM_CallBack2)(void);
static void (*s_TIM_CallBack3)(void);
static void (*s_TIM_CallBack4)(void);


/*
*********************************************************************************************************
*	函 数 名: bsp_StartHardTimer
*	功能说明: 使用TIM2-5做单次定时器使用, 定时时间到后执行回调函数。可以同时启动4个定时器通道，互不干扰。
*             定时精度正负1us （主要耗费在调用本函数的执行时间）
*			  TIM2和TIM5 是32位定时器。定时范围很大
*			  TIM3和TIM4 是16位定时器。
*	形    参: _CC : 捕获比较通道几，1，2，3, 4
*             _uiTimeOut : 超时时间, 单位 1us. 对于16位定时器，最大 65.5ms; 对于32位定时器，最大 4294秒
*             _pCallBack : 定时时间到后，被执行的函数
*	返 回 值: 无
*********************************************************************************************************
*/
void bsp_StartHardTimer(uint8_t _CC, uint32_t _uiTimeOut, void * _pCallBack)
{
    uint32_t cnt_now;
    uint32_t cnt_tar;
		TIM_TypeDef* TIMx = TIM_HARD;
	
    /* H743速度较快，无需补偿延迟，实测精度正负1us */
    
    cnt_now = TIMx->CNT; 
    cnt_tar = cnt_now + _uiTimeOut;			/* 计算捕获的计数器值 */
    if (_CC == 1)
    {
        s_TIM_CallBack1 = (void (*)(void))_pCallBack;

		TIMx->CCR1 = cnt_tar; 			    /* 设置捕获比较计数器CC1 */
        TIMx->SR = (uint16_t)~TIM_IT_CC1;   /* 清除CC1中断标志 */
		TIMx->DIER |= TIM_IT_CC1;			/* 使能CC1中断 */
	}
    else if (_CC == 2)
    {
		s_TIM_CallBack2 = (void (*)(void))_pCallBack;

		TIMx->CCR2 = cnt_tar;				/* 设置捕获比较计数器CC2 */
        TIMx->SR = (uint16_t)~TIM_IT_CC2;	/* 清除CC2中断标志 */
		TIMx->DIER |= TIM_IT_CC2;			/* 使能CC2中断 */
    }
    else if (_CC == 3)
    {
        s_TIM_CallBack3 = (void (*)(void))_pCallBack;

		TIMx->CCR3 = cnt_tar;				/* 设置捕获比较计数器CC3 */
        TIMx->SR = (uint16_t)~TIM_IT_CC3;	/* 清除CC3中断标志 */
		TIMx->DIER |= TIM_IT_CC3;			/* 使能CC3中断 */
    }
    else if (_CC == 4)
    {
        s_TIM_CallBack4 = (void (*)(void))_pCallBack;

		TIMx->CCR4 = cnt_tar;				/* 设置捕获比较计数器CC4 */
        TIMx->SR = (uint16_t)~TIM_IT_CC4;	/* 清除CC4中断标志 */
		TIMx->DIER |= TIM_IT_CC4;			/* 使能CC4中断 */
    }
	else
    {
        return;
    }
}

/*
*********************************************************************************************************
*	函 数 名: TIMx_IRQHandler
*	功能说明: TIM 中断服务程序
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/
void TIM_HARD_IRQHandler(void)
{
	uint16_t itstatus = 0x0, itenable = 0x0;
	TIM_TypeDef* TIMx = TIM_HARD;
	
    
  	itstatus = TIMx->SR & TIM_IT_CC1;
	itenable = TIMx->DIER & TIM_IT_CC1;
    
	if ((itstatus != (uint16_t)RESET) && (itenable != (uint16_t)RESET))
	{
		TIMx->SR = (uint16_t)~TIM_IT_CC1;
		TIMx->DIER &= (uint16_t)~TIM_IT_CC1;		/* 禁能CC1中断 */	

        /* 先关闭中断，再执行回调函数。因为回调函数可能需要重启定时器 */    
		s_TIM_CallBack1();
    }

	itstatus = TIMx->SR & TIM_IT_CC2;
	itenable = TIMx->DIER & TIM_IT_CC2;
	if ((itstatus != (uint16_t)RESET) && (itenable != (uint16_t)RESET))
	{
		TIMx->SR = (uint16_t)~TIM_IT_CC2;
		TIMx->DIER &= (uint16_t)~TIM_IT_CC2;		/* 禁能CC2中断 */	

        /* 先关闭中断，再执行回调函数。因为回调函数可能需要重启定时器 */
        s_TIM_CallBack2();
    }

	itstatus = TIMx->SR & TIM_IT_CC3;
	itenable = TIMx->DIER & TIM_IT_CC3;
	if ((itstatus != (uint16_t)RESET) && (itenable != (uint16_t)RESET))
	{
		TIMx->SR = (uint16_t)~TIM_IT_CC3;
		TIMx->DIER &= (uint16_t)~TIM_IT_CC3;		/* 禁能CC2中断 */	

        /* 先关闭中断，再执行回调函数。因为回调函数可能需要重启定时器 */
        s_TIM_CallBack3();
    }

	itstatus = TIMx->SR & TIM_IT_CC4;
	itenable = TIMx->DIER & TIM_IT_CC4;
	if ((itstatus != (uint16_t)RESET) && (itenable != (uint16_t)RESET))
	{
		TIMx->SR = (uint16_t)~TIM_IT_CC4;
		TIMx->DIER &= (uint16_t)~TIM_IT_CC4;		/* 禁能CC4中断 */	

        /* 先关闭中断，再执行回调函数。因为回调函数可能需要重启定时器 */
        s_TIM_CallBack4();
    }	
}

/*
*********************************************************************************************************
*	函 数 名: osDeleyUS
*	功能说明: 微秒分辨率延迟实现
*	形    参：_uiTimeOut 延迟时间，单位us
*             _CC 使用的通道，1，2，3, 4
*	返 回 值: 无
*********************************************************************************************************
*/

void  AppTask1CallBack    (void){osSemaphoreRelease(g_tDelayUS[0].SemaphoreId);}
void  AppTask2CallBack    (void){osSemaphoreRelease(g_tDelayUS[1].SemaphoreId);}
void  AppTask3CallBack    (void){osSemaphoreRelease(g_tDelayUS[2].SemaphoreId);}
void  AppTask4CallBack		(void){osSemaphoreRelease(g_tDelayUS[3].SemaphoreId);}

void osDelayUs(uint32_t _uiTimeOut, delay_us_cc_t _CC)
{
	bsp_StartHardTimer(g_tDelayUS[_CC-1].CC, _uiTimeOut, g_tDelayUS[_CC-1]._pCallBack);
	osSemaphoreAcquire(g_tDelayUS[_CC-1].SemaphoreId,osWaitForever);
}
