移植自VESC的代码,用来适配AxDr-L驱动板。\
AxDr-L驱动板的资料可以看 [立创开源广场](https://oshwhub.com/lylssy/foc_driver)\
使用CUBMX生成的工程，方便底层与FOC分离，后续移植会更方便。\
将ChibisOS改为个人更熟悉的FreeRTOS。\
标准库改成HAL库。\
目前还没有移植通信协议，使用按键控制。\
KEY2-打印当前RTOS任务状态\
KEY3-识别电机参数，写入flash\
KEY4-运行电机\
电位器-调节占空比（控制电机速度）
