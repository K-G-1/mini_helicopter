
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx.h"


#include "stdio.h"
#include "nvic.h"
#include "structconfig.h"
#include "delay.h"
#include "led.h"
#include "usart.h"
#include "iic.h"
#include "spi.h"
#include "exit.h"
#include "si24r1.h"
#include "tim.h"
#include "motor.h"
#include "mpu6050.h"
#include "imu.h"
#include "fbm320.h"
#include "pid.h"
#include "control.h"
#include "flash.h"
#include "paramsave.h"
#include "ANO_DT.h"
#include "power.h"
#include "remotedata.h"
#include "bmp280.h"
#include "altitude.h"


void System_Init(void);
void Task_Schedule(void);
#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
