#ifndef   _MAIN_H
#define   _MAIN_H

#include "stdio.h"
#include "stm32f10x.h"
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
#include "altitude.h"
#include "bmp280.h"

void System_Init(void);
void Task_Schedule(void);

#endif

