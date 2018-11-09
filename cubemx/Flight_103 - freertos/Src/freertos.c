/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

/* USER CODE BEGIN Includes */     
#include "tim.h"
#include "usart.h"
#include "gpio.h"

#include "imu.h"
#include "mpu6050.h"
#include "sand_data.h"
#include "24l01.h"
#include "rc.h"
#include "control.h"
#include "param.h"
/* USER CODE END Includes */

/* Variables -----------------------------------------------------------------*/
osThreadId defaultTaskHandle;
osThreadId LEDTaskHandle;
osThreadId MPU_TaskHandle;
osThreadId NRF_TaskHandle;
osThreadId ControlTaskHandle;
osThreadId SandTaskHandle;
osThreadId Receive_taskHandle;
osTimerId IMU_timerHandle;
osMutexId USARTMutexHandle;
osSemaphoreId ReadIMUHandle;
osSemaphoreId USART_SamHandle;

/* USER CODE BEGIN Variables */

/* USER CODE END Variables */

/* Function prototypes -------------------------------------------------------*/
void StartDefaultTask(void const * argument);
void StartLEDTask(void const * argument);
void StartMPU_Task(void const * argument);
void StartNRF_Task(void const * argument);
void StartControlTask(void const * argument);
void StartSandTask(void const * argument);
void StartReceive_task(void const * argument);
void IMU_Calculation(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

/* Hook prototypes */

/* Init FreeRTOS */

void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
       
  /* USER CODE END Init */

  /* Create the mutex(es) */
  /* definition and creation of USARTMutex */
  osMutexDef(USARTMutex);
  USARTMutexHandle = osMutexCreate(osMutex(USARTMutex));

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* definition and creation of ReadIMU */
  osSemaphoreDef(ReadIMU);
  ReadIMUHandle = osSemaphoreCreate(osSemaphore(ReadIMU), 1);

  /* definition and creation of USART_Sam */
  osSemaphoreDef(USART_Sam);
  USART_SamHandle = osSemaphoreCreate(osSemaphore(USART_Sam), 1);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* Create the timer(s) */
  /* definition and creation of IMU_timer */
  osTimerDef(IMU_timer, IMU_Calculation);
  IMU_timerHandle = osTimerCreate(osTimer(IMU_timer), osTimerPeriodic, NULL);

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityIdle, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of LEDTask */
  osThreadDef(LEDTask, StartLEDTask, osPriorityLow, 0, 128);
  LEDTaskHandle = osThreadCreate(osThread(LEDTask), NULL);

  /* definition and creation of MPU_Task */
  osThreadDef(MPU_Task, StartMPU_Task, osPriorityAboveNormal, 0, 128);
  MPU_TaskHandle = osThreadCreate(osThread(MPU_Task), NULL);

  /* definition and creation of NRF_Task */
  osThreadDef(NRF_Task, StartNRF_Task, osPriorityNormal, 0, 128);
  NRF_TaskHandle = osThreadCreate(osThread(NRF_Task), NULL);

  /* definition and creation of ControlTask */
  osThreadDef(ControlTask, StartControlTask, osPriorityHigh, 0, 128);
  ControlTaskHandle = osThreadCreate(osThread(ControlTask), NULL);

  /* definition and creation of SandTask */
  osThreadDef(SandTask, StartSandTask, osPriorityBelowNormal, 0, 128);
  SandTaskHandle = osThreadCreate(osThread(SandTask), NULL);

  /* definition and creation of Receive_task */
  osThreadDef(Receive_task, StartReceive_task, osPriorityAboveNormal, 0, 256);
  Receive_taskHandle = osThreadCreate(osThread(Receive_task), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  osTimerStart(IMU_timerHandle,5);
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
}

/* StartDefaultTask function */
void StartDefaultTask(void const * argument)
{

  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
//    osThreadResume(SandTaskHandle);
    osDelay(100);
  }
  /* USER CODE END StartDefaultTask */
}

/* StartLEDTask function */
void StartLEDTask(void const * argument)
{
  /* USER CODE BEGIN StartLEDTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(100);
  }
  /* USER CODE END StartLEDTask */
}

/* StartMPU_Task function */
void StartMPU_Task(void const * argument)
{
  /* USER CODE BEGIN StartMPU_Task */
  Read_Acc_Gyro_offest();
  /* Infinite loop */
  for(;;)
  {
    READ_MPU6050();
    Prepare_6050_Data();
    
    HAL_GPIO_TogglePin(LED0_GPIO_Port,LED0_Pin);
    osDelay(1);
  }
  /* USER CODE END StartMPU_Task */
}

/* StartNRF_Task function */
void StartNRF_Task(void const * argument)
{
  /* USER CODE BEGIN StartNRF_Task */
  extern int IRQ_timeout;
  extern uint16_t BAT_Value;
  extern uint8_t Rx_buff[30];
  uint8_t sta = 0;
  uint16_t bat_cnt = 0;
  /* Infinite loop */
  for(;;)
  {
    bat_cnt ++;
    IRQ_timeout++;
    
    
    Deblocking();
    if(bat_cnt >1000)
    {
      bat_cnt = 0;
      BAT_Value = Get_Adc(ADC_CHANNEL_3);
      NRF_sand_BAT(BAT_Value);
    }
    
    if(IRQ_timeout >=1000)
    {
      IRQ_timeout = 0 ;
      sta=NRF24L01_Read_Reg(STATUS);  //读取状态寄存器的值    	 
        NRF24L01_Write_Reg(NRF_WRITE_REG+STATUS,sta); //清除TX_DS或MAX_RT中断标志
        NRF24L01_Write_Reg(FLUSH_RX,0xff);//清除RX FIFO寄存器 
    }
    osDelay(1);
  }
  /* USER CODE END StartNRF_Task */
}

/* StartControlTask function */
void StartControlTask(void const * argument)
{
  /* USER CODE BEGIN StartControlTask */
  Read_PID_shell();
  Read_PID_core();
  /* Infinite loop */
  for(;;)
  {
    CONTROL(angle.roll,angle.pitch,angle.yaw);
    osDelay(5);
  }
  /* USER CODE END StartControlTask */
}

/* StartSandTask function */
void StartSandTask(void const * argument)
{
  /* USER CODE BEGIN StartSandTask */
  uint16_t sand_cnt= 0;
  /* Infinite loop */
  for(;;)
  {
    sand_cnt++;
    if(osMutexWait(USARTMutexHandle,0x10) == osOK)
    {
      if(sand_cnt==1)
        sand_IMU_data();
      else if(sand_cnt==2)
        sand_ACC_GYRO_data();
      else if(sand_cnt==3)
        sand_RC_data();
      else if(sand_cnt==4)
        sand_Motor_data();
      else 
        sand_cnt = 0;
    }
    
    osMutexRelease(USARTMutexHandle);
    
    osDelay(10);
  }
  /* USER CODE END StartSandTask */
}

/* StartReceive_task function */
void StartReceive_task(void const * argument)
{
  /* USER CODE BEGIN StartReceive_task */
  extern uint8_t RxBuffer[50],_data_cnt;
  /* Infinite loop */
  for(;;)
  {
    if(osSemaphoreWait(USART_SamHandle,10) == osOK)
      Data_Receive_Anl(RxBuffer,_data_cnt+5);
    osDelay(100);
  }
  /* USER CODE END StartReceive_task */
}

/* IMU_Calculation function */
void IMU_Calculation(void const * argument)
{
  /* USER CODE BEGIN IMU_Calculation */
  Get_Attitude();
  /* USER CODE END IMU_Calculation */
}

/* USER CODE BEGIN Application */
     
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
