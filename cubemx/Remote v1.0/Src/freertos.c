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
#include "adc.h"
#include "oled.h"
#include "usart.h"
#include "24l01.h"
#include "gpio.h"
#include "stmflash.h"
/* USER CODE END Includes */

/* Variables -----------------------------------------------------------------*/
osThreadId defaultTaskHandle;
osThreadId myTask_ADCHandle;
osThreadId myTask_NrfHandle;
osThreadId myTask_LEDHandle;
osThreadId myTask_OLEDHandle;
osThreadId myTask_KeyHandle;
osThreadId Param_TaskHandle;
osSemaphoreId NRF_statusHandle;
osSemaphoreId Data_saveHandle;

/* USER CODE BEGIN Variables */
osMailQId ADC_ValueHandle;
osMailQId Nrf_TX_BuffHandle;
osMailQId RC_Offest_buffHandle;

extern uint16_t RC_ADC_Buff[4];
int16_t RC_offest[4];
extern  uint8_t Tx_buff[30] ;
extern uint8_t key_status;
/* USER CODE END Variables */

/* Function prototypes -------------------------------------------------------*/
void StartDefaultTask(void const * argument);
void StartTask_ADC(void const * argument);
void StartTask_NRF(void const * argument);
void StartTask_LED(void const * argument);
void StartTask_OLED(void const * argument);
void StartTask_KEY(void const * argument);
void Start_Param_Task(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

/* Hook prototypes */

/* Init FreeRTOS */

void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
       
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* definition and creation of NRF_status */
  osSemaphoreDef(NRF_status);
  NRF_statusHandle = osSemaphoreCreate(osSemaphore(NRF_status), 1);

  /* definition and creation of Data_save */
  osSemaphoreDef(Data_save);
  Data_saveHandle = osSemaphoreCreate(osSemaphore(Data_save), 1);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of myTask_ADC */
  osThreadDef(myTask_ADC, StartTask_ADC, osPriorityHigh, 0, 128);
  myTask_ADCHandle = osThreadCreate(osThread(myTask_ADC), NULL);

  /* definition and creation of myTask_Nrf */
  osThreadDef(myTask_Nrf, StartTask_NRF, osPriorityNormal, 0, 128);
  myTask_NrfHandle = osThreadCreate(osThread(myTask_Nrf), NULL);

  /* definition and creation of myTask_LED */
  osThreadDef(myTask_LED, StartTask_LED, osPriorityLow, 0, 128);
  myTask_LEDHandle = osThreadCreate(osThread(myTask_LED), NULL);

  /* definition and creation of myTask_OLED */
  osThreadDef(myTask_OLED, StartTask_OLED, osPriorityLow, 0, 128);
  myTask_OLEDHandle = osThreadCreate(osThread(myTask_OLED), NULL);

  /* definition and creation of myTask_Key */
  osThreadDef(myTask_Key, StartTask_KEY, osPriorityNormal, 0, 128);
  myTask_KeyHandle = osThreadCreate(osThread(myTask_Key), NULL);

  /* definition and creation of Param_Task */
  osThreadDef(Param_Task, Start_Param_Task, osPriorityLow, 0, 128);
  Param_TaskHandle = osThreadCreate(osThread(Param_Task), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* definition and creation of ADC_Value */
  /* what about the sizeof here??? cd native code */
  osMailQDef(ADC_Value, 4, uint16_t);
  ADC_ValueHandle = osMailCreate(osMailQ(ADC_Value), NULL);

  /* definition and creation of Nrf_TX_Buff */
  /* what about the sizeof here??? cd native code */
  osMailQDef(Nrf_TX_Buff, 30, uint16_t);
  Nrf_TX_BuffHandle = osMailCreate(osMailQ(Nrf_TX_Buff), NULL);

  /* definition and creation of RC_Offest_buff */
  /* what about the sizeof here??? cd native code */
  osMailQDef(RC_Offest_buff, 4, int16_t);
  RC_Offest_buffHandle = osMailCreate(osMailQ(RC_Offest_buff), NULL);
  /* USER CODE END RTOS_QUEUES */
}

/* StartDefaultTask function */
void StartDefaultTask(void const * argument)
{

  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* StartTask_ADC function */
void StartTask_ADC(void const * argument)
{
  /* USER CODE BEGIN StartTask_ADC */
  uint8_t i = 0;
  /* Infinite loop */
  for(;;)
  {
    Get_Adc_Average(10);
    for (i = 0;i<4 ;i++)
      RC_ADC_Buff[i] += RC_offest[i];
    osMailPut(ADC_ValueHandle,RC_ADC_Buff);
    osMailPut(ADC_ValueHandle,Nrf_TX_BuffHandle);
    osDelay(10);
  }
  /* USER CODE END StartTask_ADC */
}

/* StartTask_NRF function */
void StartTask_NRF(void const * argument)
{
  /* USER CODE BEGIN StartTask_NRF */
  osEvent TX_mail;
  
  NRF24L01_Init();
  while(NRF24L01_Check() != 0)
  { 
    HAL_GPIO_WritePin(LED1_GPIO_Port,LED2_Pin,GPIO_PIN_RESET);
    osDelay(100);
  }
  NRF24L01_TX_Mode();
  HAL_GPIO_WritePin(LED2_GPIO_Port,LED2_Pin,GPIO_PIN_SET);
  /* Infinite loop */
  for(;;)
  {
    nrf_sand_rc();
    
    if(NRF24L01_TxPacket(Tx_buff) == TX_OK)
    {
      osSemaphoreRelease(NRF_statusHandle);
    }
    
    
    osDelay(100);
  }
  /* USER CODE END StartTask_NRF */
}

/* StartTask_LED function */
void StartTask_LED(void const * argument)
{
  /* USER CODE BEGIN StartTask_LED */
  /* Infinite loop */
  for(;;)
  {
    if(osSemaphoreWait(NRF_statusHandle,0x10) == osOK)
    {
      osSemaphoreRelease(NRF_statusHandle);
      HAL_GPIO_TogglePin(LED2_GPIO_Port,LED2_Pin);
    }
    
    
    HAL_GPIO_TogglePin(LED1_GPIO_Port,LED1_Pin);
    osDelay(1);
  }
  /* USER CODE END StartTask_LED */
}

/* StartTask_OLED function */
void StartTask_OLED(void const * argument)
{
  /* USER CODE BEGIN StartTask_OLED */
  osDelay(1000);
  osEvent display_mail;
  /* Infinite loop */
  for(;;)
  {
    display_mail = osMailGet(ADC_ValueHandle,10);
    if(display_mail.status == osEventMail)
      oled_show_RC_data(display_mail.value.p);
    
    display_mail = osMailGet(RC_Offest_buffHandle,10);
    if(display_mail.status == osEventMail)
      oled_show_offest_data(display_mail.value.p);
    
    if(osSemaphoreWait(NRF_statusHandle,0xff) == osOK)
    {
      OLED_P6x8Str(72,0,"connected");
    }
    else 
    {
      OLED_P6x8Str(72,0," vanished");
    }
    change_offest(key_status);

    osDelay(1);
  }
  /* USER CODE END StartTask_OLED */
}

/* StartTask_KEY function */
void StartTask_KEY(void const * argument)
{
  /* USER CODE BEGIN StartTask_KEY */
  uint8_t i = 0;
  uint16_t err;

  /* Infinite loop */
  for(;;)
  {
    KEY_Scan(1);
    osMailPut(RC_Offest_buffHandle,RC_offest);
    osDelay(100);
  }
  /* USER CODE END StartTask_KEY */
}

/* Start_Param_Task function */
void Start_Param_Task(void const * argument)
{
  /* USER CODE BEGIN Start_Param_Task */
  STMFLASH_Read(0x0800FC00,(uint16_t*)RC_offest,4);
  /* Infinite loop */
  for(;;)
  {
    if(osSemaphoreWait(Data_saveHandle,10) == osOK)
    {
      STMFLASH_Write(0x0800FC00,(uint16_t*)RC_offest,4);
    }
    
    osDelay(1);
  }
  /* USER CODE END Start_Param_Task */
}

/* USER CODE BEGIN Application */
     
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
