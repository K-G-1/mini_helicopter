#include "timer3.h"
#include  "IMU.h"
#include "sand_data.h"
#include "led.h"
#include "mpu9250.h"


void tim3_init(u16 arr,u16 psc)
{
	
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
	NVIC_InitTypeDef NVIC_InitStruct;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);
	
	//timer3中断配置

	TIM_TimeBaseInitStruct.TIM_Period=arr;
	TIM_TimeBaseInitStruct.TIM_Prescaler=psc;
	TIM_TimeBaseInitStruct.TIM_ClockDivision= TIM_CKD_DIV1;
	TIM_TimeBaseInitStruct.TIM_CounterMode= TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM3,&TIM_TimeBaseInitStruct);
	
	//中断优先级配置
	NVIC_InitStruct.NVIC_IRQChannel=TIM3_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelCmd=ENABLE ;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority=0x01;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority=0x03;
	NVIC_Init(&NVIC_InitStruct);

	
	//使能timer3中断
	
	TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE);
	TIM_Cmd(TIM3, ENABLE);

}

int times= 0;
void TIM3_IRQHandler(void)
{
	if( TIM_GetITStatus(TIM3 ,TIM_IT_Update)==SET)
	{
		Get_Attitude();
        
        times ++;
        sand_IMU_data();
        sand_ACC_GYRO_data(); 
        
        if(times % 19 == 0)
        {   
            READ_MPU9250_MAG();
        }
        else if(times % 3 == 0)
        {
            LED0 =!LED0;
        }
//        if(times %20 == 0)
//        {
//           
//            sand_ACC_GYRO_data(); 
//        }

	}
	TIM_ClearITPendingBit(TIM3,TIM_IT_Update);
}
