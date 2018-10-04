#include "pwm.h"

#define Moto_PwmMax 1000

void pwm_init(u16 arr,u16 psc)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
	TIM_OCInitTypeDef TIM_OCInitStruct;
	GPIO_InitTypeDef GPIO_InitStruct;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE );
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);
	
	//����IO��
//	GPIO_PinAFConfig(GPIOF,GPIO_PinSource9,GPIO_AF_TIM14);
	
	GPIO_InitStruct.GPIO_Mode=GPIO_Mode_AF_PP;
	GPIO_InitStruct.GPIO_Pin=GPIO_Pin_6;
	GPIO_InitStruct.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&GPIO_InitStruct);
	
    GPIO_InitStruct.GPIO_Pin=GPIO_Pin_7;
    GPIO_Init(GPIOB,&GPIO_InitStruct);
    GPIO_InitStruct.GPIO_Pin=GPIO_Pin_8;
    GPIO_Init(GPIOB,&GPIO_InitStruct);
    GPIO_InitStruct.GPIO_Pin=GPIO_Pin_9;
    GPIO_Init(GPIOB,&GPIO_InitStruct);
    /**********************************************************
	72 000 000/72=1M
	1000 000/2500=400Hz
	���Բ�����PWMΪ400Hz
	����Ϊ2.5ms����Ӧ2500�ļ���ֵ��1ms~2ms��Ӧ�ļ���ֵΪ1000~2000��
	**********************************************************/
	//���ö�ʱ��
	TIM_TimeBaseInitStruct.TIM_ClockDivision=TIM_CKD_DIV1;
	TIM_TimeBaseInitStruct.TIM_CounterMode=TIM_CounterMode_Up;
	TIM_TimeBaseInitStruct.TIM_Period=arr;
	TIM_TimeBaseInitStruct.TIM_Prescaler=psc;
	TIM_TimeBaseInit(TIM4,&TIM_TimeBaseInitStruct);
	
	//TIM14 ͨ��
	TIM_OCInitStruct.TIM_OCMode=TIM_OCMode_PWM1;
	TIM_OCInitStruct.TIM_OutputState=TIM_OutputState_Enable;
	TIM_OCInitStruct.TIM_OCPolarity=TIM_OCPolarity_High;
    
	TIM_OC1Init(TIM4,&TIM_OCInitStruct);
	TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);
	TIM_OC2Init(TIM4,&TIM_OCInitStruct);
	TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);
    TIM_OC3Init(TIM4,&TIM_OCInitStruct);
	TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);
    TIM_OC4Init(TIM4,&TIM_OCInitStruct);
	TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);
    
	TIM_Cmd(TIM4, ENABLE);
	
	TIM_ARRPreloadConfig(TIM4,ENABLE);//ARPEʹ��
}

void Moto_PwmRflash(uint16_t MOTO1_PWM,uint16_t MOTO2_PWM,uint16_t MOTO3_PWM,uint16_t MOTO4_PWM)
{		
	if(MOTO1_PWM>Moto_PwmMax)	MOTO1_PWM = Moto_PwmMax;
	if(MOTO2_PWM>Moto_PwmMax)	MOTO2_PWM = Moto_PwmMax;
	if(MOTO3_PWM>Moto_PwmMax)	MOTO3_PWM = Moto_PwmMax;
	if(MOTO4_PWM>Moto_PwmMax)	MOTO4_PWM = Moto_PwmMax;
	
	
	TIM4->CCR1 = MOTO1_PWM;
	TIM4->CCR2 = MOTO2_PWM;
	TIM4->CCR3 = MOTO3_PWM;
	TIM4->CCR4 = MOTO4_PWM;
//	TIM2->CCR1 = MOTO1_PWM;
//	TIM2->CCR2 = MOTO2_PWM;
//	TIM2->CCR3 = MOTO3_PWM;
//	TIM2->CCR4 = MOTO4_PWM;

}



