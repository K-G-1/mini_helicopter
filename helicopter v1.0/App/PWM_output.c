

#include "PWM_output.h"

#define Motor_PORT_PWM12	GPIOA
#define Motor_PORT_PWM34	GPIOB	

#define PWM1	GPIO_Pin_1
#define PWM2	GPIO_Pin_0
#define PWM3	GPIO_Pin_7
#define PWM4	GPIO_Pin_6

volatile uint16_t g_u16_Pwm1MC_CMS =00u;
volatile uint16_t g_u16_Pwm2MC_CMS =00u;
volatile uint16_t g_u16_Pwm3MC_CMS =00u;
volatile uint16_t g_u16_Pwm4MC_CMS =00u;

/*
 * ��������PWM_GPIO_Config
 * ����  ������TIM2�������PWMʱ�õ���I/O
 * ����  ����
 * ���  ����
 * ����  ���ڲ�����
 */
static void PWM_GPIO_Config(void) 
{
	GPIO_InitTypeDef GPIO_InitStructure;	 
	/* TIM2 clock enable */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM4, ENABLE);//PCLK1����2��Ƶ����ΪTIM2��ʱ��Դ����72MHz
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB, ENABLE); /* GPIOA clock enable */
	/*GPIOA Configuration: TIM2 channel 1 and 2 as alternate function push-pull */
	GPIO_InitStructure.GPIO_Pin =  PWM1 | PWM2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;		    // �����������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(Motor_PORT_PWM12, &GPIO_InitStructure);
	
	/*GPIOB Configuration: TIM4 channel 3 and 4 as alternate function push-pull */
	GPIO_InitStructure.GPIO_Pin =  PWM3 | PWM4;
	GPIO_Init(Motor_PORT_PWM34, &GPIO_InitStructure);
//	GPIO_ResetBits(Motor_PORT_PWM12,PWM1|PWM2);
//	GPIO_ResetBits(Motor_PORT_PWM34,PWM3|PWM4);
}

/*
 * ��������PWM_Mode_Config
 * ����  ������TIM2/4�����PWM�źŵ�ģʽ�������ڡ����ԡ�ռ�ձ�
 * ����  ����
 * ���  ����   
 * ����  ���ڲ�����
 */
static void PWM_Mode_Config(void)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;

	/* PWM�źŵ�ƽ����ֵ */

/* -----------------------------------------------------------------------
    TIM3 Configuration: generate 4 PWM signals with 4 different duty cycles:
    TIM3CLK = 72 MHz, Prescaler = 0x0, TIM3 counter clock = 72 MHz
    TIM3 ARR Register = 999 => TIM3 Frequency = TIM3 counter clock/(ARR + 1)
    TIM3 Frequency = 72 KHz.
    TIM3 Channel1 duty cycle = (TIM3_CCR1/ TIM3_ARR)* 100 = 50%
    TIM3 Channel2 duty cycle = (TIM3_CCR2/ TIM3_ARR)* 100 = 37.5%
    TIM3 Channel3 duty cycle = (TIM3_CCR3/ TIM3_ARR)* 100 = 25%
    TIM3 Channel4 duty cycle = (TIM3_CCR4/ TIM3_ARR)* 100 = 12.5%
  ----------------------------------------------------------------------- */

	/* Time base configuration */		 
	TIM_TimeBaseStructure.TIM_Period = 3599;       //����ʱ����0������3599����Ϊ3600�Σ�Ϊһ����ʱ����
	TIM_TimeBaseStructure.TIM_Prescaler = 0;	    //����Ԥ��Ƶ��0��Ƶ����Ϊ20KHz
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;	//����ʱ�ӷ�Ƶϵ��������Ƶ
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //���ϼ���ģʽ

	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);	

	/* PWM1 Mode configuration: Channel1 */
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;	    //����ΪPWMģʽ1
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;	
	TIM_OCInitStructure.TIM_Pulse = g_u16_Pwm1MC_CMS;	   //��������ֵ�������������������ֵʱ����ƽ��������
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;  //����ʱ������ֵС��CCR1_ValʱΪ�ߵ�ƽ
	TIM_OC1Init(TIM2, &TIM_OCInitStructure);	 //����ͨ��1
	TIM_OC1Init(TIM4, &TIM_OCInitStructure);	 //����ͨ��1
	TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable);
	TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);

	/* PWM1 Mode configuration: Channel2 */
	TIM_OCInitStructure.TIM_Pulse = g_u16_Pwm2MC_CMS;	  //����ͨ��2�ĵ�ƽ����ֵ���������һ��ռ�ձȵ�PWM
	TIM_OC2Init(TIM2, &TIM_OCInitStructure);	  //����ͨ��2
	TIM_OC2Init(TIM4, &TIM_OCInitStructure);	  //����ͨ��2
	TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);
	TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);

	TIM_ARRPreloadConfig(TIM2, ENABLE);	// ʹ��TIM2���ؼĴ���ARR
	TIM_ARRPreloadConfig(TIM4, ENABLE);	// ʹ��TIM4���ؼĴ���ARR
	TIM_Cmd(TIM2, ENABLE);	//ʹ�ܶ�ʱ��2/* TIM2 enable counter */
	TIM_Cmd(TIM4, ENABLE);	//ʹ�ܶ�ʱ��4/* TIM4 enable counter */   
}

/*
 * ��������g_v_PwmInit
 * ����  ��TIM2��TIM4���PWM�źų�ʼ��     
 * ����  ����
 * ���  ����
 * ����  ���ⲿ����
 */
void g_v_PwmInit(void)
{
	PWM_GPIO_Config();
	PWM_Mode_Config();	
}

void g_v_PwmUpdateDuty(void)
{
	if(g_u16_Pwm1MC_CMS > 3600u)
	{
		g_u16_Pwm1MC_CMS = 3600u;
	}
	if(g_u16_Pwm2MC_CMS > 3600u)
	{
		g_u16_Pwm2MC_CMS = 3600u;
	}
	if(g_u16_Pwm3MC_CMS > 3600u)
	{
		g_u16_Pwm3MC_CMS = 3600u;
	}
	if(g_u16_Pwm4MC_CMS > 3600u)
	{
		g_u16_Pwm4MC_CMS = 3600u;
	}
	TIM_SetCompare1(TIM2 , g_u16_Pwm2MC_CMS);//Q1
	TIM_SetCompare2(TIM2 , g_u16_Pwm1MC_CMS);//Q3
	TIM_SetCompare2(TIM4 , g_u16_Pwm3MC_CMS);//Q4
	TIM_SetCompare1(TIM4 , g_u16_Pwm4MC_CMS);//Q2
}











