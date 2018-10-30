

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
 * 函数名：PWM_GPIO_Config
 * 描述  ：配置TIM2复用输出PWM时用到的I/O
 * 输入  ：无
 * 输出  ：无
 * 调用  ：内部调用
 */
static void PWM_GPIO_Config(void) 
{
	GPIO_InitTypeDef GPIO_InitStructure;	 
	/* TIM2 clock enable */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM4, ENABLE);//PCLK1经过2倍频后作为TIM2的时钟源等于72MHz
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB, ENABLE); /* GPIOA clock enable */
	/*GPIOA Configuration: TIM2 channel 1 and 2 as alternate function push-pull */
	GPIO_InitStructure.GPIO_Pin =  PWM1 | PWM2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;		    // 复用推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(Motor_PORT_PWM12, &GPIO_InitStructure);
	
	/*GPIOB Configuration: TIM4 channel 3 and 4 as alternate function push-pull */
	GPIO_InitStructure.GPIO_Pin =  PWM3 | PWM4;
	GPIO_Init(Motor_PORT_PWM34, &GPIO_InitStructure);
//	GPIO_ResetBits(Motor_PORT_PWM12,PWM1|PWM2);
//	GPIO_ResetBits(Motor_PORT_PWM34,PWM3|PWM4);
}

/*
 * 函数名：PWM_Mode_Config
 * 描述  ：配置TIM2/4输出的PWM信号的模式，如周期、极性、占空比
 * 输入  ：无
 * 输出  ：无   
 * 调用  ：内部调用
 */
static void PWM_Mode_Config(void)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;

	/* PWM信号电平跳变值 */

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
	TIM_TimeBaseStructure.TIM_Period = 3599;       //当定时器从0计数到3599，即为3600次，为一个定时周期
	TIM_TimeBaseStructure.TIM_Prescaler = 0;	    //设置预分频：0分频，即为20KHz
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;	//设置时钟分频系数：不分频
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //向上计数模式

	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);	

	/* PWM1 Mode configuration: Channel1 */
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;	    //配置为PWM模式1
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;	
	TIM_OCInitStructure.TIM_Pulse = g_u16_Pwm1MC_CMS;	   //设置跳变值，当计数器计数到这个值时，电平发生跳变
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;  //当定时器计数值小于CCR1_Val时为高电平
	TIM_OC1Init(TIM2, &TIM_OCInitStructure);	 //配置通道1
	TIM_OC1Init(TIM4, &TIM_OCInitStructure);	 //配置通道1
	TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable);
	TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);

	/* PWM1 Mode configuration: Channel2 */
	TIM_OCInitStructure.TIM_Pulse = g_u16_Pwm2MC_CMS;	  //设置通道2的电平跳变值，输出另外一个占空比的PWM
	TIM_OC2Init(TIM2, &TIM_OCInitStructure);	  //配置通道2
	TIM_OC2Init(TIM4, &TIM_OCInitStructure);	  //配置通道2
	TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);
	TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);

	TIM_ARRPreloadConfig(TIM2, ENABLE);	// 使能TIM2重载寄存器ARR
	TIM_ARRPreloadConfig(TIM4, ENABLE);	// 使能TIM4重载寄存器ARR
	TIM_Cmd(TIM2, ENABLE);	//使能定时器2/* TIM2 enable counter */
	TIM_Cmd(TIM4, ENABLE);	//使能定时器4/* TIM4 enable counter */   
}

/*
 * 函数名：g_v_PwmInit
 * 描述  ：TIM2、TIM4输出PWM信号初始化     
 * 输入  ：无
 * 输出  ：无
 * 调用  ：外部调用
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











