#include "led.h"
#include "usart.h"
#include "mpu6050.h"
#include "control.h"
#include "delay.h"
#include "timer.h"
#include "PWM_output.h"
#include "nrf24l01.h"
#include "DataTransfer.h"
#include "spi.h"
#include "appconfig.h"
#include "SysTick.h"
#include "delay.h"
#include "ADC.h"
#include "myiic.h"
#include "DataTransferV26.h"
#include "param.h"
	uint16_t pwm1;
	uint16_t pwm2;
	uint16_t pwm3;
	uint16_t pwm4;
typedef enum
{
	Flag_RESET = 0,
	Flag_SET
}FlagAction;

/*
 * 函数名：NVIC_Configuration
 * 描述  ：中断优先级配置
 * 输入  ：无
 * 输出  ：无	
 */
void NVIC_Configuration(void)
{
	NVIC_InitTypeDef NVIC_InitStructure; 
	/* Configure the NVIC Preemption Priority Bits */  
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	
//	/* Enable the USARTy Interrupt */
//	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;	 
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//	NVIC_Init(&NVIC_InitStructure);

	
	/* Enable the USARTy Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;	 
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	/* Enable the TIM1 Interrupt */ 													
    NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;	  
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;	
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);	

}
u8 status;	//用于判断接收/发送状态


void system_init(void)
{
	u16 who_am_i; 
//	SysTick_Init();
//	g_v_ADCInit();
	LED_GPIO_Config();

	Tim3_Init(1000);
//	TIM1_Configuration();
	NVIC_Configuration();
	USART1_Config();


	check_mpu6050();
	mpu6050_init();
	

	who_am_i=mpu6050_read_byte(WHO_AM_I);

	g_v_PwmInit();
	g_v_PwmUpdateDuty();
	Spi2_Init();
	Nrf24l01_Init(3,40);

//	SPI_NRF_Init();
//	/*检测NRF模块与MCU的连接*/
  LED3(ON);
	status = Nrf24l01_Check(); 
	if(status == SUCCESS)	  
	{
		LED3(OFF);
	}
	else
	{
		LED3(ON);
	}
	ANO_Param_Init();
//	ANO_Param_Read();
	g_v_PIDInit();
	START_TIME;	
	LED1(ON);
	LED2(ON);
	LED3(ON);
	LED4(ON);
}
uintbl g_bl_TimeFlag2ms_CMS=0;
uintbl g_bl_TimeFlag10ms_SMS=0;
uintbl g_bl_TimeFlag50ms_SMS=0;
int main(void)
{	
static uint8_t cnt=0u;
static uint8_t cnt1=0u;
	system_init();

	delayms(3000);
	while(1)
	{		
		if(g_bl_TimeFlag2ms_CMS == 1)
		{
			
			g_bl_TimeFlag2ms_CMS = 0;
			g_v_AdcCalculate();	

		  g_v_GetMpuOffset();
			LED1_turn();
			
			
			
			g_v_UpdataPositioning();

			g_v_ControlDublePID();

			Nrf_Check_Event();
			cnt++;
			if(cnt == 5)
			{
				cnt = 0;
				g_bl_TimeFlag10ms_SMS = 1;
				if(g_bl_TimeFlag10ms_SMS == 1)
				{
					ANO_DT_Data_Exchange();
					g_bl_TimeFlag10ms_SMS = 0;
					cnt1++;
//					MPU6050_Data_Offset();
					if(5==cnt1)
					{	
						cnt1=0;
						g_bl_TimeFlag50ms_SMS=1;
						if(g_bl_TimeFlag50ms_SMS==1)
						{
							g_bl_TimeFlag50ms_SMS=0;
							PID_Save_Overtime(1500,50);
						}
					}

				}
			}
		}
	}
}










