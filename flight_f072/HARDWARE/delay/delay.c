#include "delay.h"


volatile uint32_t sysTickUptime = 0;


#define TICK_PER_SECOND 1000 
#define TICK_US	(1000000/TICK_PER_SECOND)




void  SysTick_Configuration(void)
{
	RCC_ClocksTypeDef  rcc_clocks;
  NVIC_InitTypeDef NVIC_InitStructure;
	uint32_t         cnts;

	

   RCC_DeInit();
  RCC_HSEConfig(RCC_HSE_ON);  
  
  if(RCC_WaitForHSEStartUp() == SUCCESS)    //??HSE????
    {  
    
        RCC_PLLConfig(RCC_PLLSource_HSE, RCC_PLLMul_8);  //??PLL?????9? 72Mhz
        RCC_PLLCmd(ENABLE);   //??PLL??                                  
        while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)     //??PLL????
        {  
        }  
        RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK); //??????PLL?????  
        // *           - 0x00: HSI used as system clock  
        // *           - 0x04: HSE used as system clock    
        // *           - 0x08: PLL used as system clock  
        // *           - 0x0C: HSI48 used as system clock, applicable only for STM32F072 devices    
        while(RCC_GetSYSCLKSource()!=0x08)   //???????????PLL???
        {  
        }  
        RCC_HCLKConfig(RCC_SYSCLK_Div1); //PLL????????   72MHz
    }  

  RCC_GetClocksFreq(&rcc_clocks);
	cnts = (uint32_t)rcc_clocks.HCLK_Frequency / TICK_PER_SECOND;
	cnts = cnts / 8;

	SysTick_Config(cnts);
  
	SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8);
}

uint32_t GetSysTime_us(void) 
{
	register uint32_t ms;
	uint32_t value;
	ms = sysTickUptime;
	value = ms * TICK_US + (SysTick->LOAD - SysTick->VAL) * TICK_US / SysTick->LOAD;
	return value;
}

void delay_us(uint32_t us)
{
    uint32_t now = GetSysTime_us();
    while (GetSysTime_us() - now < us);
}

void delay_ms(uint32_t ms)
{
    while (ms--)
        delay_us(1000);
}



void delay_init(void)
{
	SysTick_Configuration();
	
	
}


















