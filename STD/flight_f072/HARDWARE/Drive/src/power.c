/*******************************************************************************************
										    声 明
    本项目代码仅供个人学习使用，可以自由移植修改，但必须保留此声明信息。移植过程中出现其他
	
不可估量的BUG，天际智联不负任何责任。请勿商用！

* 程序版本：V1.01
* 程序日期：2018-8-18
* 程序作者：愤怒的小孩
* 版权所有：西安天际智联信息技术有限公司
*******************************************************************************************/
#include "structconfig.h"
#include "power.h"
#include "si24r1.h"
#include "stdio.h"
#include "filter.h"


BATT_TYPE BAT=
{
	.BattAdc = 0,        //电池电压采集ADC值
	.BattRealV = 3.31f,  //实际测量的飞机供电电压 (注意此电压必须亲测否则测量的电压不准)
	.BattMeasureV = 0,   //程序测量的实际电池电压
	.BattAlarmV = 3.2f,  //电池低电压报警瞬时值 (这个值需要根据机身不同重量实测，实测380mh是2.8v)
	.BattFullV = 4.2f,   //电池充满电值 4.2V
};
uint8_t BATT_LEDflag = 0;

/******************************************************************************************
* 函  数：void BATT_Init(void)
* 功  能：电压检测引脚初始化 以及ADC1初始化
* 参  数：无
* 返回值：无
* 备  注：无
*******************************************************************************************/
void BATT_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	ADC_InitTypeDef   ADC_InitStruct;
	
  /* GPIOC Periph clock enable */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
  
  /* ADC1 Periph clock enable */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
	
	//模拟输入模式选择       
	GPIO_InitStruct.GPIO_Pin=GPIO_Pin_3;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AN;
  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL ;
	GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* ADCs DeInit */  
  ADC_DeInit(ADC1);
  /* Initialize ADC structure */
  ADC_StructInit(&ADC_InitStruct);
  
  /* Configure the ADC1 in continuous mode with a resolution equal to 12 bits  */
  ADC_InitStruct.ADC_Resolution = ADC_Resolution_12b;
  ADC_InitStruct.ADC_ContinuousConvMode = ENABLE; 
  ADC_InitStruct.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
  ADC_InitStruct.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStruct.ADC_ScanDirection = ADC_ScanDirection_Upward;
  ADC_Init(ADC1, &ADC_InitStruct); 
	
  ADC_ChannelConfig(ADC1, ADC_Channel_3 , ADC_SampleTime_239_5Cycles);
  /* ADC Calibration */
  ADC_GetCalibrationFactor(ADC1);
  
  /* Enable the ADC peripheral */
  ADC_Cmd(ADC1, ENABLE);     
  
  /* Wait the ADRDY flag */
  while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_ADRDY)); 
  
  /* ADC1 regular Software Start Conv */ 
  ADC_StartOfConversion(ADC1);
}

/******************************************************************************************
* 函  数：uint16_t Get_BatteryAdc(uint8_t ch)
* 功  能：获取电池采样点电压的ADC值
* 参  数：ch  ADC采样通道
* 返回值：返回通道AD值
* 备  注：电池电压采样点的ADC值，电池电压采样电路见原理图
*******************************************************************************************/
uint16_t Get_BatteryAdc(uint8_t ch)
{
    /* Test EOC flag */
  while(ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET);
  /* Get ADC1 converted data */
//  ADC1ConvertedValue =ADC_GetConversionValue(ADC1);
//	ADC_RegularChannelConfig(ADC1, ch, 1, ADC_SampleTime_239Cycles5);
//	ADC_SoftwareStartConvCmd(ADC1,ENABLE);	//软件触发转换使能
//	while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC));	//等待转换结束
	return ADC_GetConversionValue(ADC1);	//返回转换结果的值
}		

/******************************************************************************************
* 函  数：void BATT_GetVoltage(void)
* 功  能：获取电池电压
* 参  数：无
* 返回值：无
* 备  注：电池电压 = ADC检测电压*2 具体原理可看原理图
*******************************************************************************************/
void BATT_GetVoltage(void)
{
	float V;
	Aver_Filter((float)Get_BatteryAdc(ADC_Channel_3),&BAT.BattAdc,6); //滑动滤波电压值，提高精度
	if(BAT.BattAdc)
	V = BAT.BattAdc * BAT.BattRealV / 4095.0f;
	BAT.BattMeasureV = 2*V; //根据原理电阻分压，可知 电池实际电压 = ADC侧量电压 * 2
//	printf("Test Voltage :%0.2f   temp:%0.0f \r\n ",BAT.BattMeasureV,BAT.BattAdc);
}

/******************************************************************************************
* 函  数：void LowVoltage_Alarm(void)
* 功  能：低电量报警
* 参  数：无
* 返回值：无
* 备  注：无
*******************************************************************************************/
void LowVoltage_Alarm(void)
{
	static uint8_t cnt=0,cnt1=0;
	BATT_GetVoltage();
	if(Airplane_Enable)
	{
		if(BAT.BattMeasureV < BAT.BattAlarmV)//飞行时测量
		{
			if(cnt1++>10)
			{
				cnt1 = 0;
				BATT_LEDflag = 1;
			}
		}
		else
		{
			cnt1 = 0;
			BATT_LEDflag = 0;
		}
	}else
	{
		if(BAT.BattMeasureV < 3.7f)//落地时测量（380mh时是3.5V）
		{
			if(cnt++>10)
			{
				Run_flag = 0;
				cnt = 0;
			  BATT_LEDflag = 1;
			}
		}
		else
		{
			Run_flag = 1;
			cnt = 0;
			BATT_LEDflag = 0;
		}
	}
}


