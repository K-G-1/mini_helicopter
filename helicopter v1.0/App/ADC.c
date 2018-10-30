#include "ADC.h"
#include "LED.h"
/*

THR 	- PA0	-	ADC12_IN0
YAW		-	PA1	-	ADC12_IN1
PITCH	-	PA2	-	ADC12_IN2
ROLL	-	PA3	-	ADC12_IN3
POWER	-	PA4	-	ADC12_IN4
KEY_L	-	PB0	-	ADC12_IN8
KEY_R	-	PB1	-	ADC12_IN9

*/
#define ADC1_DR_Address    ((u32)0x4001244cu)

__IO uint16_t g_u16a7_ADCValue_CMS[1];


void g_v_ADCInit(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	DMA_InitTypeDef DMA_InitStructure;
	ADC_InitTypeDef ADC_InitStructure;
	
	/* Enable DMA clock */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
	
	/* Enable ADC1 and GPIOC clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB2Periph_GPIOA, ENABLE);
	
	/* Configure PC.01  as analog input */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_Init(GPIOA, &GPIO_InitStructure);				
	


	
	/* DMA channel1 configuration */
	DMA_DeInit(DMA1_Channel1);
	DMA_InitStructure.DMA_PeripheralBaseAddr = ADC1_DR_Address;	 //ADC��ַ
	DMA_InitStructure.DMA_MemoryBaseAddr = (u32)&g_u16a7_ADCValue_CMS;//�ڴ��ַ
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
	DMA_InitStructure.DMA_BufferSize = 1;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//�����ַ�̶�
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;  //�ڴ��ַ�̶�
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;	//����
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;		//ѭ������
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(DMA1_Channel1, &DMA_InitStructure);
	
	/* Enable DMA channel1 */
	DMA_Cmd(DMA1_Channel1, ENABLE);
	
	/* ADC1 configuration */
	
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;	//����ADCģʽ
	ADC_InitStructure.ADC_ScanConvMode = ENABLE ; 	 //��ֹɨ��ģʽ��ɨ��ģʽ���ڶ�ͨ���ɼ�
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;	//��������ת��ģʽ������ͣ�ؽ���ADCת��
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;	//��ʹ���ⲿ����ת��
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right; 	//�ɼ������Ҷ���
	ADC_InitStructure.ADC_NbrOfChannel = 1;	 	//Ҫת����ͨ����Ŀ
	ADC_Init(ADC1, &ADC_InitStructure);
	
	/*����ADCʱ�ӣ�ΪPCLK2��6��Ƶ����12MHz,ADCƵ����߲��ܳ���14MHz*/
	RCC_ADCCLKConfig(RCC_PCLK2_Div6); 
	/*����ADC1��ͨ��11Ϊ55.	5���������ڣ�����Ϊ1 */ 

	ADC_RegularChannelConfig(ADC1, ADC_Channel_4, 1, ADC_SampleTime_55Cycles5);


	/* Enable ADC1 DMA */
	ADC_DMACmd(ADC1, ENABLE);
	
	/* Enable ADC1 */
	ADC_Cmd(ADC1, ENABLE);
	
	/*��λУ׼�Ĵ��� */   
	ADC_ResetCalibration(ADC1);
	/*�ȴ�У׼�Ĵ�����λ��� */
	while(ADC_GetResetCalibrationStatus(ADC1));
	
	/* ADCУ׼ */
	ADC_StartCalibration(ADC1);
	/* �ȴ�У׼���*/
	while(ADC_GetCalibrationStatus(ADC1));
	
	/* ����û�в����ⲿ����������ʹ���������ADCת�� */ 
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);
}

uint16_t g_u16s8_BatteryVoltage_SMS = 0u;

void g_v_AdcCalculate(void)
{
	static uint8_t i = 0u;
	static uint32_t sum = 0u;
	uint16_t temp = 0u;
	temp =(uint16_t)(g_u16a7_ADCValue_CMS[0] * 3.3 * 2 * 256 / 4095);
	sum += temp;
	
	i++;
	if(i >= 50)
	{
		i=0u;
		g_u16s8_BatteryVoltage_SMS = sum / 50u;
		sum = 0u;
		if(g_u16s8_BatteryVoltage_SMS < 3.7 * 256)
		{
			LED1_turn();
			LED2_turn();
			LED3_turn();
			LED4_turn();
		}
		else
		{
//			LED1(OFF);
//			LED2(OFF);
//			LED3(OFF);
//			LED4(OFF);	
		}
	}
}





