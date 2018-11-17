/*******************************************************************************************
										    �� ��
    ����Ŀ�����������ѧϰʹ�ã�����������ֲ�޸ģ������뱣����������Ϣ����ֲ�����г�������
	
���ɹ�����BUG��������������κ����Ρ��������ã�

* ����汾��V1.01
* �������ڣ�2018-8-18
* �������ߣ���ŭ��С��
* ��Ȩ���У��������������Ϣ�������޹�˾
*******************************************************************************************/
#include "structconfig.h"
#include "power.h"
#include "si24r1.h"
#include "stdio.h"
#include "filter.h"


BATT_TYPE BAT=
{
	.BattAdc = 0,        //��ص�ѹ�ɼ�ADCֵ
	.BattRealV = 3.31f,  //ʵ�ʲ����ķɻ������ѹ (ע��˵�ѹ�����ײ��������ĵ�ѹ��׼)
	.BattMeasureV = 0,   //���������ʵ�ʵ�ص�ѹ
	.BattAlarmV = 3.2f,  //��ص͵�ѹ����˲ʱֵ (���ֵ��Ҫ���ݻ���ͬ����ʵ�⣬ʵ��380mh��2.8v)
	.BattFullV = 4.2f,   //��س�����ֵ 4.2V
};
uint8_t BATT_LEDflag = 0;

/******************************************************************************************
* ��  ����void BATT_Init(void)
* ��  �ܣ���ѹ������ų�ʼ�� �Լ�ADC1��ʼ��
* ��  ������
* ����ֵ����
* ��  ע����
*******************************************************************************************/
void BATT_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	ADC_InitTypeDef   ADC_InitStruct;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_ADC1, ENABLE); 
	RCC_ADCCLKConfig(RCC_PCLK2_Div6);
	
	//ģ������ģʽѡ��       
	GPIO_InitStruct.GPIO_Pin=GPIO_Pin_3;
	GPIO_InitStruct.GPIO_Mode=GPIO_Mode_AIN; //ģ������
	GPIO_Init(GPIOA, &GPIO_InitStruct);

	ADC_InitStruct.ADC_Mode=ADC_Mode_Independent;						//����ģʽ
	ADC_InitStruct.ADC_DataAlign=ADC_DataAlign_Right;					//�����Ҷ���
	ADC_InitStruct.ADC_NbrOfChannel=1;									//1������ͨ��
	ADC_InitStruct.ADC_ScanConvMode=DISABLE;							//ɨ��ת��ģʽʧ��
	ADC_InitStruct.ADC_ExternalTrigConv=ADC_ExternalTrigConv_None;		//�ⲿ����ʧ��
	ADC_InitStruct.ADC_ContinuousConvMode=DISABLE;						//����ת��ʧ��
	ADC_Init(ADC1,&ADC_InitStruct);
	
	ADC_Cmd(ADC1, ENABLE); //ʹ��ADC1
	
	ADC_RegularChannelConfig(ADC1,ADC_Channel_3,1,ADC_SampleTime_239Cycles5);	//������ת��ͨ��
}

/******************************************************************************************
* ��  ����uint16_t Get_BatteryAdc(uint8_t ch)
* ��  �ܣ���ȡ��ز������ѹ��ADCֵ
* ��  ����ch  ADC����ͨ��
* ����ֵ������ͨ��ADֵ
* ��  ע����ص�ѹ�������ADCֵ����ص�ѹ������·��ԭ��ͼ
*******************************************************************************************/
uint16_t Get_BatteryAdc(uint8_t ch)
{
	ADC_RegularChannelConfig(ADC1, ch, 1, ADC_SampleTime_239Cycles5);
	ADC_SoftwareStartConvCmd(ADC1,ENABLE);	//�������ת��ʹ��
	while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC));	//�ȴ�ת������
	return ADC_GetConversionValue(ADC1);	//����ת�������ֵ
}		

/******************************************************************************************
* ��  ����void BATT_GetVoltage(void)
* ��  �ܣ���ȡ��ص�ѹ
* ��  ������
* ����ֵ����
* ��  ע����ص�ѹ = ADC����ѹ*2 ����ԭ��ɿ�ԭ��ͼ
*******************************************************************************************/
void BATT_GetVoltage(void)
{
	float V;
	Aver_Filter((float)Get_BatteryAdc(ADC_Channel_3),&BAT.BattAdc,6); //�����˲���ѹֵ����߾���
	if(BAT.BattAdc)
	V = BAT.BattAdc * BAT.BattRealV / 4095.0f;
	BAT.BattMeasureV = 2*V; //����ԭ������ѹ����֪ ���ʵ�ʵ�ѹ = ADC������ѹ * 2
//	printf("Test Voltage :%0.2f   temp:%0.0f \r\n ",BAT.BattMeasureV,BAT.BattAdc);
}

/******************************************************************************************
* ��  ����void LowVoltage_Alarm(void)
* ��  �ܣ��͵�������
* ��  ������
* ����ֵ����
* ��  ע����
*******************************************************************************************/
void LowVoltage_Alarm(void)
{
	static uint8_t cnt=0,cnt1=0;
	BATT_GetVoltage();
	if(Airplane_Enable)
	{
		if(BAT.BattMeasureV < BAT.BattAlarmV)//����ʱ����
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
		if(BAT.BattMeasureV < 3.7f)//���ʱ������380mhʱ��3.5V��
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


