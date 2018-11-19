/*******************************************************************************************
										    �� ��
    ����Ŀ�����������ѧϰʹ�ã�����������ֲ�޸ģ������뱣����������Ϣ����ֲ�����г�������
	
���ɹ�����BUG��������������κ����Ρ��������ã�

* ����汾��V1.01
* �������ڣ�2018-8-18
* �������ߣ���ŭ��С��
* ��Ȩ���У��������������Ϣ�������޹�˾
*******************************************************************************************/
#include "main.h"


  
void System_Init(void)
{
	NvicConfig(); //ϵͳ�ж����ȼ�����
//	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);
	LED_Init();	//�û�ָʾ�Ƴ�ʼ��
	Delay_Init(); //ϵͳ��ʱ��ʼ��
	USART_init(115200);	//���Դ��ڳ�ʼ��
	IIC_GPIO_Init(); //ģ��IIC��ʼ��
	TIM_Init();	//ϵͳʱ����ʼ��
	Exit_Init(); //�ⲿ�жϳ�ʼ��
	SI24R1_Init(); //SI24R1(2.4G)��ʼ�����죩
  SI24R1_Config();
	MPU6050_Init(); //MPU6050��ʼ�����̣� 
//	bmp280Init(); //FBM320��ʼ��(��ѹ����) 
	MOTOR_Init(); //��������ʼ��
	BATT_Init(); //��ص�ѹ����ʼ��
	PID_ReadFlash(); //Flash�е����ݶ�ȡ
	PidParameter_init(); //PID������ʼ��

	
//	printf("System Init Finish\n");
}

void Task_Schedule(void)
{
  uint8_t sta;
	if(ANO_Scan) //500Hz
		{
			ANO_Scan = 0;
			ANO_DT_Data_Exchange(); //�������ݵ���λ��
		}
		if(IMU_Scan) //100Hz
		{      

			IMU_Scan  = 0;
			Prepare_Data(); //��ȡ��̬������������
			IMUupdate(&Gyr_rad,&Acc_filt,&Att_Angle); //��Ԫ����̬����
			Control(&Att_Angle,&Gyr_rad,&RC_Control,Airplane_Enable); //��̬����
//			bmp280GetData(&Bmp280.bmp280_temp,&Bmp280.bmp280_press,&Bmp280.Altitude);
		}
		if(LED_Scan) //10Hz
		{
			LED_Scan = 0;
			LED_Run();
			Deblocking();
//			if(!Airplane_Enable&&Run_flag)
//			{
//				RGB_LED_Runing(); //�ɻ�����״̬��
//			}
//			BATT_Alarm_LED(); //��ص͵�ѹ����	  
		}
		if(IRQ_Scan) //5Hz
		{
			IRQ_Scan = 0;

      
			SI24R1_SingalCheck(); //2.4Gͨ�ż��
			SendToRemote(); //�������ݸ�ң����
		}
		if(Batt_Scan) //2.5Hz
		{
			Batt_Scan = 0;
//			SI24R1_GetAddr(); //����2.4G��ַ
			LowVoltage_Alarm();	//�͵�������
		}
}


