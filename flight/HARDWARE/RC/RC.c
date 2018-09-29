#include "RC.h"
#include "LED.h"

u16 Rc_Pwm_In[8];
T_RC_DATA Rc_Data;//1000~2000
u16  RC_Pwm_In_his[8];
extern u8 mode ;



/*====================================================================================================*/
/*====================================================================================================*
**���� : RcData_Refine
**���� : ����ң������
**���� : None
**��� : None
**��ע : ��
**====================================================================================================*/
/*====================================================================================================*/

extern u16 Moto_duty[4];
void RC_Data_Refine(void)
{
  u8 chan,a;	

	u16 rcDataMax[4], rcDataMin[4];
	static int16_t rcDataCache[4][4], rcDataMean[4];
	static uint8_t rcValuesIndex = 0;

	rcValuesIndex++;
	for (chan = 0; chan < 4; chan++) {
		  //����ƽ��ֵ�˲���4��
		  if(Rc_Pwm_In[chan]>2800 || Rc_Pwm_In[chan]<800) 
				Rc_Pwm_In[chan] = RC_Pwm_In_his[chan];
			rcDataCache[chan][rcValuesIndex % 4] = Rc_Pwm_In[chan] ;		
		  RC_Pwm_In_his[chan] = Rc_Pwm_In[chan];
			
			rcDataMean[chan] = 0;
		  rcDataMax[chan]  = 0;
		  rcDataMin[chan]  = 25000;
		
			for (a = 0; a < 4; a++) {
				  // ��¼���������ֵ && ��Сֵ
				  if(rcDataCache[chan][a] > rcDataMax[chan])  rcDataMax[chan] = rcDataCache[chan][a];     
					if(rcDataCache[chan][a] < rcDataMin[chan])	rcDataMin[chan] = rcDataCache[chan][a]; 
				  // ���
					rcDataMean[chan] += rcDataCache[chan][a];  
      }
			// �޳������� ���ֵ && ��Сֵ 
			rcDataMean[chan] = (rcDataMean[chan] - (rcDataMax[chan] + rcDataMin[chan])) / 2;
	} 
	
	//for(chan=0;chan<6;chan++)
	 Rc_Data.YAW   = rcDataMean[0];
	 Rc_Data.THROTTLE  = rcDataMean[1];
	 Rc_Data.PITCH  =   rcDataMean[3];
	 Rc_Data.ROLL =  rcDataMean[2];

	 Rc_AUX();
}

void Rc_AUX(void)
{
//	 u8 chan,chan1;	
//	static int16_t rcDataCache[4][4], rcDataMean[4];

//	
//	for(chan =7;chan>3;chan--)
//	{
//		if(Rc_Pwm_In[chan]>3000 || Rc_Pwm_In[chan]<100) 
//				Rc_Pwm_In[chan] = RC_Pwm_In_his[chan];
//		RC_Pwm_In_his[chan] = Rc_Pwm_In[chan];
//		
//		for(chan1=3;chan1>0;chan1--)
//		{
//			rcDataCache[chan-4][chan1]= rcDataCache[chan-4][chan1-1];
//		}
//	}
//	rcDataCache[0][0]= Rc_Pwm_In[4];
//	rcDataCache[1][0]= Rc_Pwm_In[5];
//	rcDataCache[2][0]= Rc_Pwm_In[6];
//	rcDataCache[3][0]= Rc_Pwm_In[7];
//	for(chan=0;chan<4;chan++)
//	{
//		for(chan1=0;chan1<4;chan1++)
//		{
//			rcDataMean[chan] += rcDataCache[chan][chan1];
//		}
//	}
//		

	Rc_Data.AUX1=Rc_Pwm_In[4];		
	Rc_Data.AUX2=Rc_Pwm_In[5];
	Rc_Data.AUX3=Rc_Pwm_In[6];
	Rc_Data.AUX4=Rc_Pwm_In[7];
}
extern u8 ARMED;
/*    ����&��������  */
void Deblocking(void)
{
	 static vs8 flag=1;
	 static vs16 time1=0,time2=0;
	 /*               ң������                 */
	 /*     ������������������            ������������������     */
	 /*    |         |          |         |    */
	 /*    |         |          |         |    */
	 /*    |    \    |          |   /     |    */
	 /*    |     \   |          |  /      |    */
	 /*     ������������������            ������������������     */
	 /*   �����������         ҡ���Ƶ����Ͻ�  */
   if(ARMED && Rc_Data.ROLL >= 1800 && Rc_Data.PITCH <= 1200 && Rc_Data.THROTTLE <= 1200 &&  Rc_Data.YAW <= 1200)		
	 {  
			time1++; 
	 }	
	 else 
		 time1=0;
	 if(time1>30 && ARMED) 
	 { 
			ARMED = 0; 
			time1 = 0;
			LED1=0;
	 }
   /*               ң�ؽ���                 */
	 /*     ������������������            ������������������     */
	 /*    |         |          |         |    */
	 /*    |         |          |         |    */
	 /*    |    /    |          |     \   |    */
	 /*    |   /     |          |      \  |    */
	 /*     ������������������            ������������������     */
	 /*   �����������         ҡ���Ƶ����Ͻ�  */
   if(!ARMED && Rc_Data.YAW >= 1800 && Rc_Data.PITCH <= 1200 && Rc_Data.THROTTLE <= 1200 &&  Rc_Data.ROLL <= 1200)		
		{
			time2++; 
			
		}	
	 else 
		 time2=0;
	 if(time2>=30 && !ARMED)
	 {
		  ARMED = 1; 
			time2 = 0;
		  LED1=1;
	 }

}
/***************************

mode = 0;   δ֪
mode = 1,		��̬
mode = 2;		����
mode = 3;		����


*****************************/
void mode_contrl(void)
{
	if(Rc_Data.AUX1>1700)
	{
		mode = 2 ;  //����	
		LED2= 1;
	}
	else if(Rc_Data.AUX1>1300&&Rc_Data.AUX1<1700)
	{
		LED2= 0;
		mode= 1;
	}
	else if(Rc_Data.AUX1<1300)
	{
		mode= 0 ;
		LED2= 0;
	}
		
}







