#include "RC.h"
#include "LED.h"

uint16_t Rc_Pwm_In[8];
T_RC_DATA Rc_Data;//1000~2000
uint16_t  RC_Pwm_In_his[8];
uint8_t ARMED = 0;
uint8_t mode ;



/*    ����&��������  */
void Deblocking(void)
{
	 static uint8_t flag=1;
	 static uint16_t time1=0,time2=0;
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
	}
	else if(Rc_Data.AUX1>1300&&Rc_Data.AUX1<1700)
	{
		mode= 1;
	}
	else if(Rc_Data.AUX1<1300)
	{
		mode= 0 ;
	}
		
}







