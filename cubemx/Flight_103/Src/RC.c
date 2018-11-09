#include "RC.h"

#include "Algorithm_math.h"



uint16_t Rc_Pwm_In[8];
T_RC_DATA Rc_Data;//1000~2000
T_RC_DATA RX_Data;//1000~2000
uint16_t  RC_Pwm_In_his[8];
uint8_t ARMED = 0;
uint8_t mode ;
// 适应回中的摇杆，其中temp_old其实是不需要的
void RC_Receive_Anl(void)
{
   int16_t temp;
  static uint8_t i  = 0;
  static int16_t temp_old = 0;
  i ++;
  Rc_Data.YAW = RX_Data.YAW;
  Rc_Data.PITCH = RX_Data.PITCH;
  Rc_Data.ROLL  = RX_Data.ROLL;

  temp = RX_Data.THROTTLE - 1500;
   Rc_Data.THROTTLE += (temp /50);
  
 
  Rc_Data.THROTTLE = data_limit(Rc_Data.THROTTLE,2000,1000);
  
}

/*    上锁&解锁函数  */
void Deblocking(void)
{
	 static uint8_t flag=1;
	 static uint16_t time1=0,time2=0;
	 /*               遥控上锁                 */
	 /*     —————————            —————————     */
	 /*    |         |          |         |    */
	 /*    |         |          |         |    */
	 /*    |    \    |          |   /     |    */
	 /*    |     \   |          |  /      |    */
	 /*     —————————            —————————     */
	 /*   油门拉到最低         摇杆推到左上角  */
   if(!ARMED && Rc_Data.PITCH <= 1200 && Rc_Data.ROLL <= 1200 && Rc_Data.THROTTLE <= 1200 &&  Rc_Data.YAW >= 1800)		
	 {  
			time1++; 
	 }	
	 else 
		 time1=0;
	 if(time1>3 && !ARMED) 
	 { 
			ARMED = 1; 
			time1 = 0;

	 }
   /*               遥控解锁                 */
	 /*     —————————            —————————     */
	 /*    |         |          |         |    */
	 /*    |         |          |         |    */
	 /*    |    /    |          |     \   |    */
	 /*    |   /     |          |      \  |    */
	 /*     —————————            —————————     */
	 /*   油门拉到最低         摇杆推到右上角  */
   if(ARMED && Rc_Data.YAW <= 1200 && Rc_Data.PITCH <= 1200 && Rc_Data.THROTTLE <= 1200 &&  Rc_Data.ROLL >=1800)
		{
			time2++; 
			
		}	
	 else 
		 time2=0;
	 if(time2>=3 && ARMED)
	 {
		  ARMED = 0; 
			time2 = 0;

	 }

}
/***************************

mode = 0;   未知
mode = 1,		姿态
mode = 2;		定高
mode = 3;		定点


*****************************/
void mode_contrl(void)
{
	if(Rc_Data.AUX1>1700)
	{
		mode = 2 ;  //定高	
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







