#ifndef __rc_H
#define __rc_H	 
#include "sys.h"

typedef struct int16_rcget{
				int16_t ROLL;
				int16_t PITCH;
				int16_t THROTTLE;
				int16_t YAW;
	      int16_t pitch_offset;
	      int16_t roll_offset;
	      int16_t yaw_offset;
				int16_t AUX1;
				int16_t AUX2;
				int16_t AUX3;
				int16_t AUX4;
}T_RC_DATA;

				
extern T_RC_DATA Rc_Data;//1000~2000
extern u16 Rc_Pwm_In[8],Rc_Data_5;

void PWM_IN_Init(void);
void RC_Data_Refine(void);
void Deblocking(void);
void mode_contrl(void);
void Rc_AUX(void);

				
#endif




