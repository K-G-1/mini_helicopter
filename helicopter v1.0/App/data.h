#ifndef __DATA_H
#define __DATA_H

#include "stm32f10x.h"
//#include "include.h"

typedef struct
{
	float x;
	float y;
	float z;
} _xyz_f_st;

typedef struct
{
	s16 x;
	s16 y;
	s16 z;
} _xyz_s16_st;

typedef struct
{
	float x;
	float y;
	float z;
}
__attribute__((packed)) _xyz_f_st_pk;

typedef struct
{
	s16 x;
	s16 y;
	s16 z;
}__attribute__((packed)) _xyz_s16_st_pk;

typedef struct
{
	u8 thr_low;
	u8 NS; //信号来源，0：无信号
	u8 signal_loss;
	u8 low_power;
	u8 landed;
	
}_flag_st;

typedef struct
{
	float out_weight;
	float out_weight_slow;

} _global_f_st;
 
extern _flag_st flag;
extern _global_f_st gf;
#endif

