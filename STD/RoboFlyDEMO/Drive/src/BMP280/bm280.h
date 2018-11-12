#ifndef __BM280_H  
#define __BM280_H 

#include "sys.h"

#define BMP280_ADDRESS                       0xEC  
  
#define BMP280_RESET_VALUE                   0xB6  

#define BMP280_DIG_T1_LSB_REG                0x88  
#define BMP280_DIG_T1_MSB_REG                0x89  
#define BMP280_DIG_T2_LSB_REG                0x8A  
#define BMP280_DIG_T2_MSB_REG                0x8B  
#define BMP280_DIG_T3_LSB_REG                0x8C  
#define BMP280_DIG_T3_MSB_REG                0x8D  
#define BMP280_DIG_P1_LSB_REG                0x8E  
#define BMP280_DIG_P1_MSB_REG                0x8F  
#define BMP280_DIG_P2_LSB_REG                0x90  
#define BMP280_DIG_P2_MSB_REG                0x91  
#define BMP280_DIG_P3_LSB_REG                0x92  
#define BMP280_DIG_P3_MSB_REG                0x93  
#define BMP280_DIG_P4_LSB_REG                0x94  
#define BMP280_DIG_P4_MSB_REG                0x95  
#define BMP280_DIG_P5_LSB_REG                0x96  
#define BMP280_DIG_P5_MSB_REG                0x97  
#define BMP280_DIG_P6_LSB_REG                0x98  
#define BMP280_DIG_P6_MSB_REG                0x99  
#define BMP280_DIG_P7_LSB_REG                0x9A  
#define BMP280_DIG_P7_MSB_REG                0x9B  
#define BMP280_DIG_P8_LSB_REG                0x9C  
#define BMP280_DIG_P8_MSB_REG                0x9D  
#define BMP280_DIG_P9_LSB_REG                0x9E  
#define BMP280_DIG_P9_MSB_REG                0x9F  
  
#define BMP280_CHIPID_REG                    0xD0                              /*Chip ID Register */  
#define BMP280_RESET_REG                     0xE0                              /*Softreset Register */  
#define BMP280_STATUS_REG                    0xF3                              /*Status Register */  
#define BMP280_CTRLMEAS_REG                  0xF4                              /*Ctrl Measure Register */  
#define BMP280_CONFIG_REG                    0xF5                              /*Configuration Register */  
#define BMP280_PRESSURE_MSB_REG              0xF7                              /*Pressure MSB Register */  
#define BMP280_PRESSURE_LSB_REG              0xF8                              /*Pressure LSB Register */  
#define BMP280_PRESSURE_XLSB_REG             0xF9                              /*Pressure XLSB Register */  
#define BMP280_TEMPERATURE_MSB_REG           0xFA                              /*Temperature MSB Reg */  
#define BMP280_TEMPERATURE_LSB_REG           0xFB                              /*Temperature LSB Reg */  
#define BMP280_TEMPERATURE_XLSB_REG          0xFC                              /*Temperature XLSB Reg */  
  
/* ��foreced mode�£�1s�Ĳ������ڣ��¶Ⱥ���ѹʹ����͵ľ��Ȳɼ�����ʹ����С���˲���ϵ��,���ݵĲɼ�ʱ������6ms,ƽ������Ϊ3.27uA��*/  
/* ��foreced mode�£�1s�Ĳ�������, �¶Ⱥ���ѹʹ����ߵľ��Ȳɼ�����ʹ�������˲���ϵ��,���ݵĲɼ�ʱ������70ms,ƽ������Ϊ30uA��*/  
  
typedef enum 
{  
    BMP280_T_MODE_SKIP=0x0,                                                    /*skipped*/  
    BMP280_T_MODE_1,                                                           /*x1*/  
    BMP280_T_MODE_2,                                                           /*x2*/  
    BMP280_T_MODE_3,                                                           /*x4*/  
    BMP280_T_MODE_4,                                                           /*x8*/  
    BMP280_T_MODE_5                                                            /*x16*/  
} BMP280_T_OVERSAMPLING;                                                       //������
  
typedef enum
{	
    BMP280_SLEEP_MODE=0x00,  
    BMP280_FORCED_MODE=0x01,  
    BMP280_NORMAL_MODE=0x03  
} BMP280_WORK_MODE;  
  
typedef enum
{  
    BMP280_P_MODE_SKIP=0x0,                                                    /*skipped*/  
    BMP280_P_MODE_x1,                                                          /*x1*/  
    BMP280_P_MODE_x2,                                                          /*x2*/  
    BMP280_P_MODE_x4,                                                          /*x4*/  
    BMP280_P_MODE_x8,                                                          /*x8*/  
    BMP280_P_MODE_x16                                                          /*x16*/  
} BMP280_P_OVERSAMPLING;

/************************************************/
/**@name	          ����ģʽ����
*************************************************/
typedef enum
{
	BMP280_ULTRA_LOW_POWER_MODE=0x00,
	BMP280_LOW_POWER_MODE=0x01,
	BMP280_STANDARD_RESOLUTION_MODE=0x02,
	BMP280_HIGH_RESOLUTION_MODE=0x03,
	BMP280_ULTRA_HIGH_RESOLUTION_MODE=0x04
}WORKING_MODE;

typedef enum 
{  
    BMP280_FILTER_OFF=0x0,                                                     /*filter off*/  
    BMP280_FILTER_MODE_1,                                                      /*0.223*ODR*/  
    BMP280_FILTER_MODE_2,                                                      /*0.092*ODR*/  
    BMP280_FILTER_MODE_3,                                                      /*0.042*ODR*/  
    BMP280_FILTER_MODE_4                                                       /*0.021*ODR*/  
} BMP280_FILTER_COEFFICIENT;  
  
typedef enum
{  
    BMP280_T_SB_0_5MS=0x00,                                                    /*0.5ms */  
    BMP280_T_SB_62_5MS=0x01,                                                   /*62.5ms*/  
    BMP280_T_SB_125MS=0x02,                                                    /*125ms */  
    BMP280_T_SB_250MS=0x03,                                                    /*250ms */  
    BMP280_T_SB_500MS=0x04,                                                    /*500ms */  
    BMP280_T_SB_1000MS=0x05,                                                   /*1000ms*/  
    BMP280_T_SB_2000MS=0x06,                                                   /*2000ms*/  
    BMP280_T_SB_4000MS=0x07,                                                   /*4000ms*/  
} BMP280_T_SB;  

typedef struct
{
	u16 dig_T1;                                                                /*У׼T1����*/
	s16 dig_T2;                                                                /*У׼T2����*/
	s16 dig_T3;                                                                /*У׼T3����*/
	u16 dig_P1;                                                                /*У׼P1����*/
	s16 dig_P2;                                                                /*У׼P2����*/
	s16 dig_P3;                                                                /*У׼P3����*/
	s16 dig_P4;                                                                /*У׼P4����*/
	s16 dig_P5;                                                                /*У׼P5����*/
	s16 dig_P6;                                                                /*У׼P6����*/
	s16 dig_P7;                                                                /*У׼P7����*/
	s16 dig_P8;                                                                /*У׼P8����*/
	s16 dig_P9;                                                                /*У׼P9����*/
	s32 t_fine;                                                                /*У׼t_fine����*/
}bmp280_calib_param_t;

typedef struct
{
	bmp280_calib_param_t calib_param;                                          /*У׼����*/
	u8 chip_id;                                                                /*������ID*/
	u8 dev_addr;                                                               /*������IIC��ַ*/
	u8 oversamp_temperature;                                                   /*�¶Ȳ���*/
	u8 oversamp_pressure;                                                      /*��ѹ����*/
}bmp280_t;

#define BMP280_SlaveAddr 0x76                                                  //BMP280��������ַ

/*����*/
u8  BMP280_Init(void);
u8  BMP280_CalibParam(void);
u8  BMP280_GetMode(void);
u8  BMP280_SetMode(BMP280_WORK_MODE mode);
u8  BMP280_SetPowerMode(u8 mode);
u8  BMP280_SetWorkMode(WORKING_MODE mode);
u8  BMP280_SetStandbyDurn(BMP280_T_SB v_standby_durn_u8);
u8  BMP280_GetStandbyDurn(u8* v_standby_durn_u8);
u8  BMP280_ReadUncompTemperature(s32* un_temp);
u8  BMP280_ReadUncompPressuree(s32 *un_press);
u8  BMP280_ReadUncompPressureTemperature(s32 *un_press, s32 *un_temp);
s32 BMP280_CompensateTemperatureInt32(s32 un_temp);
u32 BMP280_CompensatePressureInt32(s32 un_press);
u8  BMP280_ReadPressureTemperature(u32 *press, s32 *temp);
u8  BMP280_Write_Byte(u8 addr,u8 reg,u8 data);
u8  BMP280_Read_Byte(u8 addr,u8 reg);
u8  BMP280_Read_Len(u8 addr,u8 reg,u8 len,u8 *buf);

#endif  
