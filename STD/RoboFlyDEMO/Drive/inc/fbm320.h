#ifndef   _FBM320_H
#define   _FBM320_H

#include "stdint.h"


#define FBMAddr  0xD8   //��ѹ�ƴӻ�IIC��ַ

/****************************************��ѹ�ƼĴ�����ַ***********************************************/
#define SPI_CTRL     0x00 //SPIͨ��ʱ���üĴ��� (������ʹ��IIC���� ��Ӧ����Ϊ0x00)
#define FBM_ID       0x6B //FBM320 ��ݼĴ���
#define FBM_COEFF1   0xAA //FBM320 У׼�Ĵ���
#define FBM_COEFF2   0xBB
#define FBM_COEFF3   0xD0
#define FBM_COEFF4   0xF1
#define FBM_RESET    0xE0 //FBM320 ��λ�Ĵ���
#define FBM_CONFIG   0xF4 //FBM320 �����üĴ��� 6:7 OSR�����ȣ���0:5 101110:�¶�ת������/110100:��ѹת������
#define DATA_MSB     0xF6 //FBM320 ���ݼĴ��� 16:23
#define DATA_CSB     0xF7 //FBM320 ���ݼĴ��� 8:15
#define DATA_LSB     0xF8 //FBM320 ���ݼĴ��� 0:7

#define OSR1024  0x00
#define OSR2048  0x40
#define OSR4096  0x80
#define OSR8192  0xC0
#define PRES_CONVERSION     0x34
#define TEMP_CONVERSION     0x2E
#define FBMRESET      0xB6
#define FBMID         0x42

extern float RPFilter;

void FBM320_Init(void);
void FBM320_Check(void);
void FBM320_Init(void);
void FBM320_GetCoeff(void);
void FBM320_Calculate(int32_t UP, int32_t UT);
int32_t Abs_Altitude(int32_t Press);
uint8_t Init_Altitude(void);
void FBM320_GetAltitude(void);


#endif

