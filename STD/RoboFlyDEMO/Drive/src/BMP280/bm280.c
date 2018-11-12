#include "bm280.h"
#include "myiic.h"
#include "delay.h"                                                             //调用了延时函数
#include "usart.h"
#include "string.h"
#include <math.h>

static bmp280_t p_bmp280;                                                      //校准参数

//IIC写一个字节 
//devaddr:器件IIC地址
//reg:寄存器地址
//data:数据
//返回值:0,正常
//    其他,错误代码
u8 BMP280_Write_Byte(u8 addr,u8 reg,u8 data)
{
    IIC_Start();
    IIC_Send_Byte((addr<<1)|0); //发送器件地址+写命令
    if(IIC_Wait_Ack())          //等待应答
    {
        IIC_Stop();
        return 1;
    }
    IIC_Send_Byte(reg);         //写寄存器地址
    IIC_Wait_Ack();             //等待应答
    IIC_Send_Byte(data);        //发送数据
    if(IIC_Wait_Ack())          //等待ACK
    {
        IIC_Stop();
        return 1;
    }
    IIC_Stop();
    return 0;
}

//IIC读一个字节 
//reg:寄存器地址 
//返回值:读到的数据
u8 BMP280_Read_Byte(u8 addr,u8 reg)
{
    u8 res;
    IIC_Start();
    IIC_Send_Byte((addr<<1)|0); //发送器件地址+写命令
    IIC_Wait_Ack();             //等待应答
    IIC_Send_Byte(reg);         //写寄存器地址
    IIC_Wait_Ack();             //等待应答
	IIC_Start();                
    IIC_Send_Byte((addr<<1)|1); //发送器件地址+读命令
    IIC_Wait_Ack();             //等待应答
    res=IIC_Read_Byte(0);		    //读数据,发送nACK  
    IIC_Stop();                 //产生一个停止条件
    return res;  
}

//IIC连续读
//addr:器件地址
//reg:要读取的寄存器地址
//len:要读取的长度
//buf:读取到的数据存储区
//返回值:0,正常
//    其他,错误代码
u8 BMP280_Read_Len(u8 addr,u8 reg,u8 len,u8 *buf)
{ 
    IIC_Start();
    IIC_Send_Byte((addr<<1)|0); //发送器件地址+写命令
    if(IIC_Wait_Ack())          //等待应答
    {
        IIC_Stop();
        return 1;
    }
    IIC_Send_Byte(reg);         //写寄存器地址
    IIC_Wait_Ack();             //等待应答
	  IIC_Start();                
    IIC_Send_Byte((addr<<1)|1); //发送器件地址+读命令
    IIC_Wait_Ack();             //等待应答
    while(len)
    {
        if(len==1)*buf=IIC_Read_Byte(0);//读数据,发送nACK 
				else *buf=IIC_Read_Byte(1);		//读数据,发送ACK  
				len--;
				buf++;  
    }
    IIC_Stop();                 //产生一个停止条件
    return 0;       
}

/*
  * @brief  BMP280相关IO口初始化
  * @param  None 
  * @retval None
*/
void BMP280_GPIO_Init(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB|RCC_APB2Periph_AFIO,ENABLE); 
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable,ENABLE);                    //失能JTAG
	/*CSB初始化，防止IIC地址错误*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;				 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD; 		
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 
	GPIO_Init(GPIOB, &GPIO_InitStructure);					 
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4|GPIO_Pin_5;				 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; 		
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 
	GPIO_Init(GPIOB, &GPIO_InitStructure);					
}

/*
  * @brief  检查BMP280连接正常
  * @param  None 
  * @retval 0：正常
  *         1：连接不正常
*/
u8 BMP280_Chack(void)
{
	u16 time=0;
	u8 chip_ID=0;
	while(time<1000)
	{
		chip_ID=BMP280_Read_Byte(BMP280_SlaveAddr,BMP280_CHIPID_REG);
		if(chip_ID==0x57||chip_ID==0x58||chip_ID==0x59)
			break;                                                             //检测到芯片
		else time++;
		delay_ms(1);
	}
	if(time==1000)
		return 1;                                                              //未检测到芯片
	else 
	{
		p_bmp280.chip_id=chip_ID;                                              //记录芯片ID
		return 0;
	}
}

/*
  * @brief  BMP280软件复位，原因不明，函数使用之后芯片无法使用。
  * @param  None 
  * @retval 0：正常
  *         1：连接不正常
*/
u8 BMP280_SetSoftReset(void)
{
	if(BMP280_Write_Byte(BMP280_SlaveAddr,BMP280_RESET_REG,BMP280_RESET_VALUE))
		return 1;
	else return 0;
}

/*
  * @brief  BMP280初始化
  * @param  None 
  * @retval 0：正常
  *         1：连接不正常
*/
u8 BMP280_Init(void)
{
	IIC_Init();
	if(BMP280_Chack())
		return 1;                                                              //BMP280检测芯片
	else
	{
//		if(BMP280_SetSoftReset())
//			return 2;                                                          //软件复位使用不了，使用之后用不了
		if(BMP280_CalibParam())
			return 3;
		if(BMP280_SetPowerMode(BMP280_NORMAL_MODE))
			return 4;
		if(BMP280_SetWorkMode(BMP280_ULTRA_LOW_POWER_MODE))
			return 5;
		if(BMP280_SetStandbyDurn(BMP280_T_SB_0_5MS))
			return 6;
	}
	return 0;
}

/*
  * @brief  BMP280校准参数
  * @param  None 
  * @retval 0：正常
  *         1：连接不正常
*/
u8 BMP280_CalibParam(void)
{
	u8 a_data_u8[24],res=0;
	memset(a_data_u8,0,24*sizeof(u8));
	res=BMP280_Read_Len(BMP280_SlaveAddr,BMP280_DIG_T1_LSB_REG,24,a_data_u8);
	p_bmp280.calib_param.dig_T1=(u16)((((u16)((u8)a_data_u8[1]))<<8)|a_data_u8[0]);
	p_bmp280.calib_param.dig_T2=(s16)((((s16)((s8)a_data_u8[3]))<<8)|a_data_u8[2]);
	p_bmp280.calib_param.dig_T3=(s16)((((s16)((s8)a_data_u8[5]))<<8)|a_data_u8[4]);
	p_bmp280.calib_param.dig_P1=(u16)((((u16)((u8)a_data_u8[7]))<<8)|a_data_u8[6]);
	p_bmp280.calib_param.dig_P2=(s16)((((s16)((s8)a_data_u8[9]))<<8)|a_data_u8[8]);
	p_bmp280.calib_param.dig_P3=(s16)((((s16)((s8)a_data_u8[11]))<<8)|a_data_u8[10]);
	p_bmp280.calib_param.dig_P4=(s16)((((s16)((s8)a_data_u8[13]))<<8)|a_data_u8[12]);
	p_bmp280.calib_param.dig_P5=(s16)((((s16)((s8)a_data_u8[15]))<<8)|a_data_u8[14]);
	p_bmp280.calib_param.dig_P6=(s16)((((s16)((s8)a_data_u8[17]))<<8)|a_data_u8[16]);
	p_bmp280.calib_param.dig_P7=(s16)((((s16)((s8)a_data_u8[19]))<<8)|a_data_u8[18]);
	p_bmp280.calib_param.dig_P8=(s16)((((s16)((s8)a_data_u8[21]))<<8)|a_data_u8[20]);
	p_bmp280.calib_param.dig_P9=(s16)((((s16)((s8)a_data_u8[23]))<<8)|a_data_u8[22]);
	return res;
}

/*
  * @brief  设置BMP280电源工作模式
  * @param  mode：0,1,2,3 ，
    0：SLEEP_MODE，休眠模式
    1OR2：FORCED_MODE，读取一次后进入SLEEP_MODE.
    3：正常工作模式
  * @retval 0：正常
  *         1：连接不正常
  *         2：参数错误
*/
u8 BMP280_SetPowerMode(u8 mode)
{
	u8 v_mode_u8=0,res=0;
	if (mode<=BMP280_NORMAL_MODE) 
	{
		v_mode_u8=(p_bmp280.oversamp_temperature<<5)+(p_bmp280.oversamp_pressure<<2)+mode;
		res=BMP280_Write_Byte(BMP280_SlaveAddr,BMP280_CTRLMEAS_REG,v_mode_u8);
		}
	else res=2;
		return res;
}

/*
  * @brief  设置BMP280过采样模式设置,可以自己增加模式
  * @param  mode：
					BMP280_ULTRA_LOW_POWER_MODE    ,
					BMP280_LOW_POWER_MODE          ,
					BMP280_STANDARD_RESOLUTION_MODE,
					BMP280_HIGH_RESOLUTION_MODE    ,
					BMP280_ULTRA_HIGH_RESOLUTION_MODE
  * @retval 0：正常
  *         1：连接不正常
*/
u8 BMP280_SetWorkMode(WORKING_MODE mode)
{
	u8 res=0,v_data_u8=0;
	if (mode<=0x04) 
	{
		v_data_u8=BMP280_Read_Byte(BMP280_SlaveAddr,BMP280_CTRLMEAS_REG);//读取出控制寄存器的值
		switch(mode)
		{
			case BMP280_ULTRA_LOW_POWER_MODE:
				p_bmp280.oversamp_temperature=BMP280_P_MODE_x1;
			p_bmp280.oversamp_pressure=BMP280_P_MODE_x1;
			break;
			
			case BMP280_LOW_POWER_MODE:
				p_bmp280.oversamp_temperature=BMP280_P_MODE_x1;
			p_bmp280.oversamp_pressure=BMP280_P_MODE_x2;
			break;
			
			case BMP280_STANDARD_RESOLUTION_MODE:
				p_bmp280.oversamp_temperature=BMP280_P_MODE_x1;
			p_bmp280.oversamp_pressure=BMP280_P_MODE_x4;				
			break;
			
			case BMP280_HIGH_RESOLUTION_MODE:
				p_bmp280.oversamp_temperature=BMP280_P_MODE_x1;
			p_bmp280.oversamp_pressure=BMP280_P_MODE_x8;
			break;
			
			case BMP280_ULTRA_HIGH_RESOLUTION_MODE:
				p_bmp280.oversamp_temperature=BMP280_P_MODE_x2;
			p_bmp280.oversamp_pressure=BMP280_P_MODE_x16;
			break;
		}
		v_data_u8=((v_data_u8&~0xE0)|((p_bmp280.oversamp_temperature<<5)&0xE0));
		v_data_u8=((v_data_u8&~0x1C)|((p_bmp280.oversamp_pressure<<2)&0x1C));
		res=BMP280_Write_Byte(BMP280_SlaveAddr,BMP280_CTRLMEAS_REG,v_data_u8);
	} 
	else res=1;
	return res;
}

/*
  * @brief  闲置时长设置，即两次获取温度和气压的间隔时间长度
  * @param  standby_durn：
  *  BMP280_T_SB_0_5MS              ：0.5ms   
  *  BMP280_T_SB_62_5MS             ：62.5ms  
  *  BMP280_T_SB_125MS              ：125ms   
  *  BMP280_T_SB_250MS              ：250ms   
  *  BMP280_T_SB_500MS              ：500ms   
  *  BMP280_T_SB_1000MS             ：1000ms  
  *  BMP280_T_SB_2000MS             ：2000ms  
  *  BMP280_T_SB_4000MS             ：4000ms 
  * @retval 0：正常
  *         1：不正常
*/

u8 BMP280_SetStandbyDurn(BMP280_T_SB standby_durn)
{
	u8 v_data_u8=0;
	v_data_u8=BMP280_Read_Byte(BMP280_SlaveAddr,BMP280_CONFIG_REG);                             //读取出寄存器的值
	v_data_u8=((v_data_u8&~0xE0)|((standby_durn<<5)&0xE0));                    //高3位
	return BMP280_Write_Byte(BMP280_SlaveAddr,BMP280_CONFIG_REG,v_data_u8);
}

/*
  * @brief  获取闲置时长，即两次获取温度和气压的间隔时间长度
  * @param  v_standby_durn_u8：
  *  BMP280_T_SB_0_5MS              ：0.5ms   
  *  BMP280_T_SB_62_5MS             ：62.5ms  
  *  BMP280_T_SB_125MS              ：125ms   
  *  BMP280_T_SB_250MS              ：250ms   
  *  BMP280_T_SB_500MS              ：500ms   
  *  BMP280_T_SB_1000MS             ：1000ms  
  *  BMP280_T_SB_2000MS             ：2000ms  
  *  BMP280_T_SB_4000MS             ：4000ms 
  * @retval 0：正常
  *         1：不正常
*/
u8 BMP280_GetStandbyDurn(u8* v_standby_durn_u8)
{
	u8 res=0,v_data_u8=0;
	res=v_data_u8=BMP280_Read_Byte(BMP280_SlaveAddr,BMP280_CONFIG_REG);
	*v_standby_durn_u8=(v_data_u8>>5);
	return res;
}

/*
  * @brief  获取未补偿温度
  * @param  un_temp：数据指针
  * @retval 0：正常
  *         1：不正常
*/
u8 BMP280_ReadUncompTemperature(s32* un_temp)
{
	u8 a_data_u8r[3]={0,0,0},res=0;
	res=BMP280_Read_Len(BMP280_SlaveAddr,BMP280_TEMPERATURE_MSB_REG,3,a_data_u8r);
	*un_temp=(s32)((((u32)(a_data_u8r[0]))<<12)|(((u32)(a_data_u8r[1]))<<4)|((u32)a_data_u8r[2]>>4));
	return res;
}

/*
  * @brief  获取未补偿气压
  * @param  un_temp：数据指针
  * @retval 0：正常
  *         1：不正常
*/
u8 BMP280_ReadUncompPressuree(s32 *un_press)
{
	u8 a_data_u8r[3]={0,0,0},res = 0;
	res=BMP280_Read_Len(BMP280_SlaveAddr,BMP280_PRESSURE_MSB_REG,3,a_data_u8r);
	*un_press=(s32)((((u32)(a_data_u8r[0]))<<12)|(((u32)(a_data_u8r[1]))<<4)|((u32)a_data_u8r[2]>>4));
	return res;
}

/*
  * @brief  获取未补偿气压和温度（一起获取，一次读取6个字节数据，比分开读取速度快一丢丢）
  * @param  un_press：未补偿气压数据指针，un_temp：未补偿温度数据指针
  * @retval 0：正常
  *         1：不正常
*/
u8 BMP280_ReadUncompPressureTemperature(s32 *un_press,s32 *un_temp)
{
	u8 a_data_u8[6]={0,0,0,0,0,0},res = 0;
	res=BMP280_Read_Len(BMP280_SlaveAddr,BMP280_PRESSURE_MSB_REG,6,a_data_u8);
	*un_press=(s32)((((u32)(a_data_u8[0]))<<12)|(((u32)(a_data_u8[1]))<<4)|((u32)a_data_u8[2]>>4));/*气压*/
	*un_temp=(s32)((((u32)(a_data_u8[3]))<<12)| (((u32)(a_data_u8[4]))<<4)|((u32)a_data_u8[5]>>4));/* 温度 */
	return res;
}

/*
  * @brief  获取真实的气压
  * @param  un_temp：未补偿温度数据
  * @retval s32：温度值，例如：2255代表22.55 DegC
  *        
*/
s32 BMP280_CompensateTemperatureInt32(s32 un_temp)
{
	s32 v_x1_u32r=0;
	s32 v_x2_u32r=0;
	s32 temperature=0;
	v_x1_u32r=((((un_temp>>3)-((s32)p_bmp280.calib_param.dig_T1<<1)))*((s32)p_bmp280.calib_param.dig_T2))>>11;
	v_x2_u32r=(((((un_temp>>4)-((s32)p_bmp280.calib_param.dig_T1))*((un_temp>>4)-((s32)p_bmp280.calib_param.dig_T1)))>>12)*((s32)p_bmp280.calib_param.dig_T3))>>14;
	p_bmp280.calib_param.t_fine=v_x1_u32r+v_x2_u32r;
	temperature=(p_bmp280.calib_param.t_fine*5+128)>> 8;
	return temperature;
}

/*
  * @brief  获取真实气压
  * @param  un_press：未补偿气压
  * @retval u32：真实的气压值   
*/
u32 BMP280_CompensatePressureInt32(s32 un_press)
{
	s32 v_x1_u32r=0;
	s32 v_x2_u32r=0;
	u32 v_pressure_u32=0;
	v_x1_u32r=(((s32)p_bmp280.calib_param.t_fine)>>1)-(s32)64000;
	v_x2_u32r=(((v_x1_u32r>>2)* (v_x1_u32r>>2))>>11)*((s32)p_bmp280.calib_param.dig_P6);
	v_x2_u32r=v_x2_u32r+((v_x1_u32r *((s32)p_bmp280.calib_param.dig_P5))<< 1);
	v_x2_u32r=(v_x2_u32r>>2)+(((s32)p_bmp280.calib_param.dig_P4)<<16);
	v_x1_u32r=(((p_bmp280.calib_param.dig_P3*(((v_x1_u32r>>2)*(v_x1_u32r>>2))>>13))>>3)+((((s32)p_bmp280.calib_param.dig_P2)* v_x1_u32r)>>1))>>18;
	v_x1_u32r=((((32768 + v_x1_u32r))* ((s32)p_bmp280.calib_param.dig_P1))>>15);
	v_pressure_u32=(((u32)(((s32)1048576)-un_press)-(v_x2_u32r>>12)))* 3125;
	if(v_pressure_u32<0x80000000)
		if(v_x1_u32r!=0)
			v_pressure_u32=(v_pressure_u32<<1)/((u32)v_x1_u32r);
		else return 0;
	else if (v_x1_u32r!=0)
		v_pressure_u32=(v_pressure_u32/(u32)v_x1_u32r)*2;
	else return 0;
	v_x1_u32r=(((s32)p_bmp280.calib_param.dig_P9)*((s32)(((v_pressure_u32>>3)*(v_pressure_u32>>3))>>3)))>>12;
	v_x2_u32r=(((s32)(v_pressure_u32>>2))*((s32)p_bmp280.calib_param.dig_P8))>>13;
	v_pressure_u32=(u32)((s32)v_pressure_u32+((v_x1_u32r+v_x2_u32r+ p_bmp280.calib_param.dig_P7)>>4));
	return v_pressure_u32;
}

/*
  * @brief  获取真实气压和温度
  * @param  press：真实的气压指针，temp：真实的温度指针
  * @retval 0：正常
  *         1：不正常
*/
u8 BMP280_ReadPressureTemperature(u32 *press,s32 *temp)
{
	s32 un_press=0;
	s32 un_temp=0;
	u8 res=0;
	res=BMP280_ReadUncompPressureTemperature(&un_press,&un_temp);
	
	/* 读取真实的温度值和气压值*/
	*temp=BMP280_CompensateTemperatureInt32(un_temp);
	*press=BMP280_CompensatePressureInt32(un_press);
	return res;
}
