#include "IIC.h"
#include "delay.h"


/********************************IIC基础函数*****************************************************/
//初始化IIC
void IIC_Init(void)
{			

    
    GPIO_InitTypeDef GPIO_InitStructure;
	RCC_AHBPeriphClockCmd(	RCC_AHBPeriph_GPIOB, ENABLE );	//使能GPIOB时钟
	   
	GPIO_InitStructure.GPIO_Pin = SCL|SDA;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT ;   //推挽输出
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_Level_3;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_SetBits(GPIOB,SCL|SDA); 	//PB6,PB7 输出高
}
//产生IIC起始信号
uint16_t IIC_Start(void)
{
	SDA_OUT();     //sda线输出
	SDA_H;	  	  
	SCL_H;
	delay_us(4);
 	SDA_IN();
    if(!SDA_READ) 
        return 1;	//SDA线为低电平则总线忙,退出
    SDA_OUT();
	SDA_L;
	delay_us(1);
	return 0;
}	  
//产生IIC停止信号
void IIC_Stop(void)
{
	SDA_OUT();//sda线输出
    SCL_L;
    SDA_L;
 	delay_us(1);
    SCL_H;
	delay_us(1);
    SDA_H;
	delay_us(4);						   	
}
//等待应答信号到来
//返回值：1，接收应答失败
//        0，接收应答成功
uint8_t IIC_Wait_Ack(void)
{
	uint8_t ucErrTime=0;
    SCL_L;
	SDA_OUT();      //SDA设置为输入  
    SDA_H; 
    delay_us(1);    
    SCL_H;
    delay_us(1);
    SDA_IN();	 
	while(SDA_READ)
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
            SDA_OUT();
			IIC_Stop();
			return 1;
		}
	}
    SDA_OUT();
  SCL_L;   
	return 0;  
} 
//产生ACK应答
void IIC_Ack(void)
{
	SCL_L;
	SDA_OUT();
    SDA_L;
	delay_us(2);
    SCL_H;
	delay_us(2);
    SCL_L;
	delay_us(2);
}
//不产生ACK应答		    
void IIC_NAck(void)
{
	SCL_L;
	delay_us(2);
	SDA_OUT();
    SDA_H;
	delay_us(2);
	SCL_H;
	delay_us(2);
	SCL_L;
}	

//IIC发送一个字节
//返回从机有无应答
//1，有应答
//0，无应答			  
void IIC_Send_Byte(uint8_t txd)
{                        
    uint8_t t;   
	SDA_OUT(); 	    
    SCL_L;//拉低时钟开始数据传输
    for(t=0;t<8;t++)
    {              
        if(txd&0x80)
        {
            SDA_H;
        }
        else SDA_L;
        txd<<=1; 	  
		delay_us(2);   //对TEA5767这三个延时都是必须的
		SCL_H;
		delay_us(2); 
		SCL_L;	
		delay_us(2);
    }	 
} 

//读1个字节，ack=1时，发送ACK，ack=0，发送nACK   
uint8_t IIC_Read_Byte(void)
{
	unsigned char i,receive=0;
	SDA_IN();//SDA设置为输入
    SDA_H;
    for(i=0;i<8;i++ )
	{
        SCL_L; 
        delay_us(2);
	    SCL_H;
        receive<<=1;
        if(SDA_READ)
					receive++;   
		delay_us(1); 
    }					 

    return receive;
}

/*********************************额外IIC函数****************************************************/


/************************************************************   
* 函数名:IIC_ADD_write   
* 描述 : 向特定设备id的特定地址，写入字节 
* 输入  :设备id，内部地址，数据    
* 输出  :无    
*/
void IIC_ADD_write(uint8_t DeviceAddr,uint8_t address,uint8_t Bytes)
{
	IIC_Start();
	IIC_Send_Byte(DeviceAddr);
	IIC_Wait_Ack();
	IIC_Send_Byte(address);   //发送低地址
	IIC_Wait_Ack(); 	 										  		   
	IIC_Send_Byte(Bytes);     //发送字节							   
	IIC_Wait_Ack();  		    	   
    IIC_Stop();//产生一个停止条件 
	delay_us(10);	
}

/************************************************************   
* 函数名:I2C_ReadByte   
* 描述 : 从特定设备id的特定地址读取内容
* 输入  :设备id，内部地址   
* 输出  :读取的内容   
*/
uint8_t IIC_ADD_read(uint8_t DeviceAddr,uint8_t address)
{
	  unsigned char temp;
   IIC_Start();
   IIC_Send_Byte(DeviceAddr);
   IIC_Wait_Ack();
 
    IIC_Send_Byte(address);   //发送低地址
	IIC_Wait_Ack();	    
	IIC_Start();  	 	   
	IIC_Send_Byte(DeviceAddr+1);           //进入接收模式			   
	IIC_Wait_Ack();	 
    temp=IIC_Read_Byte();		

    IIC_Stop();//产生一个停止条件	    
	return temp;
}

/************************************************************   
* 函数名:IIC_Read_MultiBytes   
* 描述 : 从特定设备id的特定地址读取多个字节
* 输入  :设备id，内部地址，需要读的字节个数（0不读取）
* 输出  :读取的内容   
*************************************************************/
uint8_t IIC_Read_MultiBytes(uint8_t DeviceAddr,uint8_t address,uint8_t Len,uint8_t *data)
{

   IIC_Start();
   IIC_Send_Byte(DeviceAddr);
   IIC_Wait_Ack();
 
    IIC_Send_Byte(address);   //发送低地址
	IIC_Wait_Ack();	    
	IIC_Start();  	 	   
	IIC_Send_Byte(DeviceAddr+1);           //进入接收模式			   
	IIC_Wait_Ack();	 
    for (;Len>0;Len--)
    {
        if(Len != 1)
        {
            *data = IIC_Read_Byte();
            IIC_Ack();
        }            
        else 
        {
            *data = IIC_Read_Byte();
            IIC_NAck();
        }

        data ++;
    }
    IIC_Stop();//产生一个停止条件	 
	return 0;
}


/************************************************************   
* 函数名:IIC_NoAddr_WriteByte   
* 描述 : 向特定设备id，写入字节 
* 输入  :设备id，内容   
* 输出  :无    
*/
void IIC_NoAddr_WriteByte(unsigned char address,unsigned char Bytes)
{

	IIC_Start();
	IIC_Send_Byte(address);   //发送低地址
	IIC_Wait_Ack(); 	 										  		   
	IIC_Send_Byte(Bytes);     //发送字节							   
	IIC_Wait_Ack();  		    	   
    IIC_Stop();//产生一个停止条件 
	delay_ms(10);

}

//IIC连续写
//addr:器件地址 
//reg:寄存器地址
//len:写入长度
//buf:数据区
//返回值:0,正常
//    其他,错误代码
uint8_t MPU_Write_Len(uint8_t addr,uint8_t reg,uint8_t len,uint8_t *buf)
{
	uint8_t i; 
    IIC_Start(); 
	IIC_Send_Byte((addr<<1)|0);//发送器件地址+写命令	
	if(IIC_Wait_Ack())	//等待应答
	{
		IIC_Stop();		 
		return 1;		
	}
    IIC_Send_Byte(reg);	//写寄存器地址
    IIC_Wait_Ack();		//等待应答
	for(i=0;i<len;i++)
	{
		IIC_Send_Byte(buf[i]);	//发送数据
		if(IIC_Wait_Ack())		//等待ACK
		{
			IIC_Stop();	 
			return 1;		 
		}		
	}    
    IIC_Stop();	 
	return 0;	
} 
//IIC连续读
//addr:器件地址
//reg:要读取的寄存器地址
//len:要读取的长度
//buf:读取到的数据存储区
//返回值:0,正常
//    其他,错误代码
uint8_t MPU_Read_Len(uint8_t addr,uint8_t reg,uint8_t len,uint8_t *buf)
{ 
 	IIC_Start(); 
	IIC_Send_Byte((addr<<1)|0);//发送器件地址+写命令	
	if(IIC_Wait_Ack())	//等待应答
	{
		IIC_Stop();		 
		return 1;		
	}
    IIC_Send_Byte(reg);	//写寄存器地址
    IIC_Wait_Ack();		//等待应答
    IIC_Start();
	IIC_Send_Byte((addr<<1)|1);//发送器件地址+读命令	
    IIC_Wait_Ack();		//等待应答 
	while(len)
	{
		if(len==1)
        {
            *buf=IIC_Read_Byte();//读数据,发送nACK 
            IIC_NAck();
        }
		else 
        {
            *buf=IIC_Read_Byte();		//读数据,发送ACK  
            IIC_Ack(); //发送ACK 
        }            
		len--;
		buf++; 
	}    
    IIC_Stop();	//产生一个停止条件 
	return 0;	
}
/***************************************************************/











