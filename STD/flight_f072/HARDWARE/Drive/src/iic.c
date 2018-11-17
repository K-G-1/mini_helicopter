/*******************************************************************************************
										    声 明
    本项目代码仅供个人学习使用，可以自由移植修改，但必须保留此声明信息。移植过程中出现其他
	
不可估量的BUG，天际智联不负任何责任。请勿商用！

* 程序版本：V1.01
* 程序日期：2018-8-18
* 程序作者：愤怒的小孩
* 版权所有：西安天际智联信息技术有限公司
*******************************************************************************************/
#include "stm32f0xx.h"
#include "iic.h"
#include "delay.h"


////////////////////////////////////////// 移植须知！//////////////////////////////////////
//
// 移植此模拟IIC时，需要注意 IIC_WriteByteToSlave() 写从器件的地址时，这种方式的地址是已经
// 左移过1位的，注意与 ( I2C_Addr <<1 | 0 )的方式进行区分，否则不能与从器件通信,谨慎移植!
//
///////////////////////////////////////////////////////////////////////////////////////////


/******************************************************************************
* 函  数：void IIC_GPIO_Init(void)
* 功　能：模拟IIC引脚初始化
* 参  数：无
* 返回值：无
* 备  注：PB6->SCL	PB7->SDA
*******************************************************************************/
void IIC_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  RCC_AHBPeriphClockCmd(	RCC_AHBPeriph_GPIOB, ENABLE );	//使能GPIOB时钟
   
  GPIO_InitStructure.GPIO_Pin = SCL|SDA;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT ;   //推挽输出
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  GPIO_SetBits(GPIOB,SCL|SDA); 	//PB6,PB7 输出高
}

/******************************************************************************
* 函  数：void SDA_OUT(void)
* 功　能：配置模拟IIC SDA引脚为输出
* 参  数：无
* 返回值：无
* 备  注：无
*******************************************************************************/
void SDA_OUT(void)
{
//	GPIO_InitTypeDef  GPIO_InitStructure;
//	
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
//	
//	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_7 ;
//	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
//    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//	GPIO_Init(GPIOB, &GPIO_InitStructure);
  
  
//  GPIOB->CRL&=0X0FFFFFFF;GPIOB->CRL|=(uint32_t)1<<28;
}

/******************************************************************************
* 函  数：void SDA_IN(void)
* 功　能：配置模拟IIC SDA引脚为输入
* 参  数：无
* 返回值：无
* 备  注：无
*******************************************************************************/
void SDA_IN(void)
{
//	GPIO_InitTypeDef  GPIO_InitStructure;
//	
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
//	
//	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_7 ;
//	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN_FLOATING;
//	GPIO_Init(GPIOB, &GPIO_InitStructure);
//  GPIOB->CRL&=0X0FFFFFFF;GPIOB->CRL|=(uint32_t)8<<28;
}

/******************************************************************************
* 函  数：void IIC_Start(void)
* 功　能：产生IIC起始信号
* 参  数：无
* 返回值：无
* 备  注：无
*******************************************************************************/	
void IIC_Start(void)
{
	SDA_OUT();     //sda线输出
	SDA_H;	  	  
	SCL_H;
	Delay_us(4);
 	SDA_IN();
    if(!SDA_READ) 
        return ;	//SDA线为低电平则总线忙,退出
    SDA_OUT();
	SDA_L;
	Delay_us(1);
	return ;
}

/******************************************************************************
* 函  数：void IIC_Stop(void)
* 功　能：产生IIC停止信号
* 参  数：无
* 返回值：无
* 备  注：无
*******************************************************************************/	  
void IIC_Stop(void)
{
  SDA_OUT();//sda线输出
  SCL_L;
  SDA_L;
  Delay_us(1);
  SCL_H;
  Delay_us(1);
  SDA_H;
  Delay_us(4);						   	
}

/******************************************************************************
* 函  数: uint8_t IIC_WaitAck(void)
* 功　能: 等待应答信号到来 （有效应答：从机第9个 SCL=0 时 SDA 被从机拉低,
*         并且 SCL = 1时 SDA依然为低）
* 参  数：无
* 返回值：1，接收应答失败	0，接收应答成功
* 备  注：从机给主机的应答
*******************************************************************************/
uint8_t IIC_WaitAck(void)
{
  uint8_t ucErrTime=0;
  SCL_L;
  SDA_OUT();      //SDA设置为输入  
  SDA_H; 
  Delay_us(1);    
  SCL_H;
  Delay_us(1);
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
  SCL_L;//时钟输出0 	   
  return 0;  
} 

/******************************************************************************
* 函  数: void IIC_Ack(void)
* 功　能: 产生ACK应答 （主机接收完一个字节数据后，主机产生的ACK通知从机一个
*         字节数据已正确接收）
* 参  数：无
* 返回值：无
* 备  注：主机给从机的应答
*******************************************************************************/
void IIC_Ack(void)
{
  SCL_L;
  SDA_OUT();
  SDA_L;
  Delay_us(2);
  SCL_H;
  Delay_us(2);
  SCL_L;
  Delay_us(2);
}

/******************************************************************************
* 函  数: void IIC_NAck(void)
* 功　能: 产生NACK应答 （主机接收完最后一个字节数据后，主机产生的NACK通知从机
*         发送结束，释放SDA,以便主机产生停止信号）
* 参  数：无
* 返回值：无
* 备  注：主机给从机的应答
*******************************************************************************/
void IIC_NAck(void)
{
	SCL_L;
	Delay_us(2);
	SDA_OUT();
  SDA_H;
	Delay_us(2);
	SCL_H;
	Delay_us(2);
	SCL_L;
}					 				     

/******************************************************************************
* 函  数：void IIC_SendByte(uint8_t data)
* 功  能：IIC发送一个字节
* 参  数：data 要写的数据
* 返回值：无
* 备  注：主机往从机发
*******************************************************************************/		  
void IIC_SendByte(uint8_t data)
{                        
  uint8_t t;   
  SDA_OUT(); 	    
  SCL_L;//拉低时钟开始数据传输
  for(t=0;t<8;t++)
  {              
    if(data&0x80)
    {
        SDA_H;
    }
    else SDA_L;
    data<<=1; 	  
    Delay_us(2);   //对TEA5767这三个延时都是必须的
    SCL_H;
    Delay_us(2); 
    SCL_L;	
    Delay_us(2);
  }	 
} 	 
   
/******************************************************************************
* 函  数：uint8_t IIC_ReadByte(uint8_t ack)
* 功  能：IIC读取一个字节
* 参  数：ack=1 时，主机数据还没接收完 ack=0 时主机数据已全部接收完成
* 返回值：无
* 备  注：从机往主机发
*******************************************************************************/	
uint8_t IIC_ReadByte(uint8_t ack)
{
  unsigned char i,receive=0;
  SDA_IN();//SDA设置为输入
  SDA_H;
  for(i=0;i<8;i++ )
  {
    SCL_L; 
    Delay_us(2);
    SCL_H;
    receive<<=1;
    if(SDA_READ)
      receive++;   
    Delay_us(1); 
  }					 
  if(ack)
    IIC_Ack();
  else 
    IIC_NAck();
  return receive;
}

/******************************************************************************
* 函  数：uint8_t IIC_ReadByteFromSlave(uint8_t I2C_Addr,uint8_t addr)
* 功　能：读取指定设备 指定寄存器的一个值
* 参  数：I2C_Addr  目标设备地址
		  reg	    寄存器地址
          *buf      读取数据要存储的地址    
* 返回值：返回 1失败 0成功
* 备  注：无
*******************************************************************************/ 
uint8_t IIC_ReadByteFromSlave(uint8_t I2C_Addr,uint8_t reg,uint8_t *buf)
{
	IIC_Start();	
	IIC_SendByte(I2C_Addr);	 //发送从机地址
	if(IIC_WaitAck()) //如果从机未应答则数据发送失败
	{
		IIC_Stop();
		return 1;
	}
	IIC_SendByte(reg); //发送寄存器地址
	IIC_WaitAck();	  
	
	IIC_Start();
	IIC_SendByte(I2C_Addr+1); //进入接收模式			   
	IIC_WaitAck();
	*buf=IIC_ReadByte(0);	   
	IIC_Stop(); //产生一个停止条件
	return 0;
}

/*************************************************************************************
* 函  数：uint8_t IIC_WriteByteFromSlave(uint8_t I2C_Addr,uint8_t addr，uint8_t buf))
* 功　能：写入指定设备 指定寄存器的一个值
* 参  数：I2C_Addr  目标设备地址
		  reg	    寄存器地址
          buf       要写入的数据
* 返回值：1 失败 0成功
* 备  注：无
**************************************************************************************/ 
uint8_t IIC_WriteByteToSlave(uint8_t I2C_Addr,uint8_t reg,uint8_t data)
{
	IIC_Start();
	IIC_SendByte(I2C_Addr); //发送从机地址
	if(IIC_WaitAck())
	{
		IIC_Stop();
		return 1; //从机地址写入失败
	}
	IIC_SendByte(reg); //发送寄存器地址
	IIC_WaitAck();	  
	IIC_SendByte(data); 
	if(IIC_WaitAck())
	{
		IIC_Stop(); 
		return 1; //数据写入失败
	}
	IIC_Stop(); //产生一个停止条件
	return 0;
}

/***************************************************************************************
* 函  数：uint8_t IICreadBytes(uint8_t dev, uint8_t reg, uint8_t length, uint8_t *data)
* 功　能：读取指定设备 指定寄存器的 length个值
* 参  数：dev     目标设备地址
		  reg	  寄存器地址
          length  要读的字节数
		  *data   读出的数据将要存放的指针
* 返回值：1成功 0失败
* 备  注：无
***************************************************************************************/ 
uint8_t IIC_ReadMultByteFromSlave(uint8_t dev, uint8_t reg, uint8_t length, uint8_t *data)
{
	uint8_t count = 0;
	uint8_t temp;
	IIC_Start();
	IIC_SendByte(dev); //发送从机地址
	IIC_WaitAck();
  
	IIC_SendByte(reg); //发送寄存器地址
	IIC_WaitAck();	  
	IIC_Start();
	IIC_SendByte(dev+1); //进入接收模式	
	IIC_WaitAck();
	for(count=0;count<length;count++)
	{
		if(count!=(length-1))
			temp = IIC_ReadByte(1); //带ACK的读数据
		else  
			temp = IIC_ReadByte(0); //最后一个字节NACK

		data[count] = temp;
	}
    IIC_Stop(); //产生一个停止条件
	return 0;
}

/****************************************************************************************
* 函  数：uint8_t IICwriteBytes(uint8_t dev, uint8_t reg, uint8_t length, uint8_t* data)
* 功　能：将多个字节写入指定设备 指定寄存器
* 参  数：dev     目标设备地址
		  reg	  寄存器地址
		  length  要写的字节数
		  *data   要写入的数据将要存放的指针
* 返回值：1成功 0失败
* 备  注：无
****************************************************************************************/ 
uint8_t IIC_WriteMultByteToSlave(uint8_t dev, uint8_t reg, uint8_t length, uint8_t* data)
{
 	uint8_t count = 0;
	IIC_Start();
	IIC_SendByte(dev); //发送从机地址
	if(IIC_WaitAck())
	{
		IIC_Stop();
		return 1; //从机地址写入失败
	}
	IIC_SendByte(reg); //发送寄存器地址
	IIC_WaitAck();	  
	for(count=0;count<length;count++)
	{
		IIC_SendByte(data[count]); 
		if(IIC_WaitAck()) //每一个字节都要等从机应答
		{
			IIC_Stop();
			return 1; //数据写入失败
		}
	}
	IIC_Stop(); //产生一个停止条件
	return 0;
}


/************************************************************   
* 函数名:IIC_ADD_write   
* 描述 : 向特定设备id的特定地址，写入字节 
* 输入  :设备id，内部地址，数据    
* 输出  :无    
*/
void IIC_ADD_write(uint8_t DeviceAddr,uint8_t address,uint8_t Bytes)
{
	IIC_Start();
	IIC_SendByte(DeviceAddr);
	IIC_WaitAck();
	IIC_SendByte(address);   //发送低地址
	IIC_WaitAck(); 	 										  		   
	IIC_SendByte(Bytes);     //发送字节							   
	IIC_WaitAck();  		    	   
    IIC_Stop();//产生一个停止条件 
	Delay_us(10);	
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
   IIC_SendByte(DeviceAddr);
   IIC_WaitAck();
 
    IIC_SendByte(address);   //发送低地址
	IIC_WaitAck();	    
	IIC_Start();  	 	   
	IIC_SendByte(DeviceAddr+1);           //进入接收模式			   
	IIC_WaitAck();	 
    temp=IIC_ReadByte(0);		

    IIC_Stop();//产生一个停止条件	    
	return temp;
}
