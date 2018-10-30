
#include "iic_mpu6050.h"








uint16_t MPU6050_ADDRESS;

/*
 * 函数名：I2C_GPIO_Config
 * 描述  ：I2C1 I/O配置
 * 输入  ：无
 * 输出  ：无
 * 调用  ：内部调用
 */
static void I2C_GPIO_Config(void)
{
  	GPIO_InitTypeDef  GPIO_InitStructure; 

	/* 使能与 I2C1 有关的时钟 */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_FSMC, DISABLE);
  	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1,ENABLE);  
    
  /* PB6-I2C1_SCL、PB7-I2C1_SDA*/
  	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_6 | GPIO_Pin_7;
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;	       // 开漏输出
  	GPIO_Init(GPIOB, &GPIO_InitStructure);
}



/*
 * 函数名：I2C_Configuration
 * 描述  ：I2C 工作模式配置
 * 输入  ：无
 * 输出  ：无
 * 调用  ：内部调用
 */
static void I2C_Mode_Configu(void)
{
  I2C_InitTypeDef  I2C_InitStructure; 

  /* I2C 配置 */
  I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
  I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
  I2C_InitStructure.I2C_OwnAddress1 =I2C1_OWN_ADDRESS7; 
  I2C_InitStructure.I2C_Ack = I2C_Ack_Enable ;
  I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
  I2C_InitStructure.I2C_ClockSpeed = I2C_Speed;
  
  /* 使能 I2C1 */
  I2C_Cmd(I2C1, ENABLE);

  /* I2C1 初始化 */
  I2C_Init(I2C1, &I2C_InitStructure);

}




/*
 * 函数名：I2C_MPU_ByteWrite
 * 描述  ：写一个字节到I2C MPU6050中
 * 输入  ：-pBuffer 缓冲区数据
 *         -WriteAddr 接收数据的MPU6050的地址 
 * 输出  ：无
 * 返回  ：无
 * 调用  ：外部调用
 */
void I2C_MPU_ByteWrite(u8 pBuffer, u8 WriteAddr)
{
  /* Send STRAT condition */
  I2C_GenerateSTART(I2C1, ENABLE);

  /* Test on EV5 and clear it */
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));  

  /* Send MPU6050 address for write */
  I2C_Send7bitAddress(I2C1, MPU6050_ADDRESS, I2C_Direction_Transmitter);
  
  /* Test on EV6 and clear it */
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
      
  /* Send the MPU6050's internal address to write to */
  I2C_SendData(I2C1, WriteAddr);
  
  /* Test on EV8 and clear it */
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

  /* Send the byte to be written */
  I2C_SendData(I2C1, pBuffer); 
   
  /* Test on EV8 and clear it */
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
  
  /* Send STOP condition */
  I2C_GenerateSTOP(I2C1, ENABLE);
}


/*
 * 函数名：I2C_MPU_BufferRead
 * 描述  ：从MPU6050里面读取一块数据。 
 * 输入  ：-pBuffer 存放从MPU6050读取的数据的缓冲区指针。
 *         -WriteAddr 接收数据的MPU6050的地址。 
 *         -NumByteToWrite 要从MPU6050读取的字节数。
 * 输出  ：无
 * 返回  ：无
 * 调用  ：外部调用
 */
u8 I2C_MPU_BufferRead( u8 ReadAddr)
{  
	u8 pBuffer;				   
  //*((u8 *)0x4001080c) |=0x80; 
    while(I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY)); // Added by Najoua 27/08/2008
    
    
  /* Send START condition */
  I2C_GenerateSTART(I2C1, ENABLE);
  //*((u8 *)0x4001080c) &=~0x80;
  
  /* Test on EV5 and clear it */
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));

  /* Send MPU6050 address for write */
  I2C_Send7bitAddress(I2C1, MPU6050_ADDRESS, I2C_Direction_Transmitter);

  /* Test on EV6 and clear it */
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
  
  /* Clear EV6 by setting again the PE bit */
  I2C_Cmd(I2C1, ENABLE);

  /* Send the MPU6050's internal address to write to */
  I2C_SendData(I2C1, ReadAddr);  

  /* Test on EV8 and clear it */
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
  
  /* Send STRAT condition a second time */  
  I2C_GenerateSTART(I2C1, ENABLE);
  
  /* Test on EV5 and clear it */
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));
  
  /* Send MPU6050 address for read */
  I2C_Send7bitAddress(I2C1, MPU6050_ADDRESS, I2C_Direction_Receiver);
  
  /* Test on EV6 and clear it */
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));
  
  /* While there is data to be read */

      /* Disable Acknowledgement */
      I2C_AcknowledgeConfig(I2C1, DISABLE);
      
      /* Send STOP Condition */
      I2C_GenerateSTOP(I2C1, ENABLE);

    /* Test on EV7 and clear it */
    if(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED))  
    {      
      /* Read a byte from the MPU6050 */
      pBuffer = I2C_ReceiveData(I2C1); 
    }   

  /* Enable Acknowledgement to be ready for another reception */
  I2C_AcknowledgeConfig(I2C1, ENABLE);
  return pBuffer;
}

/*
 * 函数名：I2C_MPU_Init
 * 描述  ：I2C 外设(MPU6050)初始化
 * 输入  ：无
 * 输出  ：无
 * 调用  ：外部调用
 */
void I2C_MPU_Init(void)
{

  I2C_GPIO_Config(); 
 
  I2C_Mode_Configu();

/* 根据头文件i2c_ee.h中的定义来选择EEPROM要写入的地址 */
#ifdef MPU6050_Block0_ADDRESS
  /* 选择 MPU6050 Block0 来写入 */
  MPU6050_ADDRESS = MPU6050_Block0_ADDRESS;
#endif

#ifdef MPU6050_Block1_ADDRESS  
	/* 选择 MPU6050 Block1 来写入 */
  MPU6050_ADDRESS = MPU6050_Block1_ADDRESS;
#endif
	
	I2C_MPU_ByteWrite(0x00,PWR_MGMT_1);
	I2C_MPU_ByteWrite(0x07,SMPLRT_DIV);
	I2C_MPU_ByteWrite(0x06,CONFIG);
	I2C_MPU_ByteWrite(0x18,GYRO_CONFIG);
	I2C_MPU_ByteWrite(0x01,ACCEL_CONFIG);
}

//**************************************
//合成数据
//**************************************
int mpu6050_get(u8 REG_Address)
{
	u16 H;
	u8 L;
	H=I2C_MPU_BufferRead(REG_Address);
	L=I2C_MPU_BufferRead(REG_Address+1);
	return (H<<8)+L;   //合成数据
}


/*
 * 函数名：I2C_GPIO_Config
 * 描述  ：I2C1 I/O配置
 * 输入  ：无
 * 输出  ：无
 * 调用  ：内部调用
 */
static void I2C_GPIO_Config_mn(void)
{
  	GPIO_InitTypeDef  GPIO_InitStructure; 

	/* 使能与 I2C1 有关的时钟 */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
    
  /* PB6-I2C1_SCL、PB7-I2C1_SDA*/
  	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_6 | GPIO_Pin_7;
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;	       // 开漏输出
  	GPIO_Init(GPIOB, &GPIO_InitStructure);
}




