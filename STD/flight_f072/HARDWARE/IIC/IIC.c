#include "IIC.h"
#include "delay.h"


/********************************IIC��������*****************************************************/
//��ʼ��IIC
void IIC_Init(void)
{			

    
    GPIO_InitTypeDef GPIO_InitStructure;
	RCC_AHBPeriphClockCmd(	RCC_AHBPeriph_GPIOB, ENABLE );	//ʹ��GPIOBʱ��
	   
	GPIO_InitStructure.GPIO_Pin = SCL|SDA;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT ;   //�������
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_Level_3;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_SetBits(GPIOB,SCL|SDA); 	//PB6,PB7 �����
}
//����IIC��ʼ�ź�
uint16_t IIC_Start(void)
{
	SDA_OUT();     //sda�����
	SDA_H;	  	  
	SCL_H;
	delay_us(4);
 	SDA_IN();
    if(!SDA_READ) 
        return 1;	//SDA��Ϊ�͵�ƽ������æ,�˳�
    SDA_OUT();
	SDA_L;
	delay_us(1);
	return 0;
}	  
//����IICֹͣ�ź�
void IIC_Stop(void)
{
	SDA_OUT();//sda�����
    SCL_L;
    SDA_L;
 	delay_us(1);
    SCL_H;
	delay_us(1);
    SDA_H;
	delay_us(4);						   	
}
//�ȴ�Ӧ���źŵ���
//����ֵ��1������Ӧ��ʧ��
//        0������Ӧ��ɹ�
uint8_t IIC_Wait_Ack(void)
{
	uint8_t ucErrTime=0;
    SCL_L;
	SDA_OUT();      //SDA����Ϊ����  
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
//����ACKӦ��
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
//������ACKӦ��		    
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

//IIC����һ���ֽ�
//���شӻ�����Ӧ��
//1����Ӧ��
//0����Ӧ��			  
void IIC_Send_Byte(uint8_t txd)
{                        
    uint8_t t;   
	SDA_OUT(); 	    
    SCL_L;//����ʱ�ӿ�ʼ���ݴ���
    for(t=0;t<8;t++)
    {              
        if(txd&0x80)
        {
            SDA_H;
        }
        else SDA_L;
        txd<<=1; 	  
		delay_us(2);   //��TEA5767��������ʱ���Ǳ����
		SCL_H;
		delay_us(2); 
		SCL_L;	
		delay_us(2);
    }	 
} 

//��1���ֽڣ�ack=1ʱ������ACK��ack=0������nACK   
uint8_t IIC_Read_Byte(void)
{
	unsigned char i,receive=0;
	SDA_IN();//SDA����Ϊ����
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

/*********************************����IIC����****************************************************/


/************************************************************   
* ������:IIC_ADD_write   
* ���� : ���ض��豸id���ض���ַ��д���ֽ� 
* ����  :�豸id���ڲ���ַ������    
* ���  :��    
*/
void IIC_ADD_write(uint8_t DeviceAddr,uint8_t address,uint8_t Bytes)
{
	IIC_Start();
	IIC_Send_Byte(DeviceAddr);
	IIC_Wait_Ack();
	IIC_Send_Byte(address);   //���͵͵�ַ
	IIC_Wait_Ack(); 	 										  		   
	IIC_Send_Byte(Bytes);     //�����ֽ�							   
	IIC_Wait_Ack();  		    	   
    IIC_Stop();//����һ��ֹͣ���� 
	delay_us(10);	
}

/************************************************************   
* ������:I2C_ReadByte   
* ���� : ���ض��豸id���ض���ַ��ȡ����
* ����  :�豸id���ڲ���ַ   
* ���  :��ȡ������   
*/
uint8_t IIC_ADD_read(uint8_t DeviceAddr,uint8_t address)
{
	  unsigned char temp;
   IIC_Start();
   IIC_Send_Byte(DeviceAddr);
   IIC_Wait_Ack();
 
    IIC_Send_Byte(address);   //���͵͵�ַ
	IIC_Wait_Ack();	    
	IIC_Start();  	 	   
	IIC_Send_Byte(DeviceAddr+1);           //�������ģʽ			   
	IIC_Wait_Ack();	 
    temp=IIC_Read_Byte();		

    IIC_Stop();//����һ��ֹͣ����	    
	return temp;
}

/************************************************************   
* ������:IIC_Read_MultiBytes   
* ���� : ���ض��豸id���ض���ַ��ȡ����ֽ�
* ����  :�豸id���ڲ���ַ����Ҫ�����ֽڸ�����0����ȡ��
* ���  :��ȡ������   
*************************************************************/
uint8_t IIC_Read_MultiBytes(uint8_t DeviceAddr,uint8_t address,uint8_t Len,uint8_t *data)
{

   IIC_Start();
   IIC_Send_Byte(DeviceAddr);
   IIC_Wait_Ack();
 
    IIC_Send_Byte(address);   //���͵͵�ַ
	IIC_Wait_Ack();	    
	IIC_Start();  	 	   
	IIC_Send_Byte(DeviceAddr+1);           //�������ģʽ			   
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
    IIC_Stop();//����һ��ֹͣ����	 
	return 0;
}


/************************************************************   
* ������:IIC_NoAddr_WriteByte   
* ���� : ���ض��豸id��д���ֽ� 
* ����  :�豸id������   
* ���  :��    
*/
void IIC_NoAddr_WriteByte(unsigned char address,unsigned char Bytes)
{

	IIC_Start();
	IIC_Send_Byte(address);   //���͵͵�ַ
	IIC_Wait_Ack(); 	 										  		   
	IIC_Send_Byte(Bytes);     //�����ֽ�							   
	IIC_Wait_Ack();  		    	   
    IIC_Stop();//����һ��ֹͣ���� 
	delay_ms(10);

}

//IIC����д
//addr:������ַ 
//reg:�Ĵ�����ַ
//len:д�볤��
//buf:������
//����ֵ:0,����
//    ����,�������
uint8_t MPU_Write_Len(uint8_t addr,uint8_t reg,uint8_t len,uint8_t *buf)
{
	uint8_t i; 
    IIC_Start(); 
	IIC_Send_Byte((addr<<1)|0);//����������ַ+д����	
	if(IIC_Wait_Ack())	//�ȴ�Ӧ��
	{
		IIC_Stop();		 
		return 1;		
	}
    IIC_Send_Byte(reg);	//д�Ĵ�����ַ
    IIC_Wait_Ack();		//�ȴ�Ӧ��
	for(i=0;i<len;i++)
	{
		IIC_Send_Byte(buf[i]);	//��������
		if(IIC_Wait_Ack())		//�ȴ�ACK
		{
			IIC_Stop();	 
			return 1;		 
		}		
	}    
    IIC_Stop();	 
	return 0;	
} 
//IIC������
//addr:������ַ
//reg:Ҫ��ȡ�ļĴ�����ַ
//len:Ҫ��ȡ�ĳ���
//buf:��ȡ�������ݴ洢��
//����ֵ:0,����
//    ����,�������
uint8_t MPU_Read_Len(uint8_t addr,uint8_t reg,uint8_t len,uint8_t *buf)
{ 
 	IIC_Start(); 
	IIC_Send_Byte((addr<<1)|0);//����������ַ+д����	
	if(IIC_Wait_Ack())	//�ȴ�Ӧ��
	{
		IIC_Stop();		 
		return 1;		
	}
    IIC_Send_Byte(reg);	//д�Ĵ�����ַ
    IIC_Wait_Ack();		//�ȴ�Ӧ��
    IIC_Start();
	IIC_Send_Byte((addr<<1)|1);//����������ַ+������	
    IIC_Wait_Ack();		//�ȴ�Ӧ�� 
	while(len)
	{
		if(len==1)
        {
            *buf=IIC_Read_Byte();//������,����nACK 
            IIC_NAck();
        }
		else 
        {
            *buf=IIC_Read_Byte();		//������,����ACK  
            IIC_Ack(); //����ACK 
        }            
		len--;
		buf++; 
	}    
    IIC_Stop();	//����һ��ֹͣ���� 
	return 0;	
}
/***************************************************************/











