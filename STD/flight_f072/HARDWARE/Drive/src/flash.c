/*******************************************************************************************
										    �� ��
    ����Ŀ�����������ѧϰʹ�ã�����������ֲ�޸ģ������뱣����������Ϣ����ֲ�����г�������
	
���ɹ�����BUG��������������κ����Ρ��������ã�

* ����汾��V1.01
* �������ڣ�2018-8-18
* �������ߣ���ŭ��С��
* ��Ȩ���У��������������Ϣ�������޹�˾
*******************************************************************************************/
#include "stm32f0xx.h"
#include "flash.h"
#include "delay.h"
#include "usart.h" 


/******************************************************************************
* ��  ����uint32_t STMFLASH_ReadWord(uint32_t faddr)
* �����ܣ���ȡָ����ַ����(32λ����) 
* ��  ����faddr:����ַ 
* ����ֵ����Ӧ����
* ��  ע����
*******************************************************************************/
uint32_t STMFLASH_ReadWord(uint32_t faddr)
{
	return *(__IO uint32_t*)faddr; 
}  

/******************************************************************************
* ��  ����void STMFLASH_Write(uint32_t WriteAddr,uint32_t *pBuffer,uint32_t NumToWrite)	
* �����ܣ���ָ����ַ��ʼд��ָ�����ȵ����� 
* ��  ����WriteAddr:��ʼ��ַ(�˵�ַ����Ϊ4�ı���!!)
*         pBuffer:����ָ��
*         NumToWrite:��(32λ)��(����Ҫд���32λ���ݵĸ���.) 
* ����ֵ����
* ��  ע��STM32F1��Flashδд����Ĭ����0xFFF...F
*******************************************************************************/
void STMFLASH_Write(uint32_t WriteAddr,uint32_t *pBuffer,uint32_t NumToWrite)	
{ 
	FLASH_Status status = FLASH_COMPLETE;
	uint32_t addrx=0;
	uint32_t endaddr=0;	
	if(WriteAddr<STM32_FLASH_BASE||(WriteAddr>=(STM32_FLASH_BASE+1024*128)))return;	//�Ƿ���ַ
	FLASH_Unlock();					 //���� 
 
	addrx=WriteAddr;				 //д�����ʼ��ַ
	endaddr=WriteAddr+NumToWrite*4;	 //д��Ľ�����ַ
	if(addrx<0X08020000)			       //ֻ�����洢��,����Ҫִ�в�������!!
	{
		while(addrx<endaddr)		       //ɨ��һ���ϰ�.(�Է�FFFFFFFF�ĵط�,�Ȳ���)
		{
			if(STMFLASH_ReadWord(addrx)!=0XFFFFFFFF)//�з�0XFFFFFFFF�ĵط�,Ҫ�������ҳ
			{   
				status=FLASH_ErasePage(FLASH_SAVE_ADDR);
				if(status!=FLASH_COMPLETE)break;	    //����������
			}
			else 
			addrx+=4;
		} 
	}
	if(status==FLASH_COMPLETE)
	{
		while(WriteAddr<endaddr) //д����
		{
			if(FLASH_ProgramWord(WriteAddr,*pBuffer)!=FLASH_COMPLETE) //д������
			{ 
				break; //д���쳣
			}
			WriteAddr+=4;
			pBuffer++;
		} 
	}
	
	FLASH_Lock(); //����
} 

/******************************************************************************
* ��  ����void STMFLASH_Read(uint32_t ReadAddr,uint32_t *pBuffer,uint32_t NumToRead)  
* �����ܣ���ָ����ַ��ʼ����ָ�����ȵ�����
* ��  ����ReadAddr:��ʼ��ַ
*         pBuffer:����ָ��
*         NumToRead:��(4λ)��
* ����ֵ����
* ��  ע����
*******************************************************************************/
void STMFLASH_Read(uint32_t ReadAddr,uint32_t *pBuffer,uint32_t NumToRead)   	
{
	uint32_t i;
	for(i=0;i<NumToRead;i++)
	{
		pBuffer[i]=STMFLASH_ReadWord(ReadAddr);//��ȡ4���ֽ�.
		ReadAddr+=4;//ƫ��4���ֽ�.	
	}
}


