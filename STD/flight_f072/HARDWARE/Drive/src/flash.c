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
#include "flash.h"
#include "delay.h"
#include "usart.h" 


/******************************************************************************
* 函  数：uint32_t STMFLASH_ReadWord(uint32_t faddr)
* 功　能：读取指定地址的字(32位数据) 
* 参  数：faddr:读地址 
* 返回值：对应数据
* 备  注：无
*******************************************************************************/
uint32_t STMFLASH_ReadWord(uint32_t faddr)
{
	return *(__IO uint32_t*)faddr; 
}  

/******************************************************************************
* 函  数：void STMFLASH_Write(uint32_t WriteAddr,uint32_t *pBuffer,uint32_t NumToWrite)	
* 功　能：从指定地址开始写入指定长度的数据 
* 参  数：WriteAddr:起始地址(此地址必须为4的倍数!!)
*         pBuffer:数据指针
*         NumToWrite:字(32位)数(就是要写入的32位数据的个数.) 
* 返回值：无
* 备  注：STM32F1的Flash未写扇区默认是0xFFF...F
*******************************************************************************/
void STMFLASH_Write(uint32_t WriteAddr,uint32_t *pBuffer,uint32_t NumToWrite)	
{ 
	FLASH_Status status = FLASH_COMPLETE;
	uint32_t addrx=0;
	uint32_t endaddr=0;	
	if(WriteAddr<STM32_FLASH_BASE||(WriteAddr>=(STM32_FLASH_BASE+1024*128)))return;	//非法地址
	FLASH_Unlock();					 //解锁 
 
	addrx=WriteAddr;				 //写入的起始地址
	endaddr=WriteAddr+NumToWrite*4;	 //写入的结束地址
	if(addrx<0X08020000)			       //只有主存储区,才需要执行擦除操作!!
	{
		while(addrx<endaddr)		       //扫清一切障碍.(对非FFFFFFFF的地方,先擦除)
		{
			if(STMFLASH_ReadWord(addrx)!=0XFFFFFFFF)//有非0XFFFFFFFF的地方,要擦除这个页
			{   
				status=FLASH_ErasePage(FLASH_SAVE_ADDR);
				if(status!=FLASH_COMPLETE)break;	    //发生错误了
			}
			else 
			addrx+=4;
		} 
	}
	if(status==FLASH_COMPLETE)
	{
		while(WriteAddr<endaddr) //写数据
		{
			if(FLASH_ProgramWord(WriteAddr,*pBuffer)!=FLASH_COMPLETE) //写入数据
			{ 
				break; //写入异常
			}
			WriteAddr+=4;
			pBuffer++;
		} 
	}
	
	FLASH_Lock(); //上锁
} 

/******************************************************************************
* 函  数：void STMFLASH_Read(uint32_t ReadAddr,uint32_t *pBuffer,uint32_t NumToRead)  
* 功　能：从指定地址开始读出指定长度的数据
* 参  数：ReadAddr:起始地址
*         pBuffer:数据指针
*         NumToRead:字(4位)数
* 返回值：无
* 备  注：无
*******************************************************************************/
void STMFLASH_Read(uint32_t ReadAddr,uint32_t *pBuffer,uint32_t NumToRead)   	
{
	uint32_t i;
	for(i=0;i<NumToRead;i++)
	{
		pBuffer[i]=STMFLASH_ReadWord(ReadAddr);//读取4个字节.
		ReadAddr+=4;//偏移4个字节.	
	}
}


