#include "timer3.h"
#include  "IMU.h"
#include "sand_data.h"
#include "led.h"
#include "PWM.h"
#include "mpu9250.h"
#include "mpu6050.h"
#include "RC.h"
#include "control.h"
#include "24l01.h"

void tim2_init(u16 arr,u16 psc)
{
	
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
	NVIC_InitTypeDef NVIC_InitStruct;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);
	
	//timer3中断配置

	TIM_TimeBaseInitStruct.TIM_Period=arr;
	TIM_TimeBaseInitStruct.TIM_Prescaler=psc;
	TIM_TimeBaseInitStruct.TIM_ClockDivision= TIM_CKD_DIV1;
	TIM_TimeBaseInitStruct.TIM_CounterMode= TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM2,&TIM_TimeBaseInitStruct);
	
	//中断优先级配置
	NVIC_InitStruct.NVIC_IRQChannel=TIM2_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelCmd=ENABLE ;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority=0x01;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority=0x02;
	NVIC_Init(&NVIC_InitStruct);

	
	//使能timer3中断
	
	TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE);
//	TIM_Cmd(TIM2, ENABLE);

}


void tim4_init(u16 arr,u16 psc)
{
	
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
	NVIC_InitTypeDef NVIC_InitStruct;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);
	
	//timer3中断配置

	TIM_TimeBaseInitStruct.TIM_Period=arr;
	TIM_TimeBaseInitStruct.TIM_Prescaler=psc;
	TIM_TimeBaseInitStruct.TIM_ClockDivision= TIM_CKD_DIV1;
	TIM_TimeBaseInitStruct.TIM_CounterMode= TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM4,&TIM_TimeBaseInitStruct);
	
	//中断优先级配置
	NVIC_InitStruct.NVIC_IRQChannel=TIM4_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelCmd=ENABLE ;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority=0x01;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority=0x01;
	NVIC_Init(&NVIC_InitStruct);

	
	//使能timer3中断
	
	TIM_ITConfig(TIM4,TIM_IT_Update,ENABLE);
	

}

int times= 0;
int Flag_att = 0;
int att_cnt = 0;
int TIM4_times = 0;
int PWM_cnt = 0;
int PWM_value = 0;
extern int IRQ_timeout;
extern u8 Rx_buff[33];
extern u8 Tx_buff[33];
void TIM2_IRQHandler(void)
{
    u8 sta = 0;
  uint8_t PA0_status = 0;
	if( TIM_GetITStatus(TIM2 ,TIM_IT_Update)==SET)
	{
        
//实际测试时，IRQ引脚中断没有作用，这就相当于每10ms检测一次       
        IRQ_timeout++;
    
        //遥控器部分
        RC_Receive_Anl();
        Deblocking();
        mode_contrl();
        if(IRQ_timeout > 10)
        {
            IRQ_timeout = 0;

            PA0_status = GPIO_ReadOutputDataBit(GPIOA,GPIO_Pin_0);
            sta=NRF24L01_Read_Reg(STATUS);  //读取状态寄存器的值    	 
            NRF24L01_Write_Reg(NRF_WRITE_REG+STATUS,sta); //清除TX_DS或MAX_RT中断标志
            NRF24L01_Write_Reg(FLUSH_RX,0xff);//清除RX FIFO寄存器 
//            if(sta&RX_OK)//接收到数据
//            {
//                NRF24L01_Read_Buf(RD_RX_PLOAD,Rx_buff,RX_PLOAD_WIDTH);//读取数据
//                NRF24L01_Write_Reg(FLUSH_RX,0xff);//清除RX FIFO寄存器 
//                ReceiveData(Rx_buff);
//                LED1 =!LED1;
//                
//            }
        }
        
#if USE_IMU_DEVICE
		PWM_cnt++;
	
		if(PWM_cnt<50)
		{
			PWM_value = 0;
		}
		else if(PWM_cnt>=50&&PWM_cnt<250)
		{
			PWM_value+=5;
			
		}
		else{
			PWM_cnt = 0;
			PWM_value = 0;
		}

#else
        
        times ++;
        Flag_att++;
        if(Flag_att == 1)
        {
            MPU6500_Dataanl(&imu_data.mpu6500_dataacc1,&imu_data.mpu6500_datagyr1);
        }
        else
        {
            Flag_att = 0;
            MPU6500_Dataanl(&imu_data.mpu6500_dataacc2,&imu_data.mpu6500_datagyr2);
        }

        
        if(times %2 == 0)
        {
            Prepare_Data();
            Get_Attitude();
        }
        else if(times % 3 == 0)
        {
            LED0 =!LED0;
        }
        else if(times % 5 == 0)
        {   
            sand_IMU_data();
            sand_ACC_GYRO_data(); 
        }
        if(times % 23 == 0)
            READ_MPU9250_MAG();
			
#endif
	}
    TIM_ClearITPendingBit(TIM2,TIM_IT_Update);
	
}

void TIM4_IRQHandler(void)
{
  u8 sta = 0;
	if( TIM_GetITStatus(TIM4 ,TIM_IT_Update)==SET)
	{
        
#if USE_IMU_DEVICE
        //标志位改变
        TIM4_times ++;
        
        //姿态部分
        READ_6050();
        Prepare_6050_Data();
        Get_Attitude();
        //
    
//        sta=NRF24L01_Read_Reg(STATUS);  //读取状态寄存器的值    	 
//        NRF24L01_Write_Reg(NRF_WRITE_REG+STATUS,sta); //清除TX_DS或MAX_RT中断标志
//        if(sta&RX_OK)//接收到数据
//        {
//            NRF24L01_Read_Buf(RD_RX_PLOAD,Rx_buff,RX_PLOAD_WIDTH);//读取数据
//            NRF24L01_Write_Reg(FLUSH_RX,0xff);//清除RX FIFO寄存器 
//            ReceiveData(Rx_buff);
//            LED1 =!LED1;
//            
//        }
        //PID控制部分
        CONTROL(angle.roll,angle.pitch,angle.yaw);
        
        if(ARMED&& TIM4_times % 5==0)
        {
             LED0 =!LED0;
        }
        else if(TIM4_times % 7 == 0)
        {   
            sand_IMU_data();
            sand_ACC_GYRO_data(); 
        }
        else if(TIM4_times %13 == 0)
        {
            sand_RC_data();
            sand_Motor_data();
        }
        else if(!ARMED && TIM4_times %100 == 0)
          LED0 =!LED0;
        if(TIM4_times % 500 == 0)
        {
          NRF24L01_CE=0;
          NRF24L01_TX_Mode();
          NRF24L01_Write_Buf(WR_TX_PLOAD,Tx_buff,TX_PLOAD_WIDTH);//写数据到TX BUF  32个字节
          NRF24L01_CE=1;//启动发送
        }
         
        
#else
		PWM_cnt++;
	
		if(PWM_cnt<50)
		{
			PWM_value = 0;
		}
		else if(PWM_cnt>=50&&PWM_cnt<250)
		{
			PWM_value+=5;
			
		}
		else{
			PWM_cnt = 0;
			PWM_value = 0;
		}
		Moto_PwmRflash(PWM_value,PWM_value,PWM_value,PWM_value);
#endif
	}
	TIM_ClearITPendingBit(TIM4,TIM_IT_Update);
}
