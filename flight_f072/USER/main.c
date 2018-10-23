#include "main.h"
#include "LED.h"
#include "delay.h"
#include "IIC.h"
#include "mpu6050.h"


int main(void)
{

  delay_init();
  LED_Init();
  IIC_Init();
  
  while(mpu6050_init() != 0)
  {
      LED2(ON);
    delay_ms(1000);
    LED2(OFF);
    delay_ms(1000);
  }
  LED2(OFF)
  while (1)
  {
    LED0(OFF);
    delay_ms(1000);
    LED0(ON);
    delay_ms(1000);
  }
}

