/**
  ******************************************************************************
  * @file    Project/STM32F10x_StdPeriph_Template/main.c
  * @author  MCD Application Team
  * @version V3.6.0
  * @date    20-September-2021
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2011 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

#include "stm32f10x.h"

#include "main.h"
#include <stdio.h>


void IIC_Init(u32 bound, u16 address)
{

    GPIO_InitTypeDef GPIO_InitStructure;
    I2C_InitTypeDef I2C_InitTSturcture;

    I2C_StructInit( &I2C_InitTSturcture );

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    /* I2C1 Reset */
    RCC_APB1PeriphResetCmd( RCC_APB1Periph_I2C1, ENABLE );
    RCC_APB1PeriphResetCmd( RCC_APB1Periph_I2C1, DISABLE );
    I2C_SoftwareResetCmd( I2C1, ENABLE );
    I2C_SoftwareResetCmd( I2C1, DISABLE );

    I2C_InitTSturcture.I2C_ClockSpeed = bound;
    I2C_InitTSturcture.I2C_Mode = I2C_Mode_I2C;
    I2C_InitTSturcture.I2C_DutyCycle = I2C_DutyCycle_16_9;
    I2C_InitTSturcture.I2C_OwnAddress1 = address;
    I2C_InitTSturcture.I2C_Ack = I2C_Ack_Enable;
    I2C_InitTSturcture.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_Init(I2C1, &I2C_InitTSturcture);

    I2C_Cmd( I2C1, ENABLE );


#if (I2C_MODE == HOST_MODE)
    I2C_AcknowledgeConfig(I2C1, ENABLE);

#endif

}
/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */
int main(void)
{
	bmp180_t	bmp180Structure;
	int32_t		temperature, pressure ;
	float		altitude;

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    SystemCoreClockUpdate();

    IIC_Init(100000, 0x02);

    char result = BMP180_Setup( I2C1, &bmp180Structure );

    if( result == 1 )
    {

		temperature = BMP180_GetTemperature();

		//Pressure Pa
		pressure = BMP180_GetPressure( BMP180_ACC_ULTRA_HIGH_RES );

		altitude = BMP180_GetAltitude( pressure );

		//Pressure mmHg
		pressure = pressure * 0.00750062;
    }



    while (1)
    {

    	if( result == 1 )
    		temperature = BMP180_GetTemperature();

    }
}


#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif


