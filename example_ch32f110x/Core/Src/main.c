/********************************** (C) COPYRIGHT *******************************
* File Name          : main.c
* Author             : WCH
* Version            : V1.0.0
* Date               : 2019/10/15
* Description        : Main program body.
*******************************************************************************/

/*
 *@Note
 GPIO例程：
 PA0推挽输出。
*/


#include "bmp180.h"

/*******************************************************************************
* Function Name  : IIC_Init
* Description    : Initializes the IIC peripheral.
* Input          : None
* Return         : None
*******************************************************************************/
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

void setup(void)
{

}
/*******************************************************************************
* Function Name  : main
* Description    : Main program.
* Input          : None
* Return         : None
*******************************************************************************/
int main(void)
{
	bmp180_t	bmp180Structure;
	int32_t		temperature, pressure ;
	float		altitude;

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    SystemCoreClockUpdate();
    USART_Printf_Init(115200);

    printf("SystemClk:%ld\r\n", SystemCoreClock);
    printf("GPIO Toggle TEST\r\n");

    printf("IIC Host mode\r\n");

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
