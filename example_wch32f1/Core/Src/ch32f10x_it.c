/********************************** (C) COPYRIGHT *******************************
* File Name          : ch32f10x_it.c
* Author             : WCH
* Version            : V1.0.0
* Date               : 2019/10/15
* Description        : Main Interrupt Service Routines.
*******************************************************************************/ 
#include "ch32f10x_it.h" 

/*******************************************************************************
* Function Name  : NMI_Handler
* Description    : This function handles NMI exception.
* Input          : None
* Return         : None
*******************************************************************************/
void NMI_Handler(void)
{
}

/*******************************************************************************
* Function Name  : HardFault_Handler
* Description    : This function handles Hard Fault exception.
* Input          : None
* Return         : None
*******************************************************************************/
void HardFault_Handler(void)
{
  while (1)
  {
  }
}

/*******************************************************************************
* Function Name  : MemManage_Handler
* Description    : This function handles Memory Manage exception.
* Input          : None
* Return         : None
*******************************************************************************/
void MemManage_Handler(void)
{
  while (1)
  {
  }
}

/*******************************************************************************
* Function Name  : BusFault_Handler
* Description    : This function handles Bus Fault exception.
* Input          : None
* Return         : None
*******************************************************************************/
void BusFault_Handler(void)
{
  while (1)
  {
  }
}

/*******************************************************************************
* Function Name  : UsageFault_Handler
* Description    : This function handles Usage Fault exception.
* Input          : None
* Return         : None
*******************************************************************************/
void UsageFault_Handler(void)
{
  while (1)
  {
  }
}

/*******************************************************************************
* Function Name  : SVC_Handler
* Description    : This function handles SVCall exception.
* Input          : None
* Return         : None
*******************************************************************************/
void SVC_Handler(void)
{
}

/*******************************************************************************
* Function Name  : DebugMon_Handler
* Description    : This function handles Debug Monitor exception.
* Input          : None
* Return         : None
*******************************************************************************/
void DebugMon_Handler(void)
{
}

/*******************************************************************************
* Function Name  : PendSV_Handler
* Description    : This function handles PendSVC exception.
* Input          : None
* Return         : None
*******************************************************************************/
void PendSV_Handler(void)
{
}

/*******************************************************************************
* Function Name  : SysTick_Handler
* Description    : This function handles SysTick Handler.
* Input          : None
* Return         : None
*******************************************************************************/
void SysTick_Handler(void)
{
}



void I2C1_EV_IRQHandler(void)
{
    switch(I2C_GetLastEvent(I2C1))
    {
    case I2C_EVENT_SLAVE_RECEIVER_ADDRESS_MATCHED :
        break;
    case I2C_EVENT_SLAVE_BYTE_RECEIVED:
        break;
    case I2C_EVENT_SLAVE_STOP_DETECTED :

        break;
    case I2C_EVENT_SLAVE_TRANSMITTER_ADDRESS_MATCHED:

        break;
    case I2C_EVENT_SLAVE_BYTE_TRANSMITTED:
        break;
    case I2C_EVENT_SLAVE_ACK_FAILURE:
        break;
    default:
        break;
    }
}

void I2C1_ER_IRQHandler(void)
{

}


