/**
  ******************************************************************************
  * @file    BlueNRG1_it.c 
  * @author  VMA RF Application Team
  * @version V1.0.0
  * @date    September-2015
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2015 STMicroelectronics</center></h2>
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "BlueNRG1_it.h"
#include "BlueNRG1_conf.h"
#include "ble_const.h" 
#include "bluenrg1_stack.h"
#include "SDK_EVAL_Com.h"
#include "clock.h"


/** @addtogroup BlueNRG1_StdPeriph_Examples
  * @{
  */

/** @addtogroup GPIO_Examples
  * @{
  */ 

/** @addtogroup GPIO_IOToggle
  * @{
  */ 
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
#ifndef SENSOR_EMULATION


#endif

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M0 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {}
}


/**
  * @brief  This function handles SVCall exception.
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles PendSV_Handler exception.
  */
//void PendSV_Handler(void)
//{
//}

/**
  * @brief  This function handles SysTick Handler.
  */
void SysTick_Handler(void)
{
	SysCount_Handler();
}

void GPIO_Handler(void)
{
//#ifndef SENSOR_EMULATION
//  uint8_t free_fall_status;
//  /* Check if GPIO pin 12 interrupt event occured */
//  if(GPIO_GetITPendingBit(LSM6DS3_IRQ_PIN) == SET)
//  {
//    /* Clear the IRQ pending bit */
//    GPIO_ClearITPendingBit(LSM6DS3_IRQ_PIN);
//
//    /* Set the IRQ flag */
//    Imu6AxesDrvExt->Get_Status_Free_Fall_Detection(&free_fall_status);
//    if (free_fall_status == 1)
//    {
//      request_free_fall_notify = TRUE;
//    }
//
//  }
//#endif

  if (GPIO_GetITPendingBit(Get_ButtonGpioPin(BUTTON_1)) == SET){
	  GPIO_ClearITPendingBit(Get_ButtonGpioPin(BUTTON_1));
	  GPIO_ToggleBits(Get_LedGpioPin(LED2));
  }
}
/******************************************************************************/
/*                 BlueNRG-1 Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (system_bluenrg1.c).                                               */
/******************************************************************************/
/**
* @brief  This function handles UART interrupt request.
* @param  None
* @retval None
*/
void UART_Handler(void)
{  
	SdkEvalComIOUartIrqHandler();
}

void Blue_Handler(void)
{
   // Call RAL_Isr
   RAL_Isr();
}

/**
  * @}
  */ 

/**
  * @}
  */ 

/**
  * @}
  */ 

/******************* (C) COPYRIGHT 2015 STMicroelectronics *****END OF FILE****/
