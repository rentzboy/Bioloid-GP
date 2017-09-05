/**
  ******************************************************************************
  * @file    Project/STM32F10x_StdPeriph_Template/stm32f10x_it.c 
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    08-April-2011
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
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */

/******************************************************************************/
/*            INTERRUPTS FILE - IMPORT                                        */
/******************************************************************************/
/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_it.h"
#include "globals.h"
#include "motions.h"    //motionStatus_typedef

/******************************************************************************/
/*            INTERRUPTS FILE - PRIVATE DECLARATION                           */
/******************************************************************************/
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

/******************************************************************************/
/*            Cortex-M3 Processor Exceptions Handlers                         */
/******************************************************************************/
/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
   delay_ISR();
}

/******************************************************************************/
/*                 STM32F10x Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f10x_xx.s).                                            */
/******************************************************************************/   
/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
void TIM3_IRQHandler(void)  //TIMING MOTION CONTROL
{
   if (TIM_GetITStatus(TIM3, TIM_IT_CC1) != RESET) //CC1 = stepTimeInterruption
   {
      if(motionStatus != MOTION_IS_OVER) //es ++ importante PAUSE_IS_OVER 
         motionStatus = STEP_IS_OVER;    //update global
      TIM_ClearITPendingBit(TIM3, TIM_IT_CC1); //Clears the CC1IF (Capture Compare 1 Interrupt Flag)
      TIM_ITConfig(TIM3, TIM_IT_CC1, DISABLE); //Disable Interrupt
   }
   else if (TIM_GetITStatus(TIM3, TIM_IT_CC2) != RESET) //CC2 = pauseTimeInterruption
   {
      if(motionStatus != MOTION_IS_OVER)  //sometimes µC has already updated to MOTION_IS_OVER during pause_time !!
         motionStatus = PAUSE_IS_OVER;    //update global
      TIM_ClearITPendingBit(TIM3, TIM_IT_CC2); //Clears the CC2IF (Capture Compare 1 Interrupt Flag)
      TIM_ITConfig(TIM3, TIM_IT_CC2, DISABLE); //Disable Interrupt
   }
}
/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
void EXTI3_IRQHandler(void) //StartButton Interruption <CM-530>
{
    if(EXTI_GetITStatus(EXTI_Line3) != RESET) //check if Interruption comes from StartButton Line
        {
          while(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_3)){} //to avoid button bouncing
          StartButton = SET;
          EXTI_ClearITPendingBit(EXTI_Line3);
        }
}

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
void EXTI4_IRQHandler(void) //Button2 Interruption <OpenCM0.94>
{
    if(EXTI_GetITStatus(EXTI_Line4) != RESET) //check if Interruption comes from Button Line
        {
          while(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_4)){} //to avoid button bouncing
          StartButton = SET;
          EXTI_ClearITPendingBit(EXTI_Line4);
        }
}
/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
void EXTI10_IRQHandler(void) //D_Button Interruption <CM-530>
{
    if(EXTI_GetITStatus(EXTI_Line10) != RESET) //check if Interruption comes from D_Button Line
        {
          while(GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_10)){} //to avoid button bouncing
          g_actionmode = FIGHTMODE;
          EXTI_ClearITPendingBit(EXTI_Line10);
        }
}
/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
void EXTI11_IRQHandler(void) //U_Button Interruption <CM-530>
{
    if(EXTI_GetITStatus(EXTI_Line11) != RESET) //check if Interruption comes from U_Button Line
        {
          while(GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_11)){} //to avoid button bouncing
          g_actionmode = SOCCERMODE;
          EXTI_ClearITPendingBit(EXTI_Line11);
        }
}
/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
void USART1_IRQHandler(void) //DXL Interruption <OpenCM0.94> & <CM-530>
{
   DXL_TxRxByte_It(USART1);
}
/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
void USART2_IRQHandler(void) //Zig-bee Interruption <OpenCM0.94>
{
   USART2_ZBEE_Interrupt();
}
/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
void USART3_IRQHandler(void) //Terminal Interruption <CM-530> || DXL Interruption <OpenCM0.94>
{
#ifdef STM32F10X_HD /* CM-530 */
   USART3_Interrupt(); //µC <-> Pc
#else /* OpenCM0.94 */
   DXL_TxRxByte_It(USART3); //Exp_485 <-> DXL
#endif   
}
/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
void USART5_IRQHandler(void) //Zig-bee Interruption <CM-530>
{
   USART5_ZBEE_Interrupt();
}

/**
  * @}
  */ 


/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
