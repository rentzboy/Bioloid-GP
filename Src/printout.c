/**
  ******************************************************************************
  * @file    printout.c
  * @author  FX_Team
  * @version V1.0.0
  * @date    25-July-2017
  * @brief   
  *          
  * @version 
  *         
  ******************************************************************************

  ==============================================================================
                       ##### How to use this driver #####
  =============================================================================
  >> A la hora de imprimir por pantalla o a archivo hay que distinguir 2 casos:
  1- Imprimir una cadena de texto: se puede imprimir mediante printf o 
     creando nuestras propias funciones; USART_SendChar() para imprimir caracteres
     sueltos y USART_SendString() para cadenas.
  2- Si queremos imprimir variables podemos utilizar printf -print format- o sprintf,
     para imprimir a una cadena de texto. Si no podemos utilizar printf podemos utilizar sprintf para convertir la variable a cadena de texto (la parte + complicada) y luego mediante USART_SendString() mostrar la variable en putty.

  >> Si queremos crear nuestras funciones para imprimir variables hay que tener en
     cuenta variaas consideraciones:
  1- Antes de imprimir el valor de la variable hay que convertirla en una cadena de texto,
     por lo que es necesario realizar un (char) cast.
  2- Para pasar de un valor decimal a ASCII: 
     int a=5;
     char b = a + '0' (siendo '0' el codigo ASCII 0x30)
  3- También habrá que tener en cuenta si la variable es (+) o (-).
  4- Es posible que haya que convertir entre decimal <-> hexadecimal.
  *****************************************************************************/

/******************************************************************************/
/*            PRINTOUT FILE - IMPORT                                            */
/******************************************************************************/
/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include "printout.h"

/******************************************************************************/
/*            PRINTOUT FILE - DECLARATIONS                                      */
/******************************************************************************/
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define USART_TIMEOUT     0xFFFF

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
//control timing variables
static uint16_t Timeout;                //for USART functions

/* Private function prototypes -----------------------------------------------*/
static void USART_TC_Polling(USART_TypeDef* USARTx);
static void USART_CallOutTime(void);

/******************************************************************************/
/*            XXXXXX FILE - FUNCTION DEFINITIONS                              */
/******************************************************************************/
/** @defgroup ---------- UART Initialization functions ----------
  * @{
  */
/**
  * @brief  From USART1 Tx(PA9) & Rx(PA10) to Pc
  * @param  None
  * @retval None
  */
void USART1_OPENCM_PC_Initialize(uint32_t baudrate) //µc-Pc (Full-Duplex) -OpenCM0.94-
{
   /* ===== USART1 TIME BLOCK & INTERRUPTIONS CONFIGURATION ===== */
   /* Enable USART1 clock */
   RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 , ENABLE);

   /* Enable Port A (USART1_Tx) and (USART1_Rx) clock */
   RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

   /* Enable APB2 peripheral AFIO clock */
   RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

   /* Configure and set USART1 Interrupt to the lowest priority */
   NVIC_InitTypeDef NVIC_InitStructure;
   NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
   NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;
   NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;
   NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
   NVIC_Init(&NVIC_InitStructure);


	/* ===== GPIOS CONFIGURATION FOR USART1 ===== */
   /* Configure and set USART1_Tx (PA9) as input floating */
   GPIO_InitTypeDef   GPIO_InitStructure;
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; //Full-Duplex
   //GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD; //Half-Duplex
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
   GPIO_Init(GPIOA, &GPIO_InitStructure);

   /* Configure and set USART1_Rx (PA10) as input floating */
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
   GPIO_Init(GPIOA, &GPIO_InitStructure);


	/* ===== USART1 CONFIGURATION & SET ===== */
   /* Configure and set USART1 */
   USART_InitTypeDef   USART_InitStructure;
   USART_InitStructure.USART_BaudRate = baudrate;
   USART_InitStructure.USART_WordLength = USART_WordLength_8b;
   USART_InitStructure.USART_StopBits = USART_StopBits_1;
   USART_InitStructure.USART_Parity = USART_Parity_No ;
   USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
   USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
   USART_Init(USART1, &USART_InitStructure);


	/* ===== CUSTOM CONFIGURATION ===== */
   /* Enables USART1´s Half Duplex communication */
   //USART_HalfDuplexCmd(USART1, ENABLE);

   /* Enable USART1 peripheral */
   USART_Cmd(USART1, ENABLE);

   /* Enable USART1 Interruptions */
   //USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
   //USART_ITConfig(USART1, USART_IT_TXE, ENABLE);
   //USART_ITConfig(USART1, USART_IT_TC, ENABLE);

   /* Enable DMA Management for USART1 */
   //USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);
}
/**
  * @brief  From MicroUSB to PC in the CM-530
  * @param  None
  * @retval None
  */
void USART3_CM530_PC_Initialize(uint32_t baudrate) //µc-Pc (Full-Duplex) -CM-530-
{
   /* ===== USART3 TIME BLOCK & INTERRUPTIONS CONFIGURATION ===== */
   /* Enable USART3 clock */
   RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3 , ENABLE);

   /* Enable Port B (USART3_Tx) and (USART3_Rx) clock */
   RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

   /* Enable APB2 peripheral AFIO clock */
   RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

   /* Configure and set USART3 Interrupt to the lowest priority */
   NVIC_InitTypeDef NVIC_InitStructure;
   NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
   NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;
   NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;
   NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
   NVIC_Init(&NVIC_InitStructure);


	/* ===== GPIOS CONFIGURATION FOR USART3 ===== */
   /* Configure and set USART3_Tx (PB10) as input floating */
   GPIO_InitTypeDef   GPIO_InitStructure;
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; //Full-Duplex
   //GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD; //Half-Duplex
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
   GPIO_Init(GPIOB, &GPIO_InitStructure);

   /* Configure and set USART3_Rx (PB11) as input floating */
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
   GPIO_Init(GPIOB, &GPIO_InitStructure);


	/* ===== USART3 CONFIGURATION & SET ===== */
   /* Configure and set USART3 */
   USART_InitTypeDef   USART_InitStructure;
   USART_InitStructure.USART_BaudRate = baudrate;
   USART_InitStructure.USART_WordLength = USART_WordLength_8b;
   USART_InitStructure.USART_StopBits = USART_StopBits_1;
   USART_InitStructure.USART_Parity = USART_Parity_No ;
   USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
   USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
   USART_Init(USART3, &USART_InitStructure);


	/* ===== CUSTOM CONFIGURATION ===== */
   /* Enables USART3´s Half Duplex communication */
   //USART_HalfDuplexCmd(USART1, ENABLE);

   /* Enable USART3 peripheral */
   USART_Cmd(USART3, ENABLE);

   /* Enable USART3 Interruptions */
   //USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
   //USART_ITConfig(USART3, USART_IT_TXE, ENABLE);
   //USART_ITConfig(USART3, USART_IT_TC, ENABLE);

   /* Enable DMA Management for USART3 */
   //USART_DMACmd(USART3, USART_DMAReq_Tx, ENABLE);
}

/**
  * @brief  Initialization USART1 or USART3 depending on the board
  * @param  None
  * @retval None
  */
void PrintoutInitialization(uint32_t baudrate) //DEPRECATED
{
#ifdef STM32F10X_MD //OpenCM0.94(Exp_485) -USART1-
   USART1_OPENCM_PC_Initialize(baudrate);
#endif
   
#ifdef STM32F10X_HD//CM-530 -USART3-
   USART3_CM530_PC_Initialize(baudrate);
#endif
   
#ifdef STM32F10X_MD_VL//STM32F1 -USART3-
   USART3_CM530_PC_Initialize(baudrate);
#endif
}
/** @defgroup ---------- UART High level functions ----------
  * @{
  */
/**
  * @brief  Clear PUTTY screen using VT100 terminal commands
  *         (ver VT100 - PUTTY terminal commands.txt en la carpeta del proyecto)
  * @param  None
  * @retval None
  */
void USART_ClearScreen(USART_TypeDef* USARTx) //PUBLIC
{
  uint8_t cmd1[5] = {0x1B, '[', '2', 'J', '\0'};      // Clear screen
  uint8_t cmd2[4] = {0x1B, '[', 'f', '\0'};           // Cursor home

  USART_SendString(USARTx, cmd1);
  USART_SendString(USARTx, cmd2);
}

/**
  * @brief  Permite mostrar información del µC en PUTTY. Used togheter with USART_SendString
  *         Implementation using Polling since we control when we want to send data in code
  * @param  None
  * @retval None
  */
void USART_SendString (USART_TypeDef* USARTx, const uint8_t* str) //PUBLIC
{
    while(*str) //all string in C finish with a \0 (NULL o 0x00)
    {
        if(*str == '\n')
        {
            USART_SendChar(USARTx, '\r');
        }
        USART_SendChar(USARTx, *str++);
    }

    /* Polling until TC=1 (Transmission completed bit set to 1) */
    USART_TC_Polling(USARTx);
}
/**
  * @brief  Print an unsigned number
  * @param
  * @retval None
  */
void PrintUnsignedVariable (uint32_t num) //PUBLIC
{
   uint32_t resto;
   uint32_t div = 1000000000;
   char numero[11];
   uint8_t i = 0;

   while(1)
   {
      resto = num % div;
      if(resto != num) //when div > num, num % div = num
         break;
      div /= 10;
   }

   while(1)
   {
      numero[i++] = (char)((num / div) & 0x0F) + 0x30;
      num %= div;
      if(num < 10)
      {
         numero[i++] = (char)(num & 0x0F) + 0x30;
         //numero[i] = '\0'; --> lo pone el ¿compilador? automáticamente
         break;
      }
      div /= 10;
   }
   
#ifdef STM32F10X_MD //OpenCM0.94(Exp_485) -USART1-
   USART_SendString (USART1, (const uint8_t*) numero);
#endif
   
#ifdef STM32F10X_HD//CM-530 -USART3-
   USART_SendString (USART3, (const uint8_t*) numero);
#endif
}
/**
  * @brief  Print an Hex number of 8bits
  * @param
  * @retval None
  */
void Printu8h(uint8_t Num) //Print hex number 8bits long
{
   //Num is an hex number
   char out[5];
   out[0] = '0';
   out[1] = 'x';
   out[4] = '\0';

   out[2] = (char)((Num>>4) & 0x0F) + 0x30;
   if(out[2] > '9')
      out[2] += 7; //pues hay 7 signos ASCII entre el '9' y 'A'

   out[3] = (char)(Num & 0x0F) + 0x30;
   if(out[3] > '9')
      out[3] += 7;
   
#ifdef STM32F10X_MD //OpenCM0.94(Exp_485) -USART1-
   USART_SendString (USART1, (const uint8_t*) out);
#endif
   
#ifdef STM32F10X_HD//CM-530 -USART3-
   USART_SendString (USART3, (const uint8_t*) out);
#endif
}
/** @defgroup ---------- UART Low level functions ----------
  * @{
  */
/**
  * @brief  Send uint8_t data to USART shift register.
  *         Used togheter with USART_SendString
  * @param  None
  * @retval None
  */
void USART_SendChar (USART_TypeDef* USARTx, const uint8_t c) //PRIVATE
{
    /* polling untill TXE bit is 1 (data is transferred to the shift register) */
    while(USART_GetFlagStatus(USARTx, USART_FLAG_TXE) != SET)
    { 
        __NOP();
    }
    USART_SendData(USARTx, c);
    //printf("%c", c);
}
/**
  * @brief  Disable TE(Transmissor enable) & RE(Receptor enable) when TC (Transmission completed)
  * @param  None
  * @retval None
  */
void USART_TC_Polling(USART_TypeDef* USARTx) //PRIVATE
{
    Timeout = USART_TIMEOUT;
   
    /* Polling until TC=1 (Transmission completed bit set to 1) */
    while(USART_GetFlagStatus(USARTx, USART_FLAG_TC) == RESET)
    {
       if(!Timeout--)USART_CallOutTime();
    }
    USART_ClearFlag(USARTx, USART_FLAG_TC);
}

/**
  * @brief  xxxxxxxx
  * @param  None
  * @retval None
  */
void USART_CallOutTime(void) //PRIVATE
{
  /* Uncomment next line to know the Link Register (LR) from the stack */
  // __ASM volatile("BKPT #01");
  /* We can not use __FILE__ and __LINE__ because they got their value @ preprocessing */
     printf("\nUSART TimeOut !!");

  /* Infinite loop */
  while (1)
  {

  }
}
/** @defgroup ---------- UART Interruption functions ----------
  * @{
  */

