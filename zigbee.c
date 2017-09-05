/**
  ******************************************************************************
  * @file    zigbee.c
  * @author  FX_Team
  * @version V1.0.1
  * @date    01-April-2017
  * @brief   This file provides firmware functions to manage the following
  *          functionalities of the Zigbee/RC-100 :
  * @version 1.01 - 31.03.17 => FIFO queue with pointers in USART2_ZBEE_Interrupt
  *          and USART5_ZBEE_Interrupt with variable Buffer from ZBEE_FIFO_SIZE
  *            + xxxxxxxxxxxxx
  *            + xxxxxxxxxxxxx
  *            + xxxxxxxxxxxxx
  ******************************************************************************

  ==============================================================================
                       ##### How to use this driver #####
  =============================================================================
  * >> When using STM32F1 Zigbee gets connected to USART2
  * >> When using CM-530 Zigbee gets connected to UART5
  * >> ZBEE_FIFO_SIZE --> define size of the FIFO queue
  * >> ZBEE_BAUDRATE = 56700 según ficha técnica Zigbee
  * >> Para encenderlo GPIO_ResetBits(GPIOA, GPIO_Pin_12) y para apagarlo
  *    GPIO_SetBits(GPIOA, GPIO_Pin_12). Esto es solo asi en el CM-530
  *    (funciona al revés, igual que las LEDs, jodidos koreanos !!
  *    Tb podemos conectarlo directamente a 3V y solucionado.
  * >> La led del zigbee parpadea sin parar hasta que se sincroniza con
  *    el Radio Controller, que se queda encendida.
  * >> El Zigbee tiene 4 cables de salida siendo el #4 el cable gris:
       1- RXD: Conectarlo al TXD del µc
       2- TXD: Conectarlo al RXD del µc
       3- VCC: Conectarlo al pin de 3V de la STM32F1 o a cualquier pin de la STM32F3.
       Por lo visto los pins de la STM32F1 no llegan a 3V por lo que no se enciende el Zigbee (la luz roja no se enciende)
       4- GND: Conectarlo al GND del µc
  * >> El RC-100 añade el CMD 2 (Balance) despues de cada comando enviado x el usuario
  
  ==============================================================================
                       ##### Consideraciones #####
  =============================================================================
  * >> El RC-100 suele enviar los comandos duplicados/triplicados. Además 
  *    añade el cmd #2 (Balance) despues de cada comando enviado x el usuario. 
  *    Vamos a recuperar el 1er cmd recibido y en caso de que el cmd sea el #2
  *    lo eliminaremos.
  * >> ZBEE_FlagCheckStatus() lee valores de la FIFO con cada ejecución, de manera
  *    que con cada ejecución la global value cmd se va actualizando mientras
  *    MTN_ExecuteCmd() se termina de ejecutar. Esto es perfecto pues el cmd que 
  *    se pasa a MTN_ExecuteCmd() el prácticamente el último enviado x el usuario
  *    mediante el RC-100. En esto también importa el tamaño de la FIFO.  

  *****************************************************************************/

/******************************************************************************/
/*            ZIGBEE FILE - IMPORT                                            */
/******************************************************************************/
/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include "zigbee.h"
#include "globals.h"

/******************************************************************************/
/*            ZIGBEE FILE - PRIVATE DECLARATION                               */
/******************************************************************************/
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define ZBEE_TEST_CENTER_FAIL              0
#define ZBEE_TEST_CENTER_OK                1
#define ZBEE_RXBUFFER_OVERRUN              3
#define ZBEE_RXBUFFER_RTO                  4
#define ZBEE_FLAG_READY                    5
#define ZBEE_PACKET_LENGTH                 6
#define ZBEE_FIFO_FULL                     7
#define ZBEE_FIFO_EMPTY                    8
#define ZBEE_FIFO_PUT                      9
#define ZBEE_CMD_BALANCE                   10

#define PACKET_BYTE_0                      0
#define PACKET_BYTE_1                      1
#define PACKET_BYTE_2                      2
#define PACKET_BYTE_3                      3
#define PACKET_BYTE_4                      4
#define PACKET_BYTE_5                      5

#define ZBEE_BAUDRATE                      56700 //según ficha técnica zigbee

//1CMD = 6 bytes: le damos un tamaño pequeño pues al ser FIFO se van leyendo los + antiguos primero
//y no queremos que haya mucho desfase entre los que se ejecutan y los que envia el user x RC-100
#define ZBEE_FIFO_SIZE                     42    //multiple of 6 
                                             

//RC-100 button key values
// When more than 2 buttons are pressed, the sum of pressed code values will be sent.
#define GARBAGE       (0X00)                       //RC-100 adds it at the end of each transfer :(
#define BTN_U	   	 (0x01)                       //cmd #3  - Forward
#define BTN_D		    (0x02)                       //cmd #13 - Backwards
#define BTN_L		    (0x04)                       //cmd #27 - Turn left 
#define BTN_R	       (0x08)                       //cmd #28 - Turn right
#define BTN_1		    (0x10)                       //cmd #45 - Forward punch attack (fight mode)
#define BTN_2     	 (0x20)                       //cmd #41 - Left defense (soccer) / cmd #49 - Left punch (fight)
#define BTN_3		    (0x40)                       //cmd #46 - Forward jump?? (fight)
#define BTN_4	       (0x80)                       //cmd #42 - Right defense (soccer) / cmd #50 - Right punch (fight)
#define BTN_5	       (0x100)                      //cmd #51 - Right punch combo (fight mode)
#define BTN_6	       (0x200)                      //cmd #52 - Left punch combo (fight mode)
#define BTN_5_AND_6	(BTN_5 + BTN_6)               //0x300 - Defense ready (soccer) - Defense (fight)
#define BTN_U_AND_L	(BTN_U + BTN_L)               //0x05 - Walk forward left
#define BTN_U_AND_R	(BTN_U + BTN_R)               //0x09 - Walk forward right
#define BTN_D_AND_L	(BTN_D + BTN_L)               //0x06 - 
#define BTN_D_AND_R	(BTN_D + BTN_R)               //0x0A -
#define BTN_U_AND_1	(BTN_U + BTN_1)               //0x11 - Forward get up      
#define BTN_D_AND_1	(BTN_D + BTN_1)               //0x12 - Backwards get up
#define BTN_L_AND_1	(BTN_L + BTN_1)               //0x14 - 
#define BTN_R_AND_1	(BTN_R + BTN_1)               //0x18
#define BTN_U_AND_2	(BTN_U + BTN_2)               //0x21 - Chutar con la izquierda (soccer mode)
#define BTN_D_AND_2	(BTN_D + BTN_2)               //0x22 - Coz con la izquierda (soccer mode)
#define BTN_L_AND_2	(BTN_L + BTN_2)               //0x24 - Patada lateral izquierda (soccer mode)
#define BTN_R_AND_2	(BTN_R + BTN_2)               //0x28 - Pase a la derecha con la izquierda (soccer mode)
#define BTN_U_AND_4	(BTN_U + BTN_4)               //0x81 - Chutar con la derecha (soccer mode)
#define BTN_D_AND_4	(BTN_D + BTN_4)               //0x82 - Coz con la derecha (soccer mode)
#define BTN_L_AND_4	(BTN_L + BTN_4)               //0x84 - Patada lateral derecha (soccer mode)
#define BTN_R_AND_4	(BTN_R + BTN_4)               //0x88 - Pase con la izquierda a la derecha (soccer mode)
#define BTN_U_AND_6	(BTN_U + BTN_6)               //0x201 - Fast forward
#define BTN_R_AND_5	(BTN_R + BTN_5)               //0x108 - Right sidestep
#define BTN_L_AND_5	(BTN_L + BTN_5)               //0x104 - Left sidestep
#define BTN_R_AND_5_AND_6	(BTN_R + BTN_5 + BTN_6) //0x208 - Fast right sidestep
#define BTN_L_AND_5_AND_6	(BTN_L + BTN_5 + BTN_6) //0x304 - Fast left sidestep
#define BTN_R_AND_U_AND_5	(BTN_R + BTN_U + BTN_5) //0x109 - Right forward diagonal step
#define BTN_R_AND_D_AND_5	(BTN_R + BTN_D + BTN_5) //0x10A - Right backwards diagonal step
#define BTN_L_AND_U_AND_5	(BTN_L + BTN_U + BTN_5) //0x105 - Left forward diagonal step
#define BTN_L_AND_D_AND_5	(BTN_L + BTN_D + BTN_5) //0x106 - Left backwards diagonal step

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static uint8_t g_ZbeePacket[6];
static uint8_t g_ZbeePacketPointer; //Pointer for g_ZbeePacket[6]
static uint16_t g_ZbeePacketData;   //Command code from g_ZbeePacket[6]
static volatile uint8_t g_ZbeeFlagPacketReady;
static uint8_t Zbee_FIFO[ZBEE_FIFO_SIZE];
static uint8_t *gp_ZbeeRxBufferPut; //FIFO pointer, initialization @ ZBEE_Initialize()
static uint8_t *gp_ZbeeRxBufferGet; //FIFO pointer, initialization @ ZBEE_Initialize()
static uint8_t cmd_backup;      

/* Private function prototypes -----------------------------------------------*/
static void UNUSED_FUNCTION USART2_initialize(void);
static void UNUSED_FUNCTION USART5_initialize(void);
static uint8_t ZBEE_ReadRxBuffer(void);
static uint8_t ZBEE_FIFO_queue(void);
static uint16_t ZBEE_PacketData(void);
static void ZBEE_ErrorHandler(uint8_t ErrorType);
static uint8_t ZBEE_TestCenterRxBytes(uint8_t* ZbeePacket, uint8_t ZbeeBufferPointer);

/******************************************************************************/
/*            ZIGBEE FILE - FUNCTION DEFINITION                               */
/******************************************************************************/
/** @defgroup ---------- SetUp ZigBee functions ----------
  * @{
  */
/**
  * @brief  USART2 in duplex mode serial for Zigbee connection.
  *         Utilizamos esta en lugar de UART5 por que la OpenCM0.94 solo tiene 3 USART´s
  * @param
  * @retval None
  */
void UNUSED_FUNCTION USART2_initialize(void) //Private function Zigbee <OpenCM0.94>
{
   /* ===== USART2 TIME BLOCK & INTERRUPTIONS CONFIGURATION ===== */
   /* Enable USART2 clock */
   RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2 , ENABLE);

   /* Enable Port A (USART2_Tx) and (USART2_Rx) clock */
   RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

   /* Enable APB2 peripheral AFIO clock */
   RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

   /* Configure and set USART2 Interrupt to the lowest priority */
   NVIC_InitTypeDef NVIC_InitStructure;
   NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
   NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;
   NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;
   NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
   NVIC_Init(&NVIC_InitStructure);


	/* ===== GPIOS CONFIGURATION FOR USART2 ===== */
   /* Configure and set USART2_Tx (PA2) as input floating */
   GPIO_InitTypeDef   GPIO_InitStructure;
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
   GPIO_Init(GPIOA, &GPIO_InitStructure);

   /* Configure and set USART2_Rx (PA3) as input floating */
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
   GPIO_Init(GPIOA, &GPIO_InitStructure);


	/* ===== USART2 CONFIGURATION & SET ===== */
   /* Configure and set USART2 */
   USART_InitTypeDef   USART_InitStructure;
   USART_InitStructure.USART_BaudRate = ZBEE_BAUDRATE;
   USART_InitStructure.USART_WordLength = USART_WordLength_8b;
   USART_InitStructure.USART_StopBits = USART_StopBits_1;
   USART_InitStructure.USART_Parity = USART_Parity_No ;
   USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
   USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
   USART_Init(USART2, &USART_InitStructure);


	/* ===== CUSTOM CONFIGURATION ===== */
   /* Enable USART2 peripheral */
   USART_Cmd(USART2, ENABLE);

   /* Enable USART2 Interruptions */
   USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
   //USART_ITConfig(USART2, USART_IT_TXE, ENABLE); //Zigbee solo recibe, nunca envia
   //USART_ITConfig(USART2, USART_IT_TC, ENABLE);

   /* Enable DMA Management for USART2 */
   //USART_DMACmd(USART2, USART_DMAReq_Tx, ENABLE);
}

/**
  * @brief  UART5 in duplex mode serial for ZigBee.
  *         Los pins UART5_Tx y UART5_Rx son iguales para STM32F103 y STM32F303
  *         La configuración de GPIO_InitStructure es diferente para STM32F103 y STM32F303
  * @param  TX is configured as alternate function open-drain with an external pull-up para STM32F103
  * @retval None
  */
void UNUSED_FUNCTION UART5_initialize(void) //Private function Zigbee <CM-530>
{
#ifdef STM32F10X_HD //necesario pues USART5 no existe en STM32F10X_MD (da fallo al compilar)
   /* ===== UART5 TIME BLOCK & INTERRUPTIONS CONFIGURATION ===== */
   /* Enable UART5 clock */
   RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5 , ENABLE);

   /* Enable Port C (UART5_Tx) clock */
   RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

   /* Enable Port D (UART5_Rx) clock */
   RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);

   /* Enable APB2 peripheral AFIO */
   RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

   /* Configure and set UART5 Interrupt to the lowest priority */
   NVIC_InitTypeDef NVIC_InitStructure;
   NVIC_InitStructure.NVIC_IRQChannel = UART5_IRQn;
   NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;
   NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;
   NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
   NVIC_Init(&NVIC_InitStructure);


	/* ===== GPIOS CONFIGURATION FOR UART5 ===== */
   /* Configure and set UART5_Tx (PC12) as alternate function push-pull */
   GPIO_InitTypeDef   GPIO_InitStructure;
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
   GPIO_Init(GPIOC, &GPIO_InitStructure);

   /* Configure and set UART5_Rx (PD2) as alternate function push-pull */
   GPIO_InitStructure.GPIO_Pin = GPIO_Mode_IN_FLOATING;
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
   GPIO_Init(GPIOD, &GPIO_InitStructure);


	/* ===== UART5 CONFIGURATION & SET ===== */
   /* Configure and set UART5 */
   USART_InitTypeDef   USART_InitStructure;
   USART_InitStructure.USART_BaudRate = ZBEE_BAUDRATE;
   USART_InitStructure.USART_WordLength = USART_WordLength_8b;
   USART_InitStructure.USART_StopBits = USART_StopBits_1;
   USART_InitStructure.USART_Parity = USART_Parity_No ;
   USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
   USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
   USART_Init(UART5, &USART_InitStructure);


	/* ===== CUSTOM CONFIGURATION ===== */
   /* Enables UART5´s Half Duplex communication */
   //USART_HalfDuplexCmd(UART5, ENABLE);

   /* Enable UART5 peripheral */
   USART_Cmd(UART5, ENABLE);

   /* Enable UART5 Interruptions */
   //USART_ITConfig(UART5, USART_IT_RXNE, ENABLE);
   //USART_ITConfig(UART5, USART_IT_TXE, ENABLE);
   //USART_ITConfig(UART5, USART_IT_TC, ENABLE);

   /* Enable DMA Management for UART5 */
   //USART_DMACmd(UART5, USART_DMAReq_Tx, ENABLE);
#endif /* STM32F10X_HD */
}

void ZBEE_Initialize(void) //Public function
{
   /* FIFO initialization */
   gp_ZbeeRxBufferPut = gp_ZbeeRxBufferGet = &Zbee_FIFO[0];

#ifdef STM32F10X_MD //OpenCM0.94
	USART2_initialize();
   //Parece que Zigbee se conecta directamente a Vcc

#else /* STM32F10X_HD */ //CM-530
	UART5_initialize();

   /* PA12 set up as Zigbee enable/disable switch */
   GPIO_InitTypeDef   GPIO_InitStructure;
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
   GPIO_Init(GPIOA, &GPIO_InitStructure);

   //ResetBits enable Zigbee !!!
   GPIO_ResetBits(GPIOA, GPIO_Pin_12);
#endif
}

void ZBEE_Terminate(void) //Public function
{
#ifdef STM32F10X_MD_VL //STM32F1
   /* Disable USART2 peripheral */
   USART_Cmd(USART2, DISABLE);

#else /* STM32F10X_HD */ //CM-530
   /* Disable USART5 peripheral */
   USART_Cmd(UART5, DISABLE);
   /* Disable Zigbee */
	GPIO_SetBits(GPIOA, GPIO_Pin_12);
#endif
}

/** @defgroup ---------- Zigbee Transmision functions ----------
  * @{
  */
FlagStatus ZBEE_FlagCheckStatus(void) // //Public function: No lee el buffer x completo, solo un CMD
{
	while(!g_ZbeeFlagPacketReady) //exit #1 (g_ZbeePacket[6] is Ok)
   {
      if(ZBEE_ReadRxBuffer() == ZBEE_FIFO_EMPTY) //exit #2 (Rx buffer empty)
         return RESET;
   }
   g_ZbeeFlagPacketReady = RESET;
   return SET;
}

uint8_t ZBEE_ReadRxBuffer(void) //Private function
{
   while(gp_ZbeeRxBufferGet != gp_ZbeeRxBufferPut) //Still remains data to be read from FIFO
   {
      g_ZbeePacket[g_ZbeePacketPointer] = *gp_ZbeeRxBufferGet++; //Reading from FIFO to ZbeePacket[]

      //Redirigir el puntero al inicio de FIFO al llegar al último elemento
      if(gp_ZbeeRxBufferGet == &Zbee_FIFO[ZBEE_FIFO_SIZE])
         gp_ZbeeRxBufferGet = &Zbee_FIFO[0];

      //Send new byte to TestCenter
      if(ZBEE_TestCenterRxBytes(g_ZbeePacket, g_ZbeePacketPointer) != ZBEE_TEST_CENTER_OK)
      {
         //Checking for PacketReady Flag
         if(g_ZbeeFlagPacketReady)
         {
            cmd_backup = cmd;         //important !!
            cmd = ZBEE_Interpreter(); //global variable cmd is updated
         
            if(cmd != GARBAGE) 
               return ZBEE_FLAG_READY; //exit #1
            else //Turn down Balance cmd -we don´t want to send to Executecmd()-
            {
               cmd = cmd_backup;
               g_ZbeeFlagPacketReady = !g_ZbeeFlagPacketReady; //Reset flag (keep reading from the next byte)
               return ZBEE_CMD_BALANCE; //exit #2 
            }
         }
         //Manage Test Error byte
         else 
         {
            //Turn down buffered bytes untill the first byte (0xFF) of the next command
            while(*gp_ZbeeRxBufferGet++ != 0xFF) //if matched incremented is NOT applied !!
            {
               //Redirigir el puntero al inicio de FIFO al llegar al último elemento
               if(gp_ZbeeRxBufferGet == &Zbee_FIFO[ZBEE_FIFO_SIZE])
                  gp_ZbeeRxBufferGet = &Zbee_FIFO[0];
               //Salir si no quedan + elementos en el buffer
               if(gp_ZbeeRxBufferGet == gp_ZbeeRxBufferPut)
                  return ZBEE_FIFO_EMPTY;
            }
            g_ZbeePacketPointer = 0; //Reset pointer if test fails.
            ZBEE_ErrorHandler(ZBEE_TEST_CENTER_FAIL);
         }
      }
      //if ZBEE_TEST_CENTER_OK remain in the while loop untill Error or g_ZbeeFlagPacketReady
   }
   return ZBEE_FIFO_EMPTY; //exit #3 (solo sale x aqui si no ha recibido comandos)
}

/** @defgroup ---------- ZigBee Interruptions ----------
  * @{
  */

uint8_t ZBEE_FIFO_queue(void) //Private function
{
   uint8_t tmp;
   
#ifdef STM32F10X_MD /* OpenCM0.94 */
   tmp = USART_ReceiveData(USART2);
#else /* CM-530 */
   tmp = USART_ReceiveData(UART5);
#endif
   
   *gp_ZbeeRxBufferPut++ = tmp;
   if(gp_ZbeeRxBufferPut == &Zbee_FIFO[ZBEE_FIFO_SIZE]) //Point to the last FIFO´s @
      gp_ZbeeRxBufferPut = &Zbee_FIFO[0];               //Back to the first FIFO´s @
   if(gp_ZbeeRxBufferPut == gp_ZbeeRxBufferGet)         //PUT = GET + FIFO_SIZE !!
      return ZBEE_FIFO_FULL;                            //empieza a sobreescribir en los registros + antiguos
   return ZBEE_FIFO_PUT;
}
/**
  * @brief  USART2_ZBEE_Interrupt: Interrupt Routine Service for OpenCM0.94.
  *         USART5_ZBEE_Interrupt: Interrupt Routine Service for CM-530.
  * @retval None
  */
void USART2_ZBEE_Interrupt(void) //Public function <OpenCM0.94>
{
   /* RNXE bit is set to 1 when data is ready to be read */
   if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
   {
      if(ZBEE_FIFO_queue() == ZBEE_FIFO_FULL)
         ZBEE_ErrorHandler(ZBEE_FIFO_FULL);
   }
   /* ORE bit is set to 1 when Overrun Error */
   if(USART_GetITStatus(USART2, USART_IT_ORE) != RESET)
   {
      ZBEE_ErrorHandler(ZBEE_RXBUFFER_OVERRUN);
      USART_ClearITPendingBit(USART2, USART_IT_ORE);
   }
   //USART_ClearITPendingBit(USART2, USART_IT_RXNE); //NO ACTIVAR (DANGER) !!!
}
/**
  * @brief  USART2_ZBEE_Interrupt: Interrupt Routine Service for OpenCM0.94.
  *         USART5_ZBEE_Interrupt: Interrupt Routine Service for CM-530.
  * @retval None
  */
void USART5_ZBEE_Interrupt(void) //Public function <CM-530>
{
   /* RNXE bit is set to 1 when data is ready to be read */
   if(USART_GetITStatus(UART5, USART_IT_RXNE) != RESET)
   {
      if(ZBEE_FIFO_queue() == ZBEE_FIFO_FULL)
         ZBEE_ErrorHandler(ZBEE_FIFO_FULL);
   }
   /* ORE bit is set to 1 when Overrun Error */
   if(USART_GetITStatus(UART5, USART_IT_ORE) != RESET)
   {
      ZBEE_ErrorHandler(ZBEE_RXBUFFER_OVERRUN);
      USART_ClearITPendingBit(UART5, USART_IT_ORE);
   }
}

/** @defgroup ---------- Auxiliary Zigbee functions ----------
  * @{
  */
/**
  * @brief  Reset FIFO afterwards to get the 1ft cmd -Zigbee cmd´s are sent double/triple-
  *         to drop the repeated cmd´s.
  */
void ZBEE_ResetFIFO(void)
{
   gp_ZbeeRxBufferPut = gp_ZbeeRxBufferGet = &Zbee_FIFO[0];
}
uint16_t ZBEE_PacketData(void) //Private function
{
   g_ZbeePacketData = (uint16_t)((g_ZbeePacket[4]<<8) & 0xFF00);
   g_ZbeePacketData += g_ZbeePacket[2];

   /* Reset global pointer */
   g_ZbeePacketPointer = 0;

   return g_ZbeePacketData; //return in hexadecimal format
}

/**
  * @brief  Return the command for the corresponding Zigbee command
  */
uint8_t ZBEE_Interpreter(void) //Public function
{
   uint8_t command;

   switch(ZBEE_PacketData())
   {
      case GARBAGE: //(0x00)   //cmd_backup = last correct cmd
         command = cmd_backup; //RC-100 adds it at the end of each transfer (put the first one)
         break;
      case BTN_U:
         command = FORWARD_R;
         break;
      case BTN_D:
         command = BACKWARDS_R;
         break;
      case BTN_L:
         command = TURN_LEFT;
         break;
      case BTN_R:
         command = TURN_RIGHT;
         break;
      case BTN_1:
         command = FM_PUNCH;
         break;
      case BTN_2:
         if(g_actionmode == FIGHTMODE)
         {
            command = FM_PUNCH_LEFT;
            break;
         }
         else 
         {
            command = SM_PUNCH_LEFT;
            break;
         }
      case BTN_3:
         command = FM_FORWARD_JUMP;
         break;
      case BTN_4:
         if(g_actionmode == FIGHTMODE)
         {
            command = FM_PUNCH_RIGHT;
            break;
         }
         else 
         {
            command = SM_PUNCH_RIGHT;
            break;
         }
      case BTN_5:
         command = FM_RIGHT_PUNCH_COMBO;
         break;
      case BTN_6:
         command = FM_LEFT_PUNCH_COMBO;
         break;
      case BTN_5_AND_6:
         if(g_actionmode == FIGHTMODE)
         {
            command = FM_DEFENSE;
            break;
         }
         else
         {
            command = SM_DEFENSE;
            break;
         }            
      case BTN_U_AND_L:
         command = WALK_FORWARD_LEFT;
         break;
      case BTN_U_AND_R:
         command = WALK_FORWARD_RIGHT;
         break;
      case BTN_U_AND_1:
         command = FORWARD_GET_UP;
         break;
      case BTN_D_AND_1:
         command = BACKWARDS_GET_UP;
         break;
      case BTN_U_AND_2:
         command = SM_KICK_FORWARD_LEFT;
         break;
      case BTN_D_AND_2:
         command = SM_KICK_BACKWARDS_LEFT;
         break;
      case BTN_L_AND_2:
         command = SM_LATERAL_KICK_LEFT;
         break;
      case BTN_R_AND_2:
         command = SM_LEFT_PASS;
         break;
      case BTN_U_AND_4:
         command = SM_KICK_FORWARD_RIGHT;
         break;
      case BTN_D_AND_4:
         command = SM_KICK_BACKWARDS_RIGHT;
         break;
      case BTN_L_AND_4:
         command = SM_LATERAL_KICK_RIGHT;
         break;
      case BTN_R_AND_4:
         command = SM_RIGHT_PASS;
         break;
      case BTN_U_AND_6:
         command = FAST_FORWARD_R;
         break;
      case BTN_R_AND_5:
         command = RIGHT_SIDESTEP;
         break;
      case BTN_L_AND_5:
         command = LEFT_SIDESTEP;
         break;
      case BTN_R_AND_5_AND_6:
         command = FAST_RIGHT_SIDESTEP;
         break;
      case BTN_L_AND_5_AND_6:
         command = FAST_LEFT_SIDESTEP;
         break;
      case BTN_R_AND_U_AND_5:
         command = RIGHT_FORWARD_DIAGONAL;
         break;
      case BTN_R_AND_D_AND_5:
         command = RIGHT_BACKWARDS_DIAGONAL;
         break;
      case BTN_L_AND_U_AND_5:
         command = LEFT_FORWARD_DIAGONAL;
         break;
      case BTN_L_AND_D_AND_5:
         command = LEFT_BACKWARDS_DIAGONAL;
         break;
      default:
         command = cmd_backup; //NO deberiamos de pasar x aqui nunca.........
   }
   return command;
}
/** @defgroup ---------- Manage Zigbee Errors ----------
  * @{
  */
void ZBEE_ErrorHandler(uint8_t ErrorType) //Private function
{
#ifdef RELEASE
   return;
#endif

   //uint8_t ErrorMessage[80]; --> para sacar msg x USART3 en CM-530
   /* Checking parameters */
   assert_param(IS_ZBEE_ERROR_TYPE(ErrorType));
   /* To know the where ErrorHandler was called from insert a debugg
      breakpoint in this function and debug steb by step */
   switch (ErrorType)
   {
      case ZBEE_FIFO_FULL:
         printf("\nError Handler message: ZBEE_FIFO_FULL !!");
         break;
      case ZBEE_TEST_CENTER_FAIL:
         printf("\nError Handler message: ZBEE_TEST_CENTER_FAIL !!");
         break;
      case ZBEE_RXBUFFER_OVERRUN:
         printf("\nError Handler message: ZBEE_RXBUFFER_OVERRUN !!");
         break;
      case ZBEE_RXBUFFER_RTO:
         printf("\nError Handler message: ZBEE_RXBUFFER_RTO !!");
         break;
      default:
         printf("\nUnknown issues !!");
   }
   /* Block communication and all processes */
   // while (1){}
}

uint8_t ZBEE_TestCenterRxBytes(uint8_t* ZbeePacket, uint8_t ZbeeBufferPointer) //Private function
{
   uint8_t tmp;
   /* Checking parameters */
   assert_param(IS_ZBEE_POINTER(ZbeeBufferPointer));

      switch(ZbeeBufferPointer)
      {
         case(PACKET_BYTE_0):
            if(g_ZbeePacket[g_ZbeePacketPointer] == 0xFF)
               break;
            return ZBEE_TEST_CENTER_FAIL;
         case(PACKET_BYTE_1):
            if(g_ZbeePacket[g_ZbeePacketPointer] == 0x55)
               break;
            return ZBEE_TEST_CENTER_FAIL;
         case(PACKET_BYTE_2):
            break;
         case(PACKET_BYTE_3):
            tmp = ~g_ZbeePacket[3];
            if(ZbeePacket[2] == tmp)
               break;
            return ZBEE_TEST_CENTER_FAIL;
         case(PACKET_BYTE_4):
            break;
         case(PACKET_BYTE_5):
            tmp = ~g_ZbeePacket[5];
            if(ZbeePacket[4] == tmp)
            {
               g_ZbeeFlagPacketReady = 1; //importante !!
               return ZBEE_FLAG_READY;
            }
            return ZBEE_TEST_CENTER_FAIL;
         default:
            return ZBEE_TEST_CENTER_FAIL;
      }
      g_ZbeePacketPointer++; //importante !!
      return ZBEE_TEST_CENTER_OK; //when PacketByte_X test is correct
}
