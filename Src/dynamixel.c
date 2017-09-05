/*
  *****************************************************************************
  * @file    dynamixel.c
  * @author  Forex_Team
  * @version V1.0.2
  * @date    11-May-2017
  * @brief   This file provides firmware functions to manage the following
  *          functionalities of the dynamixel servos:
  *          + Protocol Type Half duplex Asynchronous Serial Communication (8bit,1stop,No Parity)
  *          + Instruction Packet: OXFF 0XFF ID LENGTH INSTRUCTION PARAMETER_1 …PARAMETER_N CHECK SUM
  *          + Status Packet: OXFF 0XFF ID LENGTH ERROR PARAMETER1 PARAMETER_2…PARAMETER_N CHECK SUM
  *			 + Library for Dynamixel AX-12

  ******************************************************************************

  ==============================================================================
            ##### How to connect the AX-12 to the Discovery board #####
  ==============================================================================
  * CONFIG #1
  * =========
  * >> Configurar USART como half-duplex, de manera que unicamente se utiliza Tx.
  * Esta salida del micro se conecta al cable data del Dynamixel, aplicandole voltaje al AX-12
  * desde el CM-530. Hay que añadir una resistencia de pull-up (a 5V) a la salida del Tx,
  * pues este pin se configura en Open-drain, de manera que cuando no transmite está en
  * alta impedancia (Hi-Z). Podemos comprobar que sin el pull up resistor la señal en este pin
  * fluctua sin parar, conectandolo al programa Terminal y veremos envios de bytes sin parar !!!

  * CONFIG #2
  * =========
  * >> En este caso configuramos USART como full-duplex, por lo que se utilizan tanto Tx como Rx.
  * Habrá que utilizar un line driver/buffer driver como el 74HC126A de Toshiba junto a 2
  * resistencias de pull up (10k). Para ver las conexiones desde el micro al line driver y
  * tb al AX-12 tenemos el tutorial https://www.newbiehack.com/MicrocontrollerDynamixelServo1.aspx
  * y el datasheet del 74HC126A en la carpeta programación/hardware

  * >> Siempre que veamos que un pin envia bytes (FF FC 00 .....) cuando NO debería de enviar nada
  * es muy probable que este en alta impedancia y lo que veamos en el logic analyser o Terminal
  * sean las fluctuaciones del pin. Por tano es necesario añadir/modificar las resistencia de
  * pull up/pull down (+/- ohmios) o modificar la tensión de los resistors
  * (5V funcionan mejor que 3V)

  ******************************************************************************
  ==============================================================================
                       ##### How to use this driver #####
  ==============================================================================
  * >> Single-wire half-duplex mode must be selected by setting the HDSEL bit in USARTx_CR3.
  *    When using half-duplex an external pull up resistor (10k) connected to VDD (++)
  *    must be inserted @ the Tx pin of the Discovery !!!! We could instead use a
  *    3 State Buffer Line Driver (74HC126 component) and an Hex Inverter (74HC04 component),
  *    that allows us to control the 74HC126 using just one pin. However, the 74HC04 can be
  *    a using a bit more of code.
  *
  * >> The TX and RX lines are internally connected (RX pin is no longer used)
  *
  * >> The TX pin is always released when no data is transmitted. Thus, it acts as a standard
  *    I/O in idle or in reception. It means that the I/O must be configured so that TX is
  *    configured as alternate function open-drain with an external pull-up.
  *
  * >> Check if a transmision is on going before sending a request:
  * 	 + STM32F303 using USARTx_ISR -> BUSY bit.
  *	 + STM32F103 using the global variable IsUsartBusy
  *
  * >> When instruction packet is transmitted with Broadcast ID = 0XFE, no status packets are returned
  *
  * >> STM32F303 hast Receive Timeout Register (RTO) that we could use to stop waiting
  *    for Status Packet. For the STM32F103 we need to implement a different solution
  *
  * >> There is not Buffer for Tx (only for Rx); we have to implement a while(){} in the main()
  *    to delay sending Instruction packet untill USARTx is FREE
  *
  * >> Comment/Uncomment #define TOSHIBA_74HC126A line depending on your assembly
  *
  ******************************************************************************

  ==============================================================================
                       ##### Montaje µC - 74HC126A - AX-12 #####
  ==============================================================================
  * >>Ver pdf del Bus Driver para referenciar los pins
  *
  * TC4HC126AP
  * pin #1  -----   Reset/Set (PB4)
  * pin #2  -----   USART1_Tx (PB6)
  * pin #3  -----   pin #12  -----  AX-12  ----- pullup resistor(5V)
  * pin #7  -----   GND
  *
  * pin #14  -----  VCC(5V)
  * pin #13  -----  Reset/Set (PB5)
  * pin #12  -----  pin #3
  * pin #11  -----  USART2_Rx (PB7)  -----  pullup resistor(5V)
  *
  ******************************************************************************

  ==============================================================================
                       ##### TODO LIST #####
  ==============================================================================
  * >> DXL_ACTION() & DXL_WRITE_BUFFER()
  * >> DXL_SYNC_WRITE () testing
  * >> No permitir utilizar DXL_WriteByte para editar la @Status Return Level,
  *    hay que utilizar  StatusPacket_Cmd, sino g_StatusReturnLevel[NUM_DXL_MAX]
  *    no se actualiza y da el error STATUS_RETURN_DISABLED.
  *
  ******************************************************************************

  ==============================================================================
                       ##### BUG CATCHER PROTOCOL FOR AX-12 #####
  ==============================================================================
  * >> Servo no gira
  * Comprobar cables conectados, USARTx(Baudrate) = AX-12(Baudrate), ID correcto
  * Comprobar que el Instruction packet que se envia es correcto desde Keil 
  * utilizando watch variables. Para comprobar el Baudrate es mejor hacerlo 
  * mediante Roboplus terminal
  *
  * >> Servo acepta write/ping/reset instructions pero no acepta read ni envia status packets
  * Comprobar que ID < 0xFE (Broadcasting)
  * Comprobar que AX-12 Status return level (0x10). Si esta desconectado (no se 
  * puede saber pues no responde), por lo que mediante RoboplusTerminal, 
  * seleccionar el servo mediante CID x (siendo x el servo ID) y aplicarle el
  * comando (write 16 2) y hacerle ping al servo para comprobar que se ha 
  * habilitado correctamente la respuesta del servo. 
  *
  *****************************************************************************/

/******************************************************************************/
/*            DYNAMIXEL FILE - IMPORT                                         */
/******************************************************************************/
/* Includes ------------------------------------------------------------------*/
#include "dynamixel.h"
#include "globals.h" //do not remove!
#include "printout.h"
/******************************************************************************/
/*            DYNAMIXEL FILE - PRIVATE DECLARATION                            */
/******************************************************************************/
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define  DXL_TIMEOUT                   (0XFFFF)

#define  TX_PARAM_MAX	               ((6+1)*NUM_DXL_MAX) //(L+1)*4, from SyncWrite()
#define  RX_PARAM_MAX	               (2) //We only allow to read 1 byte/word at a time

#define  USART_FREE                    (0)
#define  USART_BUSY                    (1)
#define  USART_TX_TIMEOUT              (2)
#define  USART_TC_TIMEOUT              (3)
#define  USART_TX_SUCCESS              (4)
#define  USART_RX_TIMEOUT              (5)
#define  USART_RX_OVERRUN              (6)
#define  USART_RX_COMPLET              (7)
#define  USART_RX_MISSING              (8)
#define  USART_RX_CORRUPT              (9)
#define  USART_ERROR_BIT               (10)
#define  USART_RX_SUCCESS              (11)
#define  USART_BROADCASTING            (12)
#define  STATUS_RETURN_DISABLED        (13)

#define  DXL_ERROR_BIT_0  (uint8_t)    0x01
#define  DXL_ERROR_BIT_1  (uint8_t)    0x02
#define  DXL_ERROR_BIT_2  (uint8_t)    0x04
#define  DXL_ERROR_BIT_3  (uint8_t)    0x08
#define  DXL_ERROR_BIT_4  (uint8_t)    0x10
#define  DXL_ERROR_BIT_5  (uint8_t)    0x20
#define  DXL_ERROR_BIT_6  (uint8_t)    0x40

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
// Private variable = Static variable in C programming; its scope is restricted to the current file.
static uint8_t g_TransferStatus; //se actualiza a medica que se realizan etapas del transfer
static StatusReturnLevel_Typedef g_StatusReturnLevel[NUM_DXL_MAX+1]; //Status packet reply (YES/NO) form AX-12
static uint8_t g_DelayTimeOut; //DXL_delay_us(), values: 1, 0
static uint8_t g_InstructionPacket[TX_PARAM_MAX+10];
static uint8_t g_StatusPacket[RX_PARAM_MAX+6]; //0xFF, 0xFF, ID, Length, Error,{nParameters}, Checksum
static uint8_t g_InstructionPacketLength;
static uint8_t g_StatusPacketLength;
static volatile uint8_t g_NumPacketTrasmitted ; //se actualiza cuando se envia un byte x TXE Interrup
static volatile uint8_t g_NumPacketReceived ; //se actualiza cuando se recibe un byte x RXNE Interrup
static volatile uint16_t TimeOut;

/* Private function prototypes -----------------------------------------------*/
//Las que no declaramos en el header, de manera que solo se pueden acceder desde este archivo
static uint8_t DXL_SendData(USART_TypeDef *USARTx);
static uint8_t DXL_GetData(USART_TypeDef *USARTx);
static uint8_t DXL_IsBusy(USART_TypeDef* USARTx);
static uint8_t DXL_TestStatusPacket(uint8_t RxPacketLength);
static uint8_t DLX_ErrorStatusPacket(uint8_t byte);
static void DXL_TimeOutCall(uint8_t TimeOutType);
static uint8_t DXL_TC_Polling(USART_TypeDef* USARTx);
static void DXL_delay_us (uint32_t retardo_ms, volatile uint8_t* VariableToCompare1, uint8_t VariableToCompare2);
static uint8_t DXL_lowbyte(uint16_t word);
static uint8_t DXL_highbyte(uint16_t word);
static uint8_t DXL_TxByte(USART_TypeDef* USARTx, uint8_t byte);
static void DXL_RxByte(USART_TypeDef* USARTx);

/******************************************************************************/
/*            DYNAMIXEL FILE - FUNCTION DEFINITION                            */
/******************************************************************************/
/** @defgroup ---------- DXL Initialization ----------
  * @{
  */
/**
  * @brief  USART1 in Single-wire full-duplex mode for Dynamixel AX-12
  *         by setting the HDSEL bit in USARTx_CR3
  * @param  TX is configured as alternate function push-pull
  * @retval None
  */
void CM530_USART1_initialize(uint32_t Baudrate) //(PUBLIC) Dynamixel - µC (FULL-DUPLEX)
{
   /* Checking parameters */
	assert_param(IS_DXL_USART_BAUDRATE(Baudrate));

   /* ===== USART1 TIME BLOCK & INTERRUPTIONS CONFIGURATION ===== */
   /* Enable USART1 clock */
   RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

   /* Enable Port B (USART1_Tx) and (USART1_Rx) clock */
   RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

   /* Enable APB2 peripheral AFIO */
   RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

   /* Configure and set USART1 Interrupt to the lowest priority */
   // Set within NVIC_PriorityGroup_0 (IRQ group), ranked #0 (top priority)
   NVIC_InitTypeDef NVIC_InitStructure;
   NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
   NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;
   NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;
   NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
   NVIC_Init(&NVIC_InitStructure);


	/* ===== GPIOS CONFIGURATION FOR USART1 ===== */
   /* Configure and set USART1_Tx(PB6) as AF (FULL-DUPLEX) */
   GPIO_InitTypeDef   GPIO_InitStructure;
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
   //GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD; //OPEN-DRAIN when Half-Duplex
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; //PUSH-PULL when Full-Duplex
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
   GPIO_Init(GPIOB, &GPIO_InitStructure);

   /* Configure and set USART1_Rx(PB7) as INPUT FLOATING */
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
   GPIO_Init(GPIOB, &GPIO_InitStructure);

   /* Enable the USART1 Pins Software Remapping */
   GPIO_PinRemapConfig(GPIO_Remap_USART1, ENABLE);


	/* ===== USART1 CONFIGURATION & SET ===== */
   /* Configure and set USART1 */
   USART_InitTypeDef   USART_InitStructure;
   USART_InitStructure.USART_BaudRate = Baudrate;
   USART_InitStructure.USART_WordLength = USART_WordLength_8b;
   USART_InitStructure.USART_StopBits = USART_StopBits_1;
   USART_InitStructure.USART_Parity = USART_Parity_No ;
   USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
   USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
   USART_Init(USART1, &USART_InitStructure);


	/* ===== CUSTOM CONFIGURATION ===== */
   /* Enable USART1 peripheral */
   USART_Cmd(USART1, ENABLE);

   /* Enable USART1 Interruptions */
   USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);

   /* Enable DMA Management for USART1 */
   //USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);
}
/**
  * @brief  USART3 in Single-wire full-duplex mode for Dynamixel AX-12
  * @param  TX is configured as alternate function open-drain with an external pull-up.
  * @retval None
  */
void OpenCM_USART3_initialize(uint32_t Baudrate) //(PUBLIC) Dynamixel µC (FULL-DUPLEX)
{
   /* Checking parameters */
	assert_param(IS_DXL_USART_BAUDRATE(Baudrate));

   /* ===== USART3 TIME BLOCK & INTERRUPTIONS CONFIGURATION ===== */
   /* Enable USART3 clock */
   RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

   /* Enable Port B (USART3_Tx) and (USART3_Rx) clock */
   RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

   /* Enable Port C (DXL_DIR_3) clock */
   RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

   /* Enable APB2 peripheral AFIO */
   RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

   /* Configure and set USART3 Interrupt to the lowest priority */
   // Set within NVIC_PriorityGroup_0 (IRQ group), ranked #0 (top priority)
   NVIC_InitTypeDef NVIC_InitStructure;
   NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
   NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;
   NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;
   NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
   NVIC_Init(&NVIC_InitStructure);


	/* ===== GPIOS CONFIGURATION FOR USART3 ===== */
   /* Configure and set USART3_Tx(PB10) as AF (FULL-DUPLEX) */
   GPIO_InitTypeDef   GPIO_InitStructure;
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
   //GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD; //OPEN-DRAIN when Half-Duplex
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; //PUSH-PULL when Full-Duplex
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
   GPIO_Init(GPIOB, &GPIO_InitStructure);

   /* Configure and set USART3_Rx(PB11) as INPUT FLOATING */
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
   GPIO_Init(GPIOB, &GPIO_InitStructure);

   /* Configure and set PC14 (USART3 Line Buffer Controller) in output pushpull mode */
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
   GPIO_Init(GPIOC, &GPIO_InitStructure);

   /* Enable the USART1 Pins Software Remapping */
   //GPIO_PinRemapConfig(GPIO_Remap_USART1, ENABLE);


	/* ===== USART3 CONFIGURATION & SET ===== */
   /* Configure and set USART3 */
   USART_InitTypeDef   USART_InitStructure;
   USART_InitStructure.USART_BaudRate = Baudrate;
   USART_InitStructure.USART_WordLength = USART_WordLength_8b;
   USART_InitStructure.USART_StopBits = USART_StopBits_1;
   USART_InitStructure.USART_Parity = USART_Parity_No ;
   USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
   USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
   USART_Init(USART3, &USART_InitStructure);


	/* ===== CUSTOM CONFIGURATION ===== */
   /* Enables USART3´s Half Duplex communication */
   //USART_HalfDuplexCmd(USART3, ENABLE);

   /* Enable USART3 peripheral */
   USART_Cmd(USART3, ENABLE);

   /* Enable USART3 Interruptions */
   USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
   //USART_ITConfig(USART3, USART_IT_TC, ENABLE);

   /* Enable DMA Management for USART3 */
   //USART_DMACmd(USART3, USART_DMAReq_Tx, ENABLE);
}
/**
  * @brief  IMPORTANTE: USART1 se puede utilizar para TTL con los servos o también
  *         para conexión con el PC, por lo que en putty.c hay otras funciones. 
  *         NO es posible utilizar ambas a la vez !!
  * @param  TX is configured as alternate function open-drain with an external pull-up.
  * @retval None
  */
void OpenCM_USART1_initialize(uint32_t Baudrate) //(PUBLIC) Dynamixel / USB / PUTTY (FULL-DUPLEX)
{
   /* Checking parameters */
	assert_param(IS_DXL_USART_BAUDRATE(Baudrate));

   /* ===== USART1 TIME BLOCK & INTERRUPTIONS CONFIGURATION ===== */
   /* Enable USART1 clock */
   RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

   /* Enable Port B (USART1_Tx) and (USART1_Rx) clock */
   RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

   /* Enable APB2 peripheral AFIO */
   RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

   /* Configure and set USART1 Interrupt to the lowest priority */
   // Set within NVIC_PriorityGroup_0 (IRQ group), ranked #0 (top priority)
   NVIC_InitTypeDef NVIC_InitStructure;
   NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
   NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;
   NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;
   NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
   NVIC_Init(&NVIC_InitStructure);


	/* ===== GPIOS CONFIGURATION FOR USART1 ===== */
   /* Configure and set USART1_Tx(PB6) as AF (FULL-DUPLEX) */
   GPIO_InitTypeDef   GPIO_InitStructure;
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
   //GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD; //OPEN-DRAIN when Half-Duplex
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; //PUSH-PULL when Full-Duplex
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
   GPIO_Init(GPIOB, &GPIO_InitStructure);

   /* Configure and set USART1_Rx(PB7) as INPUT FLOATING */
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
   GPIO_Init(GPIOB, &GPIO_InitStructure);

   /* Configure and set PB5 (USART1 Line Buffer Controller) in output pushpull mode */
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
   GPIO_Init(GPIOB, &GPIO_InitStructure);

   /* Enable the USART1 Pins Software Remapping */
   GPIO_PinRemapConfig(GPIO_Remap_USART1, ENABLE);


	/* ===== USART1 CONFIGURATION & SET ===== */
   /* Configure and set USART1 */
   USART_InitTypeDef   USART_InitStructure;
   USART_InitStructure.USART_BaudRate = Baudrate;
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
   USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
   //USART_ITConfig(USART1, USART_IT_TC, ENABLE);

   /* Enable DMA Management for USART1 */
   //USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);
}
/** @defgroup ---------- DXL Data Transfer functions ----------
  * @{
  */
/**
  * @brief  Private function, called from Interruptions
  * @param
  * @retval
  */
void DXL_TxRxByte_It(USART_TypeDef* USARTx) //(PUBLIC) - ISR
{
   /* Checking parameters */
	assert_param(IS_DXL_USART(USARTx));

   /* TXE bit is set to 1 when data is ready to be sent */
   if(USART_GetITStatus(USARTx, USART_IT_TXE) == SET)
   {
      USART_SendData(USARTx, g_InstructionPacket[g_NumPacketTrasmitted++]);
   }
   /* RNXE bit is set to 1 when data is ready to be read */
   if(USART_GetITStatus(USARTx, USART_IT_RXNE) == SET)
   {
      g_StatusPacket[g_NumPacketReceived++] = USART_ReceiveData(USARTx);
   }
   /* ORE bit is set to 1 when Overrun Error */
   if(USART_GetITStatus(USARTx, USART_IT_ORE) != RESET)
   {
      g_TransferStatus = USART_RX_OVERRUN;
      USART_ClearITPendingBit(USARTx, USART_IT_ORE);
   }
}
/**
  * @brief Called from Instruction functions
  * @param
  * @retval
  */
uint8_t DXL_SendData(USART_TypeDef *USARTx) //(PRIVATE)
{
   TimeOut = DXL_TIMEOUT;
   /* Checking parameters */
	assert_param(IS_DXL_USART(USARTx));
   
   /* Control Internal Line Driver 74HC126A */
#ifdef STM32F10X_HD /* CM-530 */
   GPIO_SetBits(GPIOB, GPIO_Pin_4);   //USART1: TX Enable
   GPIO_ResetBits(GPIOB, GPIO_Pin_5); //USART1: RX Disable
#else /* OpenCM0.94 */
   GPIO_SetBits(GPIOB, GPIO_Pin_5);   //USART1: TX Enable & RX Disable
   GPIO_SetBits(GPIOC, GPIO_Pin_14);  //USART3: TX Enable & RX Disable
#endif

   /* ===== For lower baudrates we use Interruptions and Polling for higher ones ===== */
   //A altas velocidades Interruption mode no funciona bien, se raya !!!
   
   /* Sending Instruction packet using INTERRUPTION MODE (instead of polling) 
      with HIGHEST priority, to avoid Tx transfer corruption from Zigbee or other ISR */
   USART_ITConfig(USARTx, USART_IT_TXE, ENABLE); //Interruption gets active right away
   while(g_NumPacketTrasmitted != g_InstructionPacketLength) //loop
   {
      if(!TimeOut--)
      {
         USART_ITConfig(USARTx, USART_IT_TXE, DISABLE); //Disable Interruption
         DXL_TimeOutCall(USART_TX_TIMEOUT);
         return 0;
      }
   }
   USART_ITConfig(USARTx, USART_IT_TXE, DISABLE); //Disable Interruption. END INTERRUPTION MODE

   /* Sending Instruction using POLLING MODE
   while(g_NumPacketTrasmitted != g_InstructionPacketLength)
   {
      if(DXL_TxByte(USARTx, g_InstructionPacket[g_NumPacketTrasmitted++]) == USART_TX_TIMEOUT)
         return 0;
   }//END POLLING MODE */

   /* Waiting for TC=1 */
   if(DXL_TC_Polling(USARTx) != USART_TX_SUCCESS)
   {
      DXL_TimeOutCall(USART_TC_TIMEOUT);
      return 0;
   }

#ifdef STM32F10X_HD /* CM-530 */
   /* Control Internal Line Driver 74HC126A */
   GPIO_ResetBits(GPIOB, GPIO_Pin_4);  //USART1: TX Disable
   GPIO_SetBits(GPIOB, GPIO_Pin_5);    //USART1: RX Enable
#else /* OpenCM0.94 */
   GPIO_ResetBits(GPIOB, GPIO_Pin_5);  //USART1: RX Enable & TX Disable
   GPIO_ResetBits(GPIOC, GPIO_Pin_14); //USART3: RX Enable & TX Disable
#endif

   /* Tx packet send successfully */
   return USART_TX_SUCCESS;
}
/**
  * @brief Called from Instruction functions
  * @param
  * @retval
  */
uint8_t DXL_GetData(USART_TypeDef *USARTx) //(PRIVATE)
{
   /* Wait to receive status packet using Interruptions.
    * Return delay Time (default value: 160µs, could be configured from @ 0x05)
    * We add n ms to allow status packets to be received by interruptions */

   /* Add delay to give time to AX-12 to reply */
   DXL_delay_us(2, &g_NumPacketReceived, g_StatusPacketLength); //realmente son ms....
   //DXL_delay_us(100, g_StatusPacketLength);

   if(g_DelayTimeOut)
   {
      if(g_TransferStatus == USART_RX_OVERRUN)
         DXL_TimeOutCall(USART_RX_OVERRUN);
      else
         DXL_TimeOutCall(USART_RX_TIMEOUT);
      return 0;
   }
   return USART_RX_COMPLET;
}

/** @defgroup ---------- DXL Transfer Control functions ----------
  * @{
  */
/**
  * @brief  Check from Communication errors and printf them using DXL_TimeOutCall()
  * @param  None
  * @retval None
  */
uint8_t DXL_TestStatusPacket(uint8_t RxPacketLength) //(PRIVATE)
{
   /* Checking parameters */
   assert_param(IS_DXL_STATUSPACKET_LENGTH(RxPacketLength));

   uint8_t checksum = 0; //local variables not initilized to 0 by GCC
   uint8_t i; //loop counter

   /* Header packets */
   if(g_StatusPacket[0] != 0xFF || g_StatusPacket[1] != 0xFF)
   {
      DXL_TimeOutCall(USART_RX_CORRUPT);
      return USART_RX_CORRUPT;
   }

   /* ID packet */
   if(g_StatusPacket[2] != g_InstructionPacket[2])
   {
      DXL_TimeOutCall(USART_RX_CORRUPT);
      return USART_RX_CORRUPT;
   }

   /* Length packet */
   if(g_InstructionPacket[4] == INSTRUCTION_READ)
   {
      if(g_StatusPacket[3] != (g_InstructionPacket[6] + 0x02)) //Nº of parameters + 2 (Length code & Error code)
      {
         DXL_TimeOutCall(USART_RX_CORRUPT);
         return USART_RX_CORRUPT;
      }
   }
   else //For Ping, Write & Reset Instructions (Action y Sync_ do not send Status packets)
   {
      if(g_StatusPacket[3] != 0x02) //Length code & Error code
      {
         DXL_TimeOutCall(USART_RX_CORRUPT);
         return USART_RX_CORRUPT;
      }
   }

   /* Error packet (DXL_TimeOutCall() from DLX_ErrorStatusPacket()) */
   if(DLX_ErrorStatusPacket(g_StatusPacket[4]) != ERRBIT_NONE)
      return USART_ERROR_BIT;

   /* Checksum packet */
   for(i=2; i< (RxPacketLength-1); i++)
   {
      checksum += g_StatusPacket[i];
   }
   checksum = (~checksum);
   //if(checksum > 255)
   //   checksum &= ((uint8_t)0xFF); //only the low byte defines the checksum
   if(checksum != g_StatusPacket[RxPacketLength-1])
   {
      DXL_TimeOutCall(USART_RX_CORRUPT);
      return USART_RX_CORRUPT;
   }
   //All control bits are OK
   g_TransferStatus = USART_RX_SUCCESS;
   return USART_RX_SUCCESS;
}
/**
  * @brief  Call from DXL_TestStatusPacket()
  * @param  None
  * @retval None
  */
uint8_t DLX_ErrorStatusPacket(uint8_t byte) //(PRIVATE)
{
   if(byte & DXL_ERROR_BIT_0)
   {
      printf("\nID = %d: nInput Voltaje Error", g_StatusPacket[2]);
      g_TransferStatus = ERRBIT_VOLTAGE;
      return ERRBIT_VOLTAGE;
   }
   else if(byte & DXL_ERROR_BIT_1)
   {
      printf("\nID = %d: Angle Limit Error", g_StatusPacket[2]);
      g_TransferStatus = ERRBIT_ANGLE;
      return ERRBIT_ANGLE;
   }
   else if(byte & DXL_ERROR_BIT_2)
   {
      printf("\nID = %d: Overheating Error", g_StatusPacket[2]);
      g_TransferStatus = ERRBIT_OVERHEAT;
      return ERRBIT_OVERHEAT;
   }
   else if(byte & DXL_ERROR_BIT_3)
   {
      printf("\nID = %d: Range Error", g_StatusPacket[2]);
      g_TransferStatus = ERRBIT_RANGE;
      return ERRBIT_RANGE;
   }
   else if(byte & DXL_ERROR_BIT_4)
   {
      printf("\nID = %d: Checksum Error", g_StatusPacket[2]);
      g_TransferStatus = ERRBIT_CHECKSUM;
      return ERRBIT_CHECKSUM;
   }
   else if(byte & DXL_ERROR_BIT_5)
   {
      printf("\nID = %d: Overload Error", g_StatusPacket[2]);
      g_TransferStatus = ERRBIT_OVERLOAD;
      return ERRBIT_OVERLOAD;
   }
   else if(byte & DXL_ERROR_BIT_6)
   {
      printf("\nID = %d: Instruction Error", g_StatusPacket[2]);
      g_TransferStatus = ERRBIT_INSTRUCTION;
      return ERRBIT_INSTRUCTION;
   }
   else
   {
      g_TransferStatus = ERRBIT_NONE;
      return ERRBIT_NONE;
   }
}
/**
  * @brief  Basic management of the timeout situation.
  * @param  None.
  * @retval None.
  */
void DXL_TimeOutCall(uint8_t TimeOutType)  //(PRIVATE) - Active f(USE_FULL_ASSERT)
{
#ifdef  USE_FULL_ASSERT
   /* Checking parameters */
   assert_param(IS_DXL_RX_TIMEOUT(TimeOutType));

   /* To know the where DXL_TimeOutCall was called from insert a debugg
      breakpoint in this function and debug steb by step */

   char UNUSED_VAR error_msg[50];
   switch (TimeOutType)
   {
      case USART_BUSY:
         g_TransferStatus = USART_BUSY;
         sprintf(error_msg, "\nTimeOut issue: USART_BUSY");
         break;
      case USART_TX_TIMEOUT:
         g_TransferStatus = USART_TX_TIMEOUT;
         sprintf(error_msg, "\nTimeOut issue: USART_TX_TIMEOUT");
         break;
      case USART_TC_TIMEOUT:
         g_TransferStatus = USART_TC_TIMEOUT;
         sprintf(error_msg, "\nTimeOut issue: USART_TX_TIMEOUT");
         break;
      case USART_RX_TIMEOUT:
         g_TransferStatus = USART_RX_TIMEOUT;
         sprintf(error_msg, "\nTimeOut issue: USART_RX_TIMEOUT");
         break;
      case USART_RX_OVERRUN:
         g_TransferStatus = USART_RX_OVERRUN;
         sprintf(error_msg, "\nTimeOut issue: USART_RX_OVERRUN");
         break;
      case USART_RX_MISSING:
         g_TransferStatus = USART_RX_MISSING;
         sprintf(error_msg, "\nTimeOut issue: USART_RX_MISSING");
         break;
      case USART_RX_CORRUPT:
         g_TransferStatus = USART_RX_CORRUPT;
         sprintf(error_msg, "\nTimeOut issue: USART_RX_CORRUPT");
         break;
      case USART_BROADCASTING:
         g_TransferStatus = USART_BROADCASTING;
         sprintf(error_msg, "\nTimeOut issue: USART_BROADCASTING");
         break;
      case STATUS_RETURN_DISABLED:
         g_TransferStatus = STATUS_RETURN_DISABLED;
         sprintf(error_msg, "\nTimeOut issue: STATUS_RETURN_DISABLED");
         break;   
   }
#ifdef STM32F10X_HD /* CM-530 */
   USART_SendString (USART3, (const uint8_t*) error_msg);
#endif
   
#ifdef STM32F10X_MD /* OpenCM */
   printf("%s", error_msg);
#endif

#endif /* USE_FULL_ASSERT */
}

/** @defgroup ---------- DXL High Communication Methods ----------
  ==============================================================================
                       ##### Packets Configuration #####
  ==============================================================================
  READ_DATA INSTRUCTION (según User Manual AX-12)
  ************************************* 1 byte reading ******************************
    >>>>> Instruction packet <<<<<             | >>>>> Status packet <<<<<
    ->length = 0x04                            | ->length = 0x03
    ->lengthInstructionPacket = length + 0x04  | ->lengthStatusPacket = length + 0x04

  ************************************* 2 bytes reading ******************************
    >>>>> Instruction packet <<<<<             | >>>>> Status packet <<<<<
    ->length = 0x04                            | ->length = 0x04
    ->lengthInstructionPacket = length + 0x04  | ->lengthStatusPacket = length + 0x04

  WRITE_DATA INSTRUCTION (según User Manual AX-12)
  ************************************* 1 byte writing ******************************
    >>>>> Instruction packet <<<<<             | >>>>> Status packet <<<<<
    ->length = 0x05                            | ->length = 0x02
    ->lengthInstructionPacket = length + 0x04  | ->lengthStatusPacket = length + 0x04

  ************************************* 2 bytes writing ******************************
    >>>>> Instruction packet <<<<<             | >>>>> Status packet <<<<<
    ->length = 0x05                            | ->length = 0x02
    ->lengthInstructionPacket = length + 0x04  | ->lengthStatusPacket = length + 0x04
*/
/**
  * @brief  La función esta adaptada exclusivamente para modificar 6 registros:
  *         CW slope, CCW slope, position goal(L), position goal(H), speed(L) & speed(H).
  *         Ver el Manual del AX-12 (pag.23) para una explicación de los parámetros de la función.
  * @param  ID=0xFE, Length=(L+1)*4 + 4= (4+1)*4 + 4 = 0x18, Instruction=0x83,
  *         parameter1 = Starting @, parameter2 = (4 * NUM_DXL_MAX)
  *         RegLength = Number of register to write for each servo
  *         StartAddress = First register @ to write
  * @retval
  */
uint16_t DXL_SyncWrite(USART_TypeDef *USARTx, uint8_t IndexMotion, uint8_t IndexStep, uint8_t StartAddress, uint8_t RegLength) //(PUBLIC)
{
   /* Checking parameters */
	assert_param(IS_DXL_USART(USARTx));
	assert_param(IS_MOTION_ID(IndexMotion));
	assert_param(IS_MOTION_STEP(IndexStep));
	assert_param(IS_MOTION_ADDRESS(StartAddress));
	assert_param(IS_MOTION_LENGTH(RegLength));

   /* Checking for USART Bus status */
   if(DXL_IsBusy(USARTx) != USART_FREE)
   {
      DXL_TimeOutCall(USART_BUSY);
      return 0;
   }

   /* Blocking new incoming Tx transfers */
   g_TransferStatus = USART_BUSY;

   /* ----- SyncWrite does not return status packet ----- */
   /* Variables initialization */
   //PRIVATE
   uint8_t length = ((RegLength +1) * NUM_DXL_MAX) + 4;
   //uint8_t length = ((RegLength +1) * 2) + 4; -->  ELIMINAR, SOLO PARA DEBUGGING
   uint8_t parameter = 5;
   uint8_t checksum = 0; //Initialization of local variables
   //GLOBAL
   g_NumPacketTrasmitted = 0; //importante ponerlo a 0; se actualiza desde interruption
   g_InstructionPacketLength = length + 4; //tx_length + 0xFF + 0xFF + Broadcasting_ID + checksum

   /* Fill InstructionPacket[] = {0xFF, 0xFF, ID, LENGTH, INSTRUCTION #, PARAMETER0...N, CHECKSUM} */
   g_InstructionPacket[0] = 0xFF;
   g_InstructionPacket[1] = 0xFF;
   g_InstructionPacket[2] = BROADCAST_ID;           //0xFE
   g_InstructionPacket[3] = length;
   g_InstructionPacket[4] = INSTRUCTION_SYNC_WRITE; //0x83
   g_InstructionPacket[parameter++] = StartAddress; //0x1E(Goal Position) - 0x1C(Compliance Slope)
   g_InstructionPacket[parameter++] = RegLength;
   
   //Write Compliance Slope, Goal position & Moving Speed (6 registers) */
   if(StartAddress == CW_SLOPE)
   {
      for(uint8_t i=0; i<NUM_DXL_MAX; i++)
      {
         g_InstructionPacket[parameter++] = (i + 1); //no hay servo_ID = 0
         g_InstructionPacket[parameter++] = p_motion_table[IndexMotion]->torque_joints[i];
         g_InstructionPacket[parameter++] = p_motion_table[IndexMotion]->torque_joints[i];
         g_InstructionPacket[parameter++] = DXL_lowbyte (p_motion_table[IndexMotion]->position[IndexStep][i]);
         g_InstructionPacket[parameter++] = DXL_highbyte(p_motion_table[IndexMotion]->position[IndexStep][i]);
         g_InstructionPacket[parameter++] = DXL_lowbyte (MTN_GetSpeedMotion(IndexStep, i));
         g_InstructionPacket[parameter++] = DXL_highbyte(MTN_GetSpeedMotion(IndexStep, i));
      }
   }
   
      //Write Goal position (2 registers) -->  ELIMINAR, SOLO PARA DEBUGGING */
   if(StartAddress == GOALPOSITION)
   {
      for(uint8_t i=0; i<2; i++)
      {
         g_InstructionPacket[parameter++] = (i + 1); //no hay servo_ID = 0
         //g_InstructionPacket[parameter++] = p_motion_table[IndexMotion]->torque_joints[i];
         //g_InstructionPacket[parameter++] = p_motion_table[IndexMotion]->torque_joints[i];
         g_InstructionPacket[parameter++] = DXL_lowbyte (p_motion_table[IndexMotion]->position[IndexStep][i]);
         g_InstructionPacket[parameter++] = DXL_highbyte(p_motion_table[IndexMotion]->position[IndexStep][i]);
         //g_InstructionPacket[parameter++] = DXL_lowbyte (MTN_GetSpeedMotion(IndexStep, i));
         //g_InstructionPacket[parameter++] = DXL_highbyte(MTN_GetSpeedMotion(IndexStep, i));
      }
   }
   
	 /* Calculate checksum */
	 for(uint8_t i=2; i<g_InstructionPacketLength-1; i++)
	 {
		checksum += g_InstructionPacket[i]; //según Korea, checksum es uint8_t .......
	 }
 
	 /* Fill checksum within g_InstructionPacket[] */
	 g_InstructionPacket[parameter] = ~checksum; //parameter = g_InstructionPacketLength
   
   /* Send g_InstructionPacket[] */
   if(DXL_SendData(USARTx) != USART_TX_SUCCESS)
   {
      g_TransferStatus = USART_FREE; //Free flag for next transmission
      return 0;
   }

   /* g_InstructionPacket[] sent OK */
   g_TransferStatus = USART_FREE; //Free flag for next transmission
   return 1;
}
/**
  * @brief Instruction PING
  * @note
  * @retval Return g_StatusPacket[4] = Error packet
  */
int8_t DXL_Ping(USART_TypeDef *USARTx, uint8_t id) //(PUBLIC)
{
   /* Checking parameters */
	assert_param(IS_DXL_ID(id));
	assert_param(IS_DXL_USART(USARTx));

   /* Checking for ID Broadcasting */
   if(id == BROADCAST_ID)
   {
      DXL_TimeOutCall(USART_BROADCASTING);
      return -1;
   }

   /* Checking for USART Bus status */
   if(DXL_IsBusy(USARTx) != USART_FREE)
   {
      DXL_TimeOutCall(USART_BUSY);
      return -1;
   }

   /* Blocking new incoming Tx transfers */
   g_TransferStatus = USART_BUSY;

   /* ----- Ping status packet reports only the Error Byte, without parameters ----- */
   /* Variables initialization */
   //ver en este file @defgroup: DXL High Communication Methods
   uint8_t tx_length = 0x02; //Reference Manual AX-12
   uint8_t rx_length = 0x02; //Length and Error code bytes
   uint8_t checksum;
   g_NumPacketTrasmitted = 0; //importante ponerlo a 0; se actualiza desde interruption
   g_NumPacketReceived = 0; //importante ponerlo a 0; se actualiza desde interruption
   g_InstructionPacketLength = tx_length + 4; //tx_length + 0xFF + 0xFF + ID + checksum
   g_StatusPacketLength = rx_length + 4;
   checksum = ~(id + tx_length + INSTRUCTION_PING);

   /* InstructionPacket[] = {0xFF, 0xFF, ID, LENGTH, INSTRUCTION #, PARAMETER0...N, CHECKSUM} */
   g_InstructionPacket[0] = 0xFF;
   g_InstructionPacket[1] = 0xFF;
   g_InstructionPacket[2] = id;
   g_InstructionPacket[3] = tx_length;
   g_InstructionPacket[4] = INSTRUCTION_PING;
   g_InstructionPacket[5] = checksum;

   /* Send g_InstructionPacket[] */
   if(DXL_SendData(USARTx) != USART_TX_SUCCESS)
   {
      g_TransferStatus = USART_FREE; //Free flag for next transmission
      return -1;
   }

   /* Checking for Status Return Level */
   if(g_StatusReturnLevel[id] != REPLY_ALL)
   {
      DXL_TimeOutCall(STATUS_RETURN_DISABLED);
      g_TransferStatus = USART_FREE; //Free flag for next transmission
      return -1;
   }

   /* Get g_StatusPacket[] */
   if(DXL_GetData(USARTx) != USART_RX_COMPLET)
   {
      g_TransferStatus = USART_FREE; //Free flag for next transmission
      return -1;
   }

   /* Checking StatusPacket[] */
   if(DXL_TestStatusPacket(g_StatusPacketLength) != USART_RX_SUCCESS)
   {
      g_TransferStatus = USART_FREE; //Free flag for next transmission
      return -1;
   }

   /* StatusPacket[] is OK */
   g_TransferStatus = USART_FREE; //Free flag for next transmission
   return g_StatusPacket[4]; //Error packet
}
/**
  * @brief Instruction RESET
  * @note
  * @retval Return g_StatusPacket[4] = Error packet
  */
int8_t DXL_Reset(USART_TypeDef *USARTx, uint8_t id) //(PUBLIC)
{
   /* Checking parameters */
	assert_param(IS_DXL_ID(id));
	assert_param(IS_DXL_USART(USARTx));

   /* Checking for USART Bus status */
   if(DXL_IsBusy(USARTx) != USART_FREE)
   {
      DXL_TimeOutCall(USART_BUSY);
      return -1;
   }

   /* Blocking new incoming Tx transfers */
   g_TransferStatus = USART_BUSY;

   /* ----- Reset status packet reports only the Error Byte, without parameters ----- */
   /* Variables initialization */
   //ver en este file @defgroup: DXL High Communication Methods
   uint8_t tx_length = 0x02; //Reference Manual AX-12
   uint8_t rx_length = 0x02; //Length and Error code bytes
   uint8_t checksum;
   g_NumPacketTrasmitted = 0; //importante ponerlo a 0; se actualiza desde interruption
   g_NumPacketReceived = 0; //importante ponerlo a 0; se actualiza desde interruption
   g_InstructionPacketLength = tx_length + 4; //tx_length + 0xFF + 0xFF + ID + checksum
   g_StatusPacketLength = rx_length + 4;
   checksum = ~(id + tx_length + INSTRUCTION_PING);

   /* InstructionPacket[] = {0xFF, 0xFF, ID, LENGTH, INSTRUCTION #, PARAMETER0...N, CHECKSUM} */
   g_InstructionPacket[0] = 0xFF;
   g_InstructionPacket[1] = 0xFF;
   g_InstructionPacket[2] = id;
   g_InstructionPacket[3] = tx_length;
   g_InstructionPacket[4] = INSTRUCTION_RESET;
   g_InstructionPacket[5] = checksum;

   /* Send g_InstructionPacket[] */
   if(DXL_SendData(USARTx) != USART_TX_SUCCESS)
   {
      g_TransferStatus = USART_FREE; //Free flag for next transmission
      return -1;
   }

   /* Checking for Status Return Level */
   if(g_StatusReturnLevel[id] != REPLY_ALL)
   {
      DXL_TimeOutCall(STATUS_RETURN_DISABLED);
      g_TransferStatus = USART_FREE; //Free flag for next transmission
      return -1;
   }

   /* Get g_StatusPacket[] */
   if(DXL_GetData(USARTx) != USART_RX_COMPLET)
   {
      g_TransferStatus = USART_FREE; //Free flag for next transmission
      return -1;
   }

   /* Checking StatusPacket[] */
   if(DXL_TestStatusPacket(g_StatusPacketLength) != USART_RX_SUCCESS)
   {
      g_TransferStatus = USART_FREE; //Free flag for next transmission
      return -1;
   }

   /* StatusPacket[] is OK */
   g_TransferStatus = USART_FREE; //Free flag for next transmission
   return g_StatusPacket[4]; //Error packet
}
/**
  * @brief  To edit 1-byte register in Dynamixel AX-12 address table
  * @param  None
  * @retval None
  */
int8_t DXL_WriteByte(USART_TypeDef *USARTx, uint8_t id, uint8_t address, uint8_t WriteValue) //(PUBLIC)
{
   /* Checking parameters */
	assert_param(IS_DXL_ID(id));
   assert_param(IS_DXL_USART(USARTx));
	assert_param(IS_DXL_ADDRESS(address));
   assert_param(IS_DXL_BYTE_(WriteValue));

   /* Checking for USART Bus status */
   if(DXL_IsBusy(USARTx) != USART_FREE)
   {
      DXL_TimeOutCall(USART_BUSY);
      return -1;
   }

   /* Blocking new incoming Tx transfers */
   g_TransferStatus = USART_BUSY;

   /* Variables initialization */
   //ver @defgroup: DXL High Communication Methods
   uint8_t tx_length = 4;
   uint8_t rx_length = 2;
   uint8_t checksum = ~(id + tx_length + INSTRUCTION_WRITE + address + WriteValue);

   g_NumPacketTrasmitted = 0; //importante ponerlo a 0; se actualiza desde interruption
   g_NumPacketReceived = 0;   //importante ponerlo a 0; se actualiza desde interruption
   g_InstructionPacketLength = tx_length + 4; //tx_length + 0xFF + 0xFF + ID + checksum
   g_StatusPacketLength = rx_length + 4;

   /* InstructionPacket[] = {0xFF, 0xFF, ID, LENGTH, INSTRUCTION #, PARAMETER0...N, CHECKSUM} */
   g_InstructionPacket[0] = 0xFF;
   g_InstructionPacket[1] = 0xFF;
   g_InstructionPacket[2] = id;
   g_InstructionPacket[3] = tx_length;
   g_InstructionPacket[4] = INSTRUCTION_WRITE;
   g_InstructionPacket[5] = address;
   g_InstructionPacket[6] = WriteValue;
   g_InstructionPacket[7] = checksum;

   /* Send g_InstructionPacket[] */
   if(DXL_SendData(USARTx) != USART_TX_SUCCESS)
   {
      g_TransferStatus = USART_FREE; //Free flag for next transmission
      return -1;
   }

   /* Checking for ID Broadcasting */
   if(id == BROADCAST_ID)
   {
      g_TransferStatus = USART_FREE; //Free flag for next transmission
      return -1; //Broadcasting does not send status packet
   }

   /* Checking for Status Return Level */
   if(g_StatusReturnLevel[id] != REPLY_ALL)
   {
      DXL_TimeOutCall(STATUS_RETURN_DISABLED);
      g_TransferStatus = USART_FREE; //Free flag for next transmission
      return -1;
   }

   /* Get g_StatusPacket[] */
   if(DXL_GetData(USARTx) != USART_RX_COMPLET)
   {
      g_TransferStatus = USART_FREE; //Free flag for next transmission
      return -1;
   }

   /* Checking StatusPacket[] */
   if(DXL_TestStatusPacket(g_StatusPacketLength) != USART_RX_SUCCESS)
   {
      g_TransferStatus = USART_FREE; //Free flag for next transmission
      return -1;
   }

   /* StatusPacket[] is OK */
   g_TransferStatus = USART_FREE; //Free flag for next transmission
   return g_StatusPacket[4]; //Error packet
}
/**
  * @brief  To edit 2-bytes register in Dynamixel AX-12 address table
  * @param  None
  * @retval None
  */
int8_t DXL_WriteWord(USART_TypeDef *USARTx, uint8_t id, uint8_t address, uint16_t WriteValue) //(PUBLIC)
{
   /* Checking parameters */
	assert_param(IS_DXL_ID(id));
	assert_param(IS_DXL_ADDRESS(address));
	assert_param(IS_DXL_USART(USARTx));

   /* Checking for USART Bus status */
   if(DXL_IsBusy(USARTx) != USART_FREE)
   {
      DXL_TimeOutCall(USART_BUSY);
      return -1;
   }

   /* Blocking new incoming Tx transfers */
   g_TransferStatus = USART_BUSY;

   /* Variables initialization */
   //ver @defgroup: DXL High Communication Methods
   uint8_t tx_length = 5;
   uint8_t rx_length = 2;
   uint8_t parameter1;
   uint8_t parameter2;
   uint8_t checksum;

   /* Checking if 1 or 2 register are written */
   if(WriteValue > 0xFF)
   {
      parameter1 = DXL_lowbyte(WriteValue);  //Low byte
      parameter2 = DXL_highbyte(WriteValue); //High byte
   }
   else
   {
      parameter1 = WriteValue;
      parameter2 = 0;
   }

   g_NumPacketTrasmitted = 0; //importante ponerlo a 0; se actualiza desde interruption
   g_NumPacketReceived = 0; //importante ponerlo a 0; se actualiza desde interruption
   g_InstructionPacketLength = tx_length + 4; //tx_length + 0xFF + 0xFF + ID + checksum
   g_StatusPacketLength = rx_length + 4;
   checksum = ~(id + tx_length + INSTRUCTION_WRITE + address + parameter1 + parameter2);

   /* InstructionPacket[] = {0xFF, 0xFF, ID, LENGTH, INSTRUCTION #, PARAMETER0...N, CHECKSUM} */
   g_InstructionPacket[0] = 0xFF;
   g_InstructionPacket[1] = 0xFF;
   g_InstructionPacket[2] = id;
   g_InstructionPacket[3] = tx_length;
   g_InstructionPacket[4] = INSTRUCTION_WRITE;
   g_InstructionPacket[5] = address;
   g_InstructionPacket[6] = parameter1; //@@ según tabla de @´s: @xxx = Low, @xx+1 = High
   g_InstructionPacket[7] = parameter2;
   g_InstructionPacket[8] = checksum;

   /* Send g_InstructionPacket[] */
   if(DXL_SendData(USARTx) != USART_TX_SUCCESS)
   {
      g_TransferStatus = USART_FREE; //Free flag for next transmission
      return -1;
   }

   /* Checking for ID Broadcasting */
   if(id == BROADCAST_ID)
   {
      g_TransferStatus = USART_FREE; //Free flag for next transmission
      return -1; //Broadcasting does not send status packet
   }

   /* Checking for Status Return Level */
   if(g_StatusReturnLevel[id] != REPLY_ALL)
   {
      DXL_TimeOutCall(STATUS_RETURN_DISABLED);
      g_TransferStatus = USART_FREE; //Free flag for next transmission
      return -1;
   }

   /* Get g_StatusPacket[] */
   if(DXL_GetData(USARTx) != USART_RX_COMPLET)
   {
      g_TransferStatus = USART_FREE; //Free flag for next transmission
      return -1;
   }

   /* Checking StatusPacket[] */
   if(DXL_TestStatusPacket(g_StatusPacketLength) != USART_RX_SUCCESS)
   {
      g_TransferStatus = USART_FREE; //Free flag for next transmission
      return -1;
   }

   /* StatusPacket[] is OK */
   g_TransferStatus = USART_FREE; //Free flag for next transmission
   return g_StatusPacket[4]; //Error packet
}
/**
  * @brief Instruction READ DATA, from Dynamixel AX-12 Reference Manual
  * @param id -> Sensor ID
  * @param address -> Starting address of the location where the data is to be read
  * @param NumBytes -> Number of bytes/address to read from @param address
  * @retval
  */
uint16_t DXL_ReadData(USART_TypeDef *USARTx, uint8_t id, uint16_t address, uint8_t NumBytes) //(PUBLIC)
{
   /* Checking parameters */
	assert_param(IS_DXL_ID(id));
	assert_param(IS_DXL_ADDRESS(address));
	assert_param(IS_DXL_USART(USARTx));
	assert_param(IS_DXL_NUMBYTES(NumBytes));

   /* Checking for ID Broadcasting */
   if(id == BROADCAST_ID)
   {
      DXL_TimeOutCall(USART_BROADCASTING);
      return 0;
   }

   /* Checking for USART Bus status */
   if(DXL_IsBusy(USARTx) != USART_FREE)
   {
      DXL_TimeOutCall(USART_BUSY);
      return 0;
   }

   /* Blocking new incoming Tx transfers */
   g_TransferStatus = USART_BUSY;

   /* Variables initialization */
   uint8_t tx_length;
   uint8_t rx_length;
   uint8_t parameter2;
   uint8_t checksum;

   if(NumBytes == 1)
   {
      tx_length = 0x04; //ver @defgroup: DXL High Communication Methods
      rx_length = 0x03;
      parameter2 = 1; //length of the data to be read (1byte or 2 bytes)
   }
   else //NumBytes == 2
   {
      tx_length = 0x04;
      rx_length = 0x04;
      parameter2 = 2; //length of the data to be read (1byte or 2 bytes)
   }
   /* Same for NumBytes == 1 or NumBytes == 2 */
   g_NumPacketTrasmitted = 0; //importante ponerlo a 0; se actualiza desde interruption
   g_NumPacketReceived = 0; //importante ponerlo a 0; se actualiza desde interruption
   g_InstructionPacketLength = tx_length + 4; //tx_length + 0xFF + 0xFF + ID + checksum
   g_StatusPacketLength = rx_length + 4;
   checksum = ~(id + tx_length + INSTRUCTION_READ + address + parameter2);
   //if(checksum > 255)
   //   checksum &= ((uint8_t)0xFF);

   /* InstructionPacket[] = {0xFF, 0xFF, ID, LENGTH, INSTRUCTION #, PARAMETER0...N, CHECKSUM} */
   g_InstructionPacket[0] = 0xFF;
   g_InstructionPacket[1] = 0xFF;
   g_InstructionPacket[2] = id;
   g_InstructionPacket[3] = tx_length;
   g_InstructionPacket[4] = INSTRUCTION_READ; //0x02
   g_InstructionPacket[5] = address;
   g_InstructionPacket[6] = parameter2;
   g_InstructionPacket[7] = checksum;

   /* Send g_InstructionPacket[] */
   if(DXL_SendData(USARTx) != USART_TX_SUCCESS)
   {
      g_TransferStatus = USART_FREE; //Free flag for next transmission
      return 0;
   }

   /* Checking for Status Return Level */
   if(g_StatusReturnLevel[id] == NO_REPLY)
   {
      DXL_TimeOutCall(STATUS_RETURN_DISABLED);
      g_TransferStatus = USART_FREE; //Free flag for next transmission
      return 0;
   }

   /* Get g_StatusPacket[] */
   if(DXL_GetData(USARTx) != USART_RX_COMPLET)
   {
      g_TransferStatus = USART_FREE; //Free flag for next transmission
      return 0;
   }

   /* Checking StatusPacket[] */
   if(DXL_TestStatusPacket(g_StatusPacketLength) != USART_RX_SUCCESS)
   {
      g_TransferStatus = USART_FREE; //Free flag for next transmission
      return 0;
   }

   /* ----- CASE: NumBytes = 1 ----- */
   if(NumBytes == 1)
   {
      g_TransferStatus = USART_FREE; //Free flag for next transmission
      return g_StatusPacket[5];
   }

   /* ----- CASE: NumBytes = 2 (High & Low bytes) ----- */
   else //NumBytes == 2
   {
      g_TransferStatus = USART_FREE; //Free flag for next transmission
      return (((uint16_t)(g_StatusPacket[6]<<8)) | (uint8_t)g_StatusPacket[5]); //aqui "OR" = "+"
   }
}

/** @defgroup ---------- Query DXL functions ----------
  * @{
  */
/**
  * @brief  Printf value of the registers
  * @param  
  * @retval 
  */
void DXL_DumpRegisterValues(USART_TypeDef* USARTx) //(PUBLIC)
{
   //Return ID, BAUDRATE, RETURNDELAY, STATUS_RETURN & PRESENT_POSITION 
   
   uint8_t  tmp1=0, tmp2=0;
   uint16_t tmp3=0, tmp4=0;
   
   for(uint8_t id=1; id<(NUM_DXL_MAX+1); id++)
   {
      tmp1 = DXL_ReadData(USART_DXL, id, BAUDRATE, 1);
      tmp2 = DXL_ReadData(USART_DXL, id, RETURNDELAY, 1);
      tmp3 = DXL_ReadData(USART_DXL, id, STATUS_RETURN, 1);
      tmp4 = DXL_ReadData(USART_DXL, id, PRESENT_POSITION, 1);
      
      /* Print out register values */
      printf("\n=============== ID: %d ==============================", id);
      printf("\nBAUDRATE: %3d - RETURNDELAY: %3d - STATUS_RETURN: %3d - PRESENT_POSITION: %3d\n", tmp1, tmp2, tmp3, tmp4); 
      
      //PENDING: print out using putty
   }
   tmp1 = tmp2 = tmp3 = tmp4 = 0;
}
/**
  * @brief  
  * @param  
  * @retval DXL_MOVING (1) if any servo is moving, DXL_STOPPED(0) if all servos stopped
  */
uint8_t DXL_IsRobotInMotion(USART_TypeDef* USARTx)
{
   for(uint8_t id=1; id<(NUM_DXL_MAX+1); id++)
   {
      if(DXL_ReadData(USART_DXL, id, MOVING, 1))
         return DXL_MOVING;
   }
   return DXL_STOPPED;
}
/**
  * @brief  None
  * @param  None
  * @retval None
  */
uint8_t DXL_IsBusy(USART_TypeDef* USARTx) //(PRIVATE)
{
   TimeOut = DXL_TIMEOUT;

#ifdef STM32F303xC
   while(USART_GetFlagStatus(USARTx, USART_FLAG_BUSY) != RESET)
   {
      if(!TimeOut--)
         return USART_BUSY;
   }

#else /* STM32F10X_MD_VL (STM32F1), STM32F10X_MD (OpenCM) or STM32F10X_HD (CM-530) */
      while(g_TransferStatus != USART_FREE)
      {
         if(!TimeOut--)
            return USART_BUSY;
      }
#endif
   return USART_FREE;
}
/**
  * @brief  Communication Error & Error byte are printf from DXL_Ping()
  *         Si un servo no envia señal al no estar contectado muestra ERROR
  * @param  None
  * @retval None
  */
ErrorStatus DXL_Diagnosis(USART_TypeDef *USARTx)
{
   ErrorStatus tmp = SUCCESS;
   
   for(uint8_t id=1; id<(NUM_DXL_MAX+1); id++)
   {
      if(DXL_Ping(USART_DXL, id) == RETURN_ERROR)
      {
         //Release torque for the id servo if error
         DXL_WriteByte(USARTx, id, TORQUE_ENABLE, 0);
         tmp = ERROR;
      }
   }
   return tmp;
}



/** @defgroup ---------- SET DXL functions ----------
  * @{
  */
/**
  * @brief  Enables or disables the Status packet reply from AX-12
  * @param  RegValue: new state of the @ 0x10 (Status Return Level)
  *         This parameter can be: 0 (Do not reply to any instruction
  *                                1 (Reply only to Read_data instruction
  *                                2 (Respond to all instructions)
  * @retval None
  */
void StatusPacket_Cmd(USART_TypeDef *USARTx, uint8_t id, uint8_t RegValue) //(PUBLIC) 
{
   /* Checking parameters */
	assert_param(IS_DXL_USART(USARTx));
	assert_param(IS_DXL_ID(id));
   assert_param(IS_DXL_REGVALUE(RegValue));
   
   uint8_t tmp = 0;
   uint8_t tmp_max = NUM_DXL_MAX + 1;
   
   if(id != BROADCAST_ID)
   {
      if(RegValue == NO_REPLY)
      {
         g_StatusReturnLevel[id] = NO_REPLY;
         DXL_WriteByte(USARTx, id, STATUS_RETURN, NO_REPLY);
      }
      else if(RegValue == REPLY_TO_DATA_READ)
      {
         g_StatusReturnLevel[id] = REPLY_TO_DATA_READ;
         DXL_WriteByte(USARTx, id, STATUS_RETURN, REPLY_TO_DATA_READ);
      }
      else //RegValue == REPLY_ALL
      {
         g_StatusReturnLevel[id] = REPLY_ALL;
         DXL_WriteByte(USARTx, id, STATUS_RETURN, REPLY_ALL);
      }
   }
   else //id = BROADCAST_ID
   {
      if(RegValue == NO_REPLY)
      {
         while(tmp++ < tmp_max) {g_StatusReturnLevel[tmp] = NO_REPLY;}
         DXL_WriteByte(USARTx, id, STATUS_RETURN, NO_REPLY);
      }
      else if(RegValue == REPLY_TO_DATA_READ)
      {
         while(tmp++ < tmp_max) {g_StatusReturnLevel[tmp] = REPLY_TO_DATA_READ;}
         DXL_WriteByte(USARTx, id, STATUS_RETURN, REPLY_TO_DATA_READ);
      }
      else //RegValue == REPLY_ALL
      {
         while(tmp++ < tmp_max) {g_StatusReturnLevel[tmp] = REPLY_ALL;}
         DXL_WriteByte(USARTx, id, STATUS_RETURN, REPLY_ALL);
      }
   }
}
/**
  * @brief  Adjust Baudrate for USARTx communication with Dynamixel AX-12,
  *         2 changes: first AX-12 address table (0x04) and then USARTx protocol
  * @note  
  * @retval
  */
void Baudrate_Cmd(USART_TypeDef *USARTx, uint8_t id, uint8_t NewBaudrate) //(PUBLIC) 
{
   int tmp;
   /* Checking parameters */
	assert_param(IS_DXL_BAUDRATE(NewBaudrate));

   if(NewBaudrate == DXL_BAUDRATE_1000000)
      tmp = USART_BAUDRATE_1000000;
   else if(NewBaudrate == DXL_BAUDRATE_500000)
      tmp = USART_BAUDRATE_500000;
   else if(NewBaudrate == DXL_BAUDRATE_400000)
      tmp = USART_BAUDRATE_400000;
   else if(NewBaudrate == DXL_BAUDRATE_250000)
      tmp = USART_BAUDRATE_250000;
   else if(NewBaudrate == DXL_BAUDRATE_200000)
      tmp = USART_BAUDRATE_200000;
   else if(NewBaudrate == DXL_BAUDRATE_115200)
      tmp = USART_BAUDRATE_115200;
   else if(NewBaudrate == DXL_BAUDRATE_57600)
      tmp = USART_BAUDRATE_57600;
   else if(NewBaudrate == DXL_BAUDRATE_19200 )
      tmp = USART_BAUDRATE_19200;
   else
      tmp = USART_BAUDRATE_9600;

   //Update AX-12 table, Baudrate @ 0x04
   DXL_WriteByte(USARTx, id, BAUDRATE, NewBaudrate);

#ifdef STM32F10X_MD /* OpenCM0.94 */
   USART_DeInit(USARTx); //Baudrate can NOT be changed when USART is enabled
   if(USARTx == USART1)
      OpenCM_USART1_initialize(tmp);
   else
      OpenCM_USART3_initialize(tmp);

#else /* CM-530 */
   USART_DeInit(USARTx); //Baudrate can NOT be changed when USART is enabled
   CM530_USART1_initialize(tmp);
#endif
}

/** @defgroup ---------- DXL Auxliary functions ----------
  * @{
  */
/**
  * @brief  Disable TE(Transmissor enable) & RE(Receptor enable) when TC (Transmission completed)
  * @note   TC is reset by hardware when writting to USARTx->TDR or ClearFlag()
  * @param  None
  * @retval None
  */
uint8_t DXL_TC_Polling(USART_TypeDef* USARTx) //(PRIVATE)
{
    TimeOut = DXL_TIMEOUT;
    /* Polling until TC=1 (Transmission completed bit set to 1) */
    while(USART_GetFlagStatus(USARTx, USART_FLAG_TC) == RESET)
    {
       if(!TimeOut--)
          return USART_TC_TIMEOUT;
    }
    USART_ClearFlag(USARTx, USART_FLAG_TC);
    return USART_TX_SUCCESS;
}
/**
  * @brief
  * @note   VariableToCompare1 se pasa x REFERENCIA (pues sino simplemente crea una copia
  *         y no se actualizaría nunca
  * @param  None
  * @retval None
  */
void DXL_delay_us (uint32_t retardo_ms, volatile uint8_t* VariableToCompare1, uint8_t VariableToCompare2)  //(PRIVATE) sigue contando en ms
{
   g_DelayTimeOut = 0;
   usTicks = 0; //reset usticks before entering loop
   
   //usTicks global variable updates each 100us
   while(usTicks < retardo_ms) //check que el # de interrupciones es <= al retardo
   {
      if(*VariableToCompare1 == VariableToCompare2)
         return;
   }
   g_DelayTimeOut = 1;
}

uint8_t DXL_lowbyte(uint16_t word) //(PRIVATE)
{
   return (uint8_t)((0xFF & word));
}
uint8_t DXL_highbyte(uint16_t word) //(PRIVATE)
{
   return (uint8_t)((word >> 8) & 0xFF) ;
}
/** @defgroup ---------- DXL Deprecated functions ----------
  * @{
  */
/**
  * @brief  Deprecated, utilizamos interruptions para enviar usando TXE,
  *         de manera que una interruption no corrompa el Instruction packet
  * @param
  * @retval
  */
uint8_t UNUSED_FUNCTION DXL_TxByte(USART_TypeDef* USARTx, uint8_t byte) //(PRIVATE)
{
   /* Checking parameters */
	assert_param(IS_DXL_USART(USARTx));
   assert_param(IS_DXL_BYTE(byte));

   TimeOut = DXL_TIMEOUT;
   /* Send Tx byte */
   while(USART_GetFlagStatus(USARTx, USART_FLAG_TXE) != SET)
   {
      if(!TimeOut--)
      {
         DXL_TimeOutCall(USART_TX_TIMEOUT);
         return USART_TX_TIMEOUT;
      }
   }
   USART_SendData(USARTx, byte);
   return USART_BUSY; //por devolver algo (todo ok)
}
/**
  * @brief  Private function, called from public functions.
  * @note   NO SE PUEDE UTILIZAR POLLING SOLO INTERRUPTIONS (creo que por tema del baud rate too high)
  * @param
  * @retval
  */
void UNUSED_FUNCTION DXL_RxByte(USART_TypeDef* USARTx) //(PRIVATE)
{
   /* Checking parameters */
	assert_param(IS_DXL_USART(USARTx));

   /* Get Rx bytes */
   while(g_NumPacketReceived != g_StatusPacketLength)
   {
      while(USART_GetFlagStatus(USARTx, USART_FLAG_RXNE) == RESET)
      {
         __NOP();
      }
      g_StatusPacket[g_NumPacketReceived++] = USART_ReceiveData(USARTx);
   }
}
uint8_t DXL_Sync_SendData(uint8_t *pPacket, uint8_t numPacket)
{
   #ifdef STM32F10X_HD /* CM-530 */
   /* Control Internal Line Driver 74HC126A */
   GPIO_SetBits(GPIOB, GPIO_Pin_4);   //USART1: TX Enable
   GPIO_ResetBits(GPIOB, GPIO_Pin_5); //USART1: RX Disable

   for (uint8_t i=0; i<numPacket; i++)
   {
      while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) != SET)
      {
         if(!TimeOut--)
         {
            DXL_TimeOutCall(USART_TX_TIMEOUT);
            return USART_TX_TIMEOUT;
         }
      }
      USART_SendData(USART1, pPacket[i]);
   }
      
   /* Waiting for TC=1 */
   if(DXL_TC_Polling(USART1) != USART_TX_SUCCESS)
   {
      DXL_TimeOutCall(USART_TC_TIMEOUT);
      return 0;     
   }
   
   /* Control Internal Line Driver 74HC126A */
   GPIO_ResetBits(GPIOB, GPIO_Pin_4);  //USART1: TX Disable
   GPIO_SetBits(GPIOB, GPIO_Pin_5);    //USART1: RX Enable;

#endif    
#ifdef STM32F10X_MD /* OpenCM0.94 */
   GPIO_SetBits(GPIOB, GPIO_Pin_5);   //USART1: TX Enable & RX Disable
   GPIO_SetBits(GPIOC, GPIO_Pin_14);  //USART3: TX Enable & RX Disable

   for (uint8_t i=0; i<numPacket; i++)
   {
      while(USART_GetFlagStatus(USART3, USART_FLAG_TXE) != SET)
      {
         if(!TimeOut--)
         {
            DXL_TimeOutCall(USART_TX_TIMEOUT);
            return USART_TX_TIMEOUT;
         }
      }
      USART_SendData(USART3, pPacket[i]);
   }
      
   /* Waiting for TC=1 */
   if(DXL_TC_Polling(USART3) != USART_TX_SUCCESS)
   {
      DXL_TimeOutCall(USART_TC_TIMEOUT);
      return 0;     
   }
   
   GPIO_ResetBits(GPIOB, GPIO_Pin_5);  //USART1: RX Enable & TX Disable
   GPIO_ResetBits(GPIOC, GPIO_Pin_14); //USART3: RX Enable & TX Disable

#endif   

return numPacket;
}
