/**
  ******************************************************************************
  * @file    dxl.c
  * @author  FX_Team
  * @version V1.0.0
  * @date    12-March-2017
  * @brief   This file contains the dynamixel functions for Robotis AX-12
  *          functionalities of the dynamixel servos:
  *            + xxxxxxxxxxxxx
  *            + xxxxxxxxxxxxx
  *            + xxxxxxxxxxxxx
  ******************************************************************************

  ==============================================================================
                       ##### How to use this driver #####
  =============================================================================
  * >> Instruction Packet (Command Packet)
  * >> 0xFF -> 0xFF -> ID -> LENGTH -> INSTRUCTION -> PARAM_1 -> PARAM_2 -> PARAM_N -> CHECKSUM
  * >> LENGTH = ( 1 {ID Byte} + 1 {INSTRUCTION Byte} + N {Number of PARAMETER Bytes} )
  * >> CHECKSUM = ( ~(ID + LENGTH + INSTRUCTION + PARAM_1 +...+ PARAM_N) ) & (0x00FF)

  * >> Status Packet (Return Packet)
  * >> 0xFF -> 0xFF -> ID -> LENGTH -> ERROR -> PARAM_1 -> PARAM_2 -> PARAM_N -> CHECKSUM
  * >> LENGTH = ( 1 {ID Byte} + 1 {ERROR Byte} + N {Number of PARAMETER Bytes} )
  * >> CHECKSUM = ( ~(ID + LENGTH + ERROR + PARAM_1 +...+ PARAM_N) ) & (0x00FF)
  * >> Add RxD_DXL_Interrupt() to stm32f10x_it.c
  *****************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "dxl.h"
#include "main.h"
#include <stdio.h>

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
   #define DXL_BUFFER_LENGTH               256
   #define DXL_MAXNUM_TXPARAM              160
   #define DXL_MAXNUM_RXPARAM              80
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
   static volatile uint32_t glDxlTimeoutCounter;
   static volatile uint16_t gbDxlWrite=0, gbDxlRead=0;
   static volatile uint8_t gbpDxlBuffer[DXL_BUFFER_LENGTH];
   static uint8_t gbInstructionPacket[DXL_MAXNUM_TXPARAM] = {0};
   static uint8_t gbStatusPacket[DXL_MAXNUM_RXPARAM] = {0};
   static uint8_t gbRxPacketLength = 0; //Status Packet Length
   static uint8_t gbRxGetLength = 0;
   static volatile uint16_t gbCommStatus = DXL_RXSUCCESS;
   static volatile uint8_t giBusUsing = 0;
/* Private function prototypes -----------------------------------------------*/
   void dxl_hal_clear(void);
   uint8_t dxl_hal_tx(uint8_t*, uint8_t);
   uint8_t dxl_hal_rx(uint8_t*, uint8_t);
   void dxl_hal_set_timeout(uint8_t);
   void RxD_DXL_Interrupt(void);
   void dxl_tx_packet(void);
   void dxl_rx_packet(void);
   void dxl_clear_statpkt(void);
/* Private functions ---------------------------------------------------------*/
   extern void delay_us (uint32_t retardo_ms);
/** @defgroup ---------- Dynamixel SDK platform dependent functions ----------
  * @{
  */
/**
  * @brief  None
  * @param  None
  * @retval None
  */
void dxl_hal_clear(void)
{
    // Clear communication buffer and reset buffer pointers
    uint16_t i;
    for (i=0; i<DXL_BUFFER_LENGTH; i++)
        gbpDxlBuffer[i] = 0;
    gbDxlRead = 0;
    gbDxlWrite = 0;
}

/**
  * @brief  None
  * @param  None
  * @retval None
  */
uint8_t dxl_hal_tx(uint8_t *pPacket, uint8_t numPacket)
{
    uint8_t i;
    for (i=0; i<numPacket; i++)
    {
        // RX Disable
        GPIO_ResetBits(GPIOB, GPIO_Pin_5);
        // TX Enable
        GPIO_SetBits(GPIOB, GPIO_Pin_4);

        USART_SendData(USART1,pPacket[i]);
        while( USART_GetFlagStatus(USART1, USART_FLAG_TC)==RESET );

        // TX Disable
        GPIO_ResetBits(GPIOB, GPIO_Pin_4);
        // RX Enable
        GPIO_SetBits(GPIOB, GPIO_Pin_5);
    }

    return numPacket;
}

/**
  * @brief  None
  * @param  None
  * @retval None
  */
uint8_t dxl_hal_rx(uint8_t *pPacket, uint8_t numPacket)
{
    uint8_t i;
    for (i=0; i<numPacket; i++)
    {
        if (gbDxlRead!=gbDxlWrite)
        {
            pPacket[i] = gbpDxlBuffer[gbDxlRead++];
            if (gbDxlRead>(DXL_BUFFER_LENGTH-1))
                gbDxlRead = 0;
        }
        else
            return i;
    }

    return numPacket; //ha enviado todos -> i = numPacket
}

/**
  * @brief  None
  * @param  None
  * @retval None
  */
void dxl_hal_set_timeout(uint8_t NumRcvByte)
{
   glDxlTimeoutCounter = 0;
   delay_us(NumRcvByte * 30);
   glDxlTimeoutCounter = 1;
}

/**
  * @brief  None
  * @param  None
  * @retval None
  */
void RxD_DXL_Interrupt(void)
{
   /* NO entiendo para que sirve el Buffer si unicamente leo el Rx despues de enviar
      un Tx!! nunca voy a poder leer los bytes antiguos, unicamente el último Tx */
    uint8_t temp;
    if (USART_GetITStatus(USART1, USART_IT_RXNE)!=RESET)
    {
        temp = USART_ReceiveData(USART1);
    }
    else
        return;

    if (gbDxlWrite<(DXL_BUFFER_LENGTH-1))
    {
        gbpDxlBuffer[gbDxlWrite++] = temp;
    }
    else //gbDxlWrite = (DXL_BUFFER_LENGTH-1)
    {
        gbpDxlBuffer[gbDxlWrite] = temp;
        gbDxlWrite = 0; //Ring Buffer
    }
    // Para perder el mínimo de packets si writing es mucho + rapido que el reading
    if (gbDxlRead==gbDxlWrite)
        gbDxlRead++;
    if (gbDxlRead>(DXL_BUFFER_LENGTH-1))
        gbDxlRead=0;
}

/** @defgroup ---------- Dynamixel SDK platform independent functions ----------
  * @{
  */
/**
  * @brief  None
  * @param  None
  * @retval None
  */
uint8_t CM530_dxl_initialize(uint32_t baudrate) //Full-Duplex !!
{
   /* ===== USART1 TIME BLOCK & INTERRUPTIONS CONFIGURATION ===== */
   /* Enable USART1 clock */
   RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

   /* Enable Port B (USART1_Tx) and (USART1_Rx) clock */
   RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

   /* Enable APB2 peripheral AFIO */
   RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

   /* Configure and set USART1 Interrupt to the lowest priority */
   NVIC_InitTypeDef NVIC_InitStructure;
   NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
   NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;
   NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;
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
   USART_InitStructure.USART_BaudRate = baudrate;
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
   //USART_ITConfig(USART1, USART_IT_TXE, ENABLE);
   //USART_ITConfig(USART1, USART_IT_TC, ENABLE);

   /* Enable DMA Management for USART1 */
   //USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);

    gbCommStatus = DXL_RXSUCCESS;
    giBusUsing = 0;
    return 1;
}
/**
  * @brief  None
  * @param  None
  * @retval None
  */
uint8_t STM32F1_dxl_initialize(uint32_t baudrate) //Half-Duplex !!
{
   /* ===== USART1 TIME BLOCK & INTERRUPTIONS CONFIGURATION ===== */
   /* Enable USART1 clock */
   RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

   /* Enable Port B (USART1_Tx) and (USART1_Rx) clock */
   RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

   /* Enable APB2 peripheral AFIO */
   RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

   /* Configure and set USART1 Interrupt to the lowest priority */
   NVIC_InitTypeDef NVIC_InitStructure;
   NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
   NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;
   NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;
   NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
   NVIC_Init(&NVIC_InitStructure);


	/* ===== GPIOS CONFIGURATION FOR USART1 ===== */
   /* Configure and set USART1_Tx(PB6) as AF OPEN-DRAIN (HALF-DUPLEX) */
   GPIO_InitTypeDef   GPIO_InitStructure;
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD; //OPEN-DRAIN when Half-Duplex enabled
   //GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; //PUSH-PULL when Full-Duplex
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
   GPIO_Init(GPIOB, &GPIO_InitStructure);

   /* Enable the USART1 Pins Software Remapping */
   GPIO_PinRemapConfig(GPIO_Remap_USART1, ENABLE);


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
   USART_HalfDuplexCmd(USART1, ENABLE);

   /* Enable USART1 peripheral */
   USART_Cmd(USART1, ENABLE);

   /* Enable USART1 Interruptions */
   USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
   //USART_ITConfig(USART1, USART_IT_RTO, ENABLE);
   //USART_ITConfig(USART1, USART_IT_TXE, ENABLE);
   //USART_ITConfig(USART1, USART_IT_TC, ENABLE);

   /* Enable DMA Management for USART1 */
   //USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);

    gbCommStatus = DXL_RXSUCCESS;
    giBusUsing = 0;

    return 1;
}

/**
  * @brief  None
  * @param  None
  * @retval None
  */
void dxl_terminate(void)
{
   // Disable USART1 (dynamixel)
   USART_Cmd(USART1, DISABLE);
}

/**
  * @brief  None
  * @param  None
  * @retval None
  */
void dxl_tx_packet(void)
{
   uint8_t i;
   uint8_t TxNumByte, RealTxNumByte;
   uint8_t checksum = 0;

   // NO hay buffer para Tx, habra que implementar en el main() un while{}
   // para no enviar hasta que el UART1 bus esté libre
   if(giBusUsing==1) //a communication is already ongoing
     return;

   giBusUsing = 1;

   gbCommStatus = 0;

   if(gbInstructionPacket[DXL_PKT_LEN]>(DXL_MAXNUM_TXPARAM+2))
   {
     gbCommStatus |= DXL_TXERROR;
     giBusUsing = 0;
     return;
   }
   //no veo lógico comprobar esto cada vez....
   if(   (gbInstructionPacket[DXL_PKT_INST] != INST_PING)
     && (gbInstructionPacket[DXL_PKT_INST] != INST_READ_DATA)
     && (gbInstructionPacket[DXL_PKT_INST] != INST_WRITE_DATA)
     && (gbInstructionPacket[DXL_PKT_INST] != INST_REG_WRITE)
     && (gbInstructionPacket[DXL_PKT_INST] != INST_ACTION)
     && (gbInstructionPacket[DXL_PKT_INST] != INST_RESET)
     && (gbInstructionPacket[DXL_PKT_INST] != INST_SYNC_WRITE)
     && (gbInstructionPacket[DXL_PKT_INST] != INST_CAP_REGION) )
   {
     gbCommStatus |= DXL_BAD_INST;
     giBusUsing = 0;
     return;
   }

   gbInstructionPacket[0] = 0xFF;
   gbInstructionPacket[1] = 0xFF;
   /* gbInstructionPacket[checksum] */
   for (i=0; i<(gbInstructionPacket[DXL_PKT_LEN]+1); i++)
     checksum += gbInstructionPacket[i+2];
   gbInstructionPacket[gbInstructionPacket[DXL_PKT_LEN]+3] = ~checksum;

   /* Clear pending errors */
   if (gbCommStatus&(DXL_RXFAIL | DXL_RXTIMEOUT | DXL_RXCHECKSUM | DXL_RXLENGTH | DXL_BAD_INST | DXL_BAD_ID))
   {
     dxl_hal_clear(); //Clear communication buffer and reset buffer pointers
   }

   TxNumByte = gbInstructionPacket[DXL_PKT_LEN] + 4;

   RealTxNumByte = dxl_hal_tx((uint8_t*)gbInstructionPacket, TxNumByte);

   //printf("\n# de bytes enviados por Tx: %d", RealTxNumByte);
   if (TxNumByte!=RealTxNumByte)
   {
     gbCommStatus |= DXL_TXFAIL;
     giBusUsing = 0;
     return;
   }
   /* If Instruction = READ -> add extra delay for the Servo to send reply */
   if (gbInstructionPacket[DXL_PKT_INST]==INST_READ_DATA)
     dxl_hal_set_timeout(gbInstructionPacket[DXL_PKT_PARA+1]+6);
   else
     dxl_hal_set_timeout(6); //delay_time_reply = 180 (default) -> 6 * 30us = ms (see function)

   gbCommStatus = DXL_TXSUCCESS;
}

/**
  * @brief  None
  * @param  None
  * @retval None
  */
void dxl_rx_packet(void)
{
   uint8_t i, j, nRead;
   uint8_t checksum = 0;

   if(giBusUsing==0)
      return;

   giBusUsing = 1;

   //No Status packet are sent when Broadcasting
   if(gbInstructionPacket[DXL_PKT_ID]==BROADCAST_ID)
   {
     gbCommStatus = DXL_RXSUCCESS;
     giBusUsing = 0;
     return;
   }

   if(gbCommStatus&DXL_TXSUCCESS)
   {
     gbRxGetLength = 0;
     gbRxPacketLength = 6; //Always the same length for Status packets
   }

   /* Lee el buffer hasta completar el status packet */
   nRead = dxl_hal_rx((uint8_t*)&gbStatusPacket[gbRxGetLength], gbRxPacketLength-gbRxGetLength);

   gbRxGetLength += nRead;
   printf("\n# de bytes recibidos por Rx: %d", gbRxGetLength);

   /* Checking if Status packet is completed */
   if(gbRxGetLength<gbRxPacketLength)
   {
     if(glDxlTimeoutCounter == 1) //dxl_hal_set_timeout() set @ dxl_tx_packet() is over
     {
         if(gbRxGetLength==0)
             gbCommStatus = DXL_RXTIMEOUT; //exit do while{} from dxl_txrx_packet()
         else
             gbCommStatus = DXL_RXLENGTH; //exit do while{} from dxl_txrx_packet()

         giBusUsing = 0;
         return;
     }
   }

   // Find packet header (el status packet no tiene x que estar completo !!)
   // Busca los 2 primeros packets (0xFF 0xFF) en los packets recibidos
   // gbRxGetLength > 0, sino se habria salido de la f() en el if anterior
   for(i=0; i<(gbRxGetLength-1); i++)
   {
     if ( (gbStatusPacket[i]==0xFF) && (gbStatusPacket[i+1]==0xFF) )
     {
         break;
     }
     else if ( (i==gbRxGetLength-2) && (gbStatusPacket[gbRxGetLength-1]==0xFF) ) //NO ENTIENDO ESTE ELSE IF
     {
         break;
     }
   }
   if(i>0) //Reinicia el gbStatusPacket[] posicionando los 2 header packets an inicio del array
   {
     for (j=0; j<(gbRxGetLength-i); j++)
         gbStatusPacket[j] = gbStatusPacket[j + i];

     gbRxGetLength -= i;
   }

   // Check if received full packet
   if(gbRxGetLength<gbRxPacketLength)
   {
     gbCommStatus = DXL_RXWAITING;
     return;
   }

   // Check id pairing
   if(gbInstructionPacket[DXL_PKT_ID]!=gbStatusPacket[DXL_PKT_ID])
   {
     gbCommStatus = DXL_BAD_ID | DXL_RXFAIL;
     giBusUsing = 0;
     return;
   }

   gbRxPacketLength = gbStatusPacket[DXL_PKT_LEN] + 4;
   if(gbRxGetLength<gbRxPacketLength)
   {
     nRead = dxl_hal_rx((uint8_t*)&gbStatusPacket[gbRxGetLength], gbRxPacketLength-gbRxGetLength);
     gbRxGetLength += nRead;
     if (gbRxGetLength<gbRxPacketLength)
     {
         gbCommStatus = DXL_RXWAITING;
         return;
     }
   }

   // Check checksum
   for(i=0; i<(gbStatusPacket[DXL_PKT_LEN]+1); i++)
     checksum += gbStatusPacket[i+2];
   checksum = ~checksum;

   if(gbStatusPacket[gbStatusPacket[DXL_PKT_LEN]+3]!=checksum)
   {
     gbCommStatus = DXL_RXCHECKSUM | DXL_RXFAIL;
     giBusUsing = 0;
     return;
   }

   gbCommStatus = DXL_RXSUCCESS;
   giBusUsing = 0;
}

/**
  * @brief  None
  * @param  None
  * @retval None
  */
void dxl_txrx_packet(void)
{
   dxl_tx_packet(); //incluye delay(us) to wait for AX-12 reply_time_delay

   if(!(gbCommStatus&DXL_TXSUCCESS))
     return;
   /* Clear previous gbStatusPacket */
   dxl_clear_statpkt();

   do
   {
      dxl_rx_packet();
      delay_us(50); //to fix bug, adding a delay between dxl_rx_packet() calls (see Easy functions cm530.c for further explanation)
   }while(gbCommStatus&DXL_RXWAITING);
}

/**
  * @brief  None
  * @param  None
  * @retval None
  */
uint16_t dxl_get_result(void)
{
    return gbCommStatus;
}

/**
  * @brief  None
  * @param  None
  * @retval None
  */
void dxl_set_txpacket_id(uint8_t id)
{
    gbInstructionPacket[DXL_PKT_ID] = id;
}

/**
  * @brief  None
  * @param  None
  * @retval None
  */
void dxl_set_txpacket_instruction(uint8_t instruction)
{
    gbInstructionPacket[DXL_PKT_INST] = instruction;
}

/**
  * @brief  None
  * @param  None
  * @retval None
  */
void dxl_set_txpacket_parameter(uint8_t index, uint8_t value )
{
    gbInstructionPacket[DXL_PKT_PARA+index] = value;
}

/**
  * @brief  None
  * @param  None
  * @retval None
  */
void dxl_set_txpacket_length(uint8_t length)
{
    gbInstructionPacket[DXL_PKT_LEN] = length;
}

/**
  * @brief  None
  * @param  None
  * @retval None
  */
uint8_t dxl_get_rxpacket_error(uint8_t errbit)
{
    if ((gbCommStatus&DXL_RXFAIL))
        return 0x80;

    if (gbStatusPacket[DXL_PKT_ERR]&errbit)
        return 1;

    return 0;
}

/**
  * @brief  None
  * @param  None
  * @retval None
  */
uint8_t dxl_get_rxpacket_length(void)
{
    if ((gbCommStatus&DXL_RXFAIL))
        return 0;

    return gbStatusPacket[DXL_PKT_LEN];
}

/**
  * @brief  None
  * @param  None
  * @retval None
  */
uint8_t dxl_get_rxpacket_parameter(uint8_t index)
{
    if ((gbCommStatus&DXL_RXFAIL))
        return 0;

    return gbStatusPacket[DXL_PKT_PARA+index];
}

/**
  * @brief  None
  * @param  None
  * @retval None
  */
uint16_t dxl_makeword(uint8_t lowbyte, uint8_t highbyte)
{
    uint16_t word;

    word = highbyte;
    word = word<<8;
    word = word+lowbyte;
    return word;
}

/**
  * @brief  None
  * @param  None
  * @retval None
  */
uint8_t dxl_get_lowbyte(uint16_t word)
{
    uint16_t temp = (word&0x00FF);
    return (uint8_t) temp;
}

/**
  * @brief  None
  * @param  None
  * @retval None
  */
uint8_t dxl_get_highbyte(uint16_t word)
{
    uint16_t temp = ((word&0xFF00)>>8);
    return (uint8_t) temp;
}

/**
  * @brief  None
  * @param  None
  * @retval None
  */
void dxl_ping(uint8_t id)
{
    while (giBusUsing);

    gbInstructionPacket[DXL_PKT_ID] = id;
    gbInstructionPacket[DXL_PKT_INST] = INST_PING;
    gbInstructionPacket[DXL_PKT_LEN] = 2;

    dxl_txrx_packet();
}

/**
  * @brief  None
  * @param  None
  * @retval None
  */
uint8_t dxl_read_byte(uint8_t id, uint8_t address)
{
    while(giBusUsing);

    gbInstructionPacket[DXL_PKT_ID] = id;
    gbInstructionPacket[DXL_PKT_INST] = INST_READ_DATA;
    gbInstructionPacket[DXL_PKT_PARA] = address;
    gbInstructionPacket[DXL_PKT_PARA+1] = 1;
    gbInstructionPacket[DXL_PKT_LEN] = 4;

    dxl_txrx_packet();

    if ((gbCommStatus&DXL_RXFAIL))
        return 0;

    return gbStatusPacket[DXL_PKT_PARA];
}

/**
  * @brief  None
  * @param  None
  * @retval None
  */
void dxl_write_byte(uint8_t id, uint8_t address, uint8_t value)
{
    while(giBusUsing);

    gbInstructionPacket[DXL_PKT_ID] = id;
    gbInstructionPacket[DXL_PKT_INST] = INST_WRITE_DATA;
    gbInstructionPacket[DXL_PKT_PARA] = address;
    gbInstructionPacket[DXL_PKT_PARA+1] = value;
    gbInstructionPacket[DXL_PKT_LEN] = 4;

    dxl_txrx_packet();
}

/**
  * @brief  None
  * @param  None
  * @retval None
  */
uint16_t dxl_read_word(uint8_t id, uint8_t address)
{
    while (giBusUsing);

    gbInstructionPacket[DXL_PKT_ID] = id;
    gbInstructionPacket[DXL_PKT_INST] = INST_READ_DATA;
    gbInstructionPacket[DXL_PKT_PARA] = address;
    gbInstructionPacket[DXL_PKT_PARA+1] = 2;
    gbInstructionPacket[DXL_PKT_LEN] = 4;

    dxl_txrx_packet();

    if ((gbCommStatus&DXL_RXFAIL))
        return 0;

    return dxl_makeword(gbStatusPacket[DXL_PKT_PARA], gbStatusPacket[DXL_PKT_PARA+1]);
}

/**
  * @brief  None
  * @param  None
  * @retval None
  */
void dxl_write_word(uint8_t id, uint8_t address, uint16_t value)
{
    while (giBusUsing);

    gbInstructionPacket[DXL_PKT_ID] = id;
    gbInstructionPacket[DXL_PKT_INST] = INST_WRITE_DATA;
    gbInstructionPacket[DXL_PKT_PARA] = address;
    gbInstructionPacket[DXL_PKT_PARA+1] = dxl_get_lowbyte(value);
    gbInstructionPacket[DXL_PKT_PARA+2] = dxl_get_highbyte(value);
    gbInstructionPacket[DXL_PKT_LEN] = 5;

    dxl_txrx_packet();
}

/**
  * @brief  Clear previous gbStatusPacket
  * @param  vaya manera de complicar el tema !!!
  * @retval None
  */
void dxl_clear_statpkt(void)
{
    uint8_t i, max=gbStatusPacket[DXL_PKT_LEN];
    if ( (max>0) && (max<DXL_MAXNUM_RXPARAM) )
    {
        for (i=0; i<(max+4); i++)
            gbStatusPacket[i]=0;
    }
    else
    {
        for (i=0; i<6; i++)
            gbStatusPacket[i]=0;
    }
}











