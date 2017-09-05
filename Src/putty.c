/*
  ******************************************************************************
  * @file    putty.c
  * @author  FX_Team
  * @version V1.0.0
  * @date    11-April-2017
  * @brief   This file contains the functions prototypes to connect the CM-530 to PC
  *            + xxxxxxxxxxxxx
  *            + xxxxxxxxxxxxx
  *            + xxxxxxxxxxxxx
  ******************************************************************************

  ==============================================================================
                       ##### How to use this driver #####
  =============================================================================
  * >> Es necesario añadir USART3_Interrupt() a ISR file.
  * >>
  * >>
  *****************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include "putty.h"
#include "main.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
   #define PC_UART_BUFFER_LENGTH           128
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
   static volatile uint32_t glPcuTimeoutCounter;
   static volatile uint16_t gbPcuWrite, gbPcuRead;
   static volatile uint8_t gbpPcuBuffer[PC_UART_BUFFER_LENGTH]={0};
/* Private function prototypes -----------------------------------------------*/
   void pcu_timeout(uint8_t);
   uint8_t pcu_get_queue(void);
   uint8_t pcu_peek_queue(void);
   uint8_t pcu_get_qstate(void);
   void pcu_clear_queue(void);
/* Private functions ---------------------------------------------------------*/
   extern void delay_us (uint32_t retardo_ms);
/** @defgroup ---------- PC UART Low level functions ----------
  * @{
  */
/**
  * @brief  None
  * @param  None
  * @retval None
  */
void pcu_timeout(uint8_t NumRcvByte)
{
   glPcuTimeoutCounter = 0;
   // 200us; ~180 us to transmit one byte at 57600 bps
   delay_us(NumRcvByte * 200);
   glPcuTimeoutCounter = 1;
}

/**
  * @brief  None
  * @param  None
  * @retval None
  */
uint8_t pcu_get_queue(void)
{
   if(gbPcuWrite == gbPcuRead)
      return 0xFF;

   if(gbPcuRead > (PC_UART_BUFFER_LENGTH-1))
      gbPcuRead = 0;

   return gbpPcuBuffer[gbPcuRead++];
}

/**
  * @brief  None
  * @param  None
  * @retval None
  */
uint8_t pcu_peek_queue(void)
{
   if(gbPcuWrite == gbPcuRead)
      return 0xFF;

   return gbpPcuBuffer[gbPcuRead];
}

/**
  * @brief  None
  * @param  None
  * @retval None
  */
void pcu_clear_queue(void)
{
   gbPcuWrite = 0;
   gbPcuRead = 0;
}

/**
  * @brief  None
  * @param  None
  * @retval None
  */
uint8_t pcu_get_qstate(void)
{
   if (gbPcuWrite == gbPcuRead)
   {
     pcu_clear_queue();
     return 0;
   }
   else if (gbPcuRead < gbPcuWrite)
      return (uint8_t)(gbPcuWrite-gbPcuRead);
   else
      return (uint8_t)(PC_UART_BUFFER_LENGTH-(gbPcuRead-gbPcuWrite));
}

/**
  * @brief  None
  * @param  None
  * @retval None
  */
int std_putchar(char c)
{
   if (c == '\n')
   {
      USART_SendData(USART3,'\r'); //0x0D
      while (USART_GetFlagStatus(USART3, USART_FLAG_TC)==RESET);
      USART_SendData(USART3,'\n'); //0x0A
      while (USART_GetFlagStatus(USART3, USART_FLAG_TC)==RESET);
   }
   else
   {
      USART_SendData(USART3,c);
      while (USART_GetFlagStatus(USART3, USART_FLAG_TC)==RESET);
   }

   return c;
}

/**
  * @brief  None
  * @param  None
  * @retval None
  */
int std_puts(const char *str)
{
   int n=0;
   while (str[n])
      std_putchar(str[n++]);
   return n;
}

/**
  * @brief  None
  * @param  None
  * @retval None
  */
int std_getchar(void)
{
   char c;

   pcu_timeout(10); //delay

   if(pcu_get_qstate()==0)
      return 0xFF;

   c = pcu_get_queue();

   if(c=='\r')
      c = '\n';
   return c;
}

/**
  * @brief  None
  * @param  None
  * @retval None
  */
char* std_gets(char *str)
{
   uint8_t c, len=0;

   while (len<128)
   {
     pcu_timeout(10); //delay

     if(pcu_get_qstate()==0)
     {
         if(len==0)
         {
            return 0;//NULL;
         }
         else
         {
            str[len] = '\0';
            return str;
         }
     }

     c = pcu_get_queue();
     if( (c=='\n') || (c=='\0') )
     {
        if (len==0)
        {
            return 0;//NULL;
        }
        else
        {
            str[len] = '\0';
            return str;
        }
     }
     else
        str[len++] = (int8_t) c;
   }
   return str;
}

/** @defgroup ---------- PC UART High level functions ----------
  * @{
  */
/**
  * @brief  None
  * @param  None
  * @retval None
  */
void USART3_Interrupt(void)
{
   uint8_t temp;
   if (USART_GetITStatus(USART3, USART_IT_RXNE)!=RESET)
   {
      temp = USART_ReceiveData(USART3);
   }
   else
      return;

   if (gbPcuWrite < (PC_UART_BUFFER_LENGTH-1))
   {
      gbpPcuBuffer[gbPcuWrite++] = temp;
   }
   else
   {
      gbpPcuBuffer[gbPcuWrite] = temp;
      gbPcuWrite = 0;
   }

   if(gbPcuRead==gbPcuWrite)
      gbPcuRead++;
   if(gbPcuRead>(PC_UART_BUFFER_LENGTH-1))
      gbPcuRead=0;
}

/**
  * @brief  None
  * @param  None
  * @retval None
  */
void USART3_terminate(void)
{
   /* Disable USART3 (PC UART) */
   USART_Cmd(USART3, DISABLE);
}

/**
  * @brief  None
  * @param  None
  * @retval None
  */
void PrintCommStatus(uint16_t Status) //Disabled
{ /*
   if(Status & DXL_TXFAIL)
     std_puts("\nDXL_TXFAIL: Failed transmit instruction packet!\n");

   if(Status & DXL_RXFAIL)
     std_puts("\nDXL_RXFAIL: Failed get status packet from device!\n");

   if(Status & DXL_TXERROR)
     std_puts("\nDXL_TXERROR: Incorrect instruction packet!\n");

   if(Status & DXL_BAD_INST)
     std_puts("\nDXL_BAD_INST: Invalid Instruction byte\n");

   if(Status & DXL_BAD_ID)
     std_puts("\nDXL_BAD_ID: ID's not same for instruction and status packets\n");

   if(Status & DXL_RXWAITING)
     std_puts("\nDXL_RXWAITING: Now receiving status packet!\n");

   if(Status & DXL_RXTIMEOUT)
     std_puts("\nDXL_RXTIMEOUT: There is no status packet!\n");

   if(Status & DXL_RXCHECKSUM)
     std_puts("\nDXL_RXCHECKSUM: Incorrect status packet checksum!\n */

   //else
   //std_puts("\nThis is unknown error code!\n");
}

/**
  * @brief  None
  * @param  None
  * @retval None
  */
void PrintErrorCode(void) //Disabled
{/*
    if(dxl_get_rxpacket_error(ERRBIT_VOLTAGE) == 1)
        std_puts("\nInput voltage error!\n");

    if(dxl_get_rxpacket_error(ERRBIT_ANGLE) == 1)
        std_puts("\nAngle limit error!\n");

    if(dxl_get_rxpacket_error(ERRBIT_OVERHEAT) == 1)
        std_puts("\nOverheat error!\n");

    if(dxl_get_rxpacket_error(ERRBIT_RANGE) == 1)
        std_puts("\nOut of range error!\n");

    if(dxl_get_rxpacket_error(ERRBIT_CHECKSUM) == 1)
        std_puts("\nChecksum error!\n");

    if(dxl_get_rxpacket_error(ERRBIT_OVERLOAD) == 1)
        std_puts("\nOverload error!\n");

    if(dxl_get_rxpacket_error(ERRBIT_INSTRUCTION) == 1)
        std_puts("\nInstruction code error!\n"); */
}

/**
  * @brief  None
  * @param  None
  * @retval None
  */
int PrintChar(char c){return std_putchar(c);}

/**
  * @brief  None
  * @param  None
  * @retval None
  */
int PrintString(const char* s){return std_puts(s);}

/**
  * @brief  None
  * @param  None
  * @retval None
  */
int GetChar(void){return std_getchar();}

/**
  * @brief  None
  * @param  None
  * @retval None
  */
char* GetString(char* s){return std_gets(s);}

/**
  * @brief  None
  * @param  None
  * @retval None
  */
void Printu32d(uint32_t lNum) //Print unsigned decimal number 32bits long
{
   uint32_t temp, div=1000000000;
   char out[11];
   uint8_t i, j;

   for(i=0; i<10; i++)
   {
      temp = (char)(lNum/div);
      lNum = (lNum%div);
      //        lNum -= (uint32_t) (temp*div);
      //        out[i] = (char) (temp & 0x0000000F)+0x30;
      out[i] = (char)((temp & 0x0F) + 0x30); // 0x30 = '\0' --> from int to char in ASCII table
      div /= 10;
   }
   out[i] = '\0';

   for (i=0; i<10; i++)
   {
     if(out[0]=='0')
     {
         for(j=0; j<10; j++)
         {
             out[j] = out[j+1];
             if(out[j]=='\0')
                 break;
         }
     }
   }
   std_puts(out);
   return;
}

/**
  * @brief  None
  * @param  None
  * @retval None
  */
void Prints32d(int32_t lNumS) //Print signed decimal number 32bits long
{
   uint32_t temp, lNum, div=1000000000;
   char out[12];
   uint8_t i, j;

   if(lNumS<0)
   {
     out[0] = '-';
     lNum = (uint32_t) ((~lNumS)+1);
   }
   else
   {
     out[0] = '+';
     lNum = (uint32_t) (lNumS);
   }

   for(i=1; i<11; i++)
   {
     temp = (lNum/div);
     lNum = (lNum%div);
   //        lNum -= (uint32_t) (temp*div);
   //        out[i] = (char) (temp & 0x0000000F)+0x30;
     out[i] = (char) ((temp & 0x0F)+0x30);
     div /= 10;
   }
   out[i] = '\0';

   for (i=0; i<11; i++)
   {
     if (out[0]=='0')
     {
         for (j=0; j<11; j++)
         {
             out[j] = out[j+1];
             if (out[j]=='\0')
                 break;
         }
     }
   }

   std_puts(out);
   return;
}

/**
  * @brief  None
  * @param  None
  * @retval None
  */
void Printu16h(uint16_t wNum) //Print hex number 16bits long
{
   char out[7];
   out[0] = '0';
   out[1] = 'x';
   out[6] = '\0';

   out[2] = (char) ((wNum>>12) & 0x0F) + 0x30;
   if(out[2] > '9')
     out[2] += 7;

   out[3] = (char) ((wNum>>8) & 0x0F) + 0x30;
   if(out[3] > '9')
     out[3] += 7;

   out[4] = (char) ((wNum>>4) & 0x0F) + 0x30;
   if(out[4] > '9')
     out[4] += 7;

   out[5] = (char) (wNum & 0x0F) + 0x30;
   if(out[5] > '9')
     out[5] += 7;

   std_puts(out);
   return;
}

/**
  * @brief  None
  * @param  None
  * @retval None
  */
void Printu8h(uint8_t bNum) //Print hex number 8bits long
{
	//bNum es hexadecimal
   char out[5];
   out[0] = '0';
   out[1] = 'x';
   out[4] = '\0';

   out[2] = (char) ((bNum>>4) & 0x0F) + 0x30;
   if(out[2] > '9')
      out[2] += 7;

   out[3] = (char) (bNum & 0x0F) + 0x30;
   if(out[3] > '9')
      out[3] += 7;

   std_puts(out);
   return;
}