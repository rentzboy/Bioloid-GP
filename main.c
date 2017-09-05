/*
**
**                           Main.c
**
**
**********************************************************************/
/*
   Last committed:     $Revision: 00 $
   Last changed by:    $Author: $
   Last changed date:  $Date:  $
   ID:                 $Id:  $

**********************************************************************/

/******************************************************************************/
/*            MAIN FILE - IMPORT                                            */
/******************************************************************************/
#include "main.h"
#include "globals.h"  
#include "dynamixel.h"
#include "zigbee.h"
#include "motions.h"

#include "printout.h"

/******************************************************************************/
/*            MAIN FILE - PRIVATE DECLARATION                          */
/******************************************************************************/
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
//global variables
volatile uint8_t cmd;
volatile uint8_t new_cmd_flag;
volatile StartButtonStatus StartButton; //GLOBAL VARIABLE
volatile uint8_t g_currentMotion;       //GLOBAL VARIABLE
volatile uint8_t g_lastMotion;          //GLOBAL VARIABLE
volatile int g_actionmode = -1;         //GLOBAL VARIABLE - SOCCERMODE | FIGHTMODE
volatile uint32_t usTicks = 0;          //GLOBAL VARIABLE
//debugging variables
static UNUSED_VAR uint16_t debug_var1, debug_var2;
static UNUSED_VAR uint32_t counter_systicks[100];
static UNUSED_VAR uint16_t counter_index;
static const UNUSED_VAR uint8_t TxWellcome[] = "\nHi PUTTY, this is STM32F103 !!";
static uint8_t debugVar; //para activar/desactivar lineas de código desde Command Windows

/* Private function prototypes -----------------------------------------------*/
   
int main(void)
{ 
   /* Bioloid startup sequence */
   if(BioloidInitialization() != SUCCESS)
      printf("\nBioloidInitialization failed !!");
   
   /* Initialization Loop */
   while(1)
   {  
      //1) Set robot action mode 
      while(g_actionmode == NOT_DEFINED_MODE) //updated by Interruptions (U, D buttons)
      {
         #ifdef STM32F10X_MD /* OpenCM */
         g_actionmode = FIGHTMODE;
         #endif
      }
      //2) Press Start button to move to main
      if(StartButton) //updated by Interruptions (Start button)
      {
         //Pending: turn on head led
         break; //exit initialization loop
         }
   }
   
   /* Main Loop */
   while(1)
   {
      /* Check for new Zigbee cmd */
      if(ZBEE_FlagCheckStatus() != RESET)
         new_cmd_flag = SET;
      if(debugVar)
         printf("\nEl nuevo cmd es: %d", cmd);
         
      /* Check for sensors flags */
      
      /* Check level when power on battery */
      
      /* PID algorithm */
      
      /* Check for Errors from dynamixyel */
 
      /* Execute cmd */
      if(new_cmd_flag)
         new_cmd_flag = !MTN_ExecuteCmd(); //RESET flag when MOTION_IS_OVER
   }
   
}
      /*************************************************************************
      **************************************************************************
      PENDING: 
      1) Que se ejecute CMD en bucle mientras no se levante el dedo del botón:
         Modificar MTN_ExecuteCmd() para no salir de los cmd´s bucle si los new cmd
         son iguales a los cmd actuales
      2) Utilizar USART1 (USB OpenCM0.94) para terminal windows ?
      **************************************************************************
      *************************************************************************/  


/******************************************************************************/
/*            MAIN FILE - FUNCTION DEFINITION                                 */
/******************************************************************************/ 
/* ===== BOARDS INITIALIZATION ===== */
ErrorStatus BioloidInitialization(void)
{  
   /* ---------- Configuration for each board OpenCM0.94 / CM-530 ---------- */
   SystemCoreClockUpdate(); //Mandatory when SystemCoreClock updated within system_stm32
   delay_init();            //SysTick() set-up
   LED_initialize();        //Leds set-up f(board)
   ZBEE_Initialize();       //USART2 <OpenCM> || USART5 <CM-530>

#ifdef STM32F10X_MD /* OpenCM0.94 */
   OpenCM_Buttons_Init();                               //Only button_2
   SWO_initialize();                                    //Enable printf
   printf("\nEstado printf: Activado!!\n"); 
   ///////// NO DARLE + BAUDRATE QUE DA FALLO (ver LEER IMPORTANTE.TXT) /////////
   OpenCM_USART3_initialize(USART_BAUDRATE_115200);     //DXL <-> Exp485   (USART3)
	USART1_OPENCM_PC_Initialize(USART_BAUDRATE_57600);   //PUTTY <-> OpenCM (USART1)

#else /* CM-530 */
   CM530_Buttons_Init();                                //Start, U & D buttons
   USART3_initialize(USART_BAUDRATE_57600);             //Terminal <-> µc  (USART3)
   USART3_CM530_PC_Initialize(USART_BAUDRATE_57600);    //DXL <-> µc      (USART1)
#endif
   
   /* ---------- Configuration for servos AX-12 / AX-18 using BROADCAST_ID ---------- */
   //MANDATORY: Always return Status packet for DXL 
   //-----> no quitar, rellena g_StatusReturnLevel[] <-----
   StatusPacket_Cmd(USART_DXL, BROADCAST_ID, REPLY_ALL);       //@ = 0x10
   //Set delay answer (0x50 x 2usec) for Dynamixel
   DXL_WriteByte(USART_DXL, BROADCAST_ID, RETURNDELAY, 0x50);  //@ = 0x05
   //Printf register values for all servos
   DXL_DumpRegisterValues(USART_DXL);

   //Initialize motion table
   MTN_InitializeLookupTable();
   
   //Initialize TIM3 for motion control
   MTN_TIM3_initialize();
    
   /* Execute initial pose */
   g_lastMotion = BALANCE;
   g_currentMotion = INITIAL;
   MTN_SetNewMotionFlag();
   MTN_ExecuteMotion(); //updates
   
   /* Checking no error from last motion */
   if(DXL_Diagnosis(USART_DXL) != SUCCESS) //Error is printf from Dynamixel module
      led(LED_RED_ON);
   
   //All tests are passed!!
   led(LED_BLUE_BLINK); //Manage led   
   return SUCCESS;
}

/* ===== LEDS-BUTTONS ===== */
/**
  * @brief  Leds are connected to Vcc in Exp_485 board !!!
  * @param  None
  * @retval None
  */
void led(uint8_t led)
{
   /* Checking parameters */
   assert_param(IS_LED(led));

#ifdef STM32F10X_MD /* OpenCM0.94 */
   switch(led)
   {
   case LED_BLUE_ON:
      GPIO_ResetBits(GPIOB, GPIO_Pin_14);  //ON -> RESET BITS (SHIELD EXT_485)
      break;
   case LED_GREEN_ON:
      GPIO_ResetBits(GPIOB, GPIO_Pin_13);
      break;
   case LED_RED_ON:
      GPIO_ResetBits(GPIOB, GPIO_Pin_12);
      break;
   case LED_BLUE_OFF:
      GPIO_SetBits(GPIOB, GPIO_Pin_14);
      break;
   case LED_GREEN_OFF:                      //OFF -> SET BITS 
      GPIO_SetBits(GPIOB, GPIO_Pin_13);
      break;
   case LED_RED_OFF:
      GPIO_SetBits(GPIOB, GPIO_Pin_12);
      break;
   case LED_BLUE_BLINK:
         GPIO_ResetBits(GPIOB, GPIO_Pin_14);
         delay_ms(300);
         GPIO_SetBits(GPIOB, GPIO_Pin_14);
         delay_ms(300);
      break;
   case LED_GREEN_BLINK:
         GPIO_ResetBits(GPIOB, GPIO_Pin_13);
         delay_ms(300);
         GPIO_SetBits(GPIOB, GPIO_Pin_13);
         delay_ms(300);
      break;
   case LED_RED_BLINK:
         GPIO_ResetBits(GPIOB, GPIO_Pin_12);
         delay_ms(300);
         GPIO_SetBits(GPIOB, GPIO_Pin_12);
         delay_ms(300);
      break;
   }

#else /* CM-530 */
   switch(led)
   {
      case LED_BLUE_ON:
         GPIO_SetBits(GPIOB, GPIO_Pin_14);
         break;
      case LED_GREEN_ON:
         GPIO_SetBits(GPIOB, GPIO_Pin_13);
         break;
      case LED_RED_ON:
         GPIO_SetBits(GPIOB, GPIO_Pin_15);
         break;
      case LED_BLUE_OFF:
         GPIO_ResetBits(GPIOB, GPIO_Pin_14);
         break;
      case LED_GREEN_OFF:
         GPIO_ResetBits(GPIOB, GPIO_Pin_13);
         break;
      case LED_RED_OFF:
         GPIO_ResetBits(GPIOB, GPIO_Pin_15);
         break;
      case LED_BLUE_BLINK:
            GPIO_SetBits(GPIOB, GPIO_Pin_14);
            delay_ms(300);
            GPIO_ResetBits(GPIOB, GPIO_Pin_14);
            delay_ms(300);
         break;
      case LED_GREEN_BLINK:
            GPIO_SetBits(GPIOB, GPIO_Pin_13);
            delay_ms(300);
            GPIO_ResetBits(GPIOB, GPIO_Pin_13);
            delay_ms(300);
         break;
      case LED_RED_BLINK:
            GPIO_SetBits(GPIOB, GPIO_Pin_15);
            delay_ms(300);
            GPIO_ResetBits(GPIOB, GPIO_Pin_15);
            delay_ms(300);
         break;
   }
#endif
}
/**
  * @brief  Initialize the 6 Leds in the CM-530
  * @param  None
  * @retval None
  */
void LED_initialize(void)
{
   /* Enable GPIOB Periph clock */
   RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
   /* Enable GPIOC Periph clock */
   RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

#ifdef STM32F10X_HD /* CM-530 */
   /* Configure and set PB12, PB13, PB14 & PB15 in output pushpull mode */
   GPIO_InitTypeDef GPIO_InitStructure;
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
   GPIO_Init(GPIOB, &GPIO_InitStructure);

   /* Configure and set PC13, PC14 & PC15 in output pushpull mode */
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
   GPIO_Init(GPIOC, &GPIO_InitStructure);

#else /* OpenCM0.94 */
   /* Configure and set PB9(OpenCM), PB12(Shield), PB13(Shield) & PB14(Shield) in output pushpull mode */
   GPIO_InitTypeDef GPIO_InitStructure;
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14;
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
   GPIO_Init(GPIOB, &GPIO_InitStructure);
   #endif
   
   /* Turn off board lights */
   led(LED_BLUE_OFF); 
   led(LED_GREEN_OFF);
   led(LED_RED_OFF);
}

void CM530_Buttons_Init(void)
{
//EXTI Line3 to Start_Button (PA3)
//EXTI Line10 to D_Button    (PC10)
//EXTI Line11 to U_Button    (PC11)
   
#ifdef STM32F10X_HD/* CM-530 */
   /* Enable GPIOB Periph clock */
   RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

   /* Enable GPIOC Periph clock */
   RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

   /* AFIO clock enable */
   RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

   /* Configure and set PB3 in intput pushpull mode */
   GPIO_InitTypeDef   GPIO_InitStructure;
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
   GPIO_Init(GPIOB, &GPIO_InitStructure);
   /* Configure and set PC10 & PC11 in intput pushpull mode */
   GPIO_InitTypeDef   GPIO_InitStructure;
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
   GPIO_Init(GPIOC, &GPIO_InitStructure);

   /* Mapping EXTI Line3 to PB3 pin */
   GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource3);
   /* Mapping EXTI Line10 to PC10 pin */
   GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource10);
   /* Mapping EXTI Line11 to PC11 pin */
   GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource11);

   /* Configure and set EXTI Line3 */
   EXTI_InitTypeDef   EXTI_InitStructure;
   EXTI_InitStructure.EXTI_Line = EXTI_Line3;
   EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
   EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
   EXTI_InitStructure.EXTI_LineCmd = ENABLE;
   EXTI_Init(&EXTI_InitStructure);
   /* Configure and set EXTI Line10 */
   EXTI_InitTypeDef   EXTI_InitStructure;
   EXTI_InitStructure.EXTI_Line = EXTI_Line10;
   EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
   EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
   EXTI_InitStructure.EXTI_LineCmd = ENABLE;
   EXTI_Init(&EXTI_InitStructure);
   /* Configure and set EXTI Line11 */
   EXTI_InitTypeDef   EXTI_InitStructure;
   EXTI_InitStructure.EXTI_Line = EXTI_Line11;
   EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
   EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
   EXTI_InitStructure.EXTI_LineCmd = ENABLE;
   EXTI_Init(&EXTI_InitStructure);

   /* Configure and set EXTI Line3 Interrupt to the lowest priority */
   NVIC_InitTypeDef   NVIC_InitStructure;
   NVIC_InitStructure.NVIC_IRQChannel = EXTI3_IRQn ;
   NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;
   NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;
   NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
   NVIC_Init(&NVIC_InitStructure);
   /* Configure and set EXTI Line10 Interrupt to the lowest priority */
   NVIC_InitTypeDef   NVIC_InitStructure;
   NVIC_InitStructure.NVIC_IRQChannel = EXTI10_IRQn ;
   NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;
   NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;
   NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
   NVIC_Init(&NVIC_InitStructure);
   /* Configure and set EXTI Line11 Interrupt to the lowest priority */
   NVIC_InitTypeDef   NVIC_InitStructure;
   NVIC_InitStructure.NVIC_IRQChannel = EXTI11_IRQn ;
   NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;
   NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;
   NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
   NVIC_Init(&NVIC_InitStructure);
#endif
}

void OpenCM_Buttons_Init(void)
{
//EXTI Line3 to button2 -Exp_485- (PB4)

   /* Enable GPIOB Periph clock */
   RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

   /* AFIO clock enable */
   RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

#ifdef STM32F10X_MD /* OpenCM0.94 */
   /* Configure and set PB4 in intput pushpull mode */
   GPIO_InitTypeDef   GPIO_InitStructure;
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
   GPIO_Init(GPIOB, &GPIO_InitStructure);

   /* Mapping EXTI Line4 to PB4 pin */
   GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource4);

   /* Configure and set EXTI Line4 */
   EXTI_InitTypeDef   EXTI_InitStructure;
   EXTI_InitStructure.EXTI_Line = EXTI_Line4;
   EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
   EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
   EXTI_InitStructure.EXTI_LineCmd = ENABLE;
   EXTI_Init(&EXTI_InitStructure);

   /* Configure and set EXTI Line4 Interrupt to the lowest priority */
   NVIC_InitTypeDef   NVIC_InitStructure;
   NVIC_InitStructure.NVIC_IRQChannel = EXTI4_IRQn ;
   NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;
   NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;
   NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
   NVIC_Init(&NVIC_InitStructure);
#endif
}

/* ===== DELAY ===== */
void delay(volatile uint32_t n)
{
    while (n--)
    {
        __NOP();
    }
}


void delay_init(void)
{
   /* To adjust the SysTick time base, use the following formula:
   Reload Value = SysTick Counter Clock (Hz) x  Desired SysTick Time base(s)
       - Reload Value is the parameter to be passed to SysTick_Config() function
       - Reload Value should not exceed 0xFFFFFF (SysTick timer is only 24bits)
       - Reload Value is like ARR of Timers, the counter reset value
       - Lower reload value => more interruptions => lower calculation power from µC
       - A SysTick ISR is generated for each overflow (when counter = reload value) */
   
   ///////////////////////////////////////
   //IMPORTANTE: Si bajamos de 100us empieza a dar problemas y 1us se raya
   //es posible que haya que utilizar un crystal en condiciones, no el Internal del µC
   ///////////////////////////////////////
   ///////////////////////////////////////
   //IMPORTANTE: Si modificamos la Hz de SysTick_Config() hay que modificar también
   //DXL_delay_us() en dynamixel.c pues depende de este Timer !!
   ///////////////////////////////////////
                                          //SystemCoreClock       --> 1sec
   SysTick_Config(SystemCoreClock/1000);  //SystemCoreClock/1000  --> 1ms
}                                         //SystemCoreClock/10000 --> 100us (0,1ms)
void delay_ISR(void)
{
    usTicks++; //el tiempo de ejecución de la ISR es de 100us
	 //printf("\fInterrupt #%d\n", msTicks );
}
/**
  * @brief  To use only when we want a delay of decimal part of a millisecond
  * @param  if we want a delay of 1500us = > retardo_100us = 15
  * @param  if we want a delay of  500us = > retardo_100us = 5
  * @retval None
  */
void delay_us (uint32_t retardo_us) //es como si fuera delaly_ms()
{
   /* Checking parameters */
	assert_param(IS_RETARDO_US(retardo_us));
   
   //Uncomment for when SystemCoreClock/10000 within delay_init() -no funciona bien-
//   if(retardo_us > 100)
//      retardo_us /= 100; //pues cada Sys_Interrupt se ejecuta cada 100us
//   else retardo_us = 1;

   usTicks = 0; //reset usticks before entering loop
   while (usTicks < retardo_us) //check que el # de interrupciones es <= al retardo
   {
      __NOP();
   }
}
/**
  * @brief  
  * @param  None
  * @retval None
  */
void delay_ms (uint32_t retardo_ms)
{
   /* Checking parameters */
   assert_param(IS_RETARDO_MS(retardo_ms)); //retardo_ms debe ser inferior a uint32_t_max / 10
   
   //Uncomment for when SystemCoreClock/10000 within delay_init() -no funciona bien-
   //delay_us(retardo_ms * 10); //1ms = 10 x 100us
   delay_us(retardo_ms * 1); //1ms = 10 x 100us
}

/* ===== PARAMETER´S ASSERT  ===== */
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* Uncomment next line to know the Link Register (LR) from the stack */
  // __ASM volatile("BKPT #01");
  /* User can add his own implementation to report the file name and line number */
     printf("\nWrong parameters value: file %s on line %d\r\n", file, line);

  /* Infinite loop */
  while (1)
  {

  }
}

/* ===== ERROR HANDLER  ===== */
/**
  * @brief  To use when return from ErrorStatus functions = ERROR
  * @param
  * @param
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler */
}

/* ===== TRACE-SWO ===== */
/**
  * @brief  Configura SWO para poder utilizar printf
  * @param  None
  * @retval None
  */
void SWO_initialize(void)
{
   /* Enable APB2 peripheral AFIO */
   RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

   /* SWD configuration: Disable JTAG to release TRACESWO */
   GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);

   /* Enable TRACESWO (PB3) */
   DBGMCU->CR |= DBGMCU_CR_TRACE_IOEN;

   if (!(DBGMCU->CR & DBGMCU_CR_TRACE_IOEN))
   {
      // Some STM32s don't allow writes to DBGMCU register until
      // C_DEBUGEN in CoreDebug->DHCSR is set. This cannot be set by the
      // CPU itself, so in practice you need to connect to the CPU with
      // a debugger once before resetting it.
      printf("\nNo se ha podido iniciar TRACESWO");
   }

   /* Configure Trace Port Interface Unit (TPIU) */
   /* Ya lo hace Keil desde Options for target > Debug > Trace */
//   CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; // Enable access to registers
//   TPI->ACPR = 0; // Trace clock = HCLK/(x+1) = 8MHz
//   TPI->SPPR = 2; // Pin protocol = NRZ/USART
//   TPI->FFCR = 0x102; // TPIU packet framing enabled when bit 2 is set.
//                    // You can use 0x100 if you only need DWT/ITM and not ETM.

//   /* Configure PC sampling and exception trace  */
//   DWT->CTRL = (1 << DWT_CTRL_CYCTAP_Pos) // Prescaler for PC sampling
//                                          // 0 = x32, 1 = x512
//           | (0 << DWT_CTRL_POSTPRESET_Pos) // Postscaler for PC sampling
//                                             // Divider = value + 1
//           | (1 << DWT_CTRL_PCSAMPLENA_Pos) // Enable PC sampling
//           | (2 << DWT_CTRL_SYNCTAP_Pos)    // Sync packet interval
//                                            // 0 = Off, 1 = Every 2^23 cycles,
//                                            // 2 = Every 2^25, 3 = Every 2^27
//           | (1 << DWT_CTRL_EXCTRCENA_Pos)  // Enable exception trace
//           | (1 << DWT_CTRL_CYCCNTENA_Pos); // Enable cycle counter

   /* Configure instrumentation trace macroblock (ITM) */
   /* Ya lo hace Keil desde Options for target > Debug > Trace */
//   ITM->LAR = 0xC5ACCE55; //unlock write to the other ITM registers
//   ITM->TCR = (1 << ITM_TCR_TraceBusID_Pos) // Trace bus ID for TPIU
//          | (1 << ITM_TCR_DWTENA_Pos) // Enable events from DWT
//          | (1 << ITM_TCR_SYNCENA_Pos) // Enable sync packets
//          | (1 << ITM_TCR_ITMENA_Pos); // Main enable for ITM
//   ITM->TER = 0xFFFFFFFF; // Enable all stimulus ports
}


