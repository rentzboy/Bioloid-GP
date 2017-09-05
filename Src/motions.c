/**
  ******************************************************************************
  * @file    motions.c
  * @author  FX_Team
  * @version V1.0.0
  * @date    15-May-2017
  * @brief   This file provides firmware functions to manage the following
  *          functionalities of the dynamixel servos:
  *            + xxxxxxxxxxxxx
  *            + xxxxxxxxxxxxx
  *            + xxxxxxxxxxxxx
  *            + xxxxxxxxxxxxx
  *            + xxxxxxxxxxxxx
  *            + xxxxxxxxxxxxx
  *            + xxxxxxxxxxxxx
  *            + xxxxxxxxxxxxx
  ******************************************************************************

  ==============================================================================
                       ##### How to use this driver #####
  =============================================================================
  * >>
  * >>
  * >>
  * >>
  * >>
  * >>
  * >>
  * >>
  * >>
  * >>
  *****************************************************************************/

/******************************************************************************/
/*            MOTIONS FILE - IMPORT                                           */
/******************************************************************************/
/* Includes ------------------------------------------------------------------*/
#include "motions.h"  //motiontypedef
#include "globals.h"  //do not remove!
#include "dynamixel.h"
#include "zigbee.h"
#include "roboplus_motion.h"

/******************************************************************************/
/*            MOTIONS FILE - PRIVATE DECLARATION                              */
/******************************************************************************/
/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
//#define  RESET_CMD_FLAG        0    //Return from ExecuteCmd() 
#define  NO_PAUSE              0
#define  LOOPING_PAGE          1
#define  WALKING_PAGE          2
#define  LAST_STEP             6
#define  LAST_REPETITION       7

#define  NEXT_MOTION_PAGE(x)   p_motion_table[x]->next_page
#define  EXIT_MOTION_PAGE(x)   p_motion_table[x]->exit_page
#define  MAX_REPETITIONS(x)    p_motion_table[x]->repetition
#define  MAX_STEPS(x)          p_motion_table[x]->n_steps
#define  STEP_TIME(x)          p_motion_table[x]->time_step_x1000[indexSteps]
#define  PAUSE_TIME(x)         p_motion_table[x]->time_pause_x1000[indexSteps]

#define  RIGHT_FOOT_FIRT_CMD   (cmd == 3 || cmd == 9  || cmd == 13) 
#define  LEFT_FOOT_FIRT_CMD    (cmd == 4 || cmd == 10 || cmd == 14) 

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
// Private variable = Static variable in C programming; its scope is restricted to the current file.
const motiontypedef* p_motion_table[NUM_MOTION_MAX]; //GLOBAL VARIABLE
volatile motionStatus_typedef motionStatus;
exitFoot_typedef exitFoot;  //Remember if exit walking mode using left/right food for smooth transitions

//Se recalcula cada vez para cada motion (al no ser CONST la definimos fuera de la struct)
static uint16_t speedmotion[NUM_STEPS_MAX][NUM_DXL_MAX];

//static uint8_t currentPage; //Indicates de Roboplus motion page NO UTILIZADO, funcionamos con g_currentMotion
static uint8_t indexRepetitions;
static uint8_t maxRepetitions;
static uint8_t indexSteps;
static uint8_t maxSteps;
//Flags
static uint8_t nextMoveReady_flag;
static uint8_t newCmd_flag;
static uint8_t newMotion_flag;
static uint8_t loopingMode_flag;
static uint8_t walkingMode_flag;
static uint8_t multiPage_flag;
//Timer
uint16_t stepTimeInterruption;
uint16_t pauseTimeInterruption;

/* Private function prototypes -----------------------------------------------*/
//Las que no declaramos en el header, de manera que solo se pueden acceder desde este archivo
static void AdjustWalkingFoot(void);
static void MTN_AdvanceMotionCalculations(void);
static void MTN_NewCmdCalculations(void);
static void MTN_NewMotionCalculations(void);
static void MTN_CalculateSpeedMotion(void);
static uint8_t MTN_ExecutePages(void);
static uint8_t MTN_ExecuteStepsMotion(void);
static uint8_t MTN_MoveToNextPage(void);
static void MTN_ExecuteExitPage(void);
static void MTN_MotionTimeControl(void);

/******************************************************************************/
/*            MOTIONS FILE - FUNCTION DEFINITION                              */
/******************************************************************************/
/** @defgroup ---------- Initialization Motion functions ----------
  * @{
  */
/**
  * @brief  
  * @param
  * @retval
  */
void MTN_TIM3_initialize(void) //(PUBLIC)
{
   /* -------- TIM3 Configuration: Output Compare Active Mode -----------------
   In this example TIM3 input clock (TIM3CLK) is set to 2 * APB1 clock (PCLK1)
   TIM3CLK = 2*PCLK1 & PCLK1 = HCLK/2 => TIM3CLK = HCLK = SystemCoreClock = 56MHz

   To get TIM3 counter clock at 1 KHz (1ms), the prescaler is computed as follows:
   HCLK / (Prescaler+1) = TIM3_CLK => (Prescaler+1) = HCLK / TIM3_CLK
   => Prescaler = (56000000/1000) - 1 = 55999

   Generate 2 signals with 2 different delays (delay for CNT to reach/match CCRx_Val)
   TIM3_CH1 delay = step_time/TIM3 counter clock = step_time/1000
   TIM3_CH2 delay = pause_time/TIM3 counter clock = pause_time/1000

   --------------------------------------------------------------------------- */
   /* ===== TIM3 TIME BLOCK & INTERRUPTIONS CONFIGURATION ===== */
   /* TIM3 clock enable */
   RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

   /* TIM3 Time base configuration */
   TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
   TIM_TimeBaseStructure.TIM_Period = 0xFFFF; //ARR (counter reset value)
   TIM_TimeBaseStructure.TIM_Prescaler = 55999; //PSC
   TIM_TimeBaseStructure.TIM_ClockDivision = 0; //only when using filters
   TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
   TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

   /* Enable the TIM3 global Interrupt */
   NVIC_InitTypeDef   NVIC_InitStructure;
   NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
   NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
   NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
   NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
   NVIC_Init(&NVIC_InitStructure);

   /* ===== TIM3 OUTPUT CONFIGURATION: OUTPUT COMPARE MODE ===== */
   /* TIM3 Configuration Structure */
   TIM_OCInitTypeDef   TIM_OCInitStructure;
   TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_Timing; //Comparison match betweeen CNT and CCRx has no output effect (timebase mode)
   TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Disable;
   //TIM_OCInitStructure.TIM_Pulse = CCR1_Val; //TIMx->CCRx (los indicamos + abajo)
   TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

   /* Output Compare Timing Mode configuration: Channel1 */
   TIM_OCInitStructure.TIM_Pulse = stepTimeInterruption;
   TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Disable);
   TIM_OC1Init(TIM3, &TIM_OCInitStructure);

   /* Output Compare Timing Mode configuration: Channel2 */
   TIM_OCInitStructure.TIM_Pulse = pauseTimeInterruption;
   TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Disable);
   TIM_OC2Init(TIM3, &TIM_OCInitStructure);

   /* Auto-Reload @ ARR (TIMx->CR1) */
   //TIM_ARRPreloadConfig(TIM3, DISABLE);

   /* ===== TIM3 CUSTOM CONFIGURATIONS ===== */
   /* TIM enable counter */
   TIM_Cmd(TIM3, ENABLE);

   /* TIM Interrupts enable (TIMx->DIER) */
   //TIM_ITConfig(TIM3, TIM_IT_CC1 | TIM_IT_CC2, ENABLE); Enable later within MTN_MotionTimeControl()

   /* TIMx Event Source to be generated by software (TIMx->EGR) */
   //TIM_GenerateEvent(TIM3, TIM_EventSource_Update); //no le veo sentido aqui.....
}

/**
  * @brief  No es posible inicializar los valores uno por uno si no es dentro de una función
  *         Ejecutar esta función en las configuraciones iniciales al arrancar
  *         Hay que crear un puntero para pagina-motion de nuestro robot (76 para Bioloid GP)
  *         Las páginas de motions se crean con Roboplus Motions,
  *         se adaptan con Perl y se guardarn en el archivo motion_pages.h
  * @param
  * @retval
  */
void MTN_InitializeLookupTable(void) //(PUBLIC)
{
	//Hay + movimientos pero no los hemos implantado ni en esta tabla ni en el mando RC-100
   p_motion_table[INITIAL] =                  ((motiontypedef*) &Init);        //1
   p_motion_table[BALANCE] =                  ((motiontypedef*) &Balance);
   p_motion_table[FORWARD_R] =                ((motiontypedef*) &f_r1);
   p_motion_table[FORWARD_L] =                ((motiontypedef*) &f_l1);
   p_motion_table[FORWARD_R_LOOP] =           ((motiontypedef*) &f_r_l);       //5
   p_motion_table[FORWARD_L_LOOP] =           ((motiontypedef*) &f_l_r);
   p_motion_table[FORWARD_R_EXIT] =           ((motiontypedef*) &f_r2);
   p_motion_table[FORWARD_L_EXIT] =           ((motiontypedef*) &f_l2);
   p_motion_table[FAST_FORWARD_R] =           ((motiontypedef*) &Ff_r_l);
   p_motion_table[FAST_FORWARD_L] =           ((motiontypedef*) &Ff_l_r);      //10
   p_motion_table[WALK_FORWARD_LEFT] =        ((motiontypedef*) &f_l_r_l);
   p_motion_table[WALK_FORWARD_RIGHT] =       ((motiontypedef*) &f_r_l_r);
   p_motion_table[BACKWARDS_R] =              ((motiontypedef*) &b_r1);
   p_motion_table[BACKWARDS_L] =              ((motiontypedef*) &b_l1);
   p_motion_table[BACKWARDS_R_LOOP] =         ((motiontypedef*) &b_r_l);       //15
   p_motion_table[BACKWARDS_L_LOOP] =         ((motiontypedef*) &b_l_r);
   p_motion_table[BACKWARDS_R_EXIT] =         ((motiontypedef*) &b_r2);
   p_motion_table[BACKWARDS_L_EXIT] =         ((motiontypedef*) &b_l2);
   p_motion_table[RIGHT_FORWARD_DIAGONAL] =   ((motiontypedef*) &rf);
   p_motion_table[LEFT_FORWARD_DIAGONAL] =    ((motiontypedef*) &lf);	       //20
   p_motion_table[RIGHT_BACKWARDS_DIAGONAL] = ((motiontypedef*) &rb);
   p_motion_table[LEFT_BACKWARDS_DIAGONAL] =  ((motiontypedef*) &lb);
   p_motion_table[RIGHT_SIDESTEP] =           ((motiontypedef*) &r1);
   p_motion_table[LEFT_SIDESTEP] =            ((motiontypedef*) &l1);
   p_motion_table[FAST_RIGHT_SIDESTEP] =      ((motiontypedef*) &Fr1);         //25
   p_motion_table[FAST_LEFT_SIDESTEP] =       ((motiontypedef*) &Fl1);
   p_motion_table[TURN_RIGHT] =               ((motiontypedef*) &rt);
   p_motion_table[TURN_LEFT] =                ((motiontypedef*) &lt);
   p_motion_table[FORWARD_GET_UP] =           ((motiontypedef*) &F_getup);
   p_motion_table[BACKWARDS_GET_UP] =         ((motiontypedef*) &B_getup);      //30

   p_motion_table[SM_KICK_FORWARD_RIGHT] =    ((motiontypedef*) &r_f_kick);     //32
   p_motion_table[SM_KICK_FORWARD_LEFT] =     ((motiontypedef*) &l_f_kick);
   p_motion_table[SM_LATERAL_KICK_RIGHT] =    ((motiontypedef*) &r_r_kick);
   p_motion_table[SM_LATERAL_KICK_LEFT] =     ((motiontypedef*) &l_l_kick);
   p_motion_table[SM_RIGHT_PASS] =            ((motiontypedef*) &r_l_kick);
   p_motion_table[SM_LEFT_PASS] =             ((motiontypedef*) &l_r_kick);
   p_motion_table[SM_KICK_BACKWARDS_RIGHT] =  ((motiontypedef*) &r_b_kick);
   p_motion_table[SM_KICK_BACKWARDS_LEFT] =   ((motiontypedef*) &l_b_kick);
   p_motion_table[SM_DEFENSE] =               ((motiontypedef*) &DefenceReady);
   p_motion_table[SM_PUNCH_RIGHT] =           ((motiontypedef*) &RightDefence);
   p_motion_table[SM_PUNCH_LEFT] =            ((motiontypedef*) &LeftDefence);
   p_motion_table[SM_DEFENSE_STOP] =          ((motiontypedef*) &StopDefence);  //43

   p_motion_table[FM_PUNCH] =                 ((motiontypedef*) &f_attack);     //45
   p_motion_table[FM_FORWARD_JUMP] =          ((motiontypedef*) &P_fu_attack);
   p_motion_table[FM_FORWARD_JUMP_X] =        ((motiontypedef*) &P_fd_attack);
   p_motion_table[FM_COMBO_JUMP] =            ((motiontypedef*) &S_f_attack);
   p_motion_table[FM_PUNCH_RIGHT] =           ((motiontypedef*) &r_attack);
   p_motion_table[FM_PUNCH_LEFT] =            ((motiontypedef*) &l_attack);
   p_motion_table[FM_RIGHT_PUNCH_COMBO] =     ((motiontypedef*) &P_r_attack);
   p_motion_table[FM_LEFT_PUNCH_COMBO] =      ((motiontypedef*) &P_l_attack);
	p_motion_table[FM_DEFENSE] =               ((motiontypedef*) &Defence);
	p_motion_table[FM_DEFENSE_EXIT] =          ((motiontypedef*) &Defence_exit);  //54

   p_motion_table[SM_DEFENSE_EXIT] =          ((motiontypedef*) &SideDumbling_exit);  //70
}
/** @defgroup ---------- High-level Motion functions ----------
  * @{
  */
/**
  * @brief
  * @param
  * @retval
  */
uint8_t MTN_ExecuteCmd(void) //(PUBLIC)
{
   switch(motionStatus)
   {
      case(STEP_IS_MOVING): //updated by TIMER
         if(nextMoveReady_flag != SET)
            MTN_AdvanceMotionCalculations();
         return 0; //back to main()
      case(STEP_IS_OVER):  //updated by TIMER
         if(nextMoveReady_flag != SET)
            MTN_AdvanceMotionCalculations();
         if(PAUSE_TIME(g_currentMotion) == NO_PAUSE) //Pause = 0 -> execute next step
            break;
         else return 0; //back to main()
      case(PAUSE_IS_OVER):  //updated by TIMER
         break;
      case(MOTION_IS_OVER): //updated by user (updated the first time in BioloidInitialization()
         if(new_cmd_flag != SET) //updated from main()
            return 0; //back to main()
         //new cmd is available
         newCmd_flag = SET; //SET from MTN_ExecuteCmd(), RESET from MTN_NewCmdCalculations()
         g_lastMotion = g_currentMotion;
         g_currentMotion = cmd;
         break;
#ifdef DEBUG
      default:
         while(1); //ONLY FOR DEBUGGING, to catch errors...
#endif
   }
   /* ===== We are ready to send new command to servos ===== */
   MTN_ExecuteMotion();

   if(motionStatus != MOTION_IS_OVER)
      return 0;
   else
      return 1; //para resetear new_cmd_flag en main()
}
/**
  * @brief  Execute one motion (=Roboplus page)
  * @param  Pointer to the motion
  * @retval None
  */
void MTN_ExecuteMotion(void) //(PUBLIC)
{
   /* Calculations previous to execute a new cmd */
   if(newCmd_flag) //updated from MTN_ExecuteCmd()
      MTN_NewCmdCalculations();

   /* Calculations previous to execute a new motion page */
   if(newMotion_flag) //SET from MTN_NewCmdCalculations(), RESET from MTN_NewMotionCalculations() & SET from MTN_MoveToNextPage
      MTN_NewMotionCalculations();

   /* Execute motion pages */
   if(MTN_ExecutePages() == LAST_REPETITION)
   {
      if(MTN_MoveToNextPage() == EXIT) //updates g_currentMotion
         motionStatus = MOTION_IS_OVER;
   }
}
/** @defgroup ---------- Motion Time Control functions ----------
  * @{
  */
/**
  * @brief
  * @param
  * @retval Return time in ms
  */
void MTN_MotionTimeControl(void) //(PRIVATE)
{
   /* Update global status variable */
   motionStatus = STEP_IS_MOVING;

   /* Calculate Interruptions time */
   stepTimeInterruption = STEP_TIME(g_currentMotion);
   pauseTimeInterruption = stepTimeInterruption + PAUSE_TIME(g_currentMotion);
   
   /* Counter reset by software event generation */
   TIM_GenerateEvent(TIM3, TIM_EventSource_Update);

   /* SET Capture/Compare Timer variables(ms) & enable Interruptions */ 
   TIM_SetCompare1(TIM3, stepTimeInterruption);
   TIM_ITConfig(TIM3, TIM_IT_CC1, ENABLE);
   if(pauseTimeInterruption) // >0
   {
      TIM_ITConfig(TIM3, TIM_IT_CC2, ENABLE);
      TIM_SetCompare2(TIM3, pauseTimeInterruption);
   }
      
}
/** @defgroup ---------- Auxiliary Motion functions ----------
  * @{
  */
/**
  * @brief  If a walking cmd end in the right foot, the new one has to begin
  *         with the left foot for smooth transition
  * @param
  * @retval
  */
void AdjustWalkingFoot(void) //(PRIVATE)
{
   //Zigbee always send right foot cmds(3, 9 & 13), 
   //we have to convert into an alternance right/left 
   if(exitFoot == LEFT_FOOT)
   {
      //Update walking cmds
      if(cmd == FORWARD_R)
         cmd = FORWARD_L;
      else if(cmd == FAST_FORWARD_R)
         cmd = FAST_FORWARD_L;
      else if(cmd == BACKWARDS_R)
         cmd = BACKWARDS_L;
      //Update control variable for the next cmd
      exitFoot = RIGHT_FOOT;
   }
   else exitFoot = LEFT_FOOT;
} 
/**
  * @brief We use this function to use the µC spare time while STEP_IS_MOVING or PAUSE_TIME
  *        to make the calculations for the next servo moves. 
  *        We recalculate SpeedMotion for each repetition :((
  * @param
  * @retval
  */
void MTN_AdvanceMotionCalculations(void) //(PRIVATE)
{
   /* Caculate speed for the step to execute  */
   //This calculations could be wrong but in this case they will not be called from MTN_ExecuteStepsMotion()
   MTN_CalculateSpeedMotion();
   
   /* Update flag: IMPORTANT */
   nextMoveReady_flag = SET;
}
/**
  * @brief
  * @param
  * @retval
  */
void MTN_NewCmdCalculations(void) //(PRIVATE)
{
   /* Reset globlal flags for each new CMD */
   walkingMode_flag = RESET; //Update global variable
   loopingMode_flag = RESET; //Update global variable
   multiPage_flag   = RESET; //Update global variable

   /* Check for Walking, Looping & Mutiplage flags */
   //Checking if walking motion
   if(g_currentMotion > BALANCE && g_currentMotion < RIGHT_FORWARD_DIAGONAL)
   {
      walkingMode_flag = SET;
      AdjustWalkingFoot();
   }
      
   /* Checking if looping motion (balance, defense ready or defense motions) */
   if(g_currentMotion == SM_DEFENSE || g_currentMotion == FM_DEFENSE)
      loopingMode_flag = SET;

   /* Checking if multipage motion */
   if(NEXT_MOTION_PAGE(g_currentMotion) != EXIT)
      multiPage_flag = SET;

   /* Update flag to avoid re-entering */
   newCmd_flag = RESET;

   /* Mandatory for the 1ft motion page */
   newMotion_flag = SET;
}
/**
  * @brief
  * @param
  * @retval
  */
void MTN_NewMotionCalculations(void) //(PRIVATE)
{
   /* Calculate the Repetition limit for the current motion */
   maxRepetitions = MAX_REPETITIONS(g_currentMotion);

   /* Calculate the Step limit for the current motion */
   maxSteps = MAX_STEPS(g_currentMotion);
   
   /* Write CW/CCW Compliance Slope (joint softness) - DEPRECATED */
   //we use DXL_SyncWrite() for CW/CCW instead
   //uint16_t tmp = (p_motion_table[IndexMotion]->torque_joints) << 8;
   //tmp |= p_motion_table[IndexMotion]->torque_joints;
   //write same value @ CW_SLOPE & CCW_SLOPE
   //DXL_WriteWord(USART_DXL, BROADCAST_ID, CW_SLOPE, tmp);

   /* Control Inertial Force: related to ¿punch?, ¿CW/CCW compliance slope? */
   //no es importante, lo podemos obviar x ahora: además va aqui ??

   /* Update flag to avoid re-entering */
   newMotion_flag = RESET;
}
/**
  * @brief  Only the speedmotion for the current stepIndex is calculated
  *         1) Calculate the max angular speed
  *         From AX-12 manual(pag.3): @10V-->51rpm, @12V-->59rpm(max) = 0.983rps = 353º/s
  *         From AX-12 manaul(pag.16): 300º = 1023p => 360º = 1228p
  *         Max_angular_speed = 353 * 1228 / 360 = 1204p/s
  *
  *         2) Calculate the Max_register_value for the Max_angular_speed
  *         From AX-12 manual(pag.17): Max_register_value = 0x3FF (1023) --> 114rpm
  *         But since our Max_angular_speed = 59rpm => Max_register_value = (1023 * 59) / 114 = 530
  *         For a value of 530 or above of the Moving_speed register the angular speed will be 59rpm
  *         
  *         3) Calculate the angular_speed for each motion
  *         angular_speed = d / t = ((position2 - position1) / step_time) * speed_multiplier
  *         If(angular_speed > Max_angular_speed) => angular_speed = Max_angular_speed = 1204
  *
  *         4) Calculate the register_value for the angular_speed
  *         register_value | 530  |  ?            -> factor = 530/1204 = 0,44
  *         ------------------------------------
  *         speed (p/s)    | 1204 | angular_speed -> register_value = angular_speed * (530/1204)
  *           
  *         Resultados medidos por mi de las velocidades angulares: 
  *         register_value | 200 | 400 | 500 | 600 | 800(max)
  *         -------------------------------------------------
  *         speed (p/s)    | 350 | 700 | 800 | 850 | 900
  *         -------------------------------------------------   
  *         speed (rpm)    | 18  | 35  | 38  | 42  | 46
  *         -------------------------------------------------            
  *         factor         | .57 | .57 | .62 | .70 | .88 -> factor = 0,70  
  *            
  *            
  * @param
  * @retval
  */
void MTN_CalculateSpeedMotion(void) //(PRIVATE)
{
   uint16_t myposition1[NUM_DXL_MAX]; //begining position (LAST step of the previous motion)
   uint16_t myposition2[NUM_DXL_MAX]; //ending position   (FIRST step of the current motion)

   /* CASE #1: 1ft step of the motion */
   if(indexSteps == 0)
   {
      //Compute speedmotion[0][servo_id]
      for(uint8_t i=0; i<NUM_DXL_MAX; i++)
      {
         myposition1[i] = p_motion_table[g_lastMotion]->position[maxSteps-1][i];
         myposition2[i] = p_motion_table[g_currentMotion]->position[0][i];

         //Function description 3) -No olvidar x1000(time_step)-
         if(myposition1[i] > myposition2[i]) //si utilizamos ABS() NO funciona
            speedmotion[0][i] = (uint16_t)(((myposition1[i] - myposition2[i])*1000) / p_motion_table[g_currentMotion]->time_step_x1000[0]);
         else
            speedmotion[0][i] = (uint16_t)(((myposition2[i] - myposition1[i])*1000) / p_motion_table[g_currentMotion]->time_step_x1000[0]);
         
         //From angular_speed to its related register value -No olvidar x10(speed_multiplier)-
         speedmotion[0][i] *= (uint16_t) ((600 * p_motion_table[g_currentMotion]->speed_multiplier_x10) / (850 * 10));
         
         //Limit max/min moving speed register value
         if(speedmotion[0][i] > 1023)
            speedmotion[0][i] = 1023;
         if(speedmotion[0][i] < 250)
            speedmotion[0][i] = 250;
      }
   }
   /* CASE #2: 2nd to step limit of the motion */

   else //indexSteps > 0
   {
      //Fill data from motion structure
      for(uint8_t i=0; i<NUM_DXL_MAX; i++)
      {
         speedmotion[indexSteps][i] = p_motion_table[g_currentMotion]->speed[indexSteps][i];
      }
   }
}

/**
  * @brief  Execute all the motion pages from a pose (=Roboplus page)
  * @param
  * @retval None
  */
uint8_t MTN_ExecutePages(void) //(PRIVATE)
{
   /* Execute motion page */
   if(indexRepetitions < maxRepetitions)
   {
      if(MTN_ExecuteStepsMotion() == LAST_STEP)
      {
         if(++indexRepetitions == maxRepetitions)
         {
            indexRepetitions = RESET;
            return LAST_REPETITION;
         }
         else return 0;
      }
   }
#ifdef DEBUG
   else
      while(1); //ONLY FOR DEBUGGING, to catch errors...
#endif
#ifdef RELEASE
   else return 0; //no debería de pasar nunca
#endif
}
/**
  * @brief  Execute all the steps from a single motion (=Roboplus page)
  * @param
  * @retval None
  */
uint8_t MTN_ExecuteStepsMotion(void) //(PRIVATE)
{
   if(indexSteps < maxSteps)
   {
      /* Caculate speed for the step to execute */
      if(nextMoveReady_flag != SET)
         MTN_CalculateSpeedMotion();

      /* ExecuteStep (Goal position, speed & torque_joints) */
      DXL_SyncWrite(USART_DXL, g_currentMotion, indexSteps, CW_SLOPE, 6);
      
      /* Update TIMER & control variables */
      MTN_MotionTimeControl();
      
      /* Update global variable flag */
      nextMoveReady_flag = RESET;

      /* Check for last step motion */
      if(++indexSteps == maxSteps) //Increment afterwards DXL_SyncWrite() & MTN_MotionTimeControl()!!
      {
         indexSteps = RESET;
         return LAST_STEP;
      }
      else
         return 0;
   }
#ifdef DEBUG
   else while(1); //ONLY FOR DEBUGGING, to catch errors...
#endif
#ifdef RELEASE
   else return 0; //no debería de pasar nunca
#endif
}
uint8_t MTN_MoveToNextPage(void) //(PRIVATE)
{
   /* ===== At this point one motion page has been completed ===== */

   ////////////////////////////////////////////////////////////////////////////////
   //PENDING:Si estamos en modo automático ver si cmd = g_currentMotion_initial
   //y si es el mismo pues no ejecutamos la página de salida, seguimos en el bucle
   /////////////////////////////////////////////////////////////////////////////////

   /* 1ft exit call condition (without updated g_currentMotion) */
   if(multiPage_flag != SET || NEXT_MOTION_PAGE(g_currentMotion) == EXIT)
      return EXIT; //EXIT = 0

   /* Update flag to allow calculation for the next motion page */
   newMotion_flag = SET;

   /* Check if we need to prepare for Walking-Looping Exit */
   if(walkingMode_flag == SET || loopingMode_flag == SET)
   {
      MTN_ExecuteExitPage();
      return 1;
   }

   /* Update global variable -motion page- */
   else if(NEXT_MOTION_PAGE(g_currentMotion) != EXIT)
   {
      g_lastMotion = g_currentMotion;
      g_currentMotion = NEXT_MOTION_PAGE(g_currentMotion);
      return 1; 
   }
   else return EXIT;
}
/**
  * @brief  Prepare motion to get out for special commands
  * @param
  * @retval None
  */
void MTN_ExecuteExitPage(void) //(PRIVATE)
{
   /* Caso #1: LOOPING_PAGE */
   if(loopingMode_flag == SET)
   {
      /* Exit looping */
      if(g_currentMotion != g_lastMotion)
      {
         g_lastMotion = g_currentMotion;
         g_currentMotion = NEXT_MOTION_PAGE(g_currentMotion);
      }
      else 
         g_currentMotion = EXIT_MOTION_PAGE(g_currentMotion);
   }

   /* Caso #2: WALKING_PAGE */
   if(walkingMode_flag == SET)
   {
      /* Exit looping */
      if(g_lastMotion == NEXT_MOTION_PAGE(g_currentMotion))
      {
         g_lastMotion = g_currentMotion;
         g_currentMotion = EXIT_MOTION_PAGE(g_currentMotion);
      }
      else
      {
         g_lastMotion = g_currentMotion;
         g_currentMotion = NEXT_MOTION_PAGE(g_currentMotion);
      }
   }
}
/** @defgroup ---------- SET/GET Motion functions ----------
  * @{
  */
/**
  * @brief  To SET static newCmd_flag before manual ExecuteMotion() call
  * @param
  * @retval None
  */
void MTN_SetNewMotionFlag(void) //(PUBLIC)
{
   newCmd_flag = SET;
}
uint16_t MTN_GetSpeedMotion(uint8_t stepIndex, uint8_t servoIndex) //(PUBLIC)
{
   return speedmotion[stepIndex][servoIndex];
}
/** @defgroup ---------- Deprecated Motion functions ----------
  * @{
  */
const motiontypedef* MTN_MotionTableAccess(uint8_t Index) //***DEPRECATED***
{
	return p_motion_table[Index];
}


