/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f0xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f0xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "controller.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
 
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
uint8_t seven_segment_switch_flag;
uint8_t clr_button_flag;
uint8_t ocb_button_flag;
uint8_t ro_button_flag;
uint8_t cto_button_flag;
uint8_t pp1_button_flag;
uint8_t water_input_flag;
uint8_t pump_status_flag;
uint8_t end_display_flag;
uint8_t led_fan_flag;
uint8_t led_tank_full_flag;
uint8_t led_tank_full_flag_off;

uint8_t out_time_machine_flag;
uint8_t PP_Out_Time_Flag, OCB_Out_Time_Flag, CTO_Out_Time_Flag, RO_Out_Time_Flag;
uint8_t dislay_erro_trigger;

uint8_t OCB_Out_Time_Flag_off;
uint8_t PP_Out_Time_Flag_off;
uint8_t RO_Out_Time_Flag_off;
uint8_t CTO_Out_Time_Flag_off;
uint8_t Machine_Time_Flag_off;
uint8_t Water_low_Flag_off;
uint8_t lock_end_dislay_flag;
uint8_t erro_dislay_flag_off;
uint8_t watch_Dog_Flag;


uint8_t Machine_Time_Trigger_off;
uint8_t CTO_Out_Time_Trigger_off;
uint8_t PP_Out_Time_Trigger_off;
uint8_t RO_Out_Time_Trigger_off;
uint8_t OCB_Out_Time_Trigger_off;
uint8_t Water_low_trigger_off;
uint8_t erro_dislay_trigger;
uint8_t watch_Dog_Trigger;
uint8_t out_time_machine_trigger;
uint8_t Write_Flash_flag;





static int led_fan_cnt;
static int clr_button_cnt;
static int ro_button_cnt;
static int cto_button_cnt;
static int pp1_button_cnt;
static int ocb_button_cnt;

static int pump_status_cnt;
static int seven_segment_switch_cnt;
static int display_cnt;
static int out_time_machine_cnt;

static int led_tank_full_cnt;
static int OCB_Out_Time_Flag_off_cnt;
static int PP_Out_Time_Flag_off_cnt;
static int RO_Out_Time_Flag_off_cnt;
static int CTO_Out_Time_Flag_off_cnt;
static int water_low_Flag_off_cnt;
static int Machine_Out_Time_Flag_off_cnt;
static int erro_dislay_cnt;
static int watch_Dog_cnt;
static int write_Flash_cnt;

 uint32_t RO_time,CTO_time,PP1_time,OCB_time;
 uint8_t	*CTO_time_p,*RO_time_p,*PP1_time_p,*OCB_time_p;



static uint8_t clr_button_trigger;
static uint8_t ro_button_trigger;
static uint8_t cto_button_trigger;
static uint8_t pp1_button_trigger;
static uint8_t ocb_button_trigger;



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
void timer(uint8_t,uint8_t*,int*,int);
void timer2(uint8_t,uint8_t*,uint8_t*,int*,int);
void timer3(uint8_t,uint8_t*,uint8_t*,int*,int);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim6;
/* USER CODE BEGIN EV */
void timer(uint8_t trigger,uint8_t* flag,int* cnt,int interval){
	  if (trigger) {
		  (*cnt)++;
	  } else {
		  *cnt=0;
	  }
	  if (*cnt==interval){
		  *flag=1;
		  *cnt=0;
	  }
}
void timer2(uint8_t trigger,uint8_t* flag1,uint8_t* flag2, int* cnt,int interval){
	  if (trigger) {
		  (*cnt)++;
	  } else {
		  *cnt=0;
	  }
	  if (*cnt==interval){
		  *flag1=0;
		  *flag2=1;
		  *cnt=0;
	  }	else if (*cnt==interval-2) {
		  *flag1=1;
		 *flag2=0;

	}
}

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M0 Processor Interruption and Exception Handlers          */ 
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */
//	Ghi_data(0x08007000, 0x08007FFF);
  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVC_IRQn 0 */

  /* USER CODE END SVC_IRQn 0 */
  /* USER CODE BEGIN SVC_IRQn 1 */

  /* USER CODE END SVC_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F0xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f0xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles EXTI line 2 and 3 interrupts.
  */
void EXTI2_3_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI2_3_IRQn 0 */

  if((clr_button_trigger= HAL_GPIO_ReadPin(CLR_BUTTON_PORT, CLR_BUTTON_PIN))){
	  out_time_machine_trigger = 0;
	  out_time_machine_cnt = 0;
	  out_time_machine_flag = 0;

  }

  if ((ro_button_trigger=HAL_GPIO_ReadPin(RO_BUTTON_PORT, RO_BUTTON_PIN))) {
	  cto_button_flag = pp1_button_flag = ocb_button_flag =  water_input_flag = 0;
	  end_display_flag=0;
	  erro_dislay_trigger =0;
	  time_to_array(RO_time, RO_time_p);
	  lock_end_dislay_flag =1;
  }

  /* USER CODE END EXTI2_3_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_2);
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_3);
  /* USER CODE BEGIN EXTI2_3_IRQn 1 */

  /* USER CODE END EXTI2_3_IRQn 1 */
}

/**
  * @brief This function handles EXTI line 4 to 15 interrupts.
  */
void EXTI4_15_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI4_15_IRQn 0 */

  if ((cto_button_trigger=HAL_GPIO_ReadPin(CTO_BUTTON_PORT, CTO_BUTTON_PIN))){
	  ro_button_flag = ocb_button_flag = pp1_button_flag = water_input_flag=0;
	  lock_end_dislay_flag =1;
	  end_display_flag =0;
	  erro_dislay_trigger =0;
	  time_to_array(CTO_time, CTO_time_p);


  }

  if ((ocb_button_trigger=HAL_GPIO_ReadPin(OCB_BUTTON_PORT, OCB_BUTTON_PIN))){
	  ro_button_flag = cto_button_flag = pp1_button_flag =water_input_flag=0;
	  lock_end_dislay_flag =1;
	  end_display_flag=0;
	  erro_dislay_trigger =0;
	  time_to_array(OCB_time, OCB_time_p);



  }
  if((pp1_button_trigger=HAL_GPIO_ReadPin(PP1_BUTTON_PORT, PP1_BUTTON_PIN))){
	  ro_button_flag = cto_button_flag = ocb_button_flag =water_input_flag =0;
	  lock_end_dislay_flag =1;
	  end_display_flag=0;
	  erro_dislay_trigger =0;
	  time_to_array(PP1_time, PP1_time_p);

  }
  /* USER CODE END EXTI4_15_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_4);
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_5);
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_6);
  /* USER CODE BEGIN EXTI4_15_IRQn 1 */

  /* USER CODE END EXTI4_15_IRQn 1 */
}

/**
  * @brief This function handles TIM3 global interrupt.
  */
void TIM3_IRQHandler(void)
{
  /* USER CODE BEGIN TIM3_IRQn 0 */
	timer(clr_button_trigger, &clr_button_flag,&clr_button_cnt,CLR_BUTTON_INTERVAL);
	timer(cto_button_trigger, &cto_button_flag,&cto_button_cnt,BUTTON_INTERVAL);
	timer(pp1_button_trigger, &pp1_button_flag,&pp1_button_cnt,BUTTON_INTERVAL);
	timer(ocb_button_trigger, &ocb_button_flag,&ocb_button_cnt,BUTTON_INTERVAL);
	timer(ro_button_trigger, &ro_button_flag,&ro_button_cnt,BUTTON_INTERVAL);
	timer(lock_end_dislay_flag||PP_Out_Time_Trigger_off||OCB_Out_Time_Trigger_off||CTO_Out_Time_Trigger_off||RO_Out_Time_Trigger_off||
			Water_low_trigger_off||Machine_Time_Trigger_off, &seven_segment_switch_flag,&seven_segment_switch_cnt,SEVEN_SEG_REFRESH_RATE);
	timer(Write_Flash_flag, &led_fan_flag,&led_fan_cnt,350);
	timer(watch_Dog_Trigger, &watch_Dog_Flag, &watch_Dog_cnt, 150);
//
  /* USER CODE END TIM3_IRQn 0 */
  HAL_TIM_IRQHandler(&htim3);
  /* USER CODE BEGIN TIM3_IRQn 1 */

  /* USER CODE END TIM3_IRQn 1 */
}

/**
  * @brief This function handles TIM6 global interrupt.
  */
void TIM6_IRQHandler(void)
{
  /* USER CODE BEGIN TIM6_IRQn 0 */
	timer(HAL_GPIO_ReadPin(PUMP_STATUS_PORT, PUMP_STATUS_PIN), &Write_Flash_flag,&write_Flash_cnt, 5);
	timer(Write_Flash_flag, &pump_status_flag, &pump_status_cnt, 60);
	timer(Write_Flash_flag, &out_time_machine_trigger, &out_time_machine_cnt, 60); // timer machine out time
	timer2(!Write_Flash_flag,&led_tank_full_flag,&led_tank_full_flag_off,&led_tank_full_cnt,10);
	timer(lock_end_dislay_flag&&(!(ro_button_trigger||cto_button_trigger||ocb_button_trigger||pp1_button_trigger))
			,&end_display_flag,&display_cnt,DISPLAY_INTERVAL);

//	timer2(out_time_machine_trigger,&out_time_machine_flag,&end_display_flag,&LED_out_time_machine_cnt,10);

	timer(PP_Out_Time_Trigger_off,&PP_Out_Time_Flag_off,&PP_Out_Time_Flag_off_cnt,2);
	timer(OCB_Out_Time_Trigger_off,&OCB_Out_Time_Flag_off,&OCB_Out_Time_Flag_off_cnt,2);
	timer(CTO_Out_Time_Trigger_off,&CTO_Out_Time_Flag_off,&CTO_Out_Time_Flag_off_cnt,2);
	timer(RO_Out_Time_Trigger_off,&RO_Out_Time_Flag_off,&RO_Out_Time_Flag_off_cnt,2);
	timer(Water_low_trigger_off,&Water_low_Flag_off,&water_low_Flag_off_cnt,2);
	timer(Machine_Time_Trigger_off,&Machine_Time_Flag_off,&Machine_Out_Time_Flag_off_cnt,2);
	timer(erro_dislay_trigger, &erro_dislay_flag_off , &erro_dislay_cnt, 4);


  /* USER CODE END TIM6_IRQn 0 */
  HAL_TIM_IRQHandler(&htim6);
  /* USER CODE BEGIN TIM6_IRQn 1 */

  /* USER CODE END TIM6_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
