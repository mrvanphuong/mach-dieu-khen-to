/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include  "stm32f0xx_it.h"
#include "controller.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
IWDG_HandleTypeDef hiwdg;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim6;

/* USER CODE BEGIN PV */

extern uint8_t clr_button_flag;
extern uint8_t ocb_button_flag;
extern uint8_t ro_button_flag;
extern uint8_t cto_button_flag;
extern uint8_t pp1_button_flag;
extern uint8_t water_input_flag;
extern uint8_t pump_status_flag;
extern uint8_t end_display_flag;
extern uint8_t seven_segment_switch_flag;
extern uint8_t led_fan_flag;
extern uint8_t led_tank_full_flag;
extern uint8_t led_tank_full_flag_off;
extern uint8_t blink_flag;
extern uint8_t lock_end_dislay_flag;
extern uint8_t out_time_machine_flag;
extern uint8_t PP_Out_Time_Flag;
extern uint8_t OCB_Out_Time_Flag;
extern uint8_t CTO_Out_Time_Flag;
extern uint8_t RO_Out_Time_Flag;
extern uint8_t erro_dislay_flag_off;
extern uint8_t watch_Dog_Flag;

extern uint8_t dislay_erro_trigger;
extern uint8_t PP_Out_Time_Flag_off;
extern uint8_t PP_Out_Time_Trigger_off;
extern uint8_t OCB_Out_Time_Flag_off;
extern uint8_t OCB_Out_Time_Trigger_off;
extern uint8_t CTO_Out_Time_Flag_off;
extern uint8_t CTO_Out_Time_Trigger_off;
extern uint8_t RO_Out_Time_Flag_off;
extern uint8_t RO_Out_Time_Trigger_off;
extern uint8_t Machine_Time_Flag_off;
extern uint8_t Machine_Time_Trigger_off;
extern uint8_t Water_low_trigger_off;
extern uint8_t Water_low_Flag_off;
extern uint8_t erro_dislay_trigger;
extern uint8_t watch_Dog_Trigger;
extern uint8_t watch_Dog_Flag;
extern uint8_t out_time_machine_trigger;
extern uint8_t Write_Flash_flag;


uint32_t 	RO_time  ;
uint32_t 	CTO_time ;
uint32_t	PP1_time ;
uint32_t	OCB_time ;
uint32_t    Machine_time_run;
uint32_t	write_data;
uint8_t		CTO_time_p[3],RO_time_p[3],PP1_time_p[3],OCB_time_p[3];
uint8_t 	dislay_status_erro_cnt;

GPIO_TypeDef *(fan_led_port[10])={
	LED_C1_PORT,
	LED_C2_PORT,
	LED_C3_PORT,
	LED_C4_PORT,
	LED_C5_PORT,
	LED_C6_PORT,
	LED_C7_PORT,
	LED_C8_PORT,
	LED_C9_PORT,
	LED_C10_PORT
};

uint32_t fan_led_pin[10]={
	LED_C1_PIN,
	LED_C2_PIN,
	LED_C3_PIN,
	LED_C4_PIN,
	LED_C5_PIN,
	LED_C6_PIN,
	LED_C7_PIN,
	LED_C8_PIN,
	LED_C9_PIN,
	LED_C10_PIN
};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM6_Init(void);
static void MX_IWDG_Init(void);
/* USER CODE BEGIN PFP */
void display(void);
void on_end_display(uint8_t*);
void display_erro(void);
void led_fan_action(uint8_t*);
void led_tanks_full(void);
void out_time_fillter(void);
void clear_timer_fillter(void);
void write_Flash_Timer(void);



/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t test[3]={L_L,L_O,L_I};
uint8_t ro_alarm[3]={L_BLANK,L_R,L_O};
uint8_t out_time_Machine[3]={L_C, L_FIVE, L_H};
uint8_t out_time_PP[3]={L_P, L_P, L_ONE};
uint8_t out_time_ocb[3] = {L_U, L_D, L_F};
uint8_t out_time_cto[3] = {L_P, L_P, L_TWO};
uint8_t out_time_ro[3] = {L_R, L_O, L_BLANK};
uint8_t	blank_time[3] = {L_BLANK, L_BLANK, L_BLANK};

void check_out_time_machine(){
	if((out_time_machine_trigger)&&(HAL_GPIO_ReadPin(PUMP_STATUS_PORT, PUMP_STATUS_PIN))){
			Machine_time_run ++;
			out_time_machine_trigger = 0 ;
			write_data ++;


		}else
		if((!HAL_GPIO_ReadPin(PUMP_STATUS_PORT, PUMP_STATUS_PIN))){
			(Machine_time_run = 0);
			out_time_machine_trigger =0;
	}
}
// write da sua phai lai xem co lưu khong !!!
void write_Flash_Timer(){
	if (Write_Flash_flag&&((!HAL_GPIO_ReadPin(PUMP_STATUS_PORT, PUMP_STATUS_PIN)||(write_data==30)))){

		Write_Flash_flag = 0;
		write_data =0;
		Ghi_data(0x08007000, 0x08007FFF);


	}

}
void display (){
	if (cto_button_flag) {
		erro_dislay_trigger = 0;
//		CTO_time_p = time_to_array(CTO_time);
		Controller_Display_Mess(CTO_time_p,&seven_segment_switch_flag,&end_display_flag);
	}else if(ro_button_flag){

//		RO_time_p = time_to_array(RO_time);
		erro_dislay_trigger = 0;
		Controller_Display_Mess(RO_time_p,&seven_segment_switch_flag,&end_display_flag);
	}else if (ocb_button_flag) {

//		OCB_time_p = time_to_array(OCB_time);
		erro_dislay_trigger = 0;
		Controller_Display_Mess(OCB_time_p,&seven_segment_switch_flag,&end_display_flag);
	}else if (pp1_button_flag) {

//		PP1_time_p = time_to_array(PP1_time);
		erro_dislay_trigger = 0;
		Controller_Display_Mess(PP1_time_p, &seven_segment_switch_flag,&end_display_flag);
	}
}

void clear_timer_fillter(){
	if(clr_button_flag&&pp1_button_flag) {
			if((HAL_GPIO_ReadPin(PP1_BUTTON_PORT, PP1_BUTTON_PIN))&& (HAL_GPIO_ReadPin(CLR_BUTTON_PORT, CLR_BUTTON_PIN))){
				PP1_time = 0;
				clr_button_flag =0;
				pp1_button_flag =0;
				time_to_array(PP1_time,PP1_time_p);
				Ghi_data(0x08007000, 0x08007FFF);

			}
		}
	if(clr_button_flag&&ocb_button_flag) {
				if((HAL_GPIO_ReadPin(OCB_BUTTON_PORT, OCB_BUTTON_PIN))&& (HAL_GPIO_ReadPin(CLR_BUTTON_PORT, CLR_BUTTON_PIN))){
					OCB_time = 0;
					clr_button_flag = 0;
					ocb_button_flag = 0;
					time_to_array(OCB_time,OCB_time_p);
					Ghi_data(0x08007000, 0x08007FFF);

				}
		}
	if(clr_button_flag&&cto_button_flag) {
				if((HAL_GPIO_ReadPin(CTO_BUTTON_PORT, CTO_BUTTON_PIN))&& (HAL_GPIO_ReadPin(CLR_BUTTON_PORT, CLR_BUTTON_PIN))){
					CTO_time = 0;
					cto_button_flag =0;
					clr_button_flag =0;
					time_to_array(CTO_time,CTO_time_p);
					Ghi_data(0x08007000, 0x08007FFF);

				}
		}
	if(clr_button_flag&&ro_button_flag) {
				if((HAL_GPIO_ReadPin(RO_BUTTON_PORT, RO_BUTTON_PIN))&& (HAL_GPIO_ReadPin(CLR_BUTTON_PORT, CLR_BUTTON_PIN))){
					RO_time = 0;
					ro_button_flag=0;
					clr_button_flag = 0;
					time_to_array(RO_time,RO_time_p);
					Ghi_data(0x08007000, 0x08007FFF);

				}
		}
}

void display_erro (){

	if  ((HAL_GPIO_ReadPin(WARTER_INPUT_PORT, WARTER_INPUT_PIN))) {
					water_input_flag=0;
					water_input_flag =0;
					Water_low_Flag_off =0;
					Water_low_trigger_off =0;
				}


		if ((PP_Out_Time_Flag)&&!(pp1_button_flag||ocb_button_flag||cto_button_flag||ro_button_flag)){
					PP_Out_Time_Trigger_off =1;
					end_display_flag =0;
				Controller_Display_Mess(out_time_PP, &seven_segment_switch_flag,&end_display_flag);
				if (PP_Out_Time_Flag_off){
					PP_Out_Time_Flag =0;
					PP_Out_Time_Trigger_off=0;
					PP_Out_Time_Flag_off = 0;
				}
			}

		if ((!(PP_Out_Time_Flag))&&(OCB_Out_Time_Flag)&&!(pp1_button_flag||ocb_button_flag||cto_button_flag||ro_button_flag)){
					OCB_Out_Time_Trigger_off =1;
					end_display_flag =0;
				Controller_Display_Mess(out_time_ocb, &seven_segment_switch_flag,&end_display_flag);
				if (OCB_Out_Time_Flag_off){
					OCB_Out_Time_Flag =0;
					OCB_Out_Time_Trigger_off=0;
					OCB_Out_Time_Flag_off = 0;
				}
			}

		if ((!OCB_Out_Time_Flag)&&(!PP_Out_Time_Flag)&&(CTO_Out_Time_Flag)&&!(pp1_button_flag||ocb_button_flag||cto_button_flag||ro_button_flag)){
					end_display_flag =0;
					CTO_Out_Time_Trigger_off =1;
				Controller_Display_Mess(out_time_cto, &seven_segment_switch_flag,&end_display_flag);
				if (CTO_Out_Time_Flag_off){
					CTO_Out_Time_Flag =0;
					CTO_Out_Time_Trigger_off=0;
					CTO_Out_Time_Flag_off = 0;
					}
				}

		if ((!OCB_Out_Time_Flag)&&(!PP_Out_Time_Flag)&&(!CTO_Out_Time_Flag)&&(RO_Out_Time_Flag)&&!(pp1_button_flag||ocb_button_flag||cto_button_flag||ro_button_flag)){
					RO_Out_Time_Trigger_off =1;
					end_display_flag =0;
				Controller_Display_Mess(out_time_ro, &seven_segment_switch_flag,&end_display_flag);
				if (RO_Out_Time_Flag_off){
					RO_Out_Time_Flag =0;
					RO_Out_Time_Trigger_off=0;
					RO_Out_Time_Flag_off = 0;
				}
			}

		if ((!OCB_Out_Time_Flag)&&(!PP_Out_Time_Flag)&&(!CTO_Out_Time_Flag)&&(!RO_Out_Time_Flag)&&water_input_flag&&!(pp1_button_flag||ocb_button_flag||cto_button_flag||ro_button_flag)) {
							Water_low_trigger_off = 1;
							end_display_flag = 0;
					Controller_Display_Mess(test,&seven_segment_switch_flag,&end_display_flag);
						if (Water_low_Flag_off) {
							water_input_flag =0;
							Water_low_Flag_off =0;
							Water_low_trigger_off =0;
						}
					}

		if ((!OCB_Out_Time_Flag)&&(!water_input_flag)&&(!PP_Out_Time_Flag)&&(!CTO_Out_Time_Flag)&&(!RO_Out_Time_Flag)&&out_time_machine_flag&&!(pp1_button_flag||ocb_button_flag||cto_button_flag||ro_button_flag)) {
						Machine_Time_Trigger_off = 1;
						end_display_flag = 0;

				Controller_Display_Mess(out_time_Machine,&seven_segment_switch_flag,&end_display_flag);
				if (Machine_Time_Flag_off){
						out_time_machine_flag = 0;
						Machine_Time_Trigger_off = 0;
						Machine_Time_Flag_off = 0;
				}
			}
		if ((!OCB_Out_Time_Flag)&&(!water_input_flag)&&(!PP_Out_Time_Flag)&&(!CTO_Out_Time_Flag)&&(!RO_Out_Time_Flag)&&(!out_time_machine_flag)&&!
								(pp1_button_flag||ocb_button_flag||cto_button_flag||ro_button_flag)){
					erro_dislay_trigger = 1;
				on_end_display(&erro_dislay_trigger); /// sua thanh ghi
				if (erro_dislay_flag_off){
						erro_dislay_trigger =0;
						erro_dislay_flag_off = 0;
			}
		}
}



void out_time_fillter(){
	 if ((RO_time >= OVER_TIME_RO )&&(!dislay_erro_trigger)){
		 RO_Out_Time_Flag = 1;
	 }


	 if ((CTO_time >= OVER_TIME_CTO)&&(!dislay_erro_trigger)) {
		CTO_Out_Time_Flag = 1;
	}



	 if ((OCB_time >= OVER_TIME_OCB)&&(!dislay_erro_trigger)) {
		OCB_Out_Time_Flag =1 ;
	}


	if ((PP1_time >= OVER_TIME_pp1)&&(!dislay_erro_trigger)) {
		PP_Out_Time_Flag = 1;

	}
	/// thoi gian Chay qua thoi gian
	if((Machine_time_run >=300)&&(!dislay_erro_trigger)){
		out_time_machine_flag = 1;

	}
	if ((!HAL_GPIO_ReadPin(WARTER_INPUT_PORT, WARTER_INPUT_PIN))&&(!dislay_erro_trigger)) {
		water_input_flag =1;
	}
	if (RO_Out_Time_Flag||CTO_Out_Time_Flag||OCB_Out_Time_Flag||PP_Out_Time_Flag||water_input_flag||out_time_machine_flag||erro_dislay_trigger){
			dislay_erro_trigger =1;
		if (!(ro_button_flag||cto_button_flag||ocb_button_flag||pp1_button_flag)) {
			lock_end_dislay_flag = 0;
		}else {
			lock_end_dislay_flag = 1;
		}
	}else {
		dislay_erro_trigger =0;
		lock_end_dislay_flag = 1;
	}
 }
void on_end_display(uint8_t *flag){
	if(*flag){
		HAL_GPIO_WritePin(LED7_3_PORT, LED7_3_PIN, 0);
		HAL_GPIO_WritePin(LED7_2_PORT, LED7_2_PIN, 0);
		HAL_GPIO_WritePin(LED7_1_PORT, LED7_1_PIN, 0);
		*flag = 0;
		lock_end_dislay_flag = 0;
		ro_button_flag = cto_button_flag = ocb_button_flag = pp1_button_flag =0;



//		free(PP1_time_p);
//		free(OCB_time_p);
//		free(CTO_time_p);
//		free(RO_time_p);

	}
}

void led_fan_action(uint8_t* led_fan_flag){
	static int counting;
	static int fw_done;
	if ( (fw_done==0)&&(counting < 10)) {
		if (*led_fan_flag) {

			HAL_GPIO_WritePin(fan_led_port[counting], fan_led_pin[counting], 0);
			*led_fan_flag=0;
			HAL_GPIO_WritePin(fan_led_port[counting], fan_led_pin[counting-3], 1);
			counting++;

		}
		if(counting==10){
			fw_done=1;
			counting=0;
		}
	}
	if ((fw_done==1)&&(counting < 10)) {
		if (*led_fan_flag) {
			HAL_GPIO_WritePin(fan_led_port[9-counting], fan_led_pin[9-counting], 1);
			*led_fan_flag=0;
			counting++;

		}
		if(counting==10){
			fw_done=0;
			counting=0;
		}
	}
}

void led_tanks_full( ){
	if (led_tank_full_flag_off){

				HAL_GPIO_WritePin(LED_C1_PORT, LED_C1_PIN, 1);
				HAL_GPIO_WritePin(LED_C2_PORT, LED_C2_PIN, 1);
				HAL_GPIO_WritePin(LED_C3_PORT, LED_C3_PIN, 1);
				HAL_GPIO_WritePin(LED_C4_PORT, LED_C4_PIN, 1);
				HAL_GPIO_WritePin(LED_C5_PORT, LED_C5_PIN, 1);
				HAL_GPIO_WritePin(LED_C6_PORT, LED_C6_PIN, 1);
				HAL_GPIO_WritePin(LED_C7_PORT, LED_C7_PIN, 1);
				HAL_GPIO_WritePin(LED_C8_PORT, LED_C8_PIN, 1);
				HAL_GPIO_WritePin(LED_C9_PORT, LED_C9_PIN, 1);
				HAL_GPIO_WritePin(LED_C10_PORT, LED_C10_PIN, 1);

		}

	if (led_tank_full_flag) {

				HAL_GPIO_WritePin(LED_C1_PORT, LED_C1_PIN, 0);
				HAL_GPIO_WritePin(LED_C2_PORT, LED_C2_PIN, 0);
				HAL_GPIO_WritePin(LED_C3_PORT, LED_C3_PIN, 0);
				HAL_GPIO_WritePin(LED_C4_PORT, LED_C4_PIN, 0);
				HAL_GPIO_WritePin(LED_C5_PORT, LED_C5_PIN, 0);
				HAL_GPIO_WritePin(LED_C6_PORT, LED_C6_PIN, 0);
				HAL_GPIO_WritePin(LED_C7_PORT, LED_C7_PIN, 0);
				HAL_GPIO_WritePin(LED_C8_PORT, LED_C8_PIN, 0);
				HAL_GPIO_WritePin(LED_C9_PORT, LED_C9_PIN, 0);
				HAL_GPIO_WritePin(LED_C10_PORT, LED_C10_PIN, 0);
		}
	}





/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM3_Init();
  MX_TIM6_Init();
  MX_IWDG_Init();
  /* USER CODE BEGIN 2 */
  	  	  HAL_TIM_Base_Start_IT(&htim3);
  	  	  HAL_TIM_Base_Start_IT(&htim6);

  	  	  PP1_time = read_Flash((0x08007000));
  	  	  OCB_time = read_Flash((0x08007000+4));
  	  	  CTO_time = read_Flash((0x08007000+8));
  	  	  RO_time = read_Flash((0x08007000+12));
  	  	  if(HAL_GPIO_ReadPin(PUMP_STATUS_PORT, PUMP_STATUS_PIN)){
  	  		  Machine_time_run = read_Flash((0x08007000+16));
  	  	  }
  		time_to_array(PP1_time,PP1_time_p);
  		time_to_array(OCB_time,OCB_time_p);
  		time_to_array(CTO_time,CTO_time_p);
  		time_to_array(RO_time,RO_time_p);
  	  	HAL_GPIO_WritePin(LED7_A_PORT, LED7_A_PIN, GPIO_PIN_SET);
  	  		HAL_GPIO_WritePin(LED7_B_PORT, LED7_B_PIN, GPIO_PIN_SET);
  	  		HAL_GPIO_WritePin(LED7_C_PORT, LED7_C_PIN, GPIO_PIN_SET);
  	  		HAL_GPIO_WritePin(LED7_D_PORT, LED7_D_PIN, GPIO_PIN_SET);
  	  		HAL_GPIO_WritePin(LED7_E_PORT, LED7_E_PIN, GPIO_PIN_SET);
  	  		HAL_GPIO_WritePin(LED7_F_PORT, LED7_F_PIN, GPIO_PIN_SET);
  	  		HAL_GPIO_WritePin(LED7_G_PORT, LED7_G_PIN, GPIO_PIN_SET);

  	  	HAL_GPIO_WritePin(LED7_1_PORT, LED7_1_PIN, GPIO_PIN_SET);
  	  	HAL_Delay(500);
  	  	HAL_GPIO_WritePin(LED7_2_PORT, LED7_2_PIN, GPIO_PIN_SET);
  	  HAL_GPIO_WritePin(LED7_1_PORT, LED7_1_PIN, GPIO_PIN_RESET);
  	  	HAL_Delay(500);
  	  	HAL_GPIO_WritePin(LED7_3_PORT, LED7_3_PIN, GPIO_PIN_SET);
  	  HAL_GPIO_WritePin(LED7_2_PORT, LED7_2_PIN, GPIO_PIN_RESET);
  	  	HAL_Delay(500);
  	  HAL_GPIO_WritePin(LED7_3_PORT, LED7_3_PIN, GPIO_PIN_RESET);
  	  HAL_GPIO_WritePin(LED7_A_PORT, LED7_A_PIN, GPIO_PIN_RESET);
  	  	 HAL_GPIO_WritePin(LED7_B_PORT, LED7_B_PIN, GPIO_PIN_RESET);
  	   HAL_GPIO_WritePin(LED7_C_PORT, LED7_C_PIN, GPIO_PIN_RESET);
  	   HAL_GPIO_WritePin(LED7_D_PORT, LED7_D_PIN, GPIO_PIN_RESET);
  	   HAL_GPIO_WritePin(LED7_E_PORT, LED7_E_PIN, GPIO_PIN_RESET);
  	   HAL_GPIO_WritePin(LED7_F_PORT, LED7_F_PIN, GPIO_PIN_RESET);
  	   HAL_GPIO_WritePin(LED7_G_PORT, LED7_G_PIN, GPIO_PIN_RESET);
  	 HAL_Delay(500);

  	  	  watch_Dog_Trigger = 1;
  	  	  IWDG->KR = 0xAAAA; // Writing 0xAAAA in the Key register prevents watchdog reset
  	  	  IWDG->KR = 0xCCCC; // Start the independent watchdog timer


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  if(watch_Dog_Flag)
	 	  		{

	 	  			IWDG->KR = 0xAAAA;
	 	  			watch_Dog_Flag = 0;
	 	  		}

	  if (Write_Flash_flag){
		  led_tank_full_flag =0;
		  led_tank_full_flag_off =0;
	  }
	  out_time_fillter();
	  increase_time(&pump_status_flag);
	  display();
	  display_erro();
	  on_end_display(&end_display_flag);
  	  led_fan_action(&led_fan_flag);
	  led_tanks_full();
	  clear_timer_fillter();
	  check_out_time_machine();
//	  if(watch_Dog_Flag){
//	  HAL_WWDG_Refresh(&hwwdg);
//	  watch_Dog_Flag =0;
//	  }
	  write_Flash_Timer();



  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief IWDG Initialization Function
  * @param None
  * @retval None
  */
static void MX_IWDG_Init(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */

  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */

  /* USER CODE END IWDG_Init 1 */
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_256;
  hiwdg.Init.Window = 4095;
  hiwdg.Init.Reload = 4095;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */

  /* USER CODE END IWDG_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 799;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 9;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 8000;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 1000;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_10|GPIO_PIN_3 
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7 
                          |GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14 
                          |GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11 
                          |GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA0 PA1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA2 PA3 PA4 PA5 
                           PA6 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5 
                          |GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB10 PB11 
                           PB12 PB13 PB14 PB15 
                           PB3 PB4 PB5 PB6 
                           PB7 PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_10|GPIO_PIN_11 
                          |GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15 
                          |GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6 
                          |GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PA8 PA9 PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA11 PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI2_3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_3_IRQn);

  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
