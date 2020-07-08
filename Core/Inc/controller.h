/*
 * controller.h
 *
 *  Created on: Feb 28, 2020
 *      Author: toanp
 */

#ifndef CONTROLLER_H_
#define CONTROLLER_H_

//seven Segment led
#define		L_ZERO		0b1111110
#define		L_ONE		0b0110000
#define		L_TWO		0b1101101
#define		L_THREE		0b1111001
#define		L_FOUR 		0b0110011
#define		L_FIVE		0b1011011
#define		L_SIX		0b1011111
#define		L_SEVEN		0b1110000
#define		L_EIGHT		0b1111111
#define		L_NINE		0b1111011
#define		L_R			0b0000101
#define		L_O			0b0011101
#define		L_A			0b1110111
#define		L_C			0b1001110
#define 	L_D			0b0111101
#define 	L_F			0b1000111
#define 	L_E			0b1001111
#define		L_L			0b0001110
#define 	L_P			0b1100111
#define 	L_BLANK		0b0000000
#define 	L_I			0b0001100
#define 	L_H			0b0110111
#define 	L_U			0b0111110



#define 	LED7_1_PORT		GPIOB
#define 	LED7_2_PORT		GPIOB
#define 	LED7_3_PORT		GPIOB
#define 	LED7_1_PIN		GPIO_PIN_13
#define 	LED7_2_PIN		GPIO_PIN_12
#define 	LED7_3_PIN		GPIO_PIN_11

#define 	LED7_A_PORT		GPIOB
#define 	LED7_B_PORT		GPIOB
#define 	LED7_C_PORT		GPIOA
#define 	LED7_D_PORT		GPIOA
#define 	LED7_E_PORT		GPIOA
#define 	LED7_F_PORT		GPIOA
#define 	LED7_G_PORT		GPIOA

#define 	LED7_A_PIN		GPIO_PIN_14
#define 	LED7_B_PIN		GPIO_PIN_15
#define		LED7_C_PIN		GPIO_PIN_10
#define 	LED7_D_PIN		GPIO_PIN_11
#define 	LED7_E_PIN		GPIO_PIN_12
#define 	LED7_F_PIN		GPIO_PIN_8
#define 	LED7_G_PIN		GPIO_PIN_9

#define     SEVEN_SEG_REFRESH_RATE	5
//seven segment led


#define		CLR_BUTTON_PORT		GPIOA
#define		RO_BUTTON_PORT		GPIOA
#define		CTO_BUTTON_PORT		GPIOA
#define		PP1_BUTTON_PORT		GPIOA
#define		OCB_BUTTON_PORT		GPIOA

#define		CLR_BUTTON_PIN		GPIO_PIN_2
#define		RO_BUTTON_PIN		GPIO_PIN_3
#define		CTO_BUTTON_PIN		GPIO_PIN_4
#define		PP1_BUTTON_PIN		GPIO_PIN_6
#define		OCB_BUTTON_PIN		GPIO_PIN_5

#define 	BUTTON_INTERVAL 			500
#define 	CLR_BUTTON_INTERVAL			500
#define		PUMP_UPDATE_INTERVAL		1
#define		DISPLAY_INTERVAL			5
#define  	OVER_TIME_pp1		10800// 300//
#define 	OVER_TIME_OCB		21600 //600//
#define 	OVER_TIME_CTO		32400  // 900//
#define 	OVER_TIME_RO		43200	//1200//	 tinh bang phut


#define		LED_C1_PORT		GPIOB
#define		LED_C2_PORT		GPIOB
#define		LED_C3_PORT		GPIOB
#define		LED_C4_PORT		GPIOB
#define		LED_C5_PORT		GPIOB
#define		LED_C6_PORT		GPIOB
#define		LED_C7_PORT		GPIOB
#define		LED_C8_PORT		GPIOB
#define		LED_C9_PORT		GPIOB
#define		LED_C10_PORT	GPIOB


#define		LED_C1_PIN		GPIO_PIN_9
#define		LED_C2_PIN		GPIO_PIN_8
#define		LED_C3_PIN		GPIO_PIN_7
#define		LED_C4_PIN		GPIO_PIN_6
#define		LED_C5_PIN		GPIO_PIN_5
#define		LED_C6_PIN		GPIO_PIN_4
#define		LED_C7_PIN		GPIO_PIN_3
#define		LED_C8_PIN		GPIO_PIN_1
#define		LED_C9_PIN		GPIO_PIN_0
#define		LED_C10_PIN		GPIO_PIN_10

#define		WARTER_INPUT_PORT 	GPIOA
#define		PUMP_STATUS_PORT	GPIOA

#define		PUMP_STATUS_PIN		GPIO_PIN_1
#define		WARTER_INPUT_PIN	GPIO_PIN_0




uint8_t to_mess(int);
void Controller_Display_Mess(uint8_t*,uint8_t*,uint8_t*);

void increase_time(uint8_t*);
void time_to_array(uint32_t,uint8_t*);
void Ghi_data(uint32_t, uint32_t);
uint32_t read_Flash(uint32_t );

#endif /* CONTROLLER_H_ */
