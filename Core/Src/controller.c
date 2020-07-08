#include "main.h"
#include "controller.h"

extern uint32_t RO_time,CTO_time,PP1_time,OCB_time, Machine_time_run;
extern uint8_t	CTO_time_p[3],RO_time_p[3],PP1_time_p[3],OCB_time_p[3];
uint8_t end_display_flag;
uint8_t PP_Out_Time_Flag,OCB_Out_Time_Flag,CTO_Out_Time_Flag,RO_Out_Time_Flag;



void time_to_array(uint32_t time, uint8_t* tim_p){
	int hour = time/60;
	*tim_p= to_mess(hour/100);
	*(tim_p + 1)=to_mess((hour%100)/10);
	*(tim_p + 2)=to_mess(hour%10);
}

uint8_t to_mess(int value){
		uint8_t result;
		switch (value) {
			case 1:
				(result)=L_ONE;
				break;
			case 2:
				(result)=L_TWO;
				break;
			case 3:
				(result)=L_THREE;
				break;
			case 4:
				(result)=L_FOUR;
				break;
			case 5:
				(result)=L_FIVE;
				break;
			case 6:
				(result)=L_SIX;
				break;
			case 7:
				(result)=L_SEVEN;
				break;
			case 8:
				(result)=L_EIGHT;
				break;
			case 9:
				(result)=L_NINE;
				break;
			case 0:
				(result)=L_ZERO;
				break;
		}
		return result;
}

void increase_time(uint8_t* flag){
	if (*flag) {
		 //san xuat
			(RO_time++);
			(CTO_time++);
			(OCB_time++);
			(PP1_time++);
		//Demo

//			(RO_time+=60);
//			(CTO_time+=60);
//			(OCB_time+=60);
//			(PP1_time+=60);

		*flag = 0;
		time_to_array(PP1_time,PP1_time_p);
		time_to_array(OCB_time,OCB_time_p);
		time_to_array(CTO_time,CTO_time_p);
		time_to_array(RO_time,RO_time_p);

		if(end_display_flag){

			end_display_flag =0;
		}
	}

}


void Ghi_data(uint32_t start_Adress, uint32_t End_Adress){
	HAL_FLASH_Unlock();
			FLASH_EraseInitTypeDef EraseInitStruct;
			EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
			EraseInitStruct.PageAddress = start_Adress;
			EraseInitStruct.NbPages = (End_Adress - start_Adress) / FLASH_PAGE_SIZE;
			uint32_t PAGEError = 0;
	HAL_FLASHEx_Erase(&EraseInitStruct, &PAGEError);
			HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, start_Adress, PP1_time);
			HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, start_Adress + 4, OCB_time);
			HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, start_Adress + 8, CTO_time);
			HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, start_Adress + 12, RO_time);
			HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, start_Adress + 16, Machine_time_run);
			HAL_FLASH_Lock();
}

uint32_t read_Flash(uint32_t addr){
			uint32_t Flash_data;
			Flash_data = *(uint32_t*) addr;
			return Flash_data;
}


void (*Activate_LED7[3]) (void);
void (*Activate_Segment[7])(int value);


static void Activate_LED7_1();
static void Activate_LED7_2();
static void Activate_LED7_3();
static void Activate_Segment_A(int value);
static void Activate_Segment_B(int value);
static void Activate_Segment_C(int value);
static void Activate_Segment_D(int value);
static void Activate_Segment_E(int value);
static void Activate_Segment_F(int value);
static void Activate_Segment_G(int value);


void Controller_Display_Mess(uint8_t *mess,uint8_t* seven_segment_switch_flag,uint8_t* end_display_flag){
	int i=0;
	*seven_segment_switch_flag=0;
	while ( i < 3 && !(*end_display_flag)) {
		if(*seven_segment_switch_flag){
			Activate_LED7[i]();
		    for (int j = 6;  j >= 0 ;  j--){
		    	int value = (*mess & (1 << j));
		    	Activate_Segment[j](value);
		    }
			*seven_segment_switch_flag=0;
			mess++;
	    	i++;
	    }
	}
}


void (*Activate_LED7[3])(void) = {
		Activate_LED7_1,
		Activate_LED7_2,
		Activate_LED7_3
};

void (*Activate_Segment[7])(int value) = {
		Activate_Segment_G,
		Activate_Segment_F,
		Activate_Segment_E,
		Activate_Segment_D,
		Activate_Segment_C,
		Activate_Segment_B,
		Activate_Segment_A
};

void Activate_LED7_1(){
	HAL_GPIO_WritePin(LED7_1_PORT, LED7_1_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LED7_2_PORT, LED7_2_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED7_3_PORT, LED7_3_PIN, GPIO_PIN_RESET);
}
void Activate_LED7_2(){
	HAL_GPIO_WritePin(LED7_1_PORT, LED7_1_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED7_2_PORT, LED7_2_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LED7_3_PORT, LED7_3_PIN, GPIO_PIN_RESET);
}

void Activate_LED7_3(){
	HAL_GPIO_WritePin(LED7_1_PORT, LED7_1_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED7_2_PORT, LED7_2_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED7_3_PORT, LED7_3_PIN, GPIO_PIN_SET);
}

void Activate_Segment_A(int value){
	HAL_GPIO_WritePin(LED7_A_PORT, LED7_A_PIN, value);
}
void Activate_Segment_B(int value){
	HAL_GPIO_WritePin(LED7_B_PORT, LED7_B_PIN, value);
}
void Activate_Segment_C(int value){
	HAL_GPIO_WritePin(LED7_C_PORT, LED7_C_PIN, value);
}
void Activate_Segment_D(int value){
	HAL_GPIO_WritePin(LED7_D_PORT, LED7_D_PIN, value);
}
void Activate_Segment_E(int value){
	HAL_GPIO_WritePin(LED7_E_PORT, LED7_E_PIN, value);
}
void Activate_Segment_F(int value){
	HAL_GPIO_WritePin(LED7_F_PORT, LED7_F_PIN, value);
}
void Activate_Segment_G(int value){
	HAL_GPIO_WritePin(LED7_G_PORT, LED7_G_PIN, value);
}


