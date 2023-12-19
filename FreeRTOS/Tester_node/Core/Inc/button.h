/**
  ******************************************************************************
  * File: button.h
  * Author: Ngo Le Tuyet Hoa - Sam
  * Author's contact: ngltuyethoa1011@gmail.com
  * Last Modification: 31/10/2023
  * Used for: STM32F103C8T6 - Could be used for other STM32 KIT/ MCU but needs modification.
  * Lib name: Button
  * @All rights reserved. Please use for learning and developing purpose
  ******************************************************************************
*/

#ifndef _INC_BUTTON_H_
#define _INC_BUTTON_H_

#include "main.h"

typedef struct
{
	uint8_t button_current;
	uint8_t button_last;
	uint8_t button_filter;
	uint8_t is_debouncing;
	uint32_t time_debounce;
	uint32_t time_btn_press;
	uint8_t is_press_timeout;
	uint8_t is_long_press;
	GPIO_TypeDef *GPIOx;
	uint16_t GPIO_Pin;
}Button_Typedef;

extern uint8_t long_press_detect; //bien luu tru detect nhan giu lau

extern uint8_t btn1_press_flag;
extern uint8_t btn2_press_flag;

extern uint8_t test_flag_twicepress;

void button_handle(Button_Typedef *ButtonX);
void button_Init(Button_Typedef *ButtonX,GPIO_TypeDef *GPIOx,uint16_t GPIO_Pin);

#endif /* INC_LED_H_ */
