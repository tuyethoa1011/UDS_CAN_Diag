/**
  ******************************************************************************
  * File: button.c
  * Author: Ngo Le Tuyet Hoa - Sam
  * Author's contact: ngltuyethoa1011@gmail.com
  * Last Modification: 31/10/2023
  * Used for: STM32F103C8T6 - Could be used for other STM32 KIT/ MCU but needs modification.
  * Lib name: Button
  * @All rights reserved. Please use for learning and developing purpose
  ******************************************************************************
*/
#include "button.h"

uint8_t long_press_detect; //bien luu tru detect nhan giu lau

uint8_t btn1_press_flag;
uint8_t btn2_press_flag;

uint8_t test_flag_twicepress;


__weak void button_pressing_callback(Button_Typedef *ButtonX)
{

}

__weak void button_release_callback(void)
{
	btn1_press_flag = 1;
	btn2_press_flag = 1;
	test_flag_twicepress = 0;
	//lam gi do sau khi 2 nut nhan giu roi tha ra trong nay
	//extended feature when pressing buttons at the same time
	//ý tưởng: Khi nhấn nhiều nút cùng lúc chỉ khi thả nút ra mới thực hiện action
}

__weak void button_shortpressing_callback_500ms(Button_Typedef *ButtonX)
{

}

__weak void button_longpressing_callback_500ms(Button_Typedef *ButtonX)
{

}

////them cac ham con lai lien quan den xu ly nut bam vao
////>= 500 ms < 500ms
void button_handle(Button_Typedef *ButtonX)
{
	uint8_t state =  HAL_GPIO_ReadPin(ButtonX->GPIOx, ButtonX->GPIO_Pin); //get button state
	//cau hoi dat ra la bay gio lam sao de xac dinh la co 2 nut bay gio
	//---- Xu ly loc nhieu ----
	if(state != ButtonX->button_filter) //trang thai
	{
		ButtonX->button_filter = state;
		ButtonX->is_debouncing = 1;
		ButtonX->time_debounce = HAL_GetTick();
	}
	//---- Xac lap tin hieu ----
	if(ButtonX->is_debouncing && (HAL_GetTick() - ButtonX->time_debounce >= 15))
	{
		ButtonX->button_current = ButtonX->button_filter;
		ButtonX->is_debouncing = 0;
	}
	//---- Xu ly tin hieu nut nhan ----
	if(ButtonX->button_current != ButtonX->button_last) //phat hien tin hieu nut nhat chinh xac sau khi xu ly thay doi
	{
		//nut bam nhan xuong
		if(ButtonX->button_current == 0)
		{
			//button_pressing_callback(ButtonX);
			ButtonX->time_btn_press = HAL_GetTick();
			ButtonX->is_press_timeout = 1;
		}
		else
		{
			if(HAL_GetTick() - ButtonX->time_btn_press < 500) //xu ly nhan nha nhanh < 500ms
			{
				button_shortpressing_callback_500ms(ButtonX);
			}

			ButtonX->is_press_timeout = 0 ;
			ButtonX->is_long_press = 0 ;
			long_press_detect = ButtonX->is_long_press;
			//reset cac bien nhan 2 nut
			if(test_flag_twicepress == 1)
			{
				button_release_callback();
			}
		}
			ButtonX->button_last = ButtonX->button_current;
	}

	//-------------Xu li nhan giu lau hon 500ms----------------
	if(ButtonX->is_press_timeout == 1 && (HAL_GetTick() - ButtonX->time_btn_press >= 500)){
		ButtonX->is_long_press = 1;
		button_longpressing_callback_500ms(ButtonX);
		ButtonX->is_press_timeout = 0;

	}
}

//tong cong co bao nhieu ham callback: press_callback, release callback, longpressing, shortpressing


void button_Init(Button_Typedef *ButtonX,GPIO_TypeDef *GPIOx,uint16_t GPIO_Pin)
{
	ButtonX->GPIOx = GPIOx;
	ButtonX->GPIO_Pin = GPIO_Pin;

	ButtonX->button_current = 1;
	ButtonX->button_last = 1;
	ButtonX->button_filter = 1;
}
