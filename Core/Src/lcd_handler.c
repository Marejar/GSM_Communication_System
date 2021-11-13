/*
 * lcd_handler.c
 *
 *  Created on: 27 wrz 2021
 *      Author: Marek Jaromin
 */

#include "lcd_handler.h"
#include "lcd16x2_i2c.h"
#include <stdbool.h>
#include <string.h>

extern TIM_HandleTypeDef htim3;

static char *lcd_message_marker = "Message:";
static char *lcd_message_no_events = "No events...";

message_t message_to_display_on_LCD = {NULL, 0};
char msg_to_display_copy[100];

static volatile uint8_t lcd_state = LCD_FREE;

/**
  * @brief STORES MESSAGE WHICH SHOULD BE DISPLAYED ON LCD TO THE GLOBAL VARIABLE(STRUCTURE)
  * @param POINTER TO MESSAGE WHICH SHOULD BE DISPLAYED ON LCD
  */
void set_msg_to_print_on_lcd(const char *msg_to_display_on_lcd){

    memcpy(msg_to_display_copy, msg_to_display_on_lcd, strlen(msg_to_display_on_lcd));
    message_to_display_on_LCD.lcd_msg_length = strlen(msg_to_display_on_lcd);
    message_to_display_on_LCD.ptr_to_LCD_msg = msg_to_display_on_lcd;

}

/**
  * @brief DISPLAYING MESSAGE ON LCD. FUNCTION SHIFTS MESSAGE ON LCD UNTILL
  * WHOLE MESSAGE IS DISPLAYED. SHIFTING IS EXECUTED IN TIMER INTERRUPT CALLBACK
  */
void print_on_lcd(void){

	//ENABLING TIMER IN IT MODE TO SHIFT MESSAGE LETTERS ON LCD
    HAL_TIM_Base_Start_IT(&htim3);
    if(lcd_state == LCD_FREE){

        lcd_state = LCD_BUSY;

        lcd16x2_i2c_clear();
		lcd16x2_i2c_setCursor(0, 0);
		lcd16x2_i2c_printf(lcd_message_marker);
		lcd16x2_i2c_setCursor(1, 0);
		lcd16x2_i2c_printf(message_to_display_on_LCD.ptr_to_LCD_msg);

        message_to_display_on_LCD.ptr_to_LCD_msg++;
        message_to_display_on_LCD.lcd_msg_length--;
    }else{

        if(message_to_display_on_LCD.lcd_msg_length >= LCD_CHARS_PER_ROW && *(message_to_display_on_LCD.ptr_to_LCD_msg) != '\0'){

        	lcd16x2_i2c_setCursor(1, 0);
    		lcd16x2_i2c_printf(message_to_display_on_LCD.ptr_to_LCD_msg);

            message_to_display_on_LCD.ptr_to_LCD_msg++;
            message_to_display_on_LCD.lcd_msg_length--;
        }else{
            message_to_display_on_LCD.lcd_msg_length = 0;
            message_to_display_on_LCD.ptr_to_LCD_msg = NULL;
            memset(msg_to_display_copy, 0, strlen(msg_to_display_copy));
            lcd_state = LCD_FREE;

			lcd16x2_i2c_clear();
			lcd16x2_i2c_setCursor(0, 0);
			lcd16x2_i2c_printf(lcd_message_no_events);

            //DISABLING TIMER AFTER WHOLE MESSAGE HAS BEEN DISPLAYED
            HAL_TIM_Base_Stop_IT(&htim3);
        }
    }
}















