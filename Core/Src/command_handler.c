/*
 * command_handler.c
 *
 *  Created on: 16 wrz 2021
 *      Author: Marek Jaromin
 */

#include <string.h>
#include <stdio.h>
#include "main.h"
#include "lcd_handler.h"
#include "command_handler.h"

extern UART_HandleTypeDef huart2;

static const char *cmd_phone_number_prefix = "CMT: ";
static const char *cmd_prefix = "Cmd: ";
static const char *cmd_turn_on_led = "turn on led";
static const char *cmd_turn_off_led = "turn off led";

static char message_to_display_on_lcd[100];


/**
  * @brief PARSES MESSAGE SENT BY SIM800L WHEN SMS IS RECEIVED TO GET
  * INFORMATIONS ABOUT SENDER AND COMMAND SENT
  * @param POINTER TO MESSAGE SENT BY SIM800L
  */
void PARSE_SMS_MESSAGE(char* message){

	uint8_t counter = 0;
	char sender_phone_number[20];
	char sender_command[50];

	char *ptr_to_phone_number;
	char *ptr_to_command;

	//SAVING SENDER PHONE NUMBER
	ptr_to_phone_number = strstr(message, cmd_phone_number_prefix);
	if(ptr_to_phone_number != NULL){

		ptr_to_phone_number += PHONE_NUMBER_PREFIX_OFFSET;

		while(*ptr_to_phone_number != '"'){
			sender_phone_number[counter] = *ptr_to_phone_number;
			counter++;
			ptr_to_phone_number++;
		}
		sender_phone_number[counter] = '\0';
		counter = 0;
	}

	//SAVING SENDER COMMAND
	ptr_to_command = strstr(message, cmd_prefix);

		if(ptr_to_command != NULL){
		ptr_to_command += COMMAND_CODE_OFFSET;

		while(*ptr_to_command != '.' && (counter<49)){
			sender_command[counter] = *ptr_to_command;
			counter++;
			ptr_to_command++;
		}
		sender_command[counter] = '\0';
	}
	//EXECUTING COMMAND
	EXECUTE_COMMAND(sender_command);
	//DISPLAYING INFORMATIONS ABOUT SENDER AND COMMAND ON LCD
	sprintf(message_to_display_on_lcd, "Message from number: %s, || %s", sender_phone_number, sender_command);
	set_msg_to_print_on_lcd(message_to_display_on_lcd);
	print_on_lcd();
	//SENDING THE SAME MESSAGE VIA UART TO THE COMPUTER
	HAL_UART_Transmit(&huart2, (uint8_t*)message_to_display_on_lcd, strlen(message_to_display_on_lcd), HAL_MAX_DELAY);
}

/**
  * @brief EXECUTES COMMAND SENT VIA SMS(HELPER FUNCTION USED IN PARSE_SMS_MESSAGE)
  * @param POINTER TO COMMAND MESSAGE
  */
void EXECUTE_COMMAND(char* command){

	if(strncmp(command, cmd_turn_on_led, strlen(cmd_turn_on_led)) == 0 ){
		turn_on_led();
	}else if(strncmp(command, cmd_turn_off_led, strlen(cmd_turn_off_led)) == 0 ){
		turn_off_led();
	}

}


//FUNCTIONS WHICH CAN BE EXECUTED
void turn_on_led(void){
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
}

void turn_off_led(void){
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
}

