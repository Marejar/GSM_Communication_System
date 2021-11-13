#include "main.h"
#include <string.h>
#include "sim800l.h"

extern UART_HandleTypeDef huart2;

void check_module_info(UART_HandleTypeDef *huart){

	uint16_t length = strlen(auto_baud_init_cmd);
	HAL_UART_Transmit(huart, (uint8_t*)auto_baud_init_cmd, length, 5);

}

void check_signal_strength(UART_HandleTypeDef *huart){

	uint16_t length = strlen(get_signal_strenght_cmd);
	HAL_UART_Transmit(huart, (uint8_t*)get_signal_strenght_cmd, length, 5);

}

void check_network_connection(UART_HandleTypeDef *huart){

	uint16_t length = strlen(check_network_connection_cmd);
	HAL_UART_Transmit(huart, (uint8_t*)check_network_connection_cmd, length, 5);

}

void check_sim_number(UART_HandleTypeDef *huart){

	uint16_t length = strlen(get_sim_number_cmd);
	HAL_UART_Transmit(huart, (uint8_t*)get_sim_number_cmd, length, 10);

}

void set_text_mode(UART_HandleTypeDef *huart){

	uint16_t length = strlen(set_text_mode_cmd);
	HAL_UART_Transmit(huart, (uint8_t*)set_text_mode_cmd, length, 10);

}

void set_sms_recieve_format(UART_HandleTypeDef *huart){

	uint16_t length = strlen(set_sms_recieve_format_cmd);
	HAL_UART_Transmit(huart, (uint8_t*)set_sms_recieve_format_cmd, length, 20);

}

void sms_send(UART_HandleTypeDef *huart, const char *message){

	uint16_t length = strlen(sms_prepare_sending_cmd);
	HAL_UART_Transmit(huart, (uint8_t*)sms_prepare_sending_cmd, length, 35);
	HAL_Delay(15);

	length = strlen(message);
	HAL_UART_Transmit(huart, (uint8_t*)message, length, 20);
	HAL_Delay(10);


	HAL_UART_Transmit(huart, &sms_copy_symbol_msg, sizeof(sms_copy_symbol_msg), 20);

	length = strlen(sms_send_carriage_return);
	HAL_UART_Transmit(huart, (uint8_t*)sms_send_carriage_return, length, 20);

}

void check_sms_sending_format(UART_HandleTypeDef *huart){

	uint16_t length = strlen(check_sms_sending_fromat);
	HAL_UART_Transmit(huart, (uint8_t*)check_sms_sending_fromat, length, 20);

}

void set_sms_send_format(UART_HandleTypeDef *huart){

	uint16_t length = strlen(set_sms_send_format_cmd);
	HAL_UART_Transmit(huart, (uint8_t*)set_sms_send_format_cmd, length, 20);

}


void EnorDi_command_echo(UART_HandleTypeDef *huart,  uint8_t EnorDi){

	uint16_t length = strlen(enable_command_echo);

	if(EnorDi == ENABLE){
	HAL_UART_Transmit(huart, (uint8_t*)enable_command_echo, length, 5);
	}else if(EnorDi == DISABLE){
	HAL_UART_Transmit(huart, (uint8_t*)disable_command_echo, length, 5);
	}

}
