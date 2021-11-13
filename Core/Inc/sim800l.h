#include "main.h"
//SIM800L DIAGNOSTIC MESSAGES
static const char* auto_baud_init_cmd = "AT\r\n";
static const char* get_signal_strenght_cmd = "AT+CSQ\r\n";
static const char* get_sim_number_cmd = "AT+CCID\r\n";
static const char* check_network_connection_cmd = "AT+CREG?\r\n";
static const char* check_sms_sending_fromat = "AT+CSCS=?\r\n";
//SIM800L CONFIGURATION MESSAGES
static const char* enable_command_echo = "ATE1\r\n";
static const char* disable_command_echo = "ATE0\r\n";
static const char* set_text_mode_cmd = "AT+CMGF=1\r\n";
static const char* set_sms_recieve_format_cmd = "AT+CNMI=1,2,0,0,0\r\n";
static const char* set_sms_send_format_cmd = "AT+CSCS=\"GSM\"\r\n";

//SMS SENDING HELPER MESSAGES
static const char* sms_prepare_sending_cmd = "AT+CMGS=\"+48xxxxxxxxx\"\r";
static const char* sms_msg_temperature_exceeded = "Temperature exceeded!\r";
static const char* sms_msg_temperature_correct = "Temperature back correct\r";
static uint8_t sms_copy_symbol_msg = 0x1A; 			//ctrl+z ascii code
static const char* sms_send_carriage_return = "\r";



void check_module_info(UART_HandleTypeDef *huart);
void check_signal_strength(UART_HandleTypeDef *huart);
void check_network_connection(UART_HandleTypeDef *huart);
void check_sim_number(UART_HandleTypeDef *huart);
void check_sms_sending_format(UART_HandleTypeDef *huart);

void set_text_mode(UART_HandleTypeDef *huart);
void set_sms_recieve_format(UART_HandleTypeDef *huart);
void set_sms_send_format(UART_HandleTypeDef *huart);

void sms_send(UART_HandleTypeDef *huart, const char *message);

void EnorDi_command_echo(UART_HandleTypeDef *huart, uint8_t EnorDi);
