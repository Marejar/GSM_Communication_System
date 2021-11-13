
#include "main.h"


#define PHONE_NUMBER_PREFIX_OFFSET  6 //HELPER OFFSET TO GET PHONE NUMBER FROM SMS MESSAGE
#define COMMAND_CODE_OFFSET			5 //HELPER OFFSET TO GET COMMAND CODE FROM SMS MESSAGE

void PARSE_SMS_MESSAGE(char* message);
void EXECUTE_COMMAND(char* command);

void turn_on_led(void);
void turn_off_led(void);
