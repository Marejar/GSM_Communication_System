/*
 * lcd_handler.h
 *
 *  Created on: 27 wrz 2021
 *      Author: PC
 */

#ifndef INC_LCD_HANDLER_H_
#define INC_LCD_HANDLER_H_

#include <stdint.h>

#define LCD_BUSY			0
#define LCD_FREE			1
#define LCD_CHARS_PER_ROW  16

typedef struct{
    char *ptr_to_LCD_msg;
    uint32_t lcd_msg_length;
}message_t;

void print_on_lcd(void);
void set_msg_to_print_on_lcd(const char *msg_to_diSplay_on_lcd);
#endif /* INC_LCD_HANDLER_H_ */
