/*
 * esp32s2_i2c_lcd.c
 *
 *  Created on: 8 Apr 2023
 *      Author: shiba
 */
#include "stm32f411xx.h"
#include "i2c_lcd_esp32s2.h"

int main()
{
	//init i2c communication
	i2c_init();

	//init lcd
	lcd_init();

	//set cursor then send string
	lcd_send_cmd(0x80 | 0x00);
	lcd_send_string("HELLO WORLD");

	delay();

	esp32s2_read();
	lcd_send_string("Xin chao");
	lcd_send_string(rcv_buf);
	while(1)
	{
		;
	}
	return 0;
}
