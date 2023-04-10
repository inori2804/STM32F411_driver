/*
 * i2c_led.h
 *
 *  Created on: 8 Apr 2023
 *      Author: shiba
 */

#ifndef INC_I2C_LCD_ESP32S2_H_
#define INC_I2C_LCD_ESP32S2_H_

#include"stm32f411xx.h"

#define SLAVE_ADDRESS_LCD 	0x27 // change this according to ur setup
#define SLAVE_ADDRESS_ESP	0x68

extern uint8_t rcv_buf[32];

void i2c_init(void);

void lcd_init (void);   // initialize lcd

void lcd_send_cmd (char cmd);  // send command to the lcd

void lcd_send_data (char data);  // send data to the lcd

void lcd_send_string (char *str);  // send string to the lcd

void lcd_clear (void);

void delay();

void esp32s2_read();


#endif /* INC_I2C_LCD_ESP32S2_H_ */
