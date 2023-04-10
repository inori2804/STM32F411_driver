/*
 * i2c_lcd.c
 *
 *  Created on: 8 Apr 2023
 *      Author: shiba
 */

#include "i2c_lcd_esp32s2.h"


static void i2c_pin_config(void);
static void i2c_config(void);

uint8_t rcv_buf[32];

I2C_Handle_t pI2C1Handle;

void i2c_init(void)
{
	//1. Config pin
	i2c_pin_config();

	//2. I2C1 config
	i2c_config();

	//3. Enable I2C
	I2C_PeripheralsControl(I2C1, ENABLE);
}

static void i2c_pin_config(void)
{
	GPIO_Handle_t I2CPins;

	I2CPins.pGPIOx = GPIOB;
	I2CPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFUNC;
	I2CPins.GPIO_PinConfig.GPIO_PinOType = GPIO_OP_TYPE_OD;
	I2CPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
	I2CPins.GPIO_PinConfig.GPIO_PinAltFunMode = 4;			//AF4 mode
	I2CPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	//SCL
	I2CPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NUMBER_6;
	GPIO_Init(&I2CPins);

	//SDA
	I2CPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NUMBER_7;
	GPIO_Init(&I2CPins);
}

static void i2c_config(void)
{
	pI2C1Handle.pI2Cx = I2C1;
	pI2C1Handle.I2C_Config.I2C_SCLSpeed = I2C_SCL_SPEED_SM;
	pI2C1Handle.I2C_Config.I2C_DeviceAddress = 0x61;
	pI2C1Handle.I2C_Config.I2C_FMDutyCycle = I2C_FM_DUTY_2;
	pI2C1Handle.I2C_Config.I2C_AckControl = I2C_ACK_ENABLE;
	I2C_Init(&pI2C1Handle);
}


void lcd_send_cmd(char cmd)
{
	char data_u, data_l;
	uint8_t data_t[4];
	data_u = (cmd & 0xf0);
	data_l = ((cmd << 4) & 0xf0);
	data_t[0] = data_u | 0x0C;  //en=1, rs=0
	data_t[1] = data_u | 0x08;  //en=0, rs=0
	data_t[2] = data_l | 0x0C;  //en=1, rs=0
	data_t[3] = data_l | 0x08;  //en=0, rs=0
	I2C_MasterSendData(&pI2C1Handle, (uint8_t*) data_t, 4, SLAVE_ADDRESS_LCD, 0);
}

void lcd_send_data(char data)
{
	char data_u, data_l;
	uint8_t data_t[4];
	data_u = (data & 0xf0);
	data_l = ((data << 4) & 0xf0);
	data_t[0] = data_u | 0x0D;  //en=1, rs=1
	data_t[1] = data_u | 0x09;  //en=0, rs=1
	data_t[2] = data_l | 0x0D;  //en=1, rs=1
	data_t[3] = data_l | 0x09;  //en=0, rs=1
	I2C_MasterSendData(&pI2C1Handle, (uint8_t*) data_t, 4, SLAVE_ADDRESS_LCD, 0);
}

void lcd_clear(void)
{
	lcd_send_cmd(0x00);
	for (int i = 0; i < 100; i++)
	{
		lcd_send_data(' ');
	}
}

void lcd_init(void)
{
	// 4 bit initialisation
	delay();  // wait for >40ms
	lcd_send_cmd(0x30);
	delay();  // wait for >4.1ms
	lcd_send_cmd(0x30);
	delay();  // wait for >100us
	lcd_send_cmd(0x30);
	delay();
	lcd_send_cmd(0x20);  // 4bit mode
	delay();

	// dislay initialisation
	lcd_send_cmd(0x28); // Function set --> DL=0 (4 bit mode), N = 1 (2 line display) F = 0 (5x8 characters)
	delay();
	lcd_send_cmd(0x08); //Display on/off control --> D=0,C=0, B=0  ---> display off
	delay();
	lcd_send_cmd(0x01);  // clear display
	delay();
	lcd_send_cmd(0x06); //Entry mode set --> I/D = 1 (increment cursor) & S = 0 (no shift)
	delay();
	lcd_send_cmd(0x0C); //Display on/off control --> D = 1, C and B = 0. (Cursor and blink, last two bits)
}

void lcd_send_string(char *str)
{
	while (*str)
		lcd_send_data(*str++);
}

void esp32s2_read()
{
	uint8_t command_code;
	command_code = 0x51;
	uint8_t Len = 0;
	//send 1st command
	I2C_MasterSendData(&pI2C1Handle, &command_code, 1, SLAVE_ADDRESS_ESP, 0);
	//Receive Len
	I2C_MasterReceiveData(&pI2C1Handle, &Len, 1 , SLAVE_ADDRESS_ESP, 0);

	command_code = 0x52;
	//send 2sd command
	I2C_MasterSendData(&pI2C1Handle, &command_code, 1, SLAVE_ADDRESS_ESP, 0);
	//Receive Data
	I2C_MasterReceiveData(&pI2C1Handle, rcv_buf, Len , SLAVE_ADDRESS_ESP, 0);
}

void delay(void)
{
	for(uint16_t i = 0; i < 1000; i++);
}

