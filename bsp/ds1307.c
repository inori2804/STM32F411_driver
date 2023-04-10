/*
 * ds1307.c
 *
 *  Created on: 6 Apr 2023
 *      Author: shiba
 */
#include "ds1307.h"
#include "stdint.h"
#include "string.h"

static void ds1307_pin_config(void);
static void ds1307_i2c_config(void);
static void ds1307_write(uint8_t reg_value, uint8_t reg_addr);
static uint8_t ds1307_read(uint8_t reg_value);
static uint8_t binary_to_bcd(uint8_t value);
static uint8_t bcd_to_binary(uint8_t value);

I2C_Handle_t Ds1307I2cHandle;


//return 1: init complete
//return 0: init fail
uint8_t ds1307_init(void)
{
	//1. Init I2C Pin
	ds1307_pin_config();

	//2. Init I2C Peripheral
	ds1307_i2c_config();

	//Enable I2C peripheral
	I2C_PeripheralsControl(DS1307_I2C, ENABLE);

	//Make clock halt equal to zero
	ds1307_write(0x00, DS1307_ADD_SEC);

	//Read clock halt bit
	uint8_t clock_state = ds1307_read(0x00);

	return ((clock_state << 7) & 0x1);
}

void ds1307_set_current_time(RTC_Time_t *pTime)
{
	uint8_t seconds = binary_to_bcd(pTime->seconds);
	seconds &= ~(0x1 << 7);
	ds1307_write(seconds, DS1307_ADD_SEC);

	uint8_t minutes = binary_to_bcd(pTime->minutes);
	ds1307_write(minutes, DS1307_ADD_MIN);

	uint8_t hours = binary_to_bcd(pTime->hours);

	if(pTime->time_format == TIME_FORMAT_24HRS)
	{
		hours &= ~(0x1 << 6);
	} else
	{
		hours |= (0x1 << 6);
		hours = (pTime->time_format == TIME_FORMAT_12HRS_PM) ? hours | (0x1 << 5) : hours & ~(0x1 << 5);
	}
	ds1307_write(hours, DS1307_ADD_HRS);
}

void ds1307_get_current_time(RTC_Time_t *pTime)
{
	uint8_t seconds = ds1307_read(DS1307_ADD_SEC);
	uint8_t minutes = ds1307_read(DS1307_ADD_MIN);
	uint8_t hours = ds1307_read(DS1307_ADD_HRS);

	seconds &= ~(0x1 << 7);
	pTime->seconds = bcd_to_binary(seconds);

	pTime->minutes = bcd_to_binary(minutes);

	if(hours & (0x1 << 6))
	{
		//this 24 format
		pTime->time_format = TIME_FORMAT_24HRS;
	} else
	{
		//this is 12 format
		if(hours & (0x1 << 5))
		{
			//this is PM
			pTime->time_format = TIME_FORMAT_12HRS_PM;
			hours &= ~(0x3 << 5);
		} else
		{
			//this is AM
			pTime->time_format = TIME_FORMAT_12HRS_AM;
			hours &= ~(0x3 << 5);
		}
	}

	pTime->hours = bcd_to_binary(hours);
}

void ds1307_set_current_date(RTC_Date_t *pDate)
{
	ds1307_write(binary_to_bcd(pDate->date), DS1307_ADD_DATE);
	ds1307_write(binary_to_bcd(pDate->day), DS1307_ADD_DAY);
	ds1307_write(binary_to_bcd(pDate->month), DS1307_ADD_MONTH);
	ds1307_write(binary_to_bcd(pDate->year), DS1307_ADD_YEAR);
}
void ds1307_get_current_date(RTC_Date_t *pDate)
{
	uint8_t date = ds1307_read(DS1307_ADD_DATE);
	uint8_t day = ds1307_read(DS1307_ADD_DAY);
	uint8_t month = ds1307_read(DS1307_ADD_MONTH);
	uint8_t year = ds1307_read(DS1307_ADD_YEAR);

	pDate->date = bcd_to_binary(date);
	pDate->date = bcd_to_binary(day);
	pDate->month = bcd_to_binary(month);
	pDate->year = bcd_to_binary(year);
}

static void ds1307_pin_config(void)
{
	GPIO_Handle_t i2c_sda, i2c_scl;
	//Reset memory to avoid garbage value
	memset(&i2c_sda, 0, sizeof(i2c_sda));
	memset(&i2c_scl, 0, sizeof(i2c_scl));

	/*
	 * I2C1_SCL ==> PB6
	 * I2C1_SDA ==> PB7
	 */

	i2c_sda.pGPIOx = DS1307_I2C_SDA_PORT;
	i2c_sda.GPIO_PinConfig.GPIO_PinNumber = DS1307_I2C_SDA_PIN;
	i2c_sda.GPIO_PinConfig.GPIO_PinAltFunMode = 4;
	i2c_sda.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFUNC;
	i2c_sda.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	i2c_sda.GPIO_PinConfig.GPIO_PinOType = GPIO_OP_TYPE_OD;
	i2c_sda.GPIO_PinConfig.GPIO_PinPuPdControl = DS1307_I2C_PUPD;

	GPIO_Init(&i2c_sda);

	i2c_scl.pGPIOx = DS1307_I2C_SCL_PORT;
	i2c_scl.GPIO_PinConfig.GPIO_PinNumber = DS1307_I2C_SCL_PIN;
	i2c_scl.GPIO_PinConfig.GPIO_PinAltFunMode = 4;
	i2c_scl.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFUNC;
	i2c_scl.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	i2c_scl.GPIO_PinConfig.GPIO_PinOType = GPIO_OP_TYPE_OD;
	i2c_scl.GPIO_PinConfig.GPIO_PinPuPdControl = DS1307_I2C_PUPD;

	GPIO_Init(&i2c_scl);
}

static void ds1307_i2c_config(void)
{
	Ds1307I2cHandle.pI2Cx = I2C1;
	Ds1307I2cHandle.I2C_Config.I2C_SCLSpeed = DS1307_I2C_SPEED;
	Ds1307I2cHandle.I2C_Config.I2C_AckControl = I2C_ACK_ENABLE;
	I2C_Init(&Ds1307I2cHandle);
}

static void ds1307_write(uint8_t reg_value, uint8_t reg_addr)
{
	uint8_t tx[2];
	tx[0] = reg_addr;
	tx[1] = reg_value;
	I2C_MasterSendData(&Ds1307I2cHandle, tx, 2, DS1307_I2C_ADDR, I2C_SR_DISABLE);
}

static uint8_t ds1307_read(uint8_t reg_addr)
{
	uint8_t data;
	I2C_MasterSendData(&Ds1307I2cHandle, &reg_addr, 1, DS1307_I2C_ADDR, I2C_SR_ENABLE);
	I2C_MasterReceiveData(&Ds1307I2cHandle, &data, 1, DS1307_I2C_ADDR, I2C_SR_DISABLE);
	return data;
}

static uint8_t binary_to_bcd(uint8_t value)
{
	uint8_t m, n;

	if(value >= 10)
	{
		m = value / 10;
		n = value % 10;
	} else
	{
		return value;
	}
	return (uint8_t) ((m << 4) | n);
}

static uint8_t bcd_to_binary(uint8_t value)
{
	uint8_t n = value & 0x0F;
	uint8_t m = value >> 4;
	return m*10 + n;
}
