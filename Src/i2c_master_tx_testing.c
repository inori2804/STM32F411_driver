/*
 * i2c_master_tx_testing.c
 *
 *  Created on: 29 Mar 2023
 *      Author: shiba
 */

#include"stm32f411xx.h"
#include"string.h"
#include"stdio.h"

/*
 * PB6 -> SCL
 * PB9 -> SDA
 */

#define MY_ADDRESS			0x61
#define SLAVE_ADDR			0x68

I2C_Handle_t I2C1Handle;
//some data
uint8_t somedata[] = "Testing I2C!\n";

void delay(void)
{
	for(uint16_t i = 0; i < 5000 ; i++);
}

void GPIO_ButtonLedInits(void)
{
	// PD12 configuration as output for toggle LED
    GPIO_Handle_t GPIOLed;
    memset(&GPIOLed, 0, sizeof(GPIOLed));

    GPIOLed.pGPIOx = GPIOD;
    GPIOLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NUMBER_12;
    GPIOLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
    GPIOLed.GPIO_PinConfig.GPIO_PinOType = GPIO_OP_TYPE_PP;
    GPIOLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
    GPIOLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

    GPIO_PeripheralClockControl(GPIOD, ENABLE);
    GPIO_Init(&GPIOLed);

    // PD5 configuration as input for User press
    GPIO_Handle_t GPIOButton;
    memset(&GPIOButton, 0, sizeof(GPIOButton));

    GPIOButton.pGPIOx = GPIOA;
    GPIOButton.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NUMBER_0;
    GPIOButton.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
    GPIOButton.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
    GPIOButton.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PD;

//    GPIO_PeripheralClockControl(GPIOD, ENABLE);
    GPIO_Init(&GPIOButton);
}

void I2C1_GPIOInits(void)
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

void I2C1_Init(void)
{
	I2C1Handle.pI2Cx = I2C1;
	I2C1Handle.I2C_Config.I2C_AckControl = I2C_ACK_ENABLE;
	I2C1Handle.I2C_Config.I2C_DeviceAddress = MY_ADDRESS;
	I2C1Handle.I2C_Config.I2C_FMDutyCycle = I2C_FM_DUTY_2;
	I2C1Handle.I2C_Config.I2C_SCLSpeed = I2C_SCL_SPEED_SM;

	I2C_Init(&I2C1Handle);
}


int main()
{
	//buton led init
	GPIO_ButtonLedInits();

	//i2c pin init
	I2C1_GPIOInits();

	//I2C1 init
	I2C1_Init();

	//enable i2c peripheral
	I2C_PeripheralsControl(I2C1, ENABLE);

	while(1)
	{
		//wait for button press
		while(!(GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NUMBER_0)));
		//avoid debouncing
		delay();
		//send some data
		I2C_MasterSendData(&I2C1Handle,somedata, strlen((char*)somedata), SLAVE_ADDR, 0);
	}
}
