/*
 * i2c_master_tx_testing.c
 *
 *  Created on: 29 Mar 2023
 *      Author: shiba
 */

#include"stm32f411xx.h"
#include"string.h"
#include"stdio.h"

//extern void initialise_monitor_handles(void);

/*
 * PB6 -> SCL
 * PB7 -> SDA
 */

#define SLAVE_ADDR			0x68
#define MY_ADDRESS			SLAVE_ADDR

I2C_Handle_t I2C1Handle;
//some data
uint8_t tx_buffer[32] = "This is reply from ST32..";

void delay(void)
{
	for(uint16_t i = 0; i < 500000; i++);
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

    GPIOButton.pGPIOx = GPIOD;
    GPIOButton.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NUMBER_5;
    GPIOButton.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
    GPIOButton.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
    GPIOButton.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;

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
	I2C1Handle.I2C_Config.I2C_AckControl = I2C_ACK_ENABLE;
	I2C1Handle.I2C_Config.I2C_DeviceAddress = MY_ADDRESS;
	I2C1Handle.I2C_Config.I2C_FMDutyCycle = I2C_FM_DUTY_2;
	I2C1Handle.I2C_Config.I2C_SCLSpeed = I2C_SCL_SPEED_SM;

	I2C_Init(&I2C1Handle);
}


int main()
{

//	initialise_monitor_handles();
//	printf("Application is running \n");

	//button led init
	GPIO_ButtonLedInits();

	//i2c pin init
	I2C1_GPIOInits();

	//I2C1 init
	I2C1_Init();

	//I2C IRQ configuration
	I2C_IRQInterruptConfig(IRQ_NO_I2C1_EV, ENABLE);
	I2C_IRQInterruptConfig(IRQ_NO_I2C1_ER, ENABLE);

	I2C_EnableDisableCallBackEvents(I2C1, ENABLE);

	//enable i2c peripheral
	I2C_PeripheralsControl(I2C1, ENABLE);

	//enable ack
	I2C_ManageAcking(I2C1, I2C_ACK_ENABLE);

	while(1);
}

void I2C1_EV_IRQHandler(void)
{
	I2C_IRQ_EVHandling(&I2C1Handle);
}

void I2C1_ER_IRQHandler(void)
{
	I2C_IRQ_ERHandling(&I2C1Handle);
}

/*
 * Application function call back
 */
void I2C_ApplicationEventCallBack(I2C_Handle_t *pI2CHandle, uint8_t AppEv)
{
	//Static means private to that function but memory allocate to it is actually in global space
	static uint8_t commandCode = 0;
	static uint8_t Cnt = 0;
	if(AppEv == I2C_EV_DATA_REQ)
	{
		//master want some data, slave has to send it
		if(commandCode == 0x51)
		{
			//send the length information for the master
			I2C_SlaveSendData(pI2CHandle->pI2Cx, strlen((char*)tx_buffer));
			commandCode = 0xff;
		} else if(commandCode == 0x52)
		{
			//send context of Tx buff
			I2C_SlaveSendData(pI2CHandle->pI2Cx, tx_buffer[Cnt++]);
			commandCode = 0xff;
		}
	} else if(AppEv == I2C_EV_DATA_RCV)
	{
		//data is waiting for the slave to read. slave has to read it
		commandCode = I2C_SlaveReceiveDataIT(pI2CHandle->pI2Cx);
	} else if(AppEv == I2C_ERROR_AF)
	{
		//this happen only during slave txing
		//master has sent nack. so slave should understand that master does not need more data
		commandCode = 0xff;
		Cnt = 0;
	} else if(AppEv == I2C_EV_STOP)
	{
		//this happen only during slave reception
		//master has ended the i2c communication with the slave.
		commandCode = 0xff;
		Cnt = 0;
	}
}
