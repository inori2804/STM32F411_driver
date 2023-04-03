/*
 * i2c_master_tx_testing.c
 *
 *  Created on: 29 Mar 2023
 *      Author: shiba
 */

#include"stm32f411xx.h"
#include"string.h"
#include"stdio.h"

extern void initialise_monitor_handles(void);

/*
 * PB6 -> SCL
 * PB7 -> SDA
 */

#define MY_ADDRESS			0x61
#define SLAVE_ADDR			0x68

I2C_Handle_t I2C1Handle;
//some data
uint8_t rcv_buf[32];

uint8_t rxComplete = RESET;

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

	initialise_monitor_handles();
	printf("Application is running \n");

	uint8_t Len;
	uint8_t command_code;
	//button led init
	GPIO_ButtonLedInits();

	//i2c pin init
	I2C1_GPIOInits();

	//I2C1 init
	I2C1_Init();

	//I2C IRQ configuration
	I2C_IRQInterruptConfig(IRQ_NO_I2C1_EV, ENABLE);
	I2C_IRQInterruptConfig(IRQ_NO_I2C1_ER, ENABLE);

	//enable i2c peripheral
	I2C_PeripheralsControl(I2C1, ENABLE);

	//enable ack
	I2C_ManageAcking(I2C1, I2C_ACK_ENABLE);

	while(1)
	{
		//wait for button press
		while(!(GPIO_ReadFromInputPin(GPIOD, GPIO_PIN_NUMBER_5)));
		//avoid debouncing
		delay();
		command_code = 0x51;
		//send 1st command
		while(I2C_MasterSendDataIT(&I2C1Handle, &command_code, 1, SLAVE_ADDR, I2C_SR_ENABLE) != I2C_READY);
		//Receive Len
		while(I2C_MasterReceiveDataIT(&I2C1Handle, &Len, 1 , SLAVE_ADDR, I2C_SR_ENABLE) != I2C_READY);
		rxComplete = RESET;
		command_code = 0x52;
		//send 2sd command
		while(I2C_MasterSendDataIT(&I2C1Handle, &command_code, 1, SLAVE_ADDR, I2C_SR_ENABLE) != I2C_READY);
		//Receive Data
		while(I2C_MasterReceiveDataIT(&I2C1Handle, rcv_buf, Len , SLAVE_ADDR, I2C_SR_DISABLE) != I2C_READY);

		while(rxComplete != SET);

		rcv_buf[Len + 1] = '\0';
		printf("Data: %s\n", rcv_buf);

		rxComplete = RESET;
	}
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
	if(AppEv == I2C_EV_TX_CMPLT)
	{
		printf("TX is completed\n");
	} else if(AppEv == I2C_EV_RX_CMPLT)
	{
		printf("RX is completed\n");
		rxComplete = SET;
	} else if(AppEv == I2C_ERROR_AF)
	{
		printf("Error: Ack failure failure\n");
		//in master	ack failure happens when slave fails to send ack for the byte
		//send from master
		I2C_CloseSendData(pI2CHandle);

		//generate stop condition to realse the bus
		I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

		//Hanging in infinite loop
		while(1);
	}
}
