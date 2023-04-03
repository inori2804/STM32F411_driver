/*
 * spi_tx_testing.c
 *
 *  Created on: 16 Mar 2023
 *      Author: shiba
 */

//PB15 -- MOSI
//PB14 -- MISO
//PB13 -- SCLK
//PB12 -- NSS

#include"stm32f411xx.h"
#include"string.h"

void GPIO_ButonInit(void)
{
    GPIO_Handle_t GPIOButton;
    memset(&GPIOButton, 0, sizeof(GPIOButton));

    GPIOButton.pGPIOx = GPIOD;
    GPIOButton.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NUMBER_5;
    GPIOButton.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;
    GPIOButton.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
    GPIOButton.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;

    GPIO_Init(&GPIOButton);
}

void SPI2_GPIOInits(void)
{
	GPIO_Handle_t SPIPins;

	SPIPins.pGPIOx = GPIOB;
	SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFUNC;
	SPIPins.GPIO_PinConfig.GPIO_PinAltFunMode = 5;			//AF5 mode
	SPIPins.GPIO_PinConfig.GPIO_PinOType = GPIO_OP_TYPE_PP;
	SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	SPIPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	//SLCK
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NUMBER_13;
	GPIO_Init(&SPIPins);

	//MOSI
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NUMBER_15;
	GPIO_Init(&SPIPins);

	//MISO
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NUMBER_14;
	GPIO_Init(&SPIPins);

	//NSS
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NUMBER_12;
	GPIO_Init(&SPIPins);
}

void SPI2_Init(void)
{
	SPI_Handle_t SPI2handle;
	SPI2handle.pSPIx = SPI2;
	SPI2handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI2handle.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
	SPI2handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV8; //generate clock 8Mhz
	SPI2handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
	SPI2handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
	SPI2handle.SPIConfig.SPI_SSM = SPI_SSM_DI;

	SPI_Init(&SPI2handle);
}

void delay(void)
{
	for(uint16_t i = 0; i < 500000; i++);
}

int main(void)
{
	char userData[] = "Hello World!";

	SPI2_GPIOInits();

	SPI2_Init();

//	SPI_SSIConfig(SPI2, ENABLE);
	SPI_SSOEConfig(SPI2, ENABLE);

	while(1)
	{
		//Read from button
		while(!(GPIO_ReadFromInputPin(GPIOD, GPIO_PIN_NUMBER_5)));
		//avoid debouncing
		delay();
		//enable SPI peripherals
		SPI_PeripheralsControl(SPI2, ENABLE);
		//first send the length of information
		uint8_t datalen = strlen(userData);
		SPI_SendData(SPI2, &datalen, 1);

		//send data
		SPI_SendData(SPI2, (uint8_t*)userData, strlen(userData));

		//lets confirm SPI is not busy
		while(SPI_GetFlagStatus(SPI2, SPI_BSY_FLAG));
		//close the SPI
		SPI_PeripheralsControl(SPI2, DISABLE);
	}
}
