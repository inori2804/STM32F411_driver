/*
 * stm32f411_rcc_driver.c
 *
 *  Created on: 4 Apr 2023
 *      Author: shiba
 */

#include"stm32f411_rcc_driver.h"

uint16_t AHB_Prescaler[8] = {2, 4, 8, 16, 64, 128, 256, 512};
uint16_t APB_Prescaler[4] = {2, 4, 8, 16};

uint32_t RCC_GetPCLK1Value(void)
{
	uint32_t pclk1, Systemclk;
	uint8_t clksrc, temp, ahbp, apb2p;

	clksrc = ((RCC->CFGR >> 2) & 0x3);
	if(clksrc == 0)
	{
		Systemclk = 16000000;
	} else if(clksrc == 1)
	{
		Systemclk = 8000000;
	} else if(clksrc == 2)
	{
		Systemclk = RCC_GetPLLOutputClock();
	}
	//for AHP1
	temp = ((RCC->CFGR >> 4) & 0xF);
	if(temp < 8)
	{
		ahbp = 1;
	} else
		ahbp = AHB_Prescaler[temp - 8];
	//for APB2
	temp = ((RCC->CFGR >> 13) & 0x7);
	if(temp < 4)
	{
		apb2p = 1;
	} else
		apb2p = APB_Prescaler[temp - 4];

	pclk1 = Systemclk / (ahbp * apb2p);
	return pclk1;
}


uint32_t RCC_GetPCLK2Value(void)
{
	uint32_t pclk1, Systemclk;
	uint8_t clksrc, temp, ahbp, apb1p;

	clksrc = ((RCC->CFGR >> 2) & 0x3);
	if(clksrc == 0)
	{
		Systemclk = 16000000;
	} else if(clksrc == 1)
	{
		Systemclk = 8000000;
	} else if(clksrc == 2)
	{
		Systemclk = RCC_GetPLLOutputClock();
	}
	//for AHP1
	temp = ((RCC->CFGR >> 4) & 0xF);
	if(temp < 8)
	{
		ahbp = 1;
	} else
		ahbp = AHB_Prescaler[temp - 8];
	//for APB1
	temp = ((RCC->CFGR >> 10) & 0x7);
	if(temp < 4)
	{
		apb1p = 1;
	} else
		apb1p = APB_Prescaler[temp - 4];

	pclk1 = Systemclk / (ahbp * apb1p);
	return pclk1;
}

uint32_t RCC_GetPLLOutputClock(void)
{
	return 0;
}

