/*
 * stm32f411_rcc_driver.h
 *
 *  Created on: 4 Apr 2023
 *      Author: shiba
 */

#ifndef INC_STM32F411_RCC_DRIVER_H_
#define INC_STM32F411_RCC_DRIVER_H_

#include"stm32f411xx.h"

//this return the APB1 clock value
uint32_t RCC_GetPCLK1Value(void);

//this return the APB2 clock value
uint32_t RCC_GetPCLK2Value(void);

//this return the PLL clock value
uint32_t RCC_GetPLLOutputClock(void);

#endif /* INC_STM32F411_RCC_DRIVER_H_ */
