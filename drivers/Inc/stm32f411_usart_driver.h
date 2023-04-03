/*
 * stm32f411_usart_driver.h
 *
 *  Created on: 2 Apr 2023
 *      Author: shiba
 */

#ifndef INC_STM32F411_USART_DRIVER_H_
#define INC_STM32F411_USART_DRIVER_H_

#include"stm32f411xx.h"

/*
 * Configuration structure for USART peripheral
 */

typedef struct
{
	uint8_t USART_Mode;
	uint32_t USART_Baud;
	uint8_t USART_NoOfStopBit;
	uint8_t USART_WordLength;
	uint8_t USART_ParityControl;
	uint8_t USART_HWFlowControl;
}USART_Config_t;

/*
 * Handle structure for USARTx peripheral
 */

typedef struct
{
	USART_RegDef_t *pUSARTx;
	USART_Config_t USART_Config;
}UASRT_Handle_t;




#endif /* INC_STM32F411_USART_DRIVER_H_ */
