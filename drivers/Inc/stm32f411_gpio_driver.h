/*
 * stm32f411_gpio_driver.h
 *
 *  Created on: 12 Mar 2023
 *      Author: shiba
 */

#ifndef INC_STM32F411_GPIO_DRIVER_H_
#define INC_STM32F411_GPIO_DRIVER_H_

#include "stm32f411xx.h"


/*
 * This is GPIO configuration Pin structure
 */

typedef struct
{
	uint8_t GPIO_PinNumber; 		/* Configure Pin number, possible value from @GPIO_PIN_NUMBERS*/
	uint8_t GPIO_PinMode;			/* Configure Pin Mode, possible value from @GPIO_PIN_MODES */
	uint8_t GPIO_PinSpeed;			/* Configure Speed, possible value from @GPIO_SPEED_MODES*/
	uint8_t GPIO_PinPuPdControl;	/* Configure Pull Up Pull Down register, possible value from @GPIO_PUPD_CFG*/
	uint8_t GPIO_PinOType;			/* Configure Output Type, possible value from @GPIO_OUTPUT_TYPE*/
	uint8_t GPIO_PinAltFunMode;		/* Configure Alternative Function mode, possible value from @GPIO_ALF_FUNC*/
}GPIO_PinConfig_t;

/*
 * This is handle structure for GPIO pin
 */

typedef struct
{
	GPIO_RegDef_t *pGPIOx; /* This will hold the base address of GPIO port to which the pin belong */
	GPIO_PinConfig_t GPIO_PinConfig; /* This will hold GPIO configuration pin setting */
}GPIO_Handle_t;

/*
 * ************Some Macros for GPIO drivers****************
 */

/*
 * @GPIO_PIN_NUMBERS
 * GPIO pin numbers
 */
#define GPIO_PIN_NUMBER_0	0
#define GPIO_PIN_NUMBER_1	1
#define GPIO_PIN_NUMBER_2	2
#define GPIO_PIN_NUMBER_3	3
#define GPIO_PIN_NUMBER_4	4
#define GPIO_PIN_NUMBER_5	5
#define GPIO_PIN_NUMBER_6	6
#define GPIO_PIN_NUMBER_7	7
#define GPIO_PIN_NUMBER_8	8
#define GPIO_PIN_NUMBER_9	9
#define GPIO_PIN_NUMBER_10	10
#define GPIO_PIN_NUMBER_11	11
#define GPIO_PIN_NUMBER_12	12
#define GPIO_PIN_NUMBER_13	13
#define GPIO_PIN_NUMBER_14	14
#define GPIO_PIN_NUMBER_15	15


/*
 * @GPIO_PIN_MODES
 * GPIO pin possible mode
 */
#define GPIO_MODE_IN		0		/* GPIO Input mode */
#define GPIO_MODE_OUT		1		/* GPIO Output mode */
#define GPIO_MODE_ALTFUNC	2		/* GPIO Alternative Function mode */
#define GPIO_MODE_ANALOG	3		/* GPIO Analog mode */
#define GPIO_MODE_IT_FT		4		/* GPIO Interrupt Falling Trigger */
#define GPIO_MODE_IT_RT		5		/* GPIO Interrupt Rising Trigger */
#define GPIO_MODE_IT_RFT	6		/* GPIO Interrupt Falling and Rising Trigger */

/*
 * @GPIO_OUTPUT_TYPE
 * GPIO pin possible output types
 */
#define GPIO_OP_TYPE_PP		0		/* Push pull mode */
#define GPIO_OP_TYPE_OD		1		/* Open drain mode */

/*
 * @GPIO_SPEED_MODES
 * GPIO pin possible output speeds
 */
#define GPIO_SPEED_LOW		0
#define GPIO_SPEED_MEDIUM	1
#define GPIO_SPEED_FAST		2
#define GPIO_SPEED_HIGH		3

/*
 * @GPIO_PUPD_CFG
 * GPIO pin Pull up Pull Down configuration
 */
#define GPIO_NO_PUPD		0
#define GPIO_PIN_PU			1
#define GPIO_PIN_PD			2

/*
 ***************** API Support by this Drivers *************************************
 ***************** For more information please check function definitions **********
 */

/*
 * Peripheral Clock Setup
 */
void GPIO_PeripheralClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi);

/*
 * Init and De-Init
 */

void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);	/* Use AHB1 Reset register so we only need base address of GPIOx */

/*
 * Read from Input Pin or Port
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);

/*
 * Write to Output Pin or Port
 */
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value);

/*
 * Toggle value of Output Pin
 */
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

/*
 * Interrupt Handling
 */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);
void GPIO_IRQHandling(uint8_t PinNumber);



#endif /* INC_STM32F411_GPIO_DRIVER_H_ */
