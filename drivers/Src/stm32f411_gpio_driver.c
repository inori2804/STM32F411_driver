/*
 * stm32f411_gpio_driver.c
 *
 *  Created on: 12 Mar 2023
 *      Author: shiba
 */

#include "stm32f411_gpio_driver.h"

/*
 * Peripheral Clock Setup
 */

/**********************************************************************
 * @fn				- GPIO_PeripheralClockControl
 *
 * @brief			- This function enables or disables peripherals clock for the given GPIO port
 *
 * @param[0]		- Base address of GPIO peripheral
 * @param[1]		- ENABLE or DISABLE macros
 *
 * @return			- none
 *
 * @Note			- none
 */
void GPIO_PeripheralClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_EN();
		}
		else if(pGPIOx == GPIOB)
		{
			GPIOB_PCLK_EN();
		}
		else if(pGPIOx == GPIOC)
		{
			GPIOC_PCLK_EN();
		}
		else if(pGPIOx == GPIOD)
		{
			GPIOD_PCLK_EN();
		}
		else if(pGPIOx == GPIOE)
		{
			GPIOE_PCLK_EN();
		}
		else if(pGPIOx == GPIOH)
		{
			GPIOH_PCLK_EN();
		}
	}else
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_DI();
		}
		else if(pGPIOx == GPIOB)
		{
			GPIOB_PCLK_DI();
		}
		else if(pGPIOx == GPIOC)
		{
			GPIOC_PCLK_DI();
		}
		else if(pGPIOx == GPIOD)
		{
			GPIOD_PCLK_DI();
		}
		else if(pGPIOx == GPIOE)
		{
			GPIOE_PCLK_DI();
		}
		else if(pGPIOx == GPIOH)
		{
			GPIOH_PCLK_DI();
		}
	}
}

/*
 * Init and De-Init
 */

/**********************************************************************
 * @fn				- GPIO_Init
 *
 * @brief			- This function initialize the GPIO Pin function
 *
 * @param[0]		- GPIOHanlde structure variable contain setting for GPIO pin
 *
 * @return			- none
 *
 * @Note			- none
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	uint32_t temp = 0; //temp register

	//Enable GPIO clock peripherals
	GPIO_PeripheralClockControl(pGPIOHandle->pGPIOx, ENABLE);

	//1. Configure mode
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
	{
		//none interrupt mode
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->MODER &= ~(0x3 << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); //clearing
		pGPIOHandle->pGPIOx->MODER |= temp; //setting
	}else
	{
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT)
		{
			//1. Configure the Falling Trigger Select Register
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			//Clear the corresponding RTSR bit
			EXTI->RTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

		}else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT)
		{
			//1. Configure the Rising Trigger Select Register
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			//Clear the corresponding FTSR bit
			EXTI->FTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

		}else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT)
		{
			//1. Configure the Rising Falling Trigger Select Register
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		//2. Configure the GPIO port selection in SYSCFG_EXTICR
		uint8_t temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4;
		uint8_t temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4;
		uint8_t portcode = GPIO_BASE_ADDR_TO_PORTCODE(pGPIOHandle->pGPIOx);
		SYSCFG_PCLK_EN();
		SYSCFG->EXTICR[temp1] &= ~(0xF << (4 * temp2)); //clearing
		SYSCFG->EXTICR[temp1] |= (portcode << (4 * temp2)); //setting
		//3. Enable the EXTI interrupt delivery using IMR
		EXTI->IMR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	}

	temp = 0;
	//2. Configure speed
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); //clearing
	pGPIOHandle->pGPIOx->OSPEEDR |= temp; //setting

	temp = 0;
	//3. Configure pull up pull down settings
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); //clearing
	pGPIOHandle->pGPIOx->PUPDR |= temp; //setting

	temp = 0;
	//4. Configure the Output Type
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOType << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OTYPER &= ~(0x1 << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); //clearing
	pGPIOHandle->pGPIOx->OTYPER |= temp; //setting

	temp = 0;
	//5. Configure the ALT functionality
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFUNC)
	{
		uint8_t temp1, temp2 = 0;

		temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;
		temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;
		pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0xF << (4 * temp2)); //clearing
		pGPIOHandle->pGPIOx->AFR[temp1] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4 * temp2)); //setting
	}

}

/**********************************************************************
 * @fn				- GPIO_Init
 *
 * @brief			- This function initialize the GPIO Pin function
 *
 * @param[0]		- GPIOHanlde structure variable contain setting for GPIO pin
 *
 * @return			- none
 *
 * @Note			- none
 */
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)	/* Use AHB1 Reset register so we only need base address of GPIOx */
{
	if(pGPIOx == GPIOA)
	{
		GPIOA_REG_RESET();
	}
	else if(pGPIOx == GPIOB)
	{
		GPIOB_REG_RESET();
	}
	else if(pGPIOx == GPIOC)
	{
		GPIOC_REG_RESET();
	}
	else if(pGPIOx == GPIOD)
	{
		GPIOD_REG_RESET();
	}
	else if(pGPIOx == GPIOE)
	{
		GPIOE_REG_RESET();
	}
	else if(pGPIOx == GPIOH)
	{
		GPIOH_REG_RESET();
	}
}
/*
 * Read from Input Pin or Port
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	uint8_t value;
	value = (uint16_t)((pGPIOx->IDR >> PinNumber) & 0x00000001);
	return value;
}
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
	uint16_t value;
	value = (uint16_t)pGPIOx->IDR;
	return value;
}

/*
 * Write to Output Pin or Port
 */
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value)
{
	if(Value == GPIO_PIN_SET)
	{
		pGPIOx->ODR |= (1 << PinNumber); //write 1 to output pin
	}else if(Value == GPIO_PIN_RESET)
	{
		pGPIOx->ODR &= ~(1 << PinNumber); //write 0 to output pin
	}
}
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value)
{
	pGPIOx->ODR = Value;
}

/*
 * Toggle value of Output Pin
 */
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	pGPIOx->ODR ^= (1 << PinNumber);
}

/*
 * Interrupt Handling Begin
 */
/**********************************************************************
 * @fn				- GPIO_IRQInterruptConfig
 *
 * @brief			-
 *
 * @param[0]		-
 *
 * @return			- none
 *
 * @Note			- none
 */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(IRQNumber <= 31)
		{
			//program on ISER0 register
			*NVIC_ISER0 |= (1 << IRQNumber);

		}else if(IRQNumber <= 63)
		{
			//program on ISER1 register
			*NVIC_ISER1 |= (1 << (IRQNumber % 32));
		}else if(IRQNumber <= 95)
		{
			//program on ISER2 register
			*NVIC_ISER2 |= (1 << (IRQNumber % 64));
		}
	}else
	{
		if(IRQNumber <= 31)
		{
			//program on ISER0 register
			*NVIC_ICER0 |= (1 << IRQNumber);

		}else if(IRQNumber <= 63)
		{
			//program on ISER1 register
			*NVIC_ICER1 |= (1 << (IRQNumber % 32));
		}else if(IRQNumber <= 95)
		{
			//program on ISER2 register
			*NVIC_ICER2 |= (1 << (IRQNumber % 64));
		}
	}
}
/**********************************************************************
 * @fn				- GPIO_IRQPriotityConfig
 *
 * @brief			-
 *
 * @param[0]		-
 *
 * @return			- none
 *
 * @Note			- none
 */
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority)
{
	//1.Find out the ipr register
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;

	uint8_t shift_amout = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);
	*(NVIC_PR_BASE_ADDR + iprx) |= (IRQPriority << shift_amout);
}

/**********************************************************************
 * @fn				- GPIO_IRQHandling
 *
 * @brief			-
 *
 * @param[0]		-
 *
 * @return			- none
 *
 * @Note			- none
 */

void GPIO_IRQHandling(uint8_t PinNumber)
{
	//clear the exti pr register corresponding to the pin number
	if(EXTI->PR & (1 << PinNumber))
	{
		//clear
		EXTI->PR |= (1 << PinNumber);
	}
}
