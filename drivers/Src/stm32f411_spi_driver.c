/*
 * stm32f411_spi_driver.c
 *
 *  Created on: 15 Mar 2023
 *      Author: shiba
 */

#include"stm32f411_spi_driver.h"


static void SPI_TXIE_InterruptHandler(SPI_Handle_t *pSPIHandle);
static void SPI_RXNEIE_InterruptHandler(SPI_Handle_t *pSPIHandle);
static void SPI_ERRIE_InterruptHandler(SPI_Handle_t *pSPIHandle);

/*
 * Peripheral Clock Setup
 */
void SPI_PeripheralClockControl(SPI_Regdef_t *pSPIx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(pSPIx == SPI1)
		{
			SPI1_PCLK_EN();
		}
		else if(pSPIx == SPI2)
		{
			SPI2_PCLK_EN();
		}
		else if(pSPIx == SPI3)
		{
			SPI3_PCLK_EN();
		}
	}else
	{
		if(pSPIx == SPI1)
		{
			SPI1_PCLK_DI();
		}
		else if(pSPIx == SPI2)
		{
			SPI2_PCLK_DI();
		}
		else if(pSPIx == SPI3)
		{
			SPI3_PCLK_DI();
		}
	}
}

/*
 * Init and De-Init
 */

void SPI_Init(SPI_Handle_t *pSPIHandle)
{
	//First Configure the SPI_CR! register
	uint32_t tempreg = 0;

	//SPIx clock peripherals enable
	SPI_PeripheralClockControl(pSPIHandle->pSPIx, ENABLE);

	//1. Configure the device mode
	tempreg |= (pSPIHandle->SPIConfig.SPI_DeviceMode << SPI_CR1_MSTR);
	//2. Configure the Bus configure
	if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD)
	{
		//bidi mode must be cleared
		tempreg &= ~(1 << SPI_CR1_BIDIMODE);
	}else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD)
	{
		//bidi mode must be set
		tempreg |= (1 << SPI_CR1_BIDIMODE);
	}else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY)
	{
		//bidi mode must be set
		tempreg |= (1 << SPI_CR1_BIDIMODE);
		//RXONLY bit must be set
		tempreg |= (1 << SPI_CR1_RXONLY);
	}
	//3. Configure the serial clock speed
	tempreg |= (pSPIHandle->SPIConfig.SPI_SclkSpeed << SPI_CR1_BR);
	//4. Configure the DFF
	tempreg |= (pSPIHandle->SPIConfig.SPI_DFF << SPI_CR1_DFF);
	//5. Configure the CPOL
	tempreg |= (pSPIHandle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL);
	//6. Configure the CPHA
	tempreg |= (pSPIHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA);
	//7. Configure the SSM
	tempreg |= (pSPIHandle->SPIConfig.SPI_SSM << SPI_CR1_SSM);

	pSPIHandle->pSPIx->CR1 = tempreg;
}


void SPI_DeInit(SPI_Regdef_t *pSPIx)
{
	if(pSPIx == SPI1)
	{
		SPI1_REG_RESET();
	}else if(pSPIx == SPI2)
	{
		SPI2_REG_RESET();
	}else if(pSPIx == SPI3)
	{
		SPI3_REG_RESET();
	}
}

uint8_t SPI_GetFlagStatus(SPI_Regdef_t *pSPIx, uint32_t FlagName)
{
	if(pSPIx->SR & FlagName)
		return FLAG_SET;
	return FLAG_RESET;
}

/*
 * Data send and Receive
 */

/**********************************************************************
 * @fn				- SPI_SendData
 *
 * @brief			-
 *
 * @param[0]		-
 *
 * @return			- none
 *
 * @Note			- This is blocking call
 */

void SPI_SendData(SPI_Regdef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len)
{
	while(Len > 0)
	{
		//1. Wait until TXE is set
		while(SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG) == FLAG_RESET);

		//2. Check the DFF bit
		if(pSPIx->CR1 & (1 << SPI_CR1_DFF))
		{
			//16bits DFF
			//1. Load the data into DR
			pSPIx->DR = *((uint16_t*)pTxBuffer); //if we use uint8_t it will send one byte into DR
			Len--;
			Len--;
			(uint16_t*)pTxBuffer++;
		}else
		{
			//8bits DFF
			pSPIx->DR = *(pTxBuffer); //if we use uint8_t it will send one byte into DR
			Len--;
			pTxBuffer++;
		}
	}
}


void SPI_ReceiveData(SPI_Regdef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len)
{
	while(Len > 0)
	{
		//1. Wait until TXE is set
		while(SPI_GetFlagStatus(pSPIx, SPI_RXNE_FLAG) == FLAG_RESET);

		//2. Check the DFF bit
		if(pSPIx->CR1 & (1 << SPI_CR1_DFF))
		{
			//16bits DFF
			//1. Load data from DR to Rx buffer
			*(uint16_t*)pRxBuffer = pSPIx->DR;
			Len--;
			Len--;
			(uint16_t*)pRxBuffer++;
		}else
		{
			//8bits DFF
			//Load data from DR to Rx buffer
			*pRxBuffer = pSPIx->DR;
			Len--;
			pRxBuffer++;
		}
	}
}

uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Len)
{
	uint8_t state = pSPIHandle->TxState;
	if(state != SPI_BUSY_IN_TX)
	{
		//1. First save the address of TxBuffer and Len to global variable
		pSPIHandle->pTxBuffer = pTxBuffer;
		pSPIHandle->TxLen = Len;
		//2. Mark the SPI state as busy in transmission so that
		//   no other code can take over same SPI peripheral until transmission is over
		pSPIHandle->TxState = SPI_BUSY_IN_TX;
		state = SPI_BUSY_IN_TX;

		//3. Enable the TXEIE control bit to get interrupt when ever TXE flag is set in SR
		pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_TXEIE);

		//4. Data transmission will be done by IRQ (we will do later)
	}
	return state;
}
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len)
{
	uint8_t state = pSPIHandle->RxState;
	if(state != SPI_BUSY_IN_RX)
	{
		//1. First save the address of TxBuffer and Len to global variable
		pSPIHandle->pRxBuffer = pRxBuffer;
		pSPIHandle->RxLen = Len;
		//2. Mark the SPI state as busy in transmission so that
		//   no other code can take over same SPI peripheral until transmission is over
		pSPIHandle->RxState = SPI_BUSY_IN_RX;
		state = SPI_BUSY_IN_RX;

		//3. Enable the TXEIE control bit to get interrupt when ever TXE flag is set in SR
		//mean that when Tx enable flag is set, an interrupt send to NVIC to inform that data should be read
		pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_RXNEIE);

		//4. Data receive will be done by IRQ (we will do later)
	}
	return state;
}

/*
 * IRQ Configuration and ISQ Handling
 */

void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
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
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{

}

//Helper function for IRQ handling
//for TX empty interupt event
void SPI_TXIE_InterruptHandler(SPI_Handle_t *pSPIHandle)
{
	// Check the DFF bit
	if(pSPIHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF))
	{
		//16bits DFF
		//1. Load the data into DR
		//TODO
		pSPIHandle->pSPIx->DR = *((uint16_t*)pSPIHandle->pTxBuffer); //if we use uint8_t it will send one byte into DR
		pSPIHandle->TxLen--;
		pSPIHandle->TxLen--;
		(uint16_t*)pSPIHandle->pTxBuffer++;
	}else
	{
		//8bits DFF
		pSPIHandle->pSPIx->DR = *pSPIHandle->pTxBuffer; //if we use uint8_t it will send one byte into DR
		pSPIHandle->TxLen--;
		pSPIHandle->pTxBuffer++;
	}
	if(!pSPIHandle->TxLen)
	{
		//Tx len is 0, close the communication and inform application that Tx is over
		// this prevents interrupt from setting up to TXE flag
		SPI_CloseTransmission(pSPIHandle);
		//inform application communication is done
		SPI_ApplicationEventCallBack(pSPIHandle, SPI_EVENT_TX_CMPLT);
	}
}

//for RX not empty interrupt event
void SPI_RXNEIE_InterruptHandler(SPI_Handle_t *pSPIHandle)
{
	// Check the DFF bit
	if(pSPIHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF))
	{
		//16bits DFF
		//1. Load data from DR to Rx buffer
		//TODO
		*((uint16_t*)pSPIHandle->pRxBuffer) = (uint16_t)pSPIHandle->pSPIx->DR;
		pSPIHandle->RxLen--;
		pSPIHandle->RxLen--;
		(uint16_t*)pSPIHandle->pRxBuffer++;
	}else
	{
		//8bits DFF
		//Load data from DR to Rx buffer
		*pSPIHandle->pRxBuffer = pSPIHandle->pSPIx->DR;
		pSPIHandle->RxLen--;
		pSPIHandle->pRxBuffer++;
	}
	if(!pSPIHandle->RxLen)
	{
		//Rx len is 0, close the communication and inform application that Tx is over
		// this prevents interrupt from setting up to TXE flag
		SPI_CloseReception(pSPIHandle);
		//inform application communication is done
		SPI_ApplicationEventCallBack(pSPIHandle, SPI_EVENT_RX_CMPLT);
	}
}

//for Error interrupt event
void SPI_ERRIE_InterruptHandler(SPI_Handle_t *pSPIHandle)
{
	uint8_t temp;
	//clear ovr flag
	if(pSPIHandle->TxState != SPI_BUSY_IN_TX)
	{
		temp = pSPIHandle->pSPIx->DR;
		temp = pSPIHandle->pSPIx->SR;
	}
	(void)temp;
	//inform the application
	SPI_ApplicationEventCallBack(pSPIHandle, SPI_EVENT_OVR_ERR);
}


void SPI_IRQHandling(SPI_Handle_t *pHandle)
{
	uint8_t temp1, temp2;
	//first lets check for TXE
	temp1 = pHandle->pSPIx->SR & (1 << SPI_SR_TXE);
	temp2 = pHandle->pSPIx->CR2 & (1 << SPI_CR2_TXEIE);
	if(temp1 && temp2)
	{
		//handle TXE
		SPI_TXIE_InterruptHandler(pHandle);
	}
	//second check for RXNE
	temp1 = pHandle->pSPIx->SR & (1 << SPI_SR_RXNE);
	temp2 = pHandle->pSPIx->CR2 & (1 << SPI_CR2_RXNEIE);
	if(temp1 && temp2)
	{
		//handle TXE
		SPI_RXNEIE_InterruptHandler(pHandle);
	}
	//check for OVR flag
	temp1 = pHandle->pSPIx->SR & (1 << SPI_SR_OVR);
	temp2 = pHandle->pSPIx->CR2 & (1 << SPI_CR2_ERRIE);
	if(temp1 && temp2)
	{
		//handle TXE
		SPI_ERRIE_InterruptHandler(pHandle);
	}
}

/*
 * SPI Close communication
 */
void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_TXEIE);
	pSPIHandle->pTxBuffer = NULL;
	pSPIHandle->TxLen = 0;
	pSPIHandle->TxState = SPI_READY;
}

void SPI_CloseReception(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_RXNEIE);
	pSPIHandle->pRxBuffer = NULL;
	pSPIHandle->RxLen = 0;
	pSPIHandle->RxState = SPI_READY;
}

void SPI_ClearOVRFlag(SPI_Regdef_t *pSPIx)
{
	uint8_t temp;
	temp = pSPIx->DR;
	temp = pSPIx->SR;
	(void)temp;
}

/*
 * Other Peripherals Controls API
 */

void SPI_PeripheralsControl(SPI_Regdef_t *pSPIx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		pSPIx->CR1 |= (1 << SPI_CR1_SPE);
	}else
	{
		pSPIx->CR1 &= ~(1 << SPI_CR1_SPE);
	}
}

void SPI_SSIConfig(SPI_Regdef_t *pSPIx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		pSPIx->CR1 |= (1 << SPI_CR1_SSI);
	}else
	{
		pSPIx->CR1 &= ~(1 << SPI_CR1_SSI);
	}
}

void SPI_SSOEConfig(SPI_Regdef_t *pSPIx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		pSPIx->CR2 |= (1 << SPI_CR2_SSOE);
	}else
	{
		pSPIx->CR2 &= ~(1 << SPI_CR2_SSOE);
	}
}

/*
 * Application function call back
 */

__attribute__((weak)) void SPI_ApplicationEventCallBack(SPI_Handle_t *pSPIHandle, uint8_t AppEv)
{
	//This is a weak implementation. the application may override this function
}
