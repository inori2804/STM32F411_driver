/*
 * stm32f411_i2c_driver.c
 *
 *  Created on: 27 Mar 2023
 *      Author: shiba
 */

#include"stm32f411_i2c_driver.h"


uint16_t AHB_Prescaler[8] = {2, 4, 8, 16, 64, 128, 256, 512};
uint16_t APB_Prescaler[4] = {2, 4, 8, 16};

static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx);
static void I2C_ExecuteAddressPhaseWrite(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr);
static void I2C_ExecuteAddressPhaseRead(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr);
static void I2C_ClearADDRFlag(I2C_Handle_t *pI2CHandle);
static void I2C_MasterHandlerRXNEInterrupt(I2C_Handle_t *pI2CHandle);
static void I2C_MasterHandleTXEInterrupt(I2C_Handle_t *pI2CHandle);

/*
 * Peripheral Clock Setup
 */
void I2C_PeripheralClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(pI2Cx == I2C1)
		{
			I2C1_PCLK_EN();
		} else if(pI2Cx == I2C2)
		{
			I2C2_PCLK_EN();
		} else if(pI2Cx == I2C3)
		{
			I2C3_PCLK_EN();
		}
	} else
	{
		if(pI2Cx == I2C1)
		{
			I2C1_PCLK_DI();
		} else if(pI2Cx == I2C2)
		{
			I2C2_PCLK_DI();
		} else if(pI2Cx == I2C3)
		{
			I2C3_PCLK_DI();
		}
	}
}

uint32_t RCC_GetPLLOutputClock(void)
{
	return 0;
}


uint32_t RCC_GetPCLK1Value(void)
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

/*
 * Init and De-Init
 */

void I2C_Init(I2C_Handle_t *pI2CHandle)
{
/*	Step for I2C init
	1. configure the mode (stand or fast)

	2. configure speed of serial clock SCL

	3. Configure the device address (Applicable when device is slave)

	4. Enable Acking

	5. Configure the rise time for I2C pins */

	//Enable clock peripherals
	I2C_PeripheralClockControl(pI2CHandle->pI2Cx, ENABLE);

	uint32_t tempreg = 0;
	//ack control bit
	tempreg |= pI2CHandle->I2C_Config.I2C_AckControl << 10;
	pI2CHandle->pI2Cx->CR1 = tempreg;

	//configure the FREG field of CR2
	tempreg = 0;
	tempreg |= RCC_GetPCLK1Value() / 1000000U;
	pI2CHandle->pI2Cx->CR2 = (tempreg & 0x3F);

	//store the slave address
	tempreg = 0;
	tempreg |= (pI2CHandle->I2C_Config.I2C_DeviceAddress << 1);
	tempreg |= (1 << 14);
	pI2CHandle->pI2Cx->OAR1 = tempreg;

	//CCR calculations
	uint16_t ccr_value = 0;
	tempreg = 0;
	if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
	{
		//mode is standard mode
		ccr_value = (RCC_GetPCLK1Value() / (2 * pI2CHandle->I2C_Config.I2C_SCLSpeed));
		tempreg |= (ccr_value & 0xFFF);
	} else
	{
		//mode is fast mode
		tempreg |= (1 << 15);
		tempreg |= (pI2CHandle->I2C_Config.I2C_FMDutyCycle << 14);
		if(pI2CHandle->I2C_Config.I2C_FMDutyCycle == I2C_FM_DUTY_2)
		{
			ccr_value = (RCC_GetPCLK1Value() / (3 * pI2CHandle->I2C_Config.I2C_SCLSpeed));
		} else
		{
			ccr_value = (RCC_GetPCLK1Value() / (25 * pI2CHandle->I2C_Config.I2C_SCLSpeed));
		}
		tempreg |= (ccr_value & 0xFFF);
	}
	pI2CHandle->pI2Cx->CCR = tempreg;

	//TRISE configuration (do later)
	uint8_t trise;
	if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
	{
		//mode is standard mode
		trise = (RCC_GetPCLK1Value() / 1000000U) + 1;
	} else
	{
		//mode is fast mode
		trise = ((RCC_GetPCLK1Value() * 300) / 1000000000U) + 1;
	}
	pI2CHandle->pI2Cx->TRISE = (trise & 0x3F);
}


void I2C_DeInit(I2C_RegDef_t *pI2Cx)
{
	if(pI2Cx == I2C1)
	{
		I2C1_REG_RESET();
	} else if(pI2Cx == I2C2)
	{
		I2C2_REG_RESET();
	} else if(pI2Cx == I2C3)
	{
		I2C3_REG_RESET();
	}
}

/*
 * Data send and Receive
 */
static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->CR1 |= (1 << I2C_CR1_START);
}

static void I2C_ExecuteAddressPhaseWrite(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr)
{
	SlaveAddr = SlaveAddr << 1;
	SlaveAddr &= ~(0x1); //Slave address plus r/nw bit
	pI2Cx->DR = SlaveAddr;
}

static void I2C_ExecuteAddressPhaseRead(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr)
{
	SlaveAddr = SlaveAddr << 1;
	SlaveAddr |= 0x1; //Slave address plus r/nw bit
	pI2Cx->DR = SlaveAddr;
}

static void I2C_ClearADDRFlag(I2C_Handle_t *pI2CHandle)
{
	uint32_t dummyread;
	//check for device mode
	if(pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_MSL))
	{
		//device is master mode
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
		{
			if(pI2CHandle->RxSize == 1)
			{
				//first disable the ack
				I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_DISABLE);
				//clear addr flag
				dummyread = pI2CHandle->pI2Cx->SR1;
				dummyread = pI2CHandle->pI2Cx->SR2;
				(void)dummyread;
			} else if(pI2CHandle->RxSize > 2)
			{
				//clear addr flag
				dummyread = pI2CHandle->pI2Cx->SR1;
				dummyread = pI2CHandle->pI2Cx->SR2;
				(void)dummyread;
			}
		}
	} else
	{
		//device is slave mode
		//clear addr flag
		dummyread = pI2CHandle->pI2Cx->SR1;
		dummyread = pI2CHandle->pI2Cx->SR2;
		(void)dummyread;
	}
}

void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->CR1 |= (1 << I2C_CR1_STOP);
}

void I2C_MasterSendData(I2C_Handle_t* pI2CHandle, uint8_t *pTxBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr)
{
	//1. Generate Start condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	//2. Confirm that start generation is completely by checking the SB flag in the SR1
	// Note: Until SB is cleared SCL will be stretched (pulled to LOW)
	while(!(I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_SB_FLAG)));

	//3. Send the address of the slave with r/nw bit set to w(0) (total 8bits)
	I2C_ExecuteAddressPhaseWrite(pI2CHandle->pI2Cx, pI2CHandle->I2C_Config.I2C_DeviceAddress);

	//4. Confirm that the address phase is completely by checking the ADDR flag in the SR1
	while(!(I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_ADDR_FLAG)));

	//5. Clear the ADDR flag according to its software sequence
	//Note: Until ADDR is cleared SCL will be stretched -  pulled to LOW
	I2C_ClearADDRFlag(pI2CHandle);

	//6. Send data until Len becomes 0
	while(Len > 0)
	{
		while(!(I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_TXE_FLAG)));
		pI2CHandle->pI2Cx->DR = *pTxBuffer;
		pTxBuffer++;
		Len--;
	}
	//7. When len becomes 0 wait for TxE = 1 and BTF = 1 before generating the STOP condition
	// Note: TxE = 1, BTF = 1, means that both SR and DR are empty and next transmission should begin
	// when BTF = 1, SCL will be stretched
	while(!(I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_TXE_FLAG)));
	while(!(I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_BTF_FLAG)));

	//8. Generate STOP condition and master need not to wait for the completion of stop condition
	//Note: generating STOP, automatically clears the BTF
	if(Sr == I2C_SR_DISABLE)
		I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
}

void I2C_MasterReceiveData(I2C_Handle_t* pI2CHandle, uint8_t *pRxBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr)
{
	//1. Generate Start condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	//2. Confirm that start generation is completely by checking the SB flag in the SR1
	// Note: Until SB is cleared SCL will be stretched (pulled to LOW)
	while(!(I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_SB_FLAG)));

	//3. Send the address of the slave with r/nw bit set to R(1) (total 8bits)
	I2C_ExecuteAddressPhaseRead(pI2CHandle->pI2Cx, pI2CHandle->I2C_Config.I2C_DeviceAddress);

	//4. Confirm that the address phase is completely by checking the ADDR flag in the SR1
	while(!(I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_ADDR_FLAG)));

	//6. Receive data until Len becomes 0
	//procedure for read only one byte
	if(Len == 1)
	{
		//Disable Acking
		I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_DISABLE);

		//clear the ADDRflag
		I2C_ClearADDRFlag(pI2CHandle);

		//wait until rxne becomes 1
		while(!(I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_RXNE_FLAG)));

		if(Sr == I2C_SR_DISABLE)
			//generate Stop condition
			I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

		//Read data into buffer
		*pRxBuffer = pI2CHandle->pI2Cx->DR;
	}
	if(Len > 1)
	{
		//clear ADDR flag
		I2C_ClearADDRFlag(pI2CHandle);

		//read the data until Len becomes zero
		for(uint32_t i = Len; i > 0; i--)
		{
			//wait until rxne becomes 1
			while(!(I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_RXNE_FLAG)));

			if(i == 2)
			{
				//clear the ack bit
				I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_DISABLE);

				if(Sr == I2C_SR_DISABLE)
					//generate stop condition
					I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
			}
			//read data into buffer
			*pRxBuffer = pI2CHandle->pI2Cx->DR;

			//increment the buffer address
			pRxBuffer++;
		}
	}
	//re-enablle Acking
	if(pI2CHandle->I2C_Config.I2C_AckControl == I2C_ACK_ENABLE)
	{
		I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_ENABLE);
	}
}

void I2C_SlaveSendData(I2C_RegDef_t *pI2C, uint8_t data)
{
	pI2C->DR = data;
}
uint8_t I2C_SlaveReceiveDataIT(I2C_RegDef_t *pI2C)
{
	return (uint8_t)pI2C->DR;
}

/*
 * Data send and receive in IT mode
 */

uint8_t I2C_MasterSendDataIT(I2C_Handle_t* pI2CHandle, uint8_t *pTxBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr)
{
	uint8_t busystate = pI2CHandle->TxRxState;
	if(busystate != I2C_BUSY_IN_RX && busystate != I2C_BUSY_IN_TX)
	{
		pI2CHandle->pTxBuffer = pTxBuffer;
		pI2CHandle->TxLen = Len;
		pI2CHandle->TxRxState = I2C_BUSY_IN_TX;
		pI2CHandle->DevAddr = SlaveAddr;
		pI2CHandle->Sr = Sr;

		//Implement the Start condition
		I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

		//Implement the code to enable ITBUFEN control bit
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITBUFEN);

		//Implement the code to enable ITEVTEN control bit
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITEVFEN);

		//Implement the code to enable ITERREN control bit
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITERREN);
	}
	return busystate;
}

uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t* pI2CHandle, uint8_t *pRxBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr)
{
	uint8_t busystate = pI2CHandle->TxRxState;
	if(busystate != I2C_BUSY_IN_RX && busystate != I2C_BUSY_IN_TX)
	{
		pI2CHandle->pTxBuffer = pRxBuffer;
		pI2CHandle->TxLen = Len;
		pI2CHandle->TxRxState = I2C_BUSY_IN_RX;
		pI2CHandle->RxSize = Len; //Rxsize is used in the ISR code to manage the data reception
		pI2CHandle->DevAddr = SlaveAddr;
		pI2CHandle->Sr = Sr;

		//Implement the Start condition
		I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

		//Implement the code to enable ITBUFEN control bit
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITBUFEN);

		//Implement the code to enable ITEVTEN control bit
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITEVFEN);

		//Implement the code to enable ITERREN control bit
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITERREN);
	}
	return busystate;
}


/*
 * IRQ Configuration and ISQ Handling
 */

void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
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
void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{

}


/*
 * Other Peripherals Controls API
 */
void I2C_PeripheralsControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		pI2Cx->CR1 |= (1 << I2C_CR1_PE);
	} else
	{
		pI2Cx->CR1 &= ~(1 << I2C_CR1_PE);
	}
}

uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t FlagName)
{
	if(pI2Cx->SR1 & FlagName)
		return FLAG_SET;
	return FLAG_RESET;
}




void I2C_ManageAcking(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
{
	if(EnorDi == I2C_ACK_ENABLE)
	{
		pI2Cx->CR1 |= (0x1 << I2C_CR1_ACK);
	} else
	{
		pI2Cx->CR1 &= ~(0x1 << I2C_CR1_ACK);
	}
}

static void I2C_MasterHandlerRXNEInterrupt(I2C_Handle_t *pI2CHandle)
{
	if(pI2CHandle->RxSize == 1)
	{
		*pI2CHandle->pRxBuffer = pI2CHandle->pI2Cx->DR;
		pI2CHandle->RxLen--;
	}

	if(pI2CHandle->RxSize > 1)
	{
		if(pI2CHandle->RxSize == 2)
		{
			//clear acking bit
			I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_DISABLE);
		}
		//read data
		*pI2CHandle->pRxBuffer = pI2CHandle->pI2Cx->DR;
		pI2CHandle->pRxBuffer++;
		pI2CHandle->RxLen--;
	}

	if(pI2CHandle->RxLen == 0)
	{
		//close the i2c communication and notify the application

		//1. generate the stop condition
		if(pI2CHandle->Sr == I2C_SR_DISABLE)
		{
			I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
		}
		//2. close the i2c rx
		I2C_CloseReceiveData(pI2CHandle);
		//3. notify the application
		I2C_ApplicationEventCallBack(pI2CHandle, I2C_EV_RX_CMPLT);
	}
}

static void I2C_MasterHandleTXEInterrupt(I2C_Handle_t *pI2CHandle)
{
	//TXE flag is set
	if(pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_MSL))
	{
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX){
			if(pI2CHandle->TxLen > 0)
			{
				//1. Load data into DR
				pI2CHandle->pI2Cx->DR = *(pI2CHandle->pTxBuffer);
				//2. decrement the Tx len
				pI2CHandle->TxLen--;
				//3. increase the buffer address
				pI2CHandle->pTxBuffer++;
			}
		}
	}
}

void I2C_IRQ_EVHandling(I2C_Handle_t* pI2CHandle)
{
	//Interrupt handling for both master and slave mode of a device

	uint32_t temp1, temp2, temp3;
	temp1 = pI2CHandle->pI2Cx->CR2 & (1 << I2C_CR2_ITEVFEN);
	temp2 = pI2CHandle->pI2Cx->CR2 & (1 << I2C_CR2_ITBUFEN);

	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_SB);
	//1. Handle for interrupt generate by SB event
	//Note: SB flag is only applicable in Master mode
	if(temp1 && temp3)
	{
		//the interrupt generated by SB event
		//this block will not be executed in slave mode because for slave SB is always zero
		//In this block lets executed the address phase
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
		{
			I2C_ExecuteAddressPhaseRead(pI2CHandle->pI2Cx, pI2CHandle->DevAddr);
		} else if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
		{
			I2C_ExecuteAddressPhaseWrite(pI2CHandle->pI2Cx, pI2CHandle->DevAddr);
		}
	}
	//2. Handle for interrupt generated by ADDR event
	//Note: When master mode: Address is sent
	//		When Slave mode: address matched with own address

	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_ADDR);
	if(temp1 && temp3)
	{
		//ADDR flag is set
		I2C_ClearADDRFlag(pI2CHandle);
	}

	//3. Handle for interrupt generated by BTF(Byte transfer Finished) event

	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_BTF);
	if(temp1 && temp3)
	{
		//BTF flag is set
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
		{
			//make sure that TXE is also set
			if(pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_TXE))
			{
				//BTF and TXE is also set
				//check transmission completed
				if(pI2CHandle->TxLen == 0){
					//1. generate stop condition
					if(pI2CHandle->Sr == I2C_SR_DISABLE)
					{
						I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
					}
					//2. reset all the member elements of the handler structure
					I2C_CloseSendData(pI2CHandle);

					//3. notify the application about transmission complete
					I2C_ApplicationEventCallBack(pI2CHandle, I2C_EV_TX_CMPLT);
				}
			} else if(pI2CHandle->pI2Cx->SR1& (1 << I2C_SR1_RXNE))
			{
				;
			}
		}
	}

	//4. Handle for interrupt generated by Stop event
	//Note: Stop detection flag is applicable only slave mode. For master this flag will be disable

	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_STOPF);
	if(temp1 && temp3)
	{
		//Stop flag is set
		//Clear the STOP flag (read SR1 and then write to CR1)
		pI2CHandle->pI2Cx->CR1 |= 0x0000;
		//notify the application Stop is detected
		I2C_ApplicationEventCallBack(pI2CHandle, I2C_EV_STOP);
	}

	//5. Handle for interrupt generated by TXE event

	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_TXE);
	if(temp1 && temp2 && temp3)
	{
		//IXE flag is set
		//We have to do the data transmisson
		if(pI2CHandle->pI2Cx->SR2 & (0x1 << I2C_SR2_MSL))
		{
			//device is master
			if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
			{
				I2C_MasterHandleTXEInterrupt(pI2CHandle);
			}
		} else
		{
			//device is slave
			//make sure that slave really in transmitter mode
			if(pI2CHandle->pI2Cx->SR2 & (0x1 << I2C_SR2_TRA))
			{
				I2C_ApplicationEventCallBack(pI2CHandle, I2C_EV_DATA_REQ);
			}
		}
	}

	//6. Handle for interrupt generated by RXNe event
	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_RXNE);
	if(temp1 && temp2 && temp3)
	{
		//check the device mode
		if(pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_MSL))
		{
			//the device is mater

			//RXNE flag is set
			if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX){
				I2C_MasterHandlerRXNEInterrupt(pI2CHandle);
			}
		} else
		{
			//slave
			//make sure thast slave really in receiver mode
			if(!(pI2CHandle->pI2Cx->SR2 & (0x1 << I2C_SR2_TRA)))
			{
				I2C_ApplicationEventCallBack(pI2CHandle, I2C_EV_DATA_RCV);
			}

		}
	}
}

void I2C_IRQ_ERHandling(I2C_Handle_t* pI2CHandle)
{
	uint32_t temp1,temp2;

    //Know the status of  ITERREN control bit in the CR2
	temp2 = (pI2CHandle->pI2Cx->CR2) & ( 1 << I2C_CR2_ITERREN);


/***********************Check for Bus error************************************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1<< I2C_SR1_BERR);
	if(temp1 && temp2)
	{
		//This is Bus error

		//Implement the code to clear the buss error flag
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_BERR);

		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallBack(pI2CHandle, I2C_ERROR_BERR);
	}

/***********************Check for arbitration lost error************************************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_ARLO );
	if(temp1  && temp2)
	{
		//This is arbitration lost error

		//Implement the code to clear the arbitration lost error flag
		pI2CHandle->pI2Cx->SR1 &= ~(0x1 << I2C_SR1_ARLO);
		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallBack(pI2CHandle, I2C_ERROR_ARLO);
	}

/***********************Check for ACK failure  error************************************/

	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_AF);
	if(temp1  && temp2)
	{
		//This is ACK failure error

	    //Implement the code to clear the ACK failure error flag
		pI2CHandle->pI2Cx->SR1 &= ~(0x1 << I2C_SR1_AF);
		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallBack(pI2CHandle, I2C_ERROR_AF);
	}

/***********************Check for Overrun/underrun error************************************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_OVR);
	if(temp1  && temp2)
	{
		//This is Overrun/underrun

	    //Implement the code to clear the Overrun/underrun error flag
		pI2CHandle->pI2Cx->SR1 &= ~(0x1 << I2C_SR1_OVR);
		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallBack(pI2CHandle, I2C_ERROR_OVR);
	}

/***********************Check for Time out error************************************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_TIMEOUT);
	if(temp1  && temp2)
	{
		//This is Time out error

	    //Implement the code to clear the Time out error flag
		pI2CHandle->pI2Cx->SR1	&= ~(0x1 << I2C_SR1_TIMEOUT);
		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallBack(pI2CHandle, I2C_ERROR_TIMEOUT);
	}

}

void I2C_CloseReceiveData(I2C_Handle_t *pI2CHandle)
{
	//Disable ITBuf control bit
	pI2CHandle->pI2Cx->CR2 &= ~(0x1 << I2C_CR2_ITBUFEN);

	//Disable ItEvfen control bit
	pI2CHandle->pI2Cx->CR2 &= ~(0x1 << I2C_CR2_ITEVFEN);

	pI2CHandle->TxRxState = I2C_READY;
	pI2CHandle->pRxBuffer = NULL;
	pI2CHandle->RxLen = 0;
	pI2CHandle->RxSize = 0;
	if(pI2CHandle->I2C_Config.I2C_AckControl == I2C_ACK_ENABLE)
		I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_ENABLE);
}

void I2C_CloseSendData(I2C_Handle_t *pI2CHandle)
{
	//Disable ITbufen control bit
	pI2CHandle->pI2Cx->CR2 &= ~(0x1 << I2C_CR2_ITBUFEN);

	//Disable ITevfen control bit
	pI2CHandle->pI2Cx->CR2 &= ~(0x1 << I2C_CR2_ITEVFEN);

	pI2CHandle->TxRxState = I2C_READY;
	pI2CHandle->pTxBuffer = NULL;
	pI2CHandle->TxLen = 0;
}

void I2C_EnableDisableCallBackEvents(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		pI2Cx->CR2 |= (0x1 << I2C_CR2_ITBUFEN);
		pI2Cx->CR2 |= (0x1 << I2C_CR2_ITERREN);
		pI2Cx->CR2 |= (0x1 << I2C_CR2_ITEVFEN);
	} else
	{
		pI2Cx->CR2 &= ~(0x1 << I2C_CR2_ITBUFEN);
		pI2Cx->CR2 &= ~(0x1 << I2C_CR2_ITERREN);
		pI2Cx->CR2 &= ~(0x1 << I2C_CR2_ITEVFEN);
	}
}
