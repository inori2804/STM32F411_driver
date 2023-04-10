/*
 * stm32f411xx.h
 *
 *  Created on: Mar 11, 2023
 *      Author: shiba
 *  @Note IMPORTANT: Just implement GPIO port A, B, C, D, E, H
 */

#ifndef INC_STM32F411XX_H_
#define INC_STM32F411XX_H_

#include "stdint.h"
#include "stddef.h"

#define __vo				volatile		/* Macro volatile keyword */
#define __weak				__attribute__((weak))

/****************START PROCESSOR SPECIFIC DETAIL**************************/
/*
 * ARM Cortex Mx Processor NVIC	ISERx register Addresses
 */
//Interrupt set-enable register
#define NVIC_ISER0			((__vo uint32_t *)0xE000E100)
#define NVIC_ISER1			((__vo uint32_t *)0XE000E104)
#define NVIC_ISER2			((__vo uint32_t *)0XE000E108)
#define NVIC_ISER3			((__vo uint32_t *)0xE000E10C)

//Interrupt clear-enable register
#define NVIC_ICER0			((__vo uint32_t *)0XE000E180)
#define NVIC_ICER1			((__vo uint32_t *)0XE000E184)
#define NVIC_ICER2			((__vo uint32_t *)0XE000E188)
#define NVIC_ICER3			((__vo uint32_t *)0XE000E18C)

//Interrupt Priority register
#define NVIC_PR_BASE_ADDR	((__vo uint32_t *)0xE000E400)


#define NO_PR_BITS_IMPLEMENTED		4
/*
 * Base addresses of Flash and SRAM memory
 */

#define FLASH_BASEADDR		0x08000000U		/* Base address of Flash memory */
#define SRAM1_BASEADDR		0X20000000U		/* Base address of SRAM1 */
#define SRAM				SRAM1_BASEADDR  /* SRAM definition */


/*
 * AHBx and APBx Bus Peripheral base addresses
 */

#define PERIPH_BASE			0x40000000U		/* Base address of Peripheral */
#define APB1PERIPH_BASE		PERIPH_BASE		/* Base address of APB1 Bus */
#define APB2PERIPH_BASE		0x40010000U		/* Base address of APB2 Bus */
#define AHB1PERIPH_BASE		0x40020000U		/* Base address of AHB1 Bus */
#define AHB2PERIPH_BASE		0x50000000U		/* Base address of AHB2 Bus */

/*
 * Base address of RCC Register
 */
#define RCC_BASEADDR		(AHB1PERIPH_BASE + 0x3800U) /* Base address of RCC register */

/*
 * Base address of peripheral which are hanging on AHB1 Bus
 */

#define GPIOA_BASEADDR		(AHB1PERIPH_BASE + 0x0000U)	/* Base address of GPIOA */
#define GPIOB_BASEADDR		(AHB1PERIPH_BASE + 0x0400U)	/* Base address of GPIOB */
#define GPIOC_BASEADDR		(AHB1PERIPH_BASE + 0x0800U)	/* Base address of GPIOC */
#define GPIOD_BASEADDR		(AHB1PERIPH_BASE + 0x0C00U)	/* Base address of GPIOD */
#define GPIOE_BASEADDR		(AHB1PERIPH_BASE + 0x1000U)	/* Base address of GPIOE */
#define GPIOH_BASEADDR		(AHB1PERIPH_BASE + 0x1C00U)	/* Base address of GPIOH */

/*
 * Base address of peripheral which are hanging on APB1 Bus
 */

#define I2C1_BASEADDR		(APB1PERIPH_BASE + 0x5400U) /* Base address of I2C1 */
#define I2C2_BASEADDR		(APB1PERIPH_BASE + 0x5800U) /* Base address of I2C2 */
#define I2C3_BASEADDR		(APB1PERIPH_BASE + 0x5C00U) /* Base address of I2C3 */

#define SPI2_BASEADDR		(APB1PERIPH_BASE + 0x3800U)	/* Base address of SPI2 */
#define SPI3_BASEADDR		(APB1PERIPH_BASE + 0x3C00U) /* Base address of SPI3 */

#define USART2_BASEADDR		(APB1PERIPH_BASE + 0x4400U) /* Base address of USART2 */

/*
 * Base address of peripheral which are hanging on APB2 Bus
 */

#define EXTI_BASEADDR		(APB2PERIPH_BASE + 0x3C00U) /* Base address of External Interrupt */
#define SYSCFG_BASEADDR		(APB2PERIPH_BASE + 0x3800U)	/* Base address of System Configuration */

#define SPI1_BASEADDR		(APB2PERIPH_BASE + 0x3000U) /* Base address of SPI1 */

#define USART1_BASEADDR		(APB2PERIPH_BASE + 0x1000U) /* Base address of USART1 */
#define USART6_BASEADDR		(APB2PERIPH_BASE + 0x1400U) /* Base address of USART6 */

/**************** Peripheral Register Definition Structure ***************/

typedef struct
{
	__vo uint32_t MODER;			/* GPIO port mode register						0x00 */
	__vo uint32_t OTYPER;			/* GPIO port output type register  				0x04 */
	__vo uint32_t OSPEEDR;			/* GPIO port output speed register 				0x08 */
	__vo uint32_t PUPDR;			/* GPIO port pull-up/pull-down register 		0x0C */
	__vo uint32_t IDR;				/* GPIO port input data register  				0x10 */
	__vo uint32_t ODR;				/* GPIO port output data register 				0x14 */
	__vo uint32_t BSRR;				/* GPIO port bit set/reset register  			0x18 */
	__vo uint32_t LCKR;				/* GPIO port configuration lock register 		0x1C */
	__vo uint32_t AFR[2];			/* GPIO alternate function low/high register 	0x20, 0x24*/
}GPIO_RegDef_t;

/*************** RCC Register Definitions Structure **********************/

typedef struct
{
	__vo uint32_t CR;			/* 							Address offset: 0x00 */
	__vo uint32_t PLLCFGR;		/* 							Address offset: 0x04 */
	__vo uint32_t CFGR;			/* 							Address offset: 0x08 */
	__vo uint32_t CIR;			/* 							Address offset: 0x0C */
	__vo uint32_t AHB1RSTR;		/* 							Address offset: 0x10 */
	__vo uint32_t AHB2RSTR;		/* 							Address offset: 0x14 */
	uint32_t RESERVED0[2];		/* Reserved 				Address offset: 0x18 + 0x1C*/
	__vo uint32_t APB1RSTR;		/* 							Address offset: 0x20 */
	__vo uint32_t APB2RSTR;		/* 							Address offset: 0x24 */
	uint32_t RESERVED1[2];		/* Reserved 				Address offset: 0x28 + 0x2C */
	__vo uint32_t AHB1ENR;		/* 							Address offset: 0x30 */
	__vo uint32_t AHB2ENR;		/* 							Address offset: 0x34 */
	uint32_t RESERVED2[2];		/* Reserved 				Address offset: 0x38 + 0x3C */
	__vo uint32_t APB1ENR;		/* 							Address offset: 0x40 */
	__vo uint32_t APB2ENR;		/* 							Address offset: 0x44 */
	uint32_t RESERVED3[2];		/* Reserved 				Address offset: 0x48 + 0x4C */
	__vo uint32_t AHB1LPENR;	/* 							Address offset: 0x50 */
	__vo uint32_t AHB2LPENR;	/* 							Address offset: 0x54 */
	uint32_t RESERVED4[2];		/* Reserved 				Address offset: 0x58 + 0x5C */
	__vo uint32_t APB1LPENR;	/* 							Address offset: 0x60 */
	__vo uint32_t APB2LPENR;	/* 							Address offset: 0x64 */
	uint32_t RESERVED5[2];		/* Reserved 				Address offset: 0x68 + 0x6C */
	__vo uint32_t BDCR;			/* 							Address offset: 0x70 */
	__vo uint32_t CSR;			/* 							Address offset: 0x74 */
	uint32_t RESERVED6[2];		/* Reserved					Address offset: 0x78 + 0x7C */
	__vo uint32_t SSCGR;		/* 							Address offset: 0x80 */
	__vo uint32_t PLLI2SCFGR;	/* 							Address offset: 0x84 */
	uint32_t RESERVED7;			/* Reserved					Address offset: 0x88 */
	__vo uint32_t DCKCFGR;		/* 							Address offset: 0x8C */
}RCC_RegDef_t;


/*************** EXTI Register Definitions Structure **********************/

typedef struct
{
	__vo uint32_t IMR;			/* Interrupt mask register				Address offset: 0x00 */
	__vo uint32_t EMR;			/* Event mask register					Address offset: 0x04 */
	__vo uint32_t RTSR;			/* Rising trigger selection register	Address offset: 0x08 */
	__vo uint32_t FTSR;			/* Falling trigger selection register	Address offset: 0x0C */
	__vo uint32_t SWIER;		/* Software interrupt event register	Address offset: 0x10 */
	__vo uint32_t PR;			/* Pending register 					Address offset: 0x14 */

}EXTI_RegDef_t;

/*************** SYSCFG Register Definitions Structure **********************/

typedef struct
{
	__vo uint32_t MEMRMP;			/* SYSCFG Memory remap register									Address offset: 0x00 */
	__vo uint32_t PMC;				/* SYSCFG peripheral mode configuration register 				Address offset: 0x04 */
	__vo uint32_t EXTICR[4];		/* SYSCFG external interrupt configuration register from 1 to 4 Address offset: 0x08-0x14 */
	uint32_t RESERVED1[2];			/* Reserved 													Address offset: 0x18-0x1C */
	__vo uint32_t CMPCR;			/* Compensation cell control register							Address offset: 0x20 */

}SYSCFG_RegDef_t;


/*************** SPI Definitions Structure **********************/


typedef struct
{
	__vo uint32_t CR1; 		/* SPI control register 1 			Address offset: 0x00 */
	__vo uint32_t CR2;		/* SPI control register 2			Address offset: 0x04 */
	__vo uint32_t SR;		/* SPI status register				Address offset: 0x08 */
	__vo uint32_t DR;		/* SPI data register				Address offset: 0x0C */
	__vo uint32_t CRCPR;	/* SPI CRC polynomial register 		Address offset: 0x10 */
	__vo uint32_t RXCRCR;	/* SPI RX CRC register				Address offset: 0x14 */
	__vo uint32_t TXCRCR;	/* SPI TX CRC register				Address offset: 0x18 */
	__vo uint32_t I2SCFGR;	/* SPI_I2S configuration register	Address offset: 0x1C */
	__vo uint32_t I2SPR;	/* SPI_I2S prescaler register		Address offset: 0x20 */
}SPI_Regdef_t;

/*************** I2C Definitions Structure **********************/

typedef struct
{
	__vo uint32_t CR1;		/* Control register 1		Address offset: 0x00 */
	__vo uint32_t CR2;		/* Control register 2		Address offset: 0x04 */
	__vo uint32_t OAR1;		/* Own address register 1	Address offset: 0x08 */
	__vo uint32_t OAR2;		/* Own address register 2	Address offset: 0x0C */
	__vo uint32_t DR;		/* Data	register			Address offset: 0x10 */
	__vo uint32_t SR1;		/* Status register 1		Address offset: 0x14 */
	__vo uint32_t SR2;		/* Status register 2		Address offset: 0x18 */
	__vo uint32_t CCR;		/* Clock control register	Address offset: 0x1C */
	__vo uint32_t TRISE;	/* TRISE register 1			Address offset: 0x20 */
	__vo uint32_t FLTR;		/* FLTR register 1			Address offset: 0x24 */
}I2C_RegDef_t;

/*************** USART Definitions Structure **********************/

typedef struct
{
	__vo uint32_t SR;		/* Status register		Address offset: 0x00 */
	__vo uint32_t DR;		/* Data register		Address offset: 0x04 */
	__vo uint32_t BRR;		/* Baud rate register	Address offset: 0x08 */
	__vo uint32_t CR1;		/* Control register 1	Address offset: 0x0C */
	__vo uint32_t CR2;		/* Control register 2		Address offset: 0x10 */
	__vo uint32_t CR3;		/* Control register 3		Address offset: 0x14 */
	__vo uint32_t GTPR;		/* Guard time and prescaler register		Address offset: 0x18 */
}USART_RegDef_t;



/*
 * Peripheral definitions
 */

#define GPIOA		(GPIO_RegDef_t*)GPIOA_BASEADDR		/* GPIOA address definition */
#define GPIOB		(GPIO_RegDef_t*)GPIOB_BASEADDR		/* GPIOB address definition */
#define GPIOC		(GPIO_RegDef_t*)GPIOC_BASEADDR		/* GPIOC address definition */
#define GPIOD		(GPIO_RegDef_t*)GPIOD_BASEADDR		/* GPIOD address definition */
#define GPIOE		(GPIO_RegDef_t*)GPIOE_BASEADDR		/* GPIOE address definition */
#define GPIOH		(GPIO_RegDef_t*)GPIOH_BASEADDR		/* GPIOH address definition */

/*
 * RCC definition
 */
#define RCC			((RCC_RegDef_t*)RCC_BASEADDR)

/*
 * EXTI definition
 */

#define EXTI		((EXTI_RegDef_t*)EXTI_BASEADDR)

/*
 * SYSCFG definition
 */

#define SYSCFG		((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)

/*
 * SPI definition
 */

#define SPI1		((SPI_Regdef_t*)SPI1_BASEADDR)
#define SPI2		((SPI_Regdef_t*)SPI2_BASEADDR)
#define SPI3		((SPI_Regdef_t*)SPI3_BASEADDR)

/*
 * I2C definition
 */

#define I2C1		((I2C_RegDef_t*)I2C1_BASEADDR)
#define I2C2		((I2C_RegDef_t*)I2C2_BASEADDR)
#define I2C3		((I2C_RegDef_t*)I2C3_BASEADDR)

/*
 * USART definition
 */

#define USART1		((USART_RegDef_t*)USART1_BASEADDR)
#define USART2		((USART_RegDef_t*)USART2_BASEADDR)
#define USART6		((USART_RegDef_t*)USART6_BASEADDR)

/*
 * Clock enable Macros for GPIOx peripherals
 */

#define GPIOA_PCLK_EN()		(RCC->AHB1ENR |= (1 << 0 ))
#define GPIOB_PCLK_EN()		(RCC->AHB1ENR |= (1 << 1 ))
#define GPIOC_PCLK_EN()		(RCC->AHB1ENR |= (1 << 2 ))
#define GPIOD_PCLK_EN()		(RCC->AHB1ENR |= (1 << 3 ))
#define GPIOE_PCLK_EN()		(RCC->AHB1ENR |= (1 << 4 ))
#define GPIOH_PCLK_EN()		(RCC->AHB1ENR |= (1 << 7 ))

/*
 * Clock enable Macros for I2Cx peripherals
 */

#define I2C1_PCLK_EN()		( RCC->APB1ENR |= ( 1 << 21 ) )
#define I2C2_PCLK_EN()		( RCC->APB1ENR |= ( 1 << 22 ) )
#define I2C3_PCLK_EN()		( RCC->APB1ENR |= ( 1 << 23 ) )

/*
 * Clock enable Macros for SPIx peripherals
 */

#define SPI1_PCLK_EN()		( RCC->APB2ENR |= ( 1 << 12 ) )
#define SPI2_PCLK_EN()		( RCC->APB1ENR |= ( 1 << 14 ) )
#define SPI3_PCLK_EN()		( RCC->APB1ENR |= ( 1 << 15 ) )
#define SPI4_PCLK_EN()		( RCC->APB2ENR |= ( 1 << 13 ) )
#define SPI5_PCLK_EN()		( RCC->APB2ENR |= ( 1 << 20 ) )

/*
 * Clock enable Macros for USARTx peripherals
 */
#define USART2_PCLK_EN()	( RCC->APB1ENR |= ( 1 << 17 ) )
#define USART1_PCLK_EN()	( RCC->APB2ENR |= ( 1 << 4 ) )
#define USART6_PCLK_EN()	( RCC->APB2ENR |= ( 1 << 5 ) )
/*
 * Clock enable Macros for SYSCFG peripherals
 */
#define SYSCFG_PCLK_EN()	( RCC->APB2ENR |= ( 1 << 14 ) )

/*
 * Clock disable Macros for GPIOx peripherals
 */
#define GPIOA_PCLK_DI()		( RCC->AHB1ENR &= ~( 1 << 0 ) )
#define GPIOB_PCLK_DI()		( RCC->AHB1ENR &= ~( 1 << 1 ) )
#define GPIOC_PCLK_DI()		( RCC->AHB1ENR &= ~( 1 << 2 ) )
#define GPIOD_PCLK_DI()		( RCC->AHB1ENR &= ~( 1 << 3 ) )
#define GPIOE_PCLK_DI()		( RCC->AHB1ENR &= ~( 1 << 4 ) )
#define GPIOH_PCLK_DI()		( RCC->AHB1ENR &= ~( 1 << 7 ) )

/*
 * Clock disable Macros for I2Cx peripherals
 */
#define I2C1_PCLK_DI()		( RCC->APB1ENR &= ~( 1 << 21 ) )
#define I2C2_PCLK_DI()		( RCC->APB1ENR &= ~( 1 << 22 ) )
#define I2C3_PCLK_DI()		( RCC->APB1ENR &= ~( 1 << 23 ) )

/*
 * Clock disable Macros for SPIx peripherals
 */
#define SPI1_PCLK_DI()		( RCC->APB2ENR &= ~( 1 << 12 ) )
#define SPI2_PCLK_DI()		( RCC->APB1ENR &= ~( 1 << 14 ) )
#define SPI3_PCLK_DI()		( RCC->APB1ENR &= ~( 1 << 15 ) )
#define SPI4_PCLK_DI()		( RCC->APB2ENR &= ~( 1 << 13 ) )
#define SPI5_PCLK_DI()		( RCC->APB2ENR &= ~( 1 << 20 ) )

/*
 * Clock disable Macros for SYSCFG peripherals
 */
#define SYSCFG_PCLK_DI()	( RCC->APB2ENR &= ~( 1 << 14 ) )

/*
 * Clock Disable Macros for USARTx peripherals
 */
#define USART2_PCLK_DI()	( RCC->APB1ENR &= ~( 1 << 17 ) )
#define USART1_PCLK_DI()	( RCC->APB2ENR &= ~( 1 << 4 ) )
#define USART6_PCLK_DI()	( RCC->APB2ENR &= ~( 1 << 5 ) )

/*
 * GPIO Peripherals Reset Macros
 */
#define GPIOA_REG_RESET()	do{ (RCC->AHB1RSTR |= (1 << 0)); (RCC->AHB1RSTR &= ~(1 << 0));} while(0)
#define GPIOB_REG_RESET()	do{ (RCC->AHB1RSTR |= (1 << 1)); (RCC->AHB1RSTR &= ~(1 << 1));} while(0)
#define GPIOC_REG_RESET()	do{ (RCC->AHB1RSTR |= (1 << 2)); (RCC->AHB1RSTR &= ~(1 << 2));} while(0)
#define GPIOD_REG_RESET()	do{ (RCC->AHB1RSTR |= (1 << 3)); (RCC->AHB1RSTR &= ~(1 << 3));} while(0)
#define GPIOE_REG_RESET()	do{ (RCC->AHB1RSTR |= (1 << 4)); (RCC->AHB1RSTR &= ~(1 << 4));} while(0)
#define GPIOH_REG_RESET()	do{ (RCC->AHB1RSTR |= (1 << 7)); (RCC->AHB1RSTR &= ~(1 << 7));} while(0)

/*
 * SPI Peripherals Reset Macros
 */
#define SPI1_REG_RESET()	do{ (RCC->APB1RSTR |= (1 << 21)); (RCC->APB1RSTR &= ~(1 << 21));} while(0)
#define SPI2_REG_RESET()	do{ (RCC->APB1RSTR |= (1 << 22)); (RCC->APB1RSTR &= ~(1 << 22));} while(0)
#define SPI3_REG_RESET()	do{ (RCC->APB1RSTR |= (1 << 23)); (RCC->APB1RSTR &= ~(1 << 23));} while(0)


/*
 * I2C Peripheral Reset Macros
 */

#define I2C1_REG_RESET()	do{ (RCC->APB1RSTR |= (1 << 12)); (RCC->AHB2RSTR &= ~(1 << 12));} while(0)
#define I2C2_REG_RESET()	do{ (RCC->AHB1RSTR |= (1 << 14)); (RCC->AHB1RSTR &= ~(1 << 14));} while(0)
#define I2C3_REG_RESET()	do{ (RCC->AHB1RSTR |= (1 << 15)); (RCC->AHB1RSTR &= ~(1 << 15));} while(0)


/*
 * Return port code for given GPIO base address
 */
#define GPIO_BASE_ADDR_TO_PORTCODE(x)	  (	(x == GPIOA) ? 0 :\
											(x == GPIOB) ? 1 :\
											(x == GPIOC) ? 2 :\
											(x == GPIOD) ? 3 :\
											(x == GPIOE) ? 4 :\
											(x == GPIOH) ? 5 : 0 )

/*
 * IRQ-Interrupt Request Number of STM32F4xx MCU
 */
#define IRQ_NO_EXTI0			6
#define IRQ_NO_EXTI1			7
#define IRQ_NO_EXTI2			8
#define IRQ_NO_EXTI3			9
#define IRQ_NO_EXTI4			10
#define IRQ_NO_EXTI9_5			23
#define IRQ_NO_EXTI15_10		40

#define IRQ_NO_SPI1				35
#define IRQ_NO_SPI2				36
#define IRQ_NO_SPI3				51

#define IRQ_NO_I2C1_EV			31
#define IRQ_NO_I2C1_ER			32

#define IRQ_NO_USART2			38
/*
 * IRQ Priority levels macros
 */
#define NVIC_IRQ_PRIO0			0
#define NVIC_IRQ_PRIO15			15
/*
 * Some generic Macros
 */
#define ENABLE 			1
#define DISABLE			0
#define RESET			DISABLE
#define SET				ENABLE
#define GPIO_PIN_SET	SET
#define	GPIO_PIN_RESET	RESET
#define FLAG_RESET		RESET
#define FLAG_SET		SET

/*
 **************** Bit position definition  of SPI peripheral********************
 */
/* For Control Register 1  */
#define SPI_CR1_CPHA			0
#define SPI_CR1_CPOL			1
#define SPI_CR1_MSTR			2
#define SPI_CR1_BR				3
#define SPI_CR1_SPE				6
#define SPI_CR1_LSB_FIRST		7
#define SPI_CR1_SSI				8
#define SPI_CR1_SSM				9
#define SPI_CR1_RXONLY			10
#define SPI_CR1_DFF				11
#define SPI_CR1_CRC_NEXT		12
#define SPI_CR1_CRC_EN			13
#define SPI_CR1_BIDIOE			14
#define SPI_CR1_BIDIMODE		15

/* For Control Register 2  */
#define SPI_CR2_RXDMAEN			0
#define SPI_CR2_TXDMAEN			1
#define SPI_CR2_SSOE			2
#define SPI_CR2_FRF				4
#define SPI_CR2_ERRIE			5
#define SPI_CR2_RXNEIE			6
#define SPI_CR2_TXEIE			7

/* For Status register */
#define SPI_SR_RXNE				0
#define SPI_SR_TXE				1
#define SPI_SR_CHSIDE			2
#define SPI_ST_UDR				3
#define SPI_SR_CRCERR			4
#define SPI_SR_MODF				5
#define SPI_SR_OVR				6
#define SPI_SR_BSY				7
#define SPI_SR_FRE				8


/*
 **************** Bit position definition  of I2C peripheral********************
 */

/* For CR1 register */

#define I2C_CR1_PE					0
#define I2C_CR1_SMBUS				1
#define I2C_CR1_SMBTYPE				3
#define I2C_CR1_ENARP				4
#define I2C_CR1_ENPEC				5
#define I2C_CR1_ENGC				6
#define I2C_CR1_NOSTRETCH			7
#define I2C_CR1_START				8
#define I2C_CR1_STOP				9
#define I2C_CR1_ACK					10
#define I2C_CR1_POS					11
#define I2C_CR1_PEC					12
#define I2C_CR1_ALERT				13
#define I2C_CR1_SWRST				15

/*
 * For I2C control register 2
 */

#define I2C_CR2_FREG				0
#define I2C_CR2_ITERREN				8
#define I2C_CR2_ITEVFEN				9
#define I2C_CR2_ITBUFEN				10
#define I2C_CR2_DMAEN				11
#define I2C_CR2_LAST				12

/*
 * for I2C status register 1
 */

#define I2C_SR1_SB					0
#define I2C_SR1_ADDR				1
#define I2C_SR1_BTF					2
#define I2C_SR1_ADD10				3
#define I2C_SR1_STOPF				4
#define I2C_SR1_RXNE				6
#define I2C_SR1_TXE					7
#define I2C_SR1_BERR				8
#define I2C_SR1_ARLO				9
#define I2C_SR1_AF					10
#define I2C_SR1_OVR					11
#define I2C_SR1_PECERR				12
#define I2C_SR1_TIMEOUT				14
#define I2C_SR1_SMBALERT			15

/*
 * For I2C status register 2
 */

#define I2C_SR2_MSL					0
#define I2C_SR2_BUSY				1
#define I2C_SR2_TRA					2
#define I2C_SR2_GENCALL				4
#define I2C_SR2_SMBDEFAULT			5
#define I2C_SR2_SMBHOST				6
#define I2C_SR2_DUALF				7
#define I2C_SR2_PEC					8

/*
 * For I2C Clock Control Register
 */
#define I2C_CCR_CCR					0
#define I2C_CCR_DUTY				14
#define I2C_CCR_FS					15

/*
 **************** Bit position definition  of USART peripheral********************
 */

/*
 * For status register
 */

#define USART_SR_PE				0
#define USART_SR_FE				1
#define USART_SR_NF				2
#define USART_SR_ORE			3
#define USART_SR_IDLE			4
#define USART_SR_RXNE			5
#define USART_SR_TC				6
#define USART_SR_TXE			7
#define USART_SR_LBD			8
#define USART_SR_CTS			9

/*
 * For Data register
 */

#define USART_DR_DR				0

/*
 * For baud rate register
 */

#define USART_BRR_DIV_Fraction 	0
#define USART_BRR_DIV_Mantissa 	4

/*
 * for control register 1
 */

#define USART_CR1_SBK			0
#define USART_CR1_RWU			1
#define USART_CR1_RE			2
#define USART_CR1_TE			3
#define USART_CR1_IDLEIE		4
#define USART_CR1_RXNEIE		5
#define USART_CR1_TCIE			6
#define USART_CR1_TXEIE			7
#define USART_CR1_PEIE			8
#define USART_CR1_PS			9
#define USART_CR1_PCE			10
#define USART_CR1_WAKE			11
#define USART_CR1_M				12
#define USART_CR1_UE			13
#define USART_CR1_OVER8			15

/*
 * For control register 2
 */

#define USART_CR2_ADD			0
#define USART_CR2_LBDL			5
#define USART_CR2_LBDIE			6
#define USART_CR2_LBCL			8
#define USART_CR2_CPHA			9
#define USART_CR2_CPOL			10
#define USART_CR2_CLKEN			11
#define USART_CR2_STOP			12
#define USART_CR2_LINEN			14

/*
 * For control register 3
 */

#define USART_CR3_EIE			0
#define USART_CR3_IREN			1
#define USART_CR3_IRLP			2
#define USART_CR3_HDSEL			3
#define USART_CR3_NACK			4
#define USART_CR3_SCEN			5
#define USART_CR3_DMAR			6
#define USART_CR3_DMAT			7
#define USART_CR3_RTSE			8
#define USART_CR3_CTSE			9
#define USART_CR3_CTSIE			10
#define USART_CR3_ONEBIT		11

/*
 * For guard time and prescaler register
 */

#define USART_GTPR_PSC			0
#define USART_GTPR_GT			8




#include"stm32f411_gpio_driver.h"
#include"stm32f411_spi_driver.h"
#include"stm32f411_i2c_driver.h"
#include"stm32f411_usart_driver.h"
#include"stm32f411_rcc_driver.h"

#endif /* INC_STM32F411XX_H_ */
