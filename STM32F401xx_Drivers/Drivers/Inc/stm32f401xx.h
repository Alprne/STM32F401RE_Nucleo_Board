/*
 * stm32f401xx.h
 *
 *  Created on: Jun 30, 2023
 *      Author: alprn
 */

#ifndef INC_STM32F401XX_H_
#define INC_STM32F401XX_H_

#define __vo volatile
#define __weak __attribute__((weak))

#include <stdint.h>
#include <stddef.h>

/******************************START: Processor Spesific Details***********************************
*
* ARM Cortex Mx Processor NVIC ISERx register Adresses
*/

#define NVIC_ISER0				((__vo uint32_t*) 0xE000E100 )
#define NVIC_ISER1				((__vo uint32_t*) 0xE000E104 )
#define NVIC_ISER2				((__vo uint32_t*) 0xE000E108 )
#define NVIC_ISER3				((__vo uint32_t*) 0xE000E10C )

/*
* ARM Cortex Mx Processor NVIC ICERx register Adresses
*/

#define NVIC_ICER0				((__vo uint32_t*) 0XE000E180 )
#define NVIC_ICER1				((__vo uint32_t*) 0xE000E184 )
#define NVIC_ICER2				((__vo uint32_t*) 0xE000E108 )
#define NVIC_ICER3				((__vo uint32_t*) 0xE000E18C )

/*
 * ARM Cortex Mx Processor Priority Register Adress Calculation
 */

#define NVIC_PR_BASE_ADDR		((__vo int32_t*) 0xE000E400)

/*
 * ARM Cortex Mx Processor number of priority bits implemented in Priority Register
 */

#define NO_PR_BITS_IMPLEMENTED				4

//Base addresses of SRAM and Flash memories.
#define FLASH_BASEADDR			0x08000000U
#define SRAM1_BASEADDR			0x2000 0000U
#define SRAM 					SRAM1_BASEADDR
#define ROM_BASEADDR			0x1FFF0000U


//AHBx and APBx Bus Peripheral Base Addresses
#define PERIPH_BASE				0x40000000U
#define APB1PERIPH_BASE			PERIPH_BASE
#define APB2PERIPH_BASE			0x40010000U

#define AHB1PERIPH_BASE			0x40020000U
#define AHB2PERIPH_BASE			0x50000000U


//Base Addresses of peripherals which hanging on AHB1 Bus Domain
#define GPIOA_BASEADDR			(AHB1PERIPH_BASE + 0x0000)
#define GPIOB_BASEADDR			(AHB1PERIPH_BASE + 0x0400)
#define GPIOC_BASEADDR			(AHB1PERIPH_BASE + 0x0800)
#define GPIOD_BASEADDR			(AHB1PERIPH_BASE + 0x0C00)
#define GPIOE_BASEADDR			(AHB1PERIPH_BASE + 0x1000)
#define GPIOH_BASEADDR			(AHB1PERIPH_BASE + 0x1C00)

#define	RCC_BASEADDR			(AHB1PERIPH_BASE + 0x3800)

//Base Addresses of peripherals which hanging on APB1 Bus Domain
#define I2C1_BASEADDR			(APB1PERIPH_BASE + 0x5400)
#define	I2C2_BASEADDR			(APB1PERIPH_BASE + 0x5800)
#define I2C3_BASEADDR			(APB1PERIPH_BASE + 0x5C00)

#define SPI2_BASEADDR			(APB1PERIPH_BASE + 0x3800)
#define SPI3_BASEADDR			(APB1PERIPH_BASE + 0x3C00)

#define USART2_BASEADDR			(APB1PERIPH_BASE + 0x4400)


//Base Addresses of peripherals which hanging on APB2 Bus Domain
#define USART1_BASEADDR			(APB2PERIPH_BASE + 0x1000)
#define USART6_BASEADDR			(APB2PERIPH_BASE + 0x1400)

#define SPI1_BASEADDR			(APB2PERIPH_BASE + 0x3000)
#define SPI4_BASEADDR			(APB2PERIPH_BASE + 0x3400)

#define SYSCFG_BASEADDR			(APB2PERIPH_BASE + 0x3800)

#define EXTI_BASEADDR			(APB2PERIPH_BASE + 0x3C00)


/*******************************peripheral register definition structures ***************************/

/*
* Note: Registers of a peripheral are specific to MCU
* e.g Number of Registers of SPI peripheral of STM32F4x family of MCUs may be different (more or less)
* Compared to number of registers of SPI peripheral of STM32LX or STM32F0x family of MCUS
* Please check your Device RM
*/

typedef struct
{
__vo uint32_t MODER;			//GPIO Port Mode Register						Address Offset: 0x00
__vo uint32_t OTYPER;			//GPIO Output Type Register						Address Offset: 0x04
__vo uint32_t OSPEEDR;			//GPIO Output Speed Register					Address Offset: 0x08
__vo uint32_t PUPDR;			//GPIO Pull-up / Pull-down Register				Address Offset: 0x0C
__vo uint32_t IDR;				//GPIO Input Data Register						Address Offset: 0x10
__vo uint32_t ODR;				//GPIO Output Data Register						Address Offset: 0x14
__vo uint32_t BSRRL;			//GPIO port bit set/reset register				Address Offset: 0x18
__vo uint32_t LCKR;				//GPIO port configuration lock register			Address Offset: 0x1C
__vo uint32_t AFR[2];			//GPIO alternate function low register [1]		Address Offset: 0x20
} GPIO_RegDef_t;				//GPIO alternate function high register[2]		Address Offset: 0x24


// Peripheral register definition structure for RCC

typedef struct
{
	__vo uint32_t CR;			//RCC clock control register										Address Offset: 0x00
	__vo uint32_t PLLCFGR;		//RCC PLL configuration register									Address Offset: 0x04
	__vo uint32_t CFGR;			//RCC clock configuration register									Address Offset: 0x08
	__vo uint32_t CIR;			//RCC clock interrupt register										Address Offset: 0x0C
	__vo uint32_t AHB1RSTR;		//RCC AHB1 peripheral reset register								Address Offset: 0x10
	__vo uint32_t AHB2RSTR;		//RCC AHB2 peripheral reset register								Address Offset: 0x14
	__vo uint32_t RESERVED0;	//																	Address Offset: 0x18
	__vo uint32_t RESERVED1;	//																	Address Offset: 0x1C
	__vo uint32_t APB1RSTR;		//RCC APB1 peripheral reset register								Address Offset: 0x20
	__vo uint32_t APB2RSTR;		//RCC APB2 peripheral reset register								Address Offset: 0x24
	__vo uint32_t RESERVED2;	//																	Address Offset: 0x28
	__vo uint32_t RESERVED3;	//																	Address Offset: 0x2C
	__vo uint32_t AHB1ENR;		//RCC AHB1 peripheral clock enable register							Address Offset: 0x30
	__vo uint32_t AHB2ENR;		//RCC AHB2 peripheral clock enable register							Address Offset: 0x34
	__vo uint32_t RESERVED4;	//																	Address Offset: 0x38
	__vo uint32_t RESERVED5;	//																	Address Offset: 0x3C
	__vo uint32_t APB1ENR;		//RCC APB1 peripheral clock enable register							Address Offset: 0x40
	__vo uint32_t APB2ENR;		//RCC APB2 peripheral clock enable register							Address Offset: 0x44
	__vo uint32_t RESERVED6;	//																	Address Offset: 0x48
	__vo uint32_t RESERVED7;	//																	Address Offset: 0x4C
	__vo uint32_t AHB1LPENR;	//RCC AHB1 peripheral clock enable in low power mode register		Address Offset: 0x50
	__vo uint32_t AHB2LPENR;	//RCC AHB2 peripheral clock enable in low power mode register		Address Offset: 0x54
	__vo uint32_t RESERVED8;	//                                                                  Address Offset: 0x58
	__vo uint32_t RESERVED9;	//																	Address Offset: 0x5C
	__vo uint32_t APB1LPENR;	//RCC APB1 peripheral clock enable in low power mode register		Address Offset: 0x60
	__vo uint32_t APB2LPENR;	//RCC APB2 peripheral clock enabled in low power mode register		Address Offset: 0x64
	__vo uint32_t RESERVED10;	//																	Address Offset: 0x68
	__vo uint32_t RESERVED11;	//																	Address Offset: 0x6C
	__vo uint32_t BDCR;			//RCC Backup domain control register								Address Offset: 0x70
	__vo uint32_t CSR;			//RCC clock control & status register								Address Offset: 0x74
	__vo uint32_t RESERVED12;	//																	Address Offset: 0x78
	__vo uint32_t RESERVED13;	//																	Address Offset: 0x7C
	__vo uint32_t SSCGR;			//RCC spread spectrum clock generation register					Address Offset: 0x80
	__vo uint32_t PLLI2SCFGR;	//RCC PLLI2S configuration register									Address Offset: 0x84
	__vo uint32_t RESERVED14;	//																	Address Offset: 0x88
	__vo uint32_t DCKCFGR;		//RCC Dedicated Clocks Configuration Register						Address Offset: 0x8C
}RCC_RegDef_t;

// Peripheral register definition structure for EXTI

typedef struct
{
__vo uint32_t IMR;				//EXTI Interrupt mask register					Address Offset: 0x00
__vo uint32_t EMR;				//EXTI Event mask register						Address Offset: 0x04
__vo uint32_t RTSR;				//EXTI Rising trigger selection register		Address Offset: 0x08
__vo uint32_t FTSR;				//EXTI Falling trigger selection register		Address Offset: 0x0C
__vo uint32_t SWIER;			//EXTI Software interrupt event register		Address Offset: 0x10
__vo uint32_t PR;				//EXTI Pending register							Address Offset: 0x14
} EXTI_RegDef_t;

// Peripheral register definition structure for SPI

typedef struct
{
__vo uint32_t CR1; 				//SPI control register 1												Address Offset: 0x00
__vo uint32_t CR2;				//SPI control register 2												Address Offset:	0x04
__vo uint32_t SR;				//SPI status register													Address Offset: 0x08
__vo uint32_t DR;				//SPI data register														Address Offset: 0x0C
__vo uint32_t CRCPR;			//SPI CRC polynomial register (SPI_CRCPR)(not used in I2Smode)			Address Offset: 0x10
__vo uint32_t RXCRCR;			//SPI RX CRC register (SPI_RXCRCR)(not used in I2S mode)				Address Offset: 0x14
__vo uint32_t TXCRCR;			//SPI TX CRC register (SPI_TXCRCR)(not used in I2S mode)				Address Offset: 0x18
__vo uint32_t I2SCFGR;			//SPI_I2S configuration register (SPI_I2SCFGR)							Address Offset: 0x1C
__vo uint32_t I2SPR;			//SPI_I2S prescaler register (SPI_I2SPR)								Address Offset: 0x20

}SPI_RegDef_t;


// Peripheral register definition structure for I2C

typedef struct
{
__vo uint32_t CR1; 				//I2C control register 1								Address Offset: 0x00
__vo uint32_t CR2;				//I2C control register 2								Address Offset:	0x04
__vo uint32_t OAR1;				//I2C Own address register 1							Address Offset: 0x08
__vo uint32_t OAR2;				//I2C Own address register 2							Address Offset: 0x0C
__vo uint32_t DR;				//I2C data register										Address Offset: 0x10
__vo uint32_t SR1;				//I2C status register 1									Address Offset: 0x14
__vo uint32_t SR2;				//I2C status register 2									Address Offset: 0x18
__vo uint32_t CCR;				//I2C Clock control register							Address Offset: 0x1C
__vo uint32_t TRISE;			//I2C TRISE register 									Address Offset: 0x20
__vo uint32_t FLTR;				//I2C FLTR register										Address Offset: 0x24

}I2C_RegDef_t;


// Peripheral register definition structure for SYSCFG

typedef struct
{
__vo uint32_t MEMRMP;			//SYSCFG Memory remap register							Address Offset: 0x00
__vo uint32_t PMC;				//SYSCFG Peripheral mode configuration register			Address Offset: 0x04
__vo uint32_t EXTICR[4];		//SYSCFG external interrupt configuration register 1	Address Offset: 0x08 - 0x14
__vo uint32_t RESERVED1;		//														Address Offset: 0x18
__vo uint32_t RESERVED2;		//														Address Offset: 0x1C
__vo uint32_t CMPCR;			//SYSCFG Compensation cell control register				Address Offset: 0x20
} SYSCFG_RegDef_t;

//Peripheral Definitions (Peripherals base addresses typecasted to xxx_RegDef_t)

#define GPIOA					((GPIO_RegDef_t*) GPIOA_BASEADDR)
#define GPIOB					((GPIO_RegDef_t*) GPIOB_BASEADDR)
#define GPIOC					((GPIO_RegDef_t*) GPIOC_BASEADDR)
#define GPIOD					((GPIO_RegDef_t*) GPIOD_BASEADDR)
#define GPIOE					((GPIO_RegDef_t*) GPIOE_BASEADDR)
#define GPIOH					((GPIO_RegDef_t*) GPIOH_BASEADDR)

#define RCC						((RCC_RegDef_t*) RCC_BASEADDR)

#define EXTI 					((EXTI_RegDef_t*) EXTI_BASEADDR)

#define SYSCFG					((SYSCFG_RegDef_t*) SYSCFG_BASEADDR)

#define SPI1					((SPI_RegDef_t*) SPI1_BASEADDR)
#define SPI2					((SPI_RegDef_t*) SPI2_BASEADDR)
#define SPI3					((SPI_RegDef_t*) SPI3_BASEADDR)
#define SPI4					((SPI_RegDef_t*) SPI4_BASEADDR)

#define I2C1					((I2C_RegDef_t*) I2C1_BASEADDR)
#define I2C2					((I2C_RegDef_t*) I2C2_BASEADDR)
#define I2C3					((I2C_RegDef_t*) I2C3_BASEADDR)

//Clock Enable Macros for GPIOx Peripherals

#define GPIOA_PCLK_EN()			(RCC->AHB1ENR |= (1 << 0))
#define GPIOB_PCLK_EN()			(RCC->AHB1ENR |= (1 << 1))
#define GPIOC_PCLK_EN()			(RCC->AHB1ENR |= (1 << 2))
#define GPIOD_PCLK_EN()			(RCC->AHB1ENR |= (1 << 3))
#define GPIOE_PCLK_EN()			(RCC->AHB1ENR |= (1 << 4))
#define GPIOH_PCLK_EN()			(RCC->AHB1ENR |= (1 << 7))

//Clock Enable Macros for I2Cx Peripherals

#define I2C1_PCLK_EN()			(RCC->APB1ENR |= (1 << 21))
#define I2C2_PCLK_EN()			(RCC->APB1ENR |= (1 << 22))
#define I2C3_PCLK_EN()			(RCC->APB1ENR |= (1 << 23))

//Clock Enable Macros for SPIx Peripherals

#define SPI1_PCLK_EN()			(RCC->APB2ENR |= (1 << 12))
#define SPI2_PCLK_EN()			(RCC->APB1ENR |= (1 << 14))
#define SPI3_PCLK_EN()			(RCC->APB1ENR |= (1 << 15))
#define SPI4_PCLK_EN()			(RCC->APB2ENR |= (1 << 13))

//Clock Enable Macros for USARTx Peripherals

#define USART1_PCLK_EN()		(RCC->APB2ENR |= (1 << 4))
#define USART2_PLCK_EN()		(RCC->APB1ENR |= (1 << 17))
#define USART6_PLCK_EN()		(RCC->APB2ENR |= (1 << 5))

//Clock Enable Macros for SYSCFG Peripheral

#define SYSCFG_PLCK_EN()		(RCC->APB2ENR |= (1 << 14))

//Clock Disable Macros for GPIOx Peripherals

#define GPIOA_PCLK_DI()			(RCC->AHB1ENR &= ~(1 << 0))
#define GPIOB_PCLK_DI()			(RCC->AHB1ENR &= ~(1 << 1))
#define GPIOC_PCLK_DI()			(RCC->AHB1ENR &= ~(1 << 2))
#define GPIOD_PCLK_DI()			(RCC->AHB1ENR &= ~(1 << 3))
#define GPIOE_PCLK_DI()			(RCC->AHB1ENR &= ~(1 << 4))
#define GPIOH_PCLK_DI()			(RCC->AHB1ENR &= ~(1 << 7))


//Clock Disable Macros for I2Cx Peripherals

#define I2C1_PCLK_DI()			(RCC->APB1ENR &= ~(1 << 21))
#define I2C2_PCLK_DI()			(RCC->APB1ENR &= ~(1 << 22))
#define I2C3_PCLK_DI()			(RCC->APB1ENR &= ~(1 << 23))

//Clock Disable Macros for SPIx Peripherals

#define SPI1_PCLK_DI()			(RCC->APB2ENR &= ~(1 << 12))
#define SPI2_PCLK_DI()			(RCC->APB1ENR &= ~(1 << 14))
#define SPI3_PCLK_DI()			(RCC->APB1ENR &= ~(1 << 15))
#define SPI4_PCLK_DI()			(RCC->APB2ENR &= ~(1 << 13))

//Clock Disable Macros for USARTx Peripherals

#define USART1_PCLK_DI()		(RCC->APB2ENR &= ~(1 << 4))
#define USART2_PLCK_DI()		(RCC->APB1ENR &= ~(1 << 17))
#define USART6_PLCK_DI()		(RCC->APB2ENR &= ~(1 << 5))

//Clock Disable Macros for SYSCFG Peripheral

#define SYSCFG_PLCK_DI()		(RCC->APB2ENR &= ~(1 << 14))

//Macros to reset GPIOx peripherals
#define GPIOA_REG_RESET()		do{ (RCC->AHB1RSTR |= (1 << 0));  (RCC->AHB1RSTR &= ~(1 << 0)); }while(0)
#define GPIOB_REG_RESET()		do{ (RCC->AHB1RSTR |= (1 << 1));  (RCC->AHB1RSTR &= ~(1 << 1)); }while(0)
#define GPIOC_REG_RESET()		do{ (RCC->AHB1RSTR |= (1 << 2));  (RCC->AHB1RSTR &= ~(1 << 2)); }while(0)
#define GPIOD_REG_RESET()		do{ (RCC->AHB1RSTR |= (1 << 3));  (RCC->AHB1RSTR &= ~(1 << 3)); }while(0)
#define GPIOE_REG_RESET()		do{ (RCC->AHB1RSTR |= (1 << 4));  (RCC->AHB1RSTR &= ~(1 << 4)); }while(0)
#define GPIOH_REG_RESET()		do{ (RCC->AHB1RSTR |= (1 << 7));  (RCC->AHB1RSTR &= ~(1 << 7)); }while(0)

//Macros to reset SPIx peripherals
#define SPI1_REG_RESET()		do{ (RCC->APB2RSTR |= (1 << 12));  (RCC->APB2RSTR &= ~(1 << 12)); }while(0)
#define SPI2_REG_RESET()		do{ (RCC->APB1RSTR |= (1 << 14));  (RCC->APB1RSTR &= ~(1 << 14)); }while(0)
#define SPI3_REG_RESET()		do{ (RCC->APB1RSTR |= (1 << 15));  (RCC->APB1RSTR &= ~(1 << 15)); }while(0)
#define SPI4_REG_RESET()		do{ (RCC->APB2RSTR |= (1 << 13));  (RCC->APB2RSTR &= ~(1 << 13)); }while(0)

//Macros to reset I2Cx peripherals
#define I2C1_REG_RESET()		do{ (RCC->APB1RSTR |= (1 << 21));  (RCC->APB2RSTR &= ~(1 << 21)); }while(0)
#define I2C2_REG_RESET()		do{ (RCC->APB1RSTR |= (1 << 22));  (RCC->APB1RSTR &= ~(1 << 22)); }while(0)
#define I2C3_REG_RESET()		do{ (RCC->APB1RSTR |= (1 << 23));  (RCC->APB1RSTR &= ~(1 << 23)); }while(0)

//This macro returns a code (between 0 to 5) for a given GPIO base address (x)

#define GPIO_BASE_TO_CODE(x)  (	(x == GPIOA) ? 0 : \
								(x == GPIOB) ? 1 : \
								(x == GPIOC) ? 2 : \
								(x == GPIOD) ? 3 : \
								(x == GPIOE) ? 4 : \
								(x == GPIOH) ? 5 : 0 )

//Macros of IRQ(Interrupt Request) Numbers of STM31F401X MCU

#define IRQ_NO_EXTI0 			6
#define IRQ_NO_EXTI1 			7
#define IRQ_NO_EXTI2 			8
#define IRQ_NO_EXTI3 			9
#define IRQ_NO_EXTI4 			10
#define IRQ_NO_EXTI9_5			23
#define IRQ_NO_EXTI15_10 		40
#define IRQ_NO_SPI1		 		35
#define IRQ_NO_SPI2	 			36
#define IRQ_NO_SPI3		 		51
#define IRQ_NO_SPI4		 		84

// Macros for all the possible priority levels

#define NVIC_IRQ_PRIO0      0
#define NVIC_IRQ_PRIO1      1
#define NVIC_IRQ_PRIO2      2
#define NVIC_IRQ_PRIO3      3
#define NVIC_IRQ_PRIO4      4
#define NVIC_IRQ_PRIO5      5
#define NVIC_IRQ_PRIO6      6
#define NVIC_IRQ_PRIO7      7
#define NVIC_IRQ_PRIO8      8
#define NVIC_IRQ_PRIO9      9
#define NVIC_IRQ_PRIO10     10
#define NVIC_IRQ_PRIO11     11
#define NVIC_IRQ_PRIO12     12
#define NVIC_IRQ_PRIO13     13
#define NVIC_IRQ_PRIO14     14
#define NVIC_IRQ_PRIO15     15
#define NVIC_IRQ_PRIO16     16


//Some Generic Macros

#define ENABLE 				1
#define DISABLE 			0
#define SET					ENABLE
#define RESET				DISABLE
#define GPIO_PIN_SET		SET
#define GPIO_PIN_RESET		RESET
#define HIGH				ENABLE
#define LOW					DISABLE
#define FLAG_RESET			RESET
#define FLAG_SET			SET

/**************************************************************************************************
 *Bit Position definitions of SPI Peripheral
 **************************************************************************************************/
//SPI_CR1_X

#define SPI_CR1_CPHA				0
#define SPI_CR1_CPOL				1
#define SPI_CR1_MSTR				2
#define SPI_CR1_BR0					3
#define SPI_CR1_BR1					4
#define SPI_CR1_BR2					5
#define SPI_CR1_SPE					6
#define SPI_CR1_LSBFIRST			7
#define SPI_CR1_SSI					8
#define SPI_CR1_SSM					9
#define SPI_CR1_RXONLY				10
#define SPI_CR1_DFF					11
#define SPI_CR1_BIDIOE				14
#define SPI_CR1_BIDIMODE			15

//SPI_CR2_X

#define SPI_CR2_RXDMAEN				0
#define SPI_CR2_TXDMAEN				1
#define SPI_CR2_SSOE				2
#define SPI_CR2_FRF					4
#define SPI_CR2_ERRIE				5
#define SPI_CR2_RXNEIE				6
#define SPI_CR2_TXNEIE				7

//SPI_SR_X
#define SPI_SR_RXNE					0
#define SPI_SR_TXE					1
#define SPI_SR_CHSIDE				2
#define SPI_SR_UDR					3
#define SPI_SR_CRCERR				4
#define SPI_SR_MODF					5
#define SPI_SR_OVR					6
#define SPI_SR_BSY					7
#define SPI_SR_FRE					8

/**************************************************************************************************
 *Bit Position definitions of I2C Peripheral
 **************************************************************************************************/
//I2C_CR1
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

//I2C_CR2
#define I2C_CR2_FREQ				0
#define I2C_CR2_ITERREN				8
#define I2C_CR2_ITEVTEN				9
#define I2C_CR2_ITBUFEN				10

//I2C_OAR1
#define I2C_OAR1_ADD0				0
#define I2C_OAR1_ADD71				1
#define I2C_OAR1_ADD98				8
#define I2C_OAR1_ADDMODE			15

//I2C_SR1
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
#define I2C_SR1_TIMEOUT				14

//I2C_SR2
#define I2C_SR2_MSL					0
#define I2C_SR2_BUSY				1
#define I2C_SR2_TRA					2
#define I2C_SR2_GENCALL				4
#define I2C_SR2_DUALF				7

//I2C_CCR
#define I2C_CCR_CCR					0
#define I2C_SR2_DUTY				14
#define I2C_SR2_FS					15

#include "stm32f401xx_gpio_driver.h"
#include "stm32f401xx_spi_driver.h"
#include "stm32f401xx_i2c_driver.h"

#endif /* INC_STM32F401XX_H_ */
