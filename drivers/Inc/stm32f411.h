/*
 * stm32fxx.h
 *
 */


#ifndef INC_STM32FXX_H_
#define INC_STM32FXX_H_

#include<stddef.h>
#include<stdint.h>

#define __vo volatile
#define __weak __attribute__((weak))


/********************************START:Processor Specific details*******************************
 * ARM Cortex Mx NVIC ISERx registers Addresses
 */

#define NVIC_ISER0		((__vo uint32_t *)0xE000E100)
#define NVIC_ISER1		((__vo uint32_t *)0xE000E104)
#define NVIC_ISER2		((__vo uint32_t *)0xE000E108)
#define NVIC_ISER3		((__vo uint32_t *)0xE000E10C)
#define NVIC_ISER4		((__vo uint32_t *)0xE000E110)
#define NVIC_ISER5		((__vo uint32_t *)0xE000E114)
#define NVIC_ISER6		((__vo uint32_t *)0xE000E118)
#define NVIC_ISER7		((__vo uint32_t *)0xE000E11C)


/*
 * ARM Cortex Mx NVIC ICERx registers Addresses
 */

#define NVIC_ICER0		((__vo uint32_t *)0xE000E180)
#define NVIC_ICER1		((__vo uint32_t *)0xE000E184)
#define NVIC_ICER2		((__vo uint32_t *)0xE000E188)
#define NVIC_ICER3		((__vo uint32_t *)0xE000E18C)
#define NVIC_ICER4		((__vo uint32_t *)0xE000E190)
#define NVIC_ICER5		((__vo uint32_t *)0xE000E194)
#define NVIC_ICER6		((__vo uint32_t *)0xE000E198)
#define NVIC_ICER7		((__vo uint32_t *)0xE000E19C)

/*
 * ARM Cortex Mx Processor Interrupt Priority Registers Addresses Calculation
 */

#define NVIC_PR_BASE_ADDR		((__vo uint32_t*)0xE000E400)

/*
 * ARM Cortex Mx Processor number of priority bits implemented in priority register
 */

#define NO_PR_BITS_IMPLEMENTED		4

/*
 * Base addresses of FLASH and SRAM memories.
 */
#define FLASH_BASEADDR					0x08000000U // base address of flash memory 512KB
#define SRAM1_BASEADDR					0x20000000U // base address of SRAM1 memory128KB
#define ROM_BASEADDR					0x1FFF0000U // base address of ROM memory
#define SRAM							SRAM1_BASEADDR

/*
 * AHBx and APBx Bus Peripheral base addresses
 */

#define PERIPH_BASEADDR					0x40000000U
#define APB1PERIPH_BASEADDR				PERIPH_BASEADDR
#define APB2PERIPH_BASEADDR				0x40010000U
#define AHB1PERIPH_BASEADDR				0x40020000U
#define AHB2PERIPH_BASEADDR				0x50000000U

/*
 * Base addresses of peripherals which are hanging on AHB1 bus
 */

#define GPIOA_BASEADDR					(AHB1PERIPH_BASEADDR + 0x0000)
#define GPIOB_BASEADDR					(AHB1PERIPH_BASEADDR + 0x0400)
#define GPIOC_BASEADDR					(AHB1PERIPH_BASEADDR + 0x0800)
#define GPIOD_BASEADDR					(AHB1PERIPH_BASEADDR + 0x0C00)
#define GPIOE_BASEADDR					(AHB1PERIPH_BASEADDR + 0x1000)
#define GPIOH_BASEADDR					(AHB1PERIPH_BASEADDR + 0x1C00)

#define RCC_BASEADDR					(AHB1PERIPH_BASEADDR + 0x3800)

/*
 * Base addresses of peripherals which are hanging on APB1 bus
 */

#define I2C1_BASEADDR					(APB1PERIPH_BASEADDR + 0x5400)
#define I2C2_BASEADDR					(APB1PERIPH_BASEADDR + 0x5800)
#define I2C3_BASEADDR					(APB1PERIPH_BASEADDR + 0x5C00)

#define SPI2_BASEADDR					(APB1PERIPH_BASEADDR + 0x3800)
#define SPI3_BASEADDR					(APB1PERIPH_BASEADDR + 0x3C00)

#define USART2_BASEADDR					(APB1PERIPH_BASEADDR + 0x4400)

/*
 * Base addresses of peripherals which are hanging on APB2 bus
 */

#define EXTI_BASEADDR					(APB2PERIPH_BASEADDR + 0x3C00)

#define USART1_BASEADDR					(APB2PERIPH_BASEADDR + 0x1000)
#define USART6_BASEADDR					(APB2PERIPH_BASEADDR + 0x1400)

#define SPI1_BASEADDR					(APB2PERIPH_BASEADDR + 0x3000)
#define SPI4_BASEADDR					(APB2PERIPH_BASEADDR + 0x3400)
#define SPI5_BASEADDR					(APB2PERIPH_BASEADDR + 0x5000)

#define SYSCFG_BASEADDR					(APB2PERIPH_BASEADDR + 0x3800)

/************************************peripheral register definition*************************************/
/*
 * Note : registers of a peripheral are specific to MCU
 * e.g. ^ Number of Registers of SPI peripheral of STM32F4x family of MCUs may be different(more or less)
 * Compared to number of registers of SPI peripheral of STM32Lx or STM32F0x family of MCUs
 * Please check your Device RM
 */

/*
 * Peripheral register definition of GPIO
 */
typedef struct
{
	__vo uint32_t MODER;		//GPIO port mode register
	__vo uint32_t OTYPER;		//GPIO port output type register
	__vo uint32_t OSPEEDR;		//GPIO port output speed register
	__vo uint32_t PUPDR;		//GPIO port pull-up/pull-down register
	__vo uint32_t IDR;			//GPIO port input data register
	__vo uint32_t ODR;			//GPIO port output data register
	__vo uint32_t BSRR;			//GPIO port bit set/reset register
	__vo uint32_t LCKR;			//GPIO port bit set/reset register
	__vo uint32_t AFR[2];		//AFR[0] : GPIO alternate function low register; AFR[1] : GPIO alternate function high register

}GPIO_RegDef_t;

/*
 * Peripheral register definition of RCC
 */
typedef struct
{
	__vo uint32_t CR;			//RCC clock control register
	__vo uint32_t PLLCFGR;		//RCC PLL configuration register
	__vo uint32_t CFGR;			//RCC clock configuration register
	__vo uint32_t CIR;			//RCC clock interrupt register
	__vo uint32_t AHB1RSTR;		//RCC AHB1 peripheral reset register
	__vo uint32_t AHB2RSTR;		//RCC AHB2 peripheral reset register
	uint32_t RESERVED0[2];		//0x18 and 0x1C are reserved
	__vo uint32_t APB1RSTR;		//RCC APB1 peripheral reset register
	__vo uint32_t APB2RSTR;		//RCC APB2 peripheral reset register
	uint32_t RESERVED1[2];		//0x28 and 0x2C are reserved
	__vo uint32_t AHB1ENR;		//RCC AHB1 peripheral clock enable register
	__vo uint32_t AHB2ENR;		//RCC AHB2 peripheral clock enable register
	uint32_t RESERVED2[2];		//0x38 and 0x3C are reserved
	__vo uint32_t APB1ENR;		//RCC APB1 peripheral clock enable register
	__vo uint32_t APB2ENR;		//RCC APB2 peripheral clock enable register
	uint32_t RESERVED3[2];		//0x48 and 0x4C are reserved
	__vo uint32_t AHB1LPENR;	//RCC AHB1 peripheral clock enable in low power mode register
	__vo uint32_t AHB2LPENR;	//RCC AHB2 peripheral clock enable in low power mode register
	uint32_t RESERVED4[2];		//0x58 and 0x5C are reserved
	__vo uint32_t APB1LPENR;	//RCC APB1 peripheral clock enable in low power mode register
	__vo uint32_t APB2LPENR;	//RCC APB2 peripheral clock enable in low power mode register
	uint32_t RESERVED5[2];		//0x68 and 0x6C are reserved
	__vo uint32_t BDCR;			//Backup domain control register
	__vo uint32_t CSR;			//RCC clock control & status register
	uint32_t RESERVED6[2];		//0x78 and 0x7C are reserved
	__vo uint32_t SSCGR;		//RCC spread spectrum clock generation register
	__vo uint32_t PLLI2SCFGR;	//RCC PLLI2S configuration register
	uint32_t RESERVED7;			//0x88 is reserved
	__vo uint32_t DCKCFGR;		//RCC Dedicated Clocks Configuration Register
}RCC_RegDef_t;

/*
 * Peripheral register definition of EXTI
 */
typedef struct
{
	__vo uint32_t IMR;		//Interrupt mask register (EXTI_IMR)
	__vo uint32_t EMR;		//Event mask register (EXTI_EMR)
	__vo uint32_t RTSR;		//Rising trigger selection register (EXTI_RTSR)
	__vo uint32_t FTSR;		//Falling trigger selection register (EXTI_FTSR)
	__vo uint32_t SWIER;	//Software interrupt event register (EXTI_SWIER)
	__vo uint32_t PR;		//Pending register (EXTI_PR)
}EXTI_RegDef_t;

/*
 * Peripheral register definition of SYSCFG
 */
typedef struct
{
	__vo uint32_t MEMRMP;		//SYSCFG memory remap register (SYSCFG_MEMRMP)
	__vo uint32_t PMC;			//SYSCFG peripheral mode configuration register (SYSCFG_PMC)
	__vo uint32_t EXTICR[4];	//SYSCFG external interrupt configuration registers (SYSCFG_EXTICR1, SYSCFG_EXTICR2, SYSCFG_EXTICR3, SYSCFG_EXTICR4)
	__vo uint32_t RESERVED[2];
	__vo uint32_t CMPCR;		//Compensation cell control register (SYSCFG_CMPCR)
}SYSCFG_RegDef_t;

/*
 * Peripheral register definition of SPI
 */
typedef struct
{
	__vo uint32_t CR1;			//SPI control register 1 (SPI_CR1)(not used in I2S mode)
	__vo uint32_t CR2;			//SPI control register 2 (SPI_CR2)
	__vo uint32_t SR;			//SPI status register (SPI_SR)
	__vo uint32_t DR;			//SPI data register (SPI_DR)
	__vo uint32_t CRCPR;		//SPI CRC polynomial register (SPI_CRCPR)(not used in I2S mode)
	__vo uint32_t RXCRCR;		//SPI RX CRC register (SPI_RXCRCR)(not used in I2S mode)
	__vo uint32_t TXCRCR;		//SPI TX CRC register (SPI_TXCRCR)(not used in I2S mode)
	__vo uint32_t I2SCFGR;		//SPI_I2S configuration register (SPI_I2SCFGR)
	__vo uint32_t I2SPR;		//SPI_I2S prescaler register (SPI_I2SPR)
}SPI_RegDef_t;

/*
 * Peripheral register definition of I2C
 */
typedef struct
{
	__vo uint32_t CR1;			//I2C Control register 1 (I2C_CR1)
	__vo uint32_t CR2;			//I2C Control register 2 (I2C_CR2)
	__vo uint32_t OAR1;			//I2C Own address register 1 (I2C_OAR1)
	__vo uint32_t OAR2;			//I2C Own address register 2 (I2C_OAR2)
	__vo uint32_t DR;			//I2C Data register (I2C_DR)
	__vo uint32_t SR1;			//I2C Status register 1 (I2C_SR1)
	__vo uint32_t SR2;			//I2C Status register 2 (I2C_SR2)
	__vo uint32_t CCR;			//I2C Clock control register (I2C_CCR)
	__vo uint32_t TRISE;		//I2C TRISE register (I2C_TRISE)
	__vo uint32_t FLTR;			//I2C FLTR register (I2C_FLTR)
}I2C_RegDef_t;

/*
 * peripheral definitions ( Peripheral base addresses typecasted to xxx_RegDef_t)
 */

#define GPIOA							((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB							((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC							((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD							((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE							((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOH							((GPIO_RegDef_t*)GPIOH_BASEADDR)

#define RCC								((RCC_RegDef_t*)RCC_BASEADDR)

#define EXTI							((EXTI_RegDef_t*)EXTI_BASEADDR)

#define SYSCFG							((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)

#define SPI1							((SPI_RegDef_t*)SPI1_BASEADDR)
#define SPI2							((SPI_RegDef_t*)SPI2_BASEADDR)
#define SPI3							((SPI_RegDef_t*)SPI3_BASEADDR)
#define SPI4							((SPI_RegDef_t*)SPI4_BASEADDR)
#define SPI5							((SPI_RegDef_t*)SPI5_BASEADDR)

#define I2C1							((I2C_RegDef_t*)I2C1_BASEADDR)
#define I2C2							((I2C_RegDef_t*)I2C2_BASEADDR)
#define I2C3							((I2C_RegDef_t*)I2C3_BASEADDR)

/*
 * Clock Enable Macros for GPIOx peripherals
 */

#define GPIOA_PCLK_EN()		( RCC->AHB1ENR |= ( 1 << 0 ) )
#define GPIOB_PCLK_EN()		( RCC->AHB1ENR |= ( 1 << 1 ) )
#define GPIOC_PCLK_EN()		( RCC->AHB1ENR |= ( 1 << 2 ) )
#define GPIOD_PCLK_EN()		( RCC->AHB1ENR |= ( 1 << 3 ) )
#define GPIOE_PCLK_EN()		( RCC->AHB1ENR |= ( 1 << 4 ) )
#define GPIOH_PCLK_EN()		( RCC->AHB1ENR |= ( 1 << 7 ) )

/*
 * Clock Enable Macros for I2Cx peripherals
 */

#define I2C1_PCLK_EN()		( RCC->APB1ENR |= ( 1 << 21 ) )
#define I2C2_PCLK_EN()		( RCC->APB1ENR |= ( 1 << 22 ) )
#define I2C3_PCLK_EN()		( RCC->APB1ENR |= ( 1 << 23 ) )

/*
 * Clock Enable Macros for SPIx peripherals
 */

#define SPI1_PCLK_EN()		( RCC->APB2ENR |= ( 1 << 12 ) )
#define SPI2_PCLK_EN()		( RCC->APB1ENR |= ( 1 << 14 ) )
#define SPI3_PCLK_EN()		( RCC->APB1ENR |= ( 1 << 15 ) )
#define SPI4_PCLK_EN()		( RCC->APB2ENR |= ( 1 << 13 ) )
#define SPI5_PCLK_EN()		( RCC->APB2ENR |= ( 1 << 20 ) )

/*
 * Clock Enable Macros for USARTx peripherals
 */
#define USART1_PCLK_EN()		( RCC->APB2ENR |= ( 1 << 4 ) )
#define USART2_PCLK_EN()		( RCC->APB1ENR |= ( 1 << 17 ) )
#define USART6_PCLK_EN()		( RCC->APB2ENR |= ( 1 << 5 ) )

/*
 * Clock Enable Macros for SYSCFG peripherals
 */

#define SYSCFG_PCLK_EN()		( RCC->APB2ENR |= ( 1 << 14 ) )

/*
 * Clock Disable Macros for GPIOx peripherals
 */

#define GPIOA_PCLK_DI()		( RCC->AHB1ENR &= ~( 1 << 0 ) )
#define GPIOB_PCLK_DI()		( RCC->AHB1ENR &= ~( 1 << 1 ) )
#define GPIOC_PCLK_DI()		( RCC->AHB1ENR &= ~( 1 << 2 ) )
#define GPIOD_PCLK_DI()		( RCC->AHB1ENR &= ~( 1 << 3 ) )
#define GPIOE_PCLK_DI()		( RCC->AHB1ENR &= ~( 1 << 4 ) )
#define GPIOH_PCLK_DI()		( RCC->AHB1ENR &= ~( 1 << 7 ) )

/*
 * Clock Disable Macros for I2Cx peripherals
 */

#define I2C1_PCLK_DI()		( RCC->APB1ENR &= ~( 1 << 21 ) )
#define I2C2_PCLK_DI()		( RCC->APB1ENR &= ~( 1 << 22 ) )
#define I2C3_PCLK_DI()		( RCC->APB1ENR &= ~( 1 << 23 ) )

/*
 * Clock Disable Macros for SPIx peripherals
 */

#define SPI1_PCLK_DI()		( RCC->APB2ENR &= ~( 1 << 12 ) )
#define SPI2_PCLK_DI()		( RCC->APB1ENR &= ~( 1 << 14 ) )
#define SPI3_PCLK_DI()		( RCC->APB1ENR &= ~( 1 << 15 ) )
#define SPI4_PCLK_DI()		( RCC->APB2ENR &= ~( 1 << 13 ) )
#define SPI5_PCLK_DI()		( RCC->APB2ENR &= ~( 1 << 20 ) )

/*
 * Clock Disable Macros for USARTx peripherals
 */

#define USART1_PCLK_DI()		( RCC->APB2ENR &= ~( 1 << 4 ) )
#define USART2_PCLK_DI()		( RCC->APB1ENR &= ~( 1 << 17 ) )
#define USART6_PCLK_DI()		( RCC->APB2ENR &= ~( 1 << 5 ) )

/*
 * Clock Disable Macros for SYSCFG peripherals
 */

#define SYSCFG_PCLK_DI()		( RCC->APB2ENR &= ~( 1 << 14 ) )

/*
 * Macros to reset GPIOxperipherals
 */

#define GPIOA_REG_RESET()		do { ( RCC->AHB1RSTR |= ( 1 << 0 ) ); ( RCC->AHB1RSTR &= ~( 1 << 0 ) ); } while(0)
#define GPIOB_REG_RESET()		do { ( RCC->AHB1RSTR |= ( 1 << 1 ) ); ( RCC->AHB1RSTR &= ~( 1 << 1 ) ); } while(0)
#define GPIOC_REG_RESET()		do { ( RCC->AHB1RSTR |= ( 1 << 2 ) ); ( RCC->AHB1RSTR &= ~( 1 << 2 ) ); } while(0)
#define GPIOD_REG_RESET()		do { ( RCC->AHB1RSTR |= ( 1 << 3 ) ); ( RCC->AHB1RSTR &= ~( 1 << 3 ) ); } while(0)
#define GPIOE_REG_RESET()		do { ( RCC->AHB1RSTR |= ( 1 << 4 ) ); ( RCC->AHB1RSTR &= ~( 1 << 4 ) ); } while(0)
#define GPIOH_REG_RESET()		do { ( RCC->AHB1RSTR |= ( 1 << 7 ) ); ( RCC->AHB1RSTR &= ~( 1 << 7 ) ); } while(0)

/*
 * Macros to reset SPIxperipherals
 */

#define SPI1_REG_RESET()		do { ( RCC->APB2RSTR |= ( 1 << 12 ) ); ( RCC->APB2RSTR &= ~( 1 << 12 ) ); } while(0)
#define SPI2_REG_RESET()		do { ( RCC->APB1RSTR |= ( 1 << 14 ) ); ( RCC->APB1RSTR &= ~( 1 << 14 ) ); } while(0)
#define SPI3_REG_RESET()		do { ( RCC->APB1RSTR |= ( 1 << 15 ) ); ( RCC->APB1RSTR &= ~( 1 << 15 ) ); } while(0)
#define SPI4_REG_RESET()		do { ( RCC->APB2RSTR |= ( 1 << 13 ) ); ( RCC->APB2RSTR &= ~( 1 << 13 ) ); } while(0)
#define SPI5_REG_RESET()		do { ( RCC->APB2RSTR |= ( 1 << 20 ) ); ( RCC->APB2RSTR &= ~( 1 << 20 ) ); } while(0)

/*
 * Macros to reset I2Cxperipherals
 */
#define I2C1_REG_RESET()		do { ( RCC->APB1RSTR |= ( 1 << 21 ) ); ( RCC->APB1RSTR &= ~( 1 << 21 ) ); } while(0)
#define I2C2_REG_RESET()		do { ( RCC->APB1RSTR |= ( 1 << 22 ) ); ( RCC->APB1RSTR &= ~( 1 << 22 ) ); } while(0)
#define I2C3_REG_RESET()		do { ( RCC->APB1RSTR |= ( 1 << 23 ) ); ( RCC->APB1RSTR &= ~( 1 << 23 ) ); } while(0)

/*
 * Return port code for given pGPIO base address
 */

#define GPIO_BASEADDR_TO_CODE(x)	( (x==GPIOA) ? 0:\
									  (x==GPIOB) ? 1:\
									  (x==GPIOC) ? 2:\
									  (x==GPIOD) ? 3:\
									  (x==GPIOE) ? 4:\
									  (x==GPIOH) ? 7:0 )

/*
 * IRQ (Interrupt request) Number of STM32F411x MCU
 */

#define IRQ_NO_EXTI0			6
#define IRQ_NO_EXTI1			7
#define IRQ_NO_EXTI2			8
#define IRQ_NO_EXTI3			9
#define IRQ_NO_EXTI4			10
#define IRQ_NO_EXTI9_5			23
#define IRQ_NO_SPI1				35
#define IRQ_NO_SPI2				36
#define IRQ_NO_EXTI15_10		40
#define IRQ_NO_SPI3				51
#define IRQ_NO_SPI4				84
#define IRQ_NO_SPI5				85

/*
 * Macros NVIC IRQ Priority levels
 */

#define NVIC_IRQ_PRIO0		0
#define NVIC_IRQ_PRIO1		1
#define NVIC_IRQ_PRIO2		2
#define NVIC_IRQ_PRIO3		3
#define NVIC_IRQ_PRIO4		4
#define NVIC_IRQ_PRIO5		5
#define NVIC_IRQ_PRIO6		6
#define NVIC_IRQ_PRIO7		7
#define NVIC_IRQ_PRIO8		8
#define NVIC_IRQ_PRIO9		9
#define NVIC_IRQ_PRIO10		10
#define NVIC_IRQ_PRIO11		11
#define NVIC_IRQ_PRIO12		12
#define NVIC_IRQ_PRIO13		13
#define NVIC_IRQ_PRIO14		14
#define NVIC_IRQ_PRIO15		15

/*
 * Generic macros
 */

#define ENABLE		1
#define DISABLE		0
#define SET			ENABLE
#define RESET		DISABLE
#define FLAG_SET	SET
#define FLAG_RESET	RESET

/****************************************************************
 * 		Bits position definitions of SPI peripheral				*
 ****************************************************************/

/*
 * Bit position definitions SPI_CR1
 */
#define SPI_CR1_CPHA		0
#define SPI_CR1_CPOL		1
#define SPI_CR1_MSTR		2
#define SPI_CR1_BR			3
#define SPI_CR1_SPE			6
#define SPI_CR1_LSBFIRST	7
#define SPI_CR1_SSI			8
#define SPI_CR1_SSM			9
#define SPI_CR1_RXONLY		10
#define SPI_CR1_DFF			11
#define SPI_CR1_CRCNEXT		12
#define SPI_CR1_CRCEN		13
#define SPI_CR1_BIDIOE		14
#define SPI_CR1_BIDIMODE	15

/*
 * Bit position definitions SPI_CR2
 */
#define SPI_CR2_RXDMAEN		0
#define SPI_CR2_TXDMAEN		1
#define SPI_CR2_SSOE		2
#define SPI_CR2_FRF			4
#define SPI_CR2_ERRIE		5
#define SPI_CR2_RXNEIE		6
#define SPI_CR2_TXEIE		7

/*
 * Bit position definitions SPI_SR
 */
#define SPI_SR_RXNE			0
#define SPI_SR_TXE			1
#define SPI_SR_CHSIDE		2
#define SPI_SR_UDR			3
#define SPI_SR_CRCERR		4
#define SPI_SR_MODF			5
#define SPI_SR_OVR			6
#define SPI_SR_BSY			7
#define SPI_SR_FRE			8

/****************************************************************
 * 		Bits position definitions of I2C peripheral				*
 ****************************************************************/

/*
 * Bit position definition I2C_CR1
 */
#define I2C_CR1_PE			0
#define I2C_CR1_SMBUS		1
#define I2C_CR1_SMBTYPE		3
#define I2C_CR1_ENARP		4
#define I2C_CR1_ENPEC		5
#define I2C_CR1_ENGC		6
#define I2C_CR1_NOSTRETCH	7
#define I2C_CR1_START		8
#define I2C_CR1_STOP		9
#define I2C_CR1_ACK			10
#define I2C_CR1_POS			11
#define I2C_CR1_PEC			12
#define I2C_CR1_ALERT		13
#define I2C_CR1_SWRST		15

/*
 * Bit position definition I2C_CR2
 */
#define I2C_CR2_FREQ		0
#define I2C_CR2_ITERREN		8
#define I2C_CR2_ITEVTEN		9
#define I2C_CR2_ITBUFEN		10
#define I2C_CR2_DMAEN		11
#define I2C_CR2_LAST		12

/*
 * Bit position definition I2C_SR1
 */
#define I2C_SR1_SB			0
#define I2C_SR1_ADDR		1
#define I2C_SR1_BTF			2
#define I2C_SR1_ADD10		3
#define I2C_SR1_STOPF		4
#define I2C_SR1_RxNE		6
#define I2C_SR1_TxE			7
#define I2C_SR1_BERR		8
#define I2C_SR1_ARLO		9
#define I2C_SR1_AF			10
#define I2C_SR1_OVR			11
#define I2C_SR1_PECERR		12
#define I2C_SR1_TIMEOUT		14
#define I2C_SR1_SMBALERT	15

/*
 * Bit position definition I2C_SR2
 */
#define I2C_SR2_MSL			0
#define I2C_SR2_BUSY		1
#define I2C_SR2_TRA			2
#define I2C_SR2_GENCALL		4
#define I2C_SR2_SMBDEFAULT	5
#define I2C_SR2_SMBHOST		6
#define I2C_SR2_DUALF		7
#define I2C_SR2_PEC			8

/*
 * Bit position definition I2C_CCR
 */
#define I2C_CCR_CCR			0
#define I2C_CCR_DUTY		14
#define I2C_CCR_F_S			15




#include "stm32f411_gpio_driver.h"
#include "stm32f411xx_SPI_driver.h"
#include "stm32f411xx_I2C_driver.h"

#endif /* INC_STM32FXX_H_ */
