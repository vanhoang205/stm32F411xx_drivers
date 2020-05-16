/*
 * stm32f411xx.h
 *
 *  Created on: Apr 30, 2020
 *      Author: vanhoang
 */

#ifndef INC_STM32F411XX_H_
#define INC_STM32F411XX_H_


#include "stdint.h"
#include "stddef.h"
/******************************************************************************/
/*                         Macro essential						              */
/******************************************************************************/


/*!< define macro atribute */
#define __vo			volatile
#define __weak __attribute__((weak))

#define TRUE			1
#define FALSE			0
#define SET				TRUE
#define RESET			FALSE
#define FLAG_SET		TRUE
#define FLAG_RESET		FALSE

#define NO_IMPLEMENT_PRIORITY_BITS 		4

/*!< define priority of interrupt */
#define PRIORITY_NO_0		0
#define PRIORITY_NO_1		1
#define PRIORITY_NO_2		2
#define PRIORITY_NO_3		3
#define PRIORITY_NO_4		4
#define PRIORITY_NO_5		5
#define PRIORITY_NO_6		6
#define PRIORITY_NO_7		7
#define PRIORITY_NO_8		8
#define PRIORITY_NO_9		9
#define PRIORITY_NO_10		10
#define PRIORITY_NO_11		11
#define PRIORITY_NO_12		12
#define PRIORITY_NO_13		13
#define PRIORITY_NO_14		14
#define PRIORITY_NO_15		15

/******************************************************************************/
/*                         Base Address Macro Of Peripheral		              */
/******************************************************************************/

/*!< NVIC_ISER memory map - using cortex M4 */
#define NVIC_ISER_BASE		0xE000E100UL

/*!< NVIC_ICER memory map - using cortex M4 */
#define NVIC_ICER_BASE		0xE000E180UL

/*!< NVIC_IPR memory map - using cortex M4 */
#define NVIC_IPR_BASE		0xE000E400UL

/*!< Bus memory map */
#define APB1PERIPH_BASE		0x40000000UL
#define APB2PERIPH_BASE		0x40010000UL
#define AHB1PERIPH_BASE		0x40020000UL
#define AHB2PERIPH_BASE		0x50000000UL


/*!< AHB1 Peripheral memory map */
#define GPIOA_BASE			(AHB1PERIPH_BASE + 0x0000UL)
#define GPIOB_BASE			(AHB1PERIPH_BASE + 0x0400UL)
#define GPIOC_BASE			(AHB1PERIPH_BASE + 0x0800UL)
#define GPIOD_BASE			(AHB1PERIPH_BASE + 0x0C00UL)
#define GPIOE_BASE			(AHB1PERIPH_BASE + 0x1000UL)
#define GPIOH_BASE			(AHB1PERIPH_BASE + 0x1C00UL)

#define RCC_BASE			(AHB1PERIPH_BASE + 0x3800UL)


/*!< APB2 Peripheral memory map */
#define EXTI_BASE			(APB2PERIPH_BASE + 0x3C00UL)

#define SYSCFG_BASE			(APB2PERIPH_BASE + 0x3800UL)

#define I2C1_BASE			(APB1PERIPH_BASE + 0x5400UL)
#define I2C2_BASE			(APB1PERIPH_BASE + 0x5800UL)
#define I2C3_BASE			(APB1PERIPH_BASE + 0x5C00UL)

/******************************************************************************/
/*                 Peripheral Register Definition Structure	                  */
/******************************************************************************/

/*!< GPIO Peripheral Structure */
typedef struct {
	__vo uint32_t MODER;	/*!< GPIO port mode register, 						Address offset: 0x00 */
	__vo uint32_t OTYPER;	/*!< GPIO port output type register, 				Address offset: 0x04*/
	__vo uint32_t OSPEEDR;	/*!< GPIO port output speed register, 				Address offset: 0x08*/
	__vo uint32_t PUPDR;	/*!< GPIO port pull-up/pull-down register, 			Address offset: 0x0C*/
	__vo uint32_t IDR;		/*!< GPIO port input data register, 				Address offset: 0x10*/
	__vo uint32_t ODR;		/*!< GPIO port output data register, 				Address offset: 0x14*/
	__vo uint32_t BSRR;		/*!< GPIO port bit set/reset register, 				Address offset: 0x18*/
	__vo uint32_t LCKR;		/*!< GPIO port configuration lock register, 		Address offset: 0x1C */
	__vo uint32_t AFR[2];	/*!< GPIO alternate function register, 				Address offset: 0x20 & 0x24 */
} GPIO_RegDef_t;


/*!< RCC Peripheral Structure */
typedef struct {
	__vo uint32_t CR;				/*!< RCC clock control register, 									Address offset: 0x00 */
	__vo uint32_t PLLCFGR;			/*!< RCC PLL configuration register, 								Address offset: 0x04 */
	__vo uint32_t CFGR;				/*!< RCC clock configuration register, 								Address offset: 0x08 */
	__vo uint32_t CIR;				/*!< RCC clock interrupt register, 									Address offset: 0x0C */
	__vo uint32_t AHB1RSTR;			/*!< RCC AHB1 peripheral reset register, 							Address offset: 0x10 */
	__vo uint32_t AHB2RSTR;			/*!< RCC AHB2 peripheral reset register, 							Address offset: 0x14 */
	uint32_t Reserved0;
	uint32_t Reserved1;
	__vo uint32_t APB1RSTR;			/*!< RCC APB1 peripheral reset register , 							Address offset: 0x20 */
	__vo uint32_t APB2RSTR;			/*!< RCC APB2 peripheral reset register, 							Address offset: 0x24 */
	uint32_t Reserved2;
	uint32_t Reserved3;
	__vo uint32_t AHB1ENR;			/*!< RCC AHB1 peripheral clock enable register, 					Address offset: 0x30 */
	__vo uint32_t AHB2ENR;			/*!< RCC AHB2 peripheral clock enable register, 					Address offset: 0x34 */
	uint32_t Reserved4;
	uint32_t Reserved5;
	__vo uint32_t APB1ENR;			/*!< RCC APB1 peripheral clock enable register, 					Address offset: 0x40 */
	__vo uint32_t APB2ENR;			/*!< RCC APB2 peripheral clock enable register, 					Address offset: 0x44 */
	uint32_t Reserved6;
	uint32_t Reserved7;
	__vo uint32_t AHB1LPENR;		/*!< RCC AHB1 peripheral clock enable in low power mode register, 	Address offset: 0x50 */
	__vo uint32_t AHB2LPENR;		/*!< RCC AHB2 peripheral clock enable in low power mode register, 	Address offset: 0x54 */
	uint32_t Reserved8;
	uint32_t Reserved9;
	__vo uint32_t APB1LPENR;		/*!< RCC APB1 peripheral clock enable in low power mode register, 	Address offset: 0x60 */
	__vo uint32_t APB2LPENR;		/*!< RCC APB2 peripheral clock enabled in low power mode register, 	Address offset: 0x64 */
	uint32_t Reserved10;
	uint32_t Reserved11;
	__vo uint32_t BDCR;				/*!< RCC Backup domain control register, 							Address offset: 0x70 */
	__vo uint32_t CSR;				/*!< RCC clock control & status register, 							Address offset: 0x74 */
	uint32_t Reserved12;
	uint32_t Reserved13;
	__vo uint32_t RCC_SSCGR;		/*!< RCC spread spectrum clock generation register, 				Address offset: 0x80 */
	__vo uint32_t PLLI2SCFGR;		/*!< RCC PLLI2S configuration register, 							Address offset: 0x84 */
	uint32_t Reserved14;
	__vo uint32_t DCKCFGR;			/*!< RCC Dedicated Clock Configuration Register, 					Address offset: 0x8C */
} RCC_RegDef_t;


/*!< EXTI Peripheral Structure */
typedef struct {
	__vo uint32_t IMR;				/*!< Interrupt mask register, 				Address offset: 0x00 */
	__vo uint32_t EMR;				/*!< Event mask register, 					Address offset: 0x04*/
	__vo uint32_t RTSR;				/*!< Rising trigger selection register, 	Address offset: 0x08*/
	__vo uint32_t FTSR;				/*!< Falling trigger selection register, 	Address offset: 0x0C*/
	__vo uint32_t SWIER;			/*!< Software interrupt event register, 	Address offset: 0x10*/
	__vo uint32_t PR;				/*!< Pending register, 						Address offset: 0x14*/
} EXTI_RegDef_t;


/*!< SYSCFG Peripheral Structure */
typedef struct {
	__vo uint32_t MEMRMP;			/*!< Memory remap register, 								Address offset: 0x00 */
	__vo uint32_t PMC;				/*!< Peripheral mode configuration register, 				Address offset: 0x04*/
	__vo uint32_t EXTICR[4];		/*!< SYSCFG external interrupt configuration register, 		Address offset: 0x08*/
	uint32_t RESERVER1;
	uint32_t RESERVER2;
	__vo uint32_t CMPCR;			/*!< Falling trigger selection register, 					Address offset: 0x0C*/
} SYSCFG_RegDef_t;

typedef struct {
	__vo uint32_t CR[2];			/*!< Control register, 				Address offset: 0x00-0x04 */
	__vo uint32_t OAR[2];			/*!< Own address register, 			Address offset: 0x08-0x0C*/
	__vo uint32_t DR;				/*!< Data register , 				Address offset: 0x10*/
	__vo uint32_t SR[2];			/*!< Status register, 				Address offset: 0x14-0x18*/
	__vo uint32_t CCR;				/*!< Clock control register, 		Address offset: 0x1C*/
	__vo uint32_t TRISE;			/*!< TRISE register , 				Address offset: 0x20*/
	__vo uint32_t FLTR;				/*!< FLTR register, 				Address offset: 0x24*/
} I2C_RegDef_t;
/******************************************************************************/
/*                         Peripheral Declaration	                  		  */
/******************************************************************************/

/*!< GPIO declaration */
#define GPIOA				((GPIO_RegDef_t *)GPIOA_BASE)
#define GPIOB				((GPIO_RegDef_t *)GPIOB_BASE)
#define GPIOC				((GPIO_RegDef_t *)GPIOC_BASE)
#define GPIOD				((GPIO_RegDef_t *)GPIOD_BASE)
#define GPIOE				((GPIO_RegDef_t *)GPIOE_BASE)
#define GPIOH				((GPIO_RegDef_t *)GPIOH_BASE)


/*!< RCC declaration */
#define RCC					((RCC_RegDef_t *)RCC_BASE)


/*!< EXTI declaration */
#define EXTI				((EXTI_RegDef_t *)EXTI_BASE)

/*!< SYSCFG declaration */
#define SYSCFG				((SYSCFG_RegDef_t *)SYSCFG_BASE)

/*!< I2C declaration */
#define I2C1				((I2C_RegDef_t *)I2C1_BASE)
#define I2C2				((I2C_RegDef_t *)I2C2_BASE)
#define I2C3				((I2C_RegDef_t *)I2C3_BASE)

/*!< NVIC declaration */
#define NVIC_ISER		((__vo uint32_t*)NVIC_ISER_BASE)
#define NVIC_ICER		((__vo uint32_t*)NVIC_ICER_BASE)
#define NVIC_IPR		((__vo uint32_t*)NVIC_IPR_BASE)

/******************************************************************************/
/*                         Define Macro function 						      */
/******************************************************************************/

/*!< Define macro function to enable/disable CLK for driver */
#define ENABLE_PCLK_GPIOA()			(RCC->AHB1ENR |= 1 << 0)
#define ENABLE_PCLK_GPIOB()			(RCC->AHB1ENR |= 1 << 1)
#define ENABLE_PCLK_GPIOC()			(RCC->AHB1ENR |= 1 << 2)
#define ENABLE_PCLK_GPIOD()			(RCC->AHB1ENR |= 1 << 3)
#define ENABLE_PCLK_GPIOE()			(RCC->AHB1ENR |= 1 << 4)
#define ENABLE_PCLK_GPIOH()			(RCC->AHB1ENR |= 1 << 7)

#define DISABLE_PCLK_GPIOA()		(RCC->AHB1ENR &= ~(1 << 0))
#define DISABLE_PCLK_GPIOB()		(RCC->AHB1ENR &= ~(1 << 1))
#define DISABLE_PCLK_GPIOC()		(RCC->AHB1ENR &= ~(1 << 2))
#define DISABLE_PCLK_GPIOD()		(RCC->AHB1ENR &= ~(1 << 3))
#define DISABLE_PCLK_GPIOE()		(RCC->AHB1ENR &= ~(1 << 4))
#define DISABLE_PCLK_GPIOH()		(RCC->AHB1ENR &= ~(1 << 7))


#define ENABLE_PCLK_I2C1()			(RCC->APB1ENR |= (1 << 21))
#define ENABLE_PCLK_I2C2()			(RCC->APB1ENR |= (1 << 22))
#define ENABLE_PCLK_I2C3()			(RCC->APB1ENR |= (1 << 23))

#define DISABLE_PCLK_I2C1()			(RCC->APB1ENR &= ~(1 << 21))
#define DISABLE_PCLK_I2C2()			(RCC->APB1ENR &= ~(1 << 22))
#define DISABLE_PCLK_I2C3()			(RCC->APB1ENR &= ~(1 << 23))


/*!< Define macro function to reset value GPIO driver */
#define RESET_GPIOA()				do {RCC->AHB1RSTR |= 1 << 0; RCC->AHB1RSTR &= ~(1 << 0);}while(0)
#define RESET_GPIOB()				do {RCC->AHB1RSTR |= 1 << 1; RCC->AHB1RSTR &= ~(1 << 1);}while(0)
#define RESET_GPIOC()				do {RCC->AHB1RSTR |= 1 << 2; RCC->AHB1RSTR &= ~(1 << 2);}while(0)
#define RESET_GPIOD()				do {RCC->AHB1RSTR |= 1 << 3; RCC->AHB1RSTR &= ~(1 << 3);}while(0)
#define RESET_GPIOE()				do {RCC->AHB1RSTR |= 1 << 4; RCC->AHB1RSTR &= ~(1 << 4);}while(0)
#define RESET_GPIOH()				do {RCC->AHB1RSTR |= 1 << 7; RCC->AHB1RSTR &= ~(1 << 7);}while(0)

/*!< define port code for EXTI x configuration */
#define PORT_CODE(x)			   ((x==GPIOA)?0:\
									(x==GPIOB)?1:\
									(x==GPIOC)?2:\
									(x==GPIOD)?3:\
									(x==GPIOE)?4:\
									(x==GPIOH)?7:0)

/******************************************************************************/
/*                      Bit position definition of peripheral                 */
/******************************************************************************/

/*
 * Bit position fefinition for I2C peripheral
 */
#define I2C_CR1_PE				0
#define I2C_CR1_NOSTRETH		7
#define I2C_CR1_START			8
#define I2C_CR1_STOP			9
#define I2C_CR1_ACK				10
#define I2C_CR1_POS				11
#define I2C_CR1_PEC				12
#define I2C_CR1_SWRST			15

#define I2C_CR2_FREQ_START		0
#define I2C_CR2_ITERREN			8
#define I2C_CR2_ITEVTEN			9
#define I2C_CR2_ITBUFEN			10

#define I2C_SR1_SB				0
#define I2C_SR1_ADDR			1
#define I2C_SR1_BTF				2
#define I2C_SR1_STOPF			4
#define I2C_SR1_RXNE			6
#define I2C_SR1_TXE				7
#define I2C_SR1_BERR			8
#define I2C_SR1_ARLO			9
#define I2C_SR1_AF				10
#define I2C_SR1_OVR				11
#define I2C_SR1_TIMEOUT			14

#define I2C_SR2_MSL				0
#define I2C_SR2_BUSY			1
#define I2C_SR2_TRA				2
#define I2C_SR2_GENCALL			4
#define I2C_SR2_DUALF			7

#define I2C_CCR_CCR_STAR		0
#define I2C_CCR_DUTY			14
#define I2C_CCR_FS				15
/******************************************************************************/
/*                         Define IRQ number of stm32f407xx	                  */
/******************************************************************************/

/*
 * EXTI IRQ number
 */
#define IRQ_NO_EXTI0			6
#define IRQ_NO_EXTI1			7
#define IRQ_NO_EXTI2			8
#define IRQ_NO_EXTI3			9
#define IRQ_NO_EXTI4			10
#define IRQ_NO_EXTI9_5			23
#define IRQ_NO_EXTI15_10		40

/*
 * I2C IRQ number
 */
#define IRQ_NO_I2C1_EV			31
#define IRQ_NO_I2C1_ER			32

#define IRQ_NO_I2C2_EV			33
#define IRQ_NO_I2C2_ER			34

#define IRQ_NO_I2C3_EV			72
#define IRQ_NO_I2C4_ER			73

#endif /* INC_STM32F411XX_H_ */
