/*
 * drv_gpio.c
 *
 *  Created on: Apr 30, 2020
 *      Author: vanhoang
 */


#include "drv_gpio.h"

static void EnableClockGPIO(GPIO_RegDef_t *pGPIOx, uint8_t isEnable) {
	if (isEnable) {
		if (pGPIOx == GPIOA) {
			ENABLE_PCLK_GPIOA();
		} else if (pGPIOx == GPIOB) {
			ENABLE_PCLK_GPIOB();
		} else if (pGPIOx == GPIOC) {
			ENABLE_PCLK_GPIOC();
		} else if (pGPIOx == GPIOD) {
			ENABLE_PCLK_GPIOD();
		} else if (pGPIOx == GPIOE) {
			ENABLE_PCLK_GPIOE();
		} else if (pGPIOx == GPIOH) {
			ENABLE_PCLK_GPIOH();
		}
	} else {
		// do nothing
	}

}

void GPIO_Init(GPIO_Handle_t *pGPIOx) {
	uint32_t temp;
	uint8_t temp1;
	uint8_t temp2;

	//1. enable clock GPIO
	EnableClockGPIO(pGPIOx->pGPIO, TRUE);

	//2. set pin mode for pin
	if (pGPIOx->pinConfig.pinMode <= GPIO_PINMODE_ANALOG) {
		temp = pGPIOx->pinConfig.pinMode << (2 * pGPIOx->pinConfig.pinNum);
		pGPIOx->pGPIO->MODER &= ~(3 << pGPIOx->pinConfig.pinNum);
		pGPIOx->pGPIO->MODER |= temp;
	} else { // setting interrupt mode
		if (pGPIOx->pinConfig.pinMode == GPIO_PINMODE_IT_FALL) {				// configure interrupt mode with falling edge
			EXTI->FTSR |= (1 << pGPIOx->pinConfig.pinNum);
			EXTI->RTSR &= ~(1 << pGPIOx->pinConfig.pinNum);

		} else if (pGPIOx->pinConfig.pinMode == GPIO_PINMODE_IT_RISE) {			// configure interrupt mode with rising edge
			EXTI->FTSR &= ~(1 << pGPIOx->pinConfig.pinNum);
			EXTI->RTSR |= (1 << pGPIOx->pinConfig.pinNum);

		} else if (pGPIOx->pinConfig.pinMode == GPIO_PINMODE_IT_RISE_FALL) {	// configure interrupt mode with falling/rising edge
			EXTI->FTSR |= (1 << pGPIOx->pinConfig.pinNum);
			EXTI->RTSR |= (1 << pGPIOx->pinConfig.pinNum);
		}

		temp1 = pGPIOx->pinConfig.pinNum / 4;
		temp2 = pGPIOx->pinConfig.pinNum % 4;
		SYSCFG->EXTICR[temp1] |= (PORT_CODE(pGPIOx->pGPIO) << (4 * temp2));
	}

	temp1 = pGPIOx->pinConfig.pinNum / 8;
	temp2 = pGPIOx->pinConfig.pinNum % 8;

	//3. set resistor type for pin
	pGPIOx->pGPIO->PUPDR |= pGPIOx->pinConfig.resPull << (2 * pGPIOx->pinConfig.pinNum);

	//4. set output type for pin
	pGPIOx->pGPIO->OTYPER |= pGPIOx->pinConfig.outType << pGPIOx->pinConfig.pinNum;

	//5. set output speed for pin
	pGPIOx->pGPIO->OSPEEDR |= pGPIOx->pinConfig.outSpeed << (2 * pGPIOx->pinConfig.pinNum);

	//6. set alternate function mode for pin
	pGPIOx->pGPIO->AFR[temp1] |= pGPIOx->pinConfig.altFunc << (4 * temp2);
}

void GPIO_DeInit(GPIO_RegDef_t *pGPIOx) {
	if (pGPIOx == GPIOA) {
		RESET_GPIOA();
	} else if (pGPIOx == GPIOB) {
		RESET_GPIOB();
	} else if (pGPIOx == GPIOC) {
		RESET_GPIOC();
	} else if (pGPIOx == GPIOD) {
		RESET_GPIOD();
	} else if (pGPIOx == GPIOE) {
		RESET_GPIOE();
	}else if (pGPIOx == GPIOH) {
		RESET_GPIOH();
	}
}

void GPIO_WritePin(GPIO_RegDef_t *pGPIOx, uint8_t pin, uint8_t value) {
	if (value == TRUE) {
		pGPIOx->ODR |= 1 << pin;
	} else if (value == FALSE) {
		pGPIOx->ODR &= ~(1 << pin);
	}
}

void GPIO_WritePort(GPIO_RegDef_t *pGPIOx, uint8_t value) {
	pGPIOx->ODR = value;
}

uint16_t GPIO_ReadPin(GPIO_RegDef_t *pGPIOx, uint8_t pin) {
	return (uint16_t)(pGPIOx->IDR & (1 << pin));
}

uint16_t GPIO_ReadPort(GPIO_RegDef_t *pGPIOx) {
	return (uint16_t)(pGPIOx->IDR);
}

void GPIO_ConfigIRQ(uint8_t IRQNum, uint8_t isEnable) {
	uint8_t temp1 = IRQNum / 32;
	uint8_t temp2 = IRQNum % 32;

	if (isEnable) {
		*(NVIC_ISER + temp1) |= (1 << temp2);
	} else {
		*(NVIC_ICER + temp1) |= (1 << temp2);
	}

}

void GPIO_SetPriorityIRQ(uint8_t IRQNum, uint8_t priority) {
	uint8_t temp1 = IRQNum / 4;
	uint8_t temp2 = IRQNum % 4;

	*(NVIC_IPR + temp1) |= priority << (8*temp2 + NO_IMPLEMENT_PRIORITY_BITS);
}
void GPIO_IRQHandler(uint8_t pin) {
	if (EXTI->PR & (1 << pin)) {
		EXTI->PR |= 1 << pin;
	}
}

