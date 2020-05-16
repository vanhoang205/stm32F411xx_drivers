/*
 * drv_gpio.h
 *
 *  Created on: Apr 30, 2020
 *      Author: vanhoang
 */

#ifndef INC_DRV_GPIO_H_
#define INC_DRV_GPIO_H_


#include "stm32f411xx.h"

/*
 * Configuration structure for GPIOx peripheral
 */
typedef struct {
	uint8_t pinNum;
	uint8_t pinMode;
	uint8_t outType;
	uint8_t	outSpeed;
	uint8_t resPull;
	uint8_t altFunc;
} GPIO_Config_t;


/*
 * Handle structure for I2Cx peripheral
 */
typedef struct {
	GPIO_RegDef_t *pGPIO;
	GPIO_Config_t pinConfig;
} GPIO_Handle_t;


/**
 * @pinNum : parameter for "pinNum" attribute
 */

#define GPIO_PINNUM_0                 0  	/* Pin 0 selected    */
#define GPIO_PINNUM_1                 1  	/* Pin 1 selected    */
#define GPIO_PINNUM_2                 2  	/* Pin 2 selected    */
#define GPIO_PINNUM_3                 3 	/* Pin 3 selected    */
#define GPIO_PINNUM_4                 4  	/* Pin 4 selected    */
#define GPIO_PINNUM_5                 5		/* Pin 5 selected    */
#define GPIO_PINNUM_6                 6  	/* Pin 6 selected    */
#define GPIO_PINNUM_7                 7 	/* Pin 7 selected    */
#define GPIO_PINNUM_8                 8		/* Pin 8 selected    */
#define GPIO_PINNUM_9                 9		/* Pin 9 selected    */
#define GPIO_PINNUM_10                10 	/* Pin 10 selected   */
#define GPIO_PINNUM_11                11	/* Pin 11 selected   */
#define GPIO_PINNUM_12                12	/* Pin 12 selected   */
#define GPIO_PINNUM_13                13	/* Pin 13 selected   */
#define GPIO_PINNUM_14                14	/* Pin 14 selected   */
#define GPIO_PINNUM_15                15	/* Pin 15 selected   */

#define GPIO_PINNUM_MASK              0x0000FFFFU /* PIN mask for assert test */


/**
 * @pinMode : parameter for "pinMode" attribute
 */
#define GPIO_PINMODE_INPUT			0x00		/*!< Input Mode 	  							 */
#define GPIO_PINMODE_OUTPUT			0x01		/*!< Output Mode 	  							 */
#define GPIO_PINMODE_ALT			0x02		/*!< Alternative Mode 							 */
#define GPIO_PINMODE_ANALOG			0x03		/*!< Analog Mode 	  							 */
#define GPIO_PINMODE_IT_RISE		0x04		/*!< Interrupt Mode with rising edge 			 */
#define GPIO_PINMODE_IT_FALL		0x05		/*!< Interrupt Mode with falling edge 			 */
#define GPIO_PINMODE_IT_RISE_FALL	0x06		/*!< Interrupt Mode with rising or falling edge  */

/**
 * @outType : parameter for "outType" attribute
 */
#define GPIO_OUTTYPE_PUPU			0x00		/*!< Low speed	  		*/
#define GPIO_OUTTYPE_OD				0x01		/*!< Medium speed 	  	*/


/**
 * @outputSpeed : parameter for "outSpeed" attribute
 */
#define GPIO_OUTSPEED_LOW			0x00		/*!< Low speed	  		*/
#define GPIO_OUTSPEED_MEDIUM		0x01		/*!< Medium speed 	  	*/
#define GPIO_OUTSPEED_HIGH			0x02		/*!< High speed 	  	*/
#define GPIO_OUTSPEED_VERYHIGH		0x03		/*!< Very high speed 	*/

/**
 * @resPull : parameter for "resPull" attribute
 */
#define GPIO_RESPULL_NONE			0x00		/*!< No register	  	*/
#define GPIO_RESPULL_UP				0x01		/*!< pull-up register 	*/
#define GPIO_RESPULL_DOWN			0x02		/*!< pull-down register */


/**
 * @altFunc : parameter for "altFunc" attribute
 */
#define GPIO_ALTFUNC_0			0		/*!< Select AF0	  	*/
#define GPIO_ALTFUNC_1			1		/*!< Select AF1 	*/
#define GPIO_ALTFUNC_2			2		/*!< Select AF2 	*/
#define GPIO_ALTFUNC_3			3		/*!< Select AF3 	*/
#define GPIO_ALTFUNC_4			4		/*!< Select AF4 	*/
#define GPIO_ALTFUNC_5			5		/*!< Select AF5 	*/
#define GPIO_ALTFUNC_6			6		/*!< Select AF6 	*/
#define GPIO_ALTFUNC_7			7		/*!< Select AF7 	*/
#define GPIO_ALTFUNC_8			8		/*!< Select AF8 	*/
#define GPIO_ALTFUNC_9			9		/*!< Select AF9 	*/
#define GPIO_ALTFUNC_10			10		/*!< Select AF10	*/
#define GPIO_ALTFUNC_11			11		/*!< Select AF11 	*/
#define GPIO_ALTFUNC_12			12		/*!< Select AF12 	*/
#define GPIO_ALTFUNC_13			13		/*!< Select AF13 	*/
#define GPIO_ALTFUNC_14			14		/*!< Select AF14	*/
#define GPIO_ALTFUNC_15			15		/*!< Select AF15	*/



void GPIO_Init(GPIO_Handle_t *pGPIOx);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

void GPIO_WritePin(GPIO_RegDef_t *pGPIOx, uint8_t pin, uint8_t value);
void GPIO_WritePort(GPIO_RegDef_t *pGPIOx, uint8_t value);
uint16_t GPIO_ReadPin(GPIO_RegDef_t *pGPIOx, uint8_t pin);
uint16_t GPIO_ReadPort(GPIO_RegDef_t *pGPIOx);

void GPIO_ConfigIRQ(uint8_t IRQNum, uint8_t isEnable);
void GPIO_SetPriorityIRQ(uint8_t IRQNum, uint8_t priority);
void GPIO_IRQHandler(uint8_t pin);





#endif /* INC_DRV_GPIO_H_ */
