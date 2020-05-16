/*
 * drv_i2c.h
 *
 *  Created on: Apr 30, 2020
 *      Author: vanhoang
 */

#ifndef INC_DRV_I2C_H_
#define INC_DRV_I2C_H_

#include "stm32f411xx.h"
typedef struct {
	uint32_t sclSpeed;
	uint8_t deviceAddress;		/*!< Master or Slave device >*/
	uint8_t ACKControl;
	uint8_t dutyCycleFM;
}I2C_Config_t;

typedef struct {
	I2C_Config_t config;		/*!< Configure structure 			*/
	I2C_RegDef_t *pI2C;			/*!< point to I2C memory map 		*/
	uint8_t currentStatus;		/*!< Ready/Tx/Rx status 			*/
	uint32_t RxSize;			/*!< Current size of receive data 	*/
	uint32_t TxLen;				/*!< Length data to transmit		*/
	uint32_t RxLen;				/*!< Length data to receive			*/
	uint8_t devAddr;			/*!< Device address of slave		*/
	uint8_t isRestared;			/*!< Is restare enable 				*/
	uint8_t *pTxBuffer;			/*!< Pointer buffer to transfer		*/
	uint8_t *pRxBuffer;			/*!< Pointer buffer to receive		*/
}I2C_Handle_t;

/*
 * @status
 */
#define I2C_STATUS_READY		0
#define I2C_STATUS_IN_RX		1
#define I2C_STATUS_IN_TX		2
#define I2C_STATUS_COMM_CMPLT	3

/*
 * @sclSpeed
 */
#define I2C_SCLSPEED_SM		100000
#define I2C_SCLSPEED_FM2K	200000
#define I2C_SCLSPEED_FM4K	400000

/*
 * @ACKControl
 */
#define I2C_ACKCONTROL_ENABLE	1
#define I2C_ACKCONTROL_DISABLE	0

/*
 * @dutyCycleFM
 */
#define I2C_DUTYCYCLEFM_2		0
#define I2C_DUTYCYCLEFM_16_9	1

/*
 * Enable control bit
 */
#define I2C_CONTROL_ITEVTEN		(1 << I2C_CR2_ITEVTEN)
#define I2C_CONTROL_ITBUFEN		(1 << I2C_CR2_ITBUFEN)
#define I2C_CONTROL_ITERREN		(1 << I2C_CR2_ITERREN)

/*
 * status flag
 */
#define I2C_FLAG_SB				I2C_SR1_SB
#define I2C_FLAG_ADDR			I2C_SR1_ADDR
#define I2C_FLAG_STOPF			I2C_SR1_STOPF
#define I2C_FLAG_BTF			I2C_SR1_BTF
#define I2C_FLAG_RXNE			I2C_SR1_RXNE
#define I2C_FLAG_TXE			I2C_SR1_TXE

#define I2C_FLAG_BERR			I2C_SR1_BERR
#define I2C_FLAG_AF				I2C_SR1_AF
#define I2C_FLAG_OVR			I2C_SR1_OVR

#define I2C_CMD_READ			1
#define I2C_CMD_WRITE			0

/*
 * I2C application events macros
 */
#define I2C_EV_DATA_TX_CMPLT		0
#define I2C_EV_DATA_RX_CMPLT		1
#define I2C_EV_STOP					2
#define I2C_ER_BERR					3
#define I2C_ER_ARLO					4
#define I2C_ER_AF					5
#define I2C_ER_OVR					6
#define I2C_ER_TIMEOUT				7
#define I2C_EV_DATA_REQ				8
#define I2C_EV_DATA_RCV				9

// Initialize/ deinitialilze I2C peripheral
void I2C_Init(I2C_Handle_t *pI2CHandle);
void I2C_DeInit(I2C_RegDef_t *pI2Cx);

// Enable clock and operate for I2C peripheral
void I2C_EnableClockPeri(I2C_RegDef_t *pI2Cx, uint8_t isEnable);
void I2C_EnablePeripheral(I2C_RegDef_t *pI2Cx);

// Transmit and Receive function from Master device (Blocking mode)
void I2C_MasterTransmit(I2C_Handle_t *pI2CHandle, uint8_t slaveAddr, uint8_t *pTxBuffer, uint8_t len, uint8_t isRestared);
void I2C_MasterReceive(I2C_Handle_t *pI2CHandle, uint8_t slaveAddr, uint8_t *pRxBuffer, uint8_t len, uint8_t isReStarted);

// Transmit and Receive function from Master device (Non-Blocking mode)
uint8_t I2C_MasterTransmitIT(I2C_Handle_t *pI2CHandle, uint8_t slaveAddr, uint8_t *pTxBuffer, uint8_t len, uint8_t isRestared);
uint8_t I2C_MasterReceiveIT(I2C_Handle_t *pI2CHandle, uint8_t slaveAddr, uint8_t *pRxBuffer, uint8_t len, uint8_t isRestared);

// IRQ Configuration
void I2C_EnableIRQ(uint8_t IRQNumber, uint8_t isEnabled);
void I2C_SetPriorityIRQ(uint8_t IRQNumber, uint32_t IRQPriority);

// ISR handling
void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle);
void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle);

// Application event callback used by user application
void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t flagEvent);


#endif /* INC_DRV_I2C_H_ */
