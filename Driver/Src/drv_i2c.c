/*
 * drv_i2c.c
 *
 *  Created on: Apr 30, 2020
 *      Author: vanhoang
 */

#include "drv_i2c.h"

uint16_t ahbPrescArrayValue[] = { 2, 4, 8, 16, 64, 128, 256, 512 };
uint8_t apbPrescArrayValue[] = { 2, 4, 8, 16 };

static void i2c_clear_flag_addr(I2C_Handle_t *pI2CHandle);
static void i2c_write_slave_addrphase(I2C_RegDef_t *pI2Cx, uint8_t slaveAddr);
static void i2c_read_slave_addrphase(I2C_RegDef_t *pI2Cx, uint8_t slaveAddr);
static uint8_t i2c_get_status_flag(I2C_RegDef_t *pI2Cx, uint8_t flag);
static void i2c_master_handle_isr_rxne(I2C_Handle_t *pI2CHandle);
static void i2c_master_handle_isr_txe(I2C_Handle_t *pI2CHandle);
static uint32_t i2c_get_clock_apb(I2C_RegDef_t *pI2Cx);
static void i2c_close_reception(I2C_Handle_t *pI2CHandle);
static void i2c_close_transmission(I2C_Handle_t *pI2CHandle);
uint32_t RCC_GetClockPLL() {
	return 0;
}

static uint32_t i2c_get_clock_apb(I2C_RegDef_t *pI2Cx) {
	uint32_t tempReg = 0;
	uint32_t swValue, ahbPrescValue, apbPrescValue;
	uint8_t temp;

	tempReg = RCC->CFGR;

	// get clock System Switch
	temp = ((tempReg >> 2) & 0x03);
	if (temp == 0) {
		swValue = 16000000;
	} else if (temp == 1) {
		swValue = 8000000;
	} else if (temp == 2) {
		swValue = RCC_GetClockPLL();
	}

	// get prescaler of AHB bus
	temp = 0;
	temp = ((tempReg >> 4) && 0x0F);
	if (temp < 8) {
		ahbPrescValue = 1;
	} else {
		ahbPrescValue = ahbPrescArrayValue[8 - temp];
	}

	// get prescaler of APB1 bus
	temp = 0;
	temp = (tempReg >> 10) && 0x07;
	if (temp < 4) {
		apbPrescValue = 1;
	} else {
		apbPrescValue = apbPrescArrayValue[4 - temp];
	}

	return (swValue / (ahbPrescValue * apbPrescValue));

}
void I2C_Init(I2C_Handle_t *pI2CHandle) {

	uint32_t tempReg = 0;
	uint32_t tempValue = 0;

	//1. enable clock of i2c peripheral
	I2C_EnableClockPeri(pI2CHandle->pI2C, TRUE);

	//3. config frequency value for I2C peripheral
	pI2CHandle->pI2C->CR[1] = (i2c_get_clock_apb(pI2CHandle->pI2C) / 1000000U) & (0x3F);

	//4. choose sm/fm mode - duty mode - config ccr
	tempReg = pI2CHandle->pI2C->CCR;
	if (pI2CHandle->config.sclSpeed == I2C_SCLSPEED_SM) {
		tempReg &= ~(1 << I2C_CCR_FS);
		tempValue = (i2c_get_clock_apb(pI2CHandle->pI2C) / (2 * pI2CHandle->config.sclSpeed));
	} else {
		if (pI2CHandle->config.dutyCycleFM == I2C_DUTYCYCLEFM_2) {
			tempValue = (i2c_get_clock_apb(pI2CHandle->pI2C) / (3 * pI2CHandle->config.sclSpeed));
		} else {
			tempReg |= 1 << I2C_CCR_DUTY;
			tempValue = (i2c_get_clock_apb(pI2CHandle->pI2C) / (25 * pI2CHandle->config.sclSpeed));
		}
	}
	tempReg |= tempValue & 0xFFF;
	pI2CHandle->pI2C->CCR = tempReg;

	//5. config own address
	tempReg = pI2CHandle->pI2C->OAR[0];
	tempReg |= 1 << 14;
	tempReg |= (pI2CHandle->config.deviceAddress && 0x7F) << 1;
	pI2CHandle->pI2C->OAR[0] = tempReg;

	//TRISE Configuration
	if (pI2CHandle->config.sclSpeed <= I2C_SCLSPEED_SM) {
		//mode is standard mode

		tempReg = (i2c_get_clock_apb(pI2CHandle->pI2C) / 1000000U) + 1;

	} else {
		//mode is fast mode
		tempReg = ((i2c_get_clock_apb(pI2CHandle->pI2C) * 300) / 1000000000U) + 1;

	}
	pI2CHandle->pI2C->TRISE = (tempReg & 0x3F);

	I2C_EnablePeripheral(pI2CHandle->pI2C);
	//2. enable ack control bit
	pI2CHandle->pI2C->CR[0] |= 1 << I2C_CR1_ACK;
}

void I2C_EnablePeripheral(I2C_RegDef_t *pI2Cx) {
	pI2Cx->CR[0] |= 1 << I2C_CR1_PE;
}
void I2C_DeInit(I2C_RegDef_t *pI2Cx) {

}

void I2C_EnableClockPeri(I2C_RegDef_t *pI2Cx, uint8_t isEnable) {
	if (isEnable) {
		if (pI2Cx == I2C1) {
			ENABLE_PCLK_I2C1();
		} else if (pI2Cx == I2C2) {
			ENABLE_PCLK_I2C2();
		} else if (pI2Cx == I2C3) {
			ENABLE_PCLK_I2C3();
		}
	} else {
		// do nothing
	}
}

void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx) {
	pI2Cx->CR[0] |= (1 << I2C_CR1_START);
}

void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx) {
	pI2Cx->CR[0] |= (1 << I2C_CR1_STOP);
}

void I2C_MasterTransmit(I2C_Handle_t *pI2CHandle, uint8_t slaveAddr,
		uint8_t *pTxBuffer, uint8_t len, uint8_t isRestared) {
	//1. Generate the START condition
	I2C_GenerateStartCondition(pI2CHandle->pI2C);

	//2. Confirm that START generation is completed by checking the SB flag in the SR1
	while (!(i2c_get_status_flag(pI2CHandle->pI2C, I2C_FLAG_SB)))
		;

	//3. Send the address of slave and command bit
	i2c_write_slave_addrphase(pI2CHandle->pI2C, slaveAddr);

	//4. Confirm that address phase is completed by checking the ADDR flag in SR1
	while (!(i2c_get_status_flag(pI2CHandle->pI2C, I2C_FLAG_ADDR)))
		;

	//5. clear the ADDR flag according to its software sequence
	i2c_clear_flag_addr(pI2CHandle);

	//6. send the data until len becomes 0
	while (len > 0) {
		while (!i2c_get_status_flag(pI2CHandle->pI2C, I2C_FLAG_TXE));

		pI2CHandle->pI2C->DR = *pTxBuffer;
		len--;
		pTxBuffer++;
	}

	//7. when len becomes zero wait for TXE=1 and BTF=1 before generating the stop condition
	while (!(i2c_get_status_flag(pI2CHandle->pI2C, I2C_FLAG_TXE)
			& i2c_get_status_flag(pI2CHandle->pI2C, I2C_FLAG_BTF)));
	//8. Generate STOP condition and master need not to wait for completion of stop condition
	if (!isRestared) {
		I2C_GenerateStopCondition(pI2CHandle->pI2C);
	}
}

static void i2c_clear_flag_addr(I2C_Handle_t *pI2CHandle) {
	uint32_t dummyRead;

	// check for device mode
	if (pI2CHandle->pI2C->SR[1] & (1 << I2C_SR2_MSL)) {
		//device is in master mode
		if (pI2CHandle->currentStatus == I2C_STATUS_IN_RX) {
			if (pI2CHandle->RxSize == 1) {
				// first disable the ack
				pI2CHandle->pI2C->CR[0] &= ~(1 << I2C_CR1_ACK);
			}
		}
	}
	//clear the ADDR flag (read SR1, read SR2)
	dummyRead = pI2CHandle->pI2C->SR[0];
	dummyRead = pI2CHandle->pI2C->SR[1];
	(void) dummyRead;
}

void I2C_MasterReceive(I2C_Handle_t *pI2CHandle, uint8_t slaveAddr,
		uint8_t *pRxBuffer, uint8_t len, uint8_t isReStarted) {
	//1. Generate Start condition
	I2C_GenerateStartCondition(pI2CHandle->pI2C);

	//2. Wait SB = 1
	while (!i2c_get_status_flag(pI2CHandle->pI2C, I2C_FLAG_SB))
		;

	//3. handle EV5 by write address slave and command bit
	i2c_read_slave_addrphase(pI2CHandle->pI2C, slaveAddr);

	//4. Wait ADDR=1
	while (!i2c_get_status_flag(pI2CHandle->pI2C, I2C_FLAG_ADDR))
		;

	// 4-1. handle EV6 when length=1
	if (len == 1) {
		// Disable ACK
		pI2CHandle->pI2C->CR[0] &= ~(1 << I2C_CR1_ACK);
		i2c_clear_flag_addr(pI2CHandle);

		//wait until  RXNE becomes 1
		while (!i2c_get_status_flag(pI2CHandle->pI2C, I2C_FLAG_RXNE))
			;
		if (isReStarted == FALSE) {
			I2C_GenerateStopCondition(pI2CHandle->pI2C);
		}
		*pRxBuffer = pI2CHandle->pI2C->DR;
	}

	//4-2. handle EV6 when length > 1
	if (len > 1) {
		i2c_clear_flag_addr(pI2CHandle);
		for (uint32_t i = len; i > 0; i--) {
			while (!i2c_get_status_flag(pI2CHandle->pI2C, I2C_FLAG_RXNE))
				;
			if (i == 2) {
				pI2CHandle->pI2C->CR[0] &= ~(1 << I2C_CR1_ACK);

				if (isReStarted == FALSE) {
					I2C_GenerateStopCondition(pI2CHandle->pI2C);
				}
			}
			*pRxBuffer = pI2CHandle->pI2C->DR;
			pRxBuffer++;
		}
	}

	//re-enable ACKing
	if (pI2CHandle->config.ACKControl == I2C_ACKCONTROL_ENABLE) {
		pI2CHandle->pI2C->CR[0] |= (1 << I2C_CR1_ACK);
	}
}


static void i2c_write_slave_addrphase(I2C_RegDef_t *pI2Cx, uint8_t slaveAddr) {
	uint32_t addrCmd = 0;

	addrCmd = (slaveAddr << 1);
	pI2Cx->DR = addrCmd;
}

static void i2c_read_slave_addrphase(I2C_RegDef_t *pI2Cx, uint8_t slaveAddr) {
	uint32_t addrCmd = 0;

	addrCmd = (slaveAddr << 1) | 0x01;
	pI2Cx->DR = addrCmd;
}

static uint8_t i2c_get_status_flag(I2C_RegDef_t *pI2Cx, uint8_t flag) {
	return (uint8_t) ((pI2Cx->SR[0] >> flag) & 0x01);
}

/************************************************************************************************
 * @brief    		- I2C Master trasmit data by using interrupt - Non Blocking mode
 *
 * @param[in]   	-
 *
 * @param[in]	    -
 *
 * @retval 		-
 *************************************************************************************************/
uint8_t I2C_MasterTransmitIT(I2C_Handle_t *pI2CHandle, uint8_t slaveAddr,
		uint8_t *pTxBuffer, uint8_t len, uint8_t isRestared) {

	if (pI2CHandle->currentStatus == I2C_STATUS_READY) {
		pI2CHandle->pTxBuffer = pTxBuffer;
		pI2CHandle->TxLen = len;
		pI2CHandle->currentStatus = I2C_STATUS_IN_TX;
		pI2CHandle->devAddr = slaveAddr;
		pI2CHandle->isRestared = isRestared;

		// Implement code to generate START condition
		I2C_GenerateStartCondition(pI2CHandle->pI2C);

		// Implement the code to enable ITBUFEN control bit
		pI2CHandle->pI2C->CR[1] |= 1 << I2C_CR2_ITBUFEN;

		//Implement the code to enable ITEVFEN Control Bit
		pI2CHandle->pI2C->CR[1] |= 1 << I2C_CR2_ITEVTEN;

		//Implement the code to enable ITERREN Control Bit
		pI2CHandle->pI2C->CR[1] |= 1 << I2C_CR2_ITERREN;
	}

	if (pI2CHandle->currentStatus == I2C_STATUS_COMM_CMPLT) {
		pI2CHandle->currentStatus = I2C_STATUS_READY;
	}
	return pI2CHandle->currentStatus;
}

/************************************************************************************************
 * @brief    		-
 *
 * @param[in]   	-
 *
 * @param[in]	    -
 *
 * @retval 		-
 *************************************************************************************************/
uint8_t I2C_MasterReceiveIT(I2C_Handle_t *pI2CHandle, uint8_t slaveAddr,
		uint8_t *pRxBuffer, uint8_t len, uint8_t isRestared) {

	if (pI2CHandle->currentStatus == I2C_STATUS_READY) {
		pI2CHandle->pRxBuffer = pRxBuffer;
		pI2CHandle->RxLen = len;
		pI2CHandle->currentStatus = I2C_STATUS_IN_RX;
		pI2CHandle->devAddr = slaveAddr;
		pI2CHandle->RxSize = len;
		pI2CHandle->isRestared = isRestared;

		// Implement code to generate START condition
		I2C_GenerateStartCondition(pI2CHandle->pI2C);

		// Implement the code to enable ITBUFEN control bit
		pI2CHandle->pI2C->CR[1] |= 1 << I2C_CR2_ITBUFEN;

		//Implement the code to enable ITEVFEN Control Bit
		pI2CHandle->pI2C->CR[1] |= 1 << I2C_CR2_ITEVTEN;

		//Implement the code to enable ITERREN Control Bit
		pI2CHandle->pI2C->CR[1] |= 1 << I2C_CR2_ITERREN;
	}

	if (pI2CHandle->currentStatus == I2C_STATUS_COMM_CMPLT) {
		pI2CHandle->currentStatus = I2C_STATUS_READY;
	}
	return pI2CHandle->currentStatus;
}

void I2C_EnableIRQ(uint8_t IRQNumber, uint8_t isEnabled) {
	uint8_t temp1 = IRQNumber / 32;
	uint8_t temp2 = IRQNumber % 32;

	if (isEnabled) {
		*(NVIC_ISER + temp1) |= (1 << temp2);
	} else {
		*(NVIC_ICER + temp1) |= (1 << temp2);
	}
}

void I2C_SetPriorityIRQ(uint8_t IRQNumber, uint32_t IRQPriority) {
	uint8_t temp1 = IRQNumber / 4;
	uint8_t temp2 = IRQNumber % 4;

	*(NVIC_IPR + temp1) |= IRQPriority << (8 * temp2 + NO_IMPLEMENT_PRIORITY_BITS);
}
void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle) {

	// Interrupt handling for both master and slave mode of a device
	uint32_t temp1, temp2, temp3;
	temp1 = pI2CHandle->pI2C->CR[1] & (1 << I2C_CR2_ITEVTEN);
	temp2 = pI2CHandle->pI2C->CR[1] & (1 << I2C_CR2_ITBUFEN);

	temp3 = pI2CHandle->pI2C->SR[0] & (1 << I2C_SR1_SB);

	//1. Handle for interrupt generated by SB event
	// Note: SB flag is only applicable in Master mode
	if (temp1 && temp3) {
		//This block is processed when SB event occurs
		//this block will process of Writting address for transfer Master
		//and Reading address for Receiver Master
		if (pI2CHandle->currentStatus == I2C_STATUS_IN_TX) {
			i2c_write_slave_addrphase(pI2CHandle->pI2C, pI2CHandle->devAddr);
		} else if (pI2CHandle->currentStatus == I2C_STATUS_IN_RX) {
			i2c_read_slave_addrphase(pI2CHandle->pI2C, pI2CHandle->devAddr);
		}
	}

	//2. Handle for interrupt generated by ADDR event
	// Note: When master mode : Address is sent
	//		 When slave mode  : Address matched with own address
	temp3 = pI2CHandle->pI2C->SR[0] & (1 << I2C_SR1_ADDR);
	if (temp1 && temp3) {
		//ADDR flag is set
		i2c_clear_flag_addr(pI2CHandle);
	}

	//3. Handle for interrupt generated by BTF(Byte transfer finished) event
	temp3 = pI2CHandle->pI2C->SR[0] & (1 << I2C_SR1_BTF);
	if (temp1 && temp3) {
		//BTF flag is set
		if (pI2CHandle->currentStatus == I2C_STATUS_IN_TX) {
			// make sure that TXE is also set
			if (pI2CHandle->pI2C->SR[0] & (1 << I2C_SR1_TXE)) {
				// BTF, TXE = 1
				if (pI2CHandle->TxLen == 0) {
				//1. Generate the stop condition
					if (pI2CHandle->isRestared == FALSE) {
						I2C_GenerateStopCondition(pI2CHandle->pI2C);
					}
				//2. reset all the member elements of the handle structure
					i2c_close_transmission(pI2CHandle);

				//3. notify the application about transmisstion complete
					I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_DATA_TX_CMPLT);
				}
			}
		} else if (pI2CHandle->currentStatus == I2C_STATUS_IN_RX) {

		}
	}

	//4. Handle for interrupt generated by STOPF event
	// Note: Stop detection flag is applicable only slave Mode.For the master will
	// 		 never be set
	//The bellow code block will not executed by the master since STOPF will not set
	//in master mode
	temp3 = pI2CHandle->pI2C->SR[0] & (1 << I2C_SR1_STOPF);
	if (temp1 && temp3) {
		//STOPF flag is set
		//Clear the STOPF by reading SR1 then write to CR1
		pI2CHandle->pI2C->CR[0] |= 0x0000;

		// Notify the application that STOP is detected
		I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_STOP);
	}

	//5. Handle for interrupt generated by TXE event
	temp3 = pI2CHandle->pI2C->SR[0] & (1 << I2C_SR1_TXE);
	if (temp1 && temp2 && temp3) {
		//TXE flag is set
		if (pI2CHandle->pI2C->SR[1] & (1 << I2C_SR2_MSL)) {
			if (pI2CHandle->currentStatus == I2C_STATUS_IN_TX) {
				i2c_master_handle_isr_txe(pI2CHandle);
			}
		} else {
			// slave
			//make sure that the slave is really in transmitter mode
			if (pI2CHandle->pI2C->SR[1] & (1 << I2C_SR2_TRA)) {
				I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_DATA_REQ);
			}
		}
	}

	//6. Handle for interrupt generated by RXNE event
	temp3 = pI2CHandle->pI2C->SR[0] & (1 << I2C_SR1_RXNE);
		if (temp1 && temp2 && temp3) {
			//RXNE flag is set
			// check device mode
			if (pI2CHandle->pI2C->SR[1] & (1 << I2C_SR2_MSL)) {
				// the device is master
				if (pI2CHandle->currentStatus == I2C_STATUS_IN_RX) {
					i2c_master_handle_isr_rxne(pI2CHandle);
				}
			}
			else {
				// slave mode
				// make sure that the slave is really in receiver mode
				if (!(pI2CHandle->pI2C->SR[1] && (1 << I2C_SR2_TRA))) {
					I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_DATA_RCV);
				}
			}
		}
}
static void i2c_close_transmission(I2C_Handle_t *pI2CHandle) {

	//disable ITBUFEN control bit
	pI2CHandle->pI2C->CR[1] &= ~(1 << I2C_CR2_ITBUFEN);

	//disable ITEVTEN control bit
	pI2CHandle->pI2C->CR[1] &= ~(1 << I2C_CR2_ITEVTEN);

	pI2CHandle->currentStatus = I2C_STATUS_COMM_CMPLT;
	pI2CHandle->devAddr = 0;
	pI2CHandle->pTxBuffer = NULL;
	pI2CHandle->TxLen = 0;
}
static void i2c_close_reception(I2C_Handle_t *pI2CHandle) {

	//disable ITBUFEN control bit
	pI2CHandle->pI2C->CR[0] &= ~(1 << I2C_CR2_ITBUFEN);

	//disable ITEVTEN control bit
	pI2CHandle->pI2C->CR[0] &= ~(1 << I2C_CR2_ITEVTEN);

	pI2CHandle->RxSize = 0;
	pI2CHandle->currentStatus = I2C_STATUS_COMM_CMPLT;
	pI2CHandle->devAddr = 0;
	pI2CHandle->pRxBuffer = NULL;
	pI2CHandle->RxLen = 0;

	if (pI2CHandle->config.ACKControl == I2C_ACKCONTROL_ENABLE) {
		pI2CHandle->pI2C->CR[0] |= 1 << I2C_CR1_ACK;
	}
}


static void i2c_master_handle_isr_txe(I2C_Handle_t *pI2CHandle) {
	if (pI2CHandle->TxLen > 0) {
	//1. Load the data into DR
	pI2CHandle->pI2C->DR = *(pI2CHandle->pTxBuffer);

	//2. decrement the TxLen
	pI2CHandle->TxLen--;

	//3. Increment the buffer address
	pI2CHandle->pTxBuffer++;
	}
}
static void i2c_master_handle_isr_rxne(I2C_Handle_t *pI2CHandle) {

	if (pI2CHandle->RxSize == 1) {
		*pI2CHandle->pRxBuffer = pI2CHandle->pI2C->DR;
		pI2CHandle->RxLen--;
	}

	if (pI2CHandle->RxSize > 1) {
		if (pI2CHandle->RxLen == 2) {
			// disable ACK
			pI2CHandle->pI2C->CR[0] &= ~(1 << I2C_CR1_ACK);
		}
			// reading data
			*(pI2CHandle->pRxBuffer) = pI2CHandle->pI2C->DR;
			pI2CHandle->pRxBuffer++;
			pI2CHandle->RxLen--;
	}

	if (pI2CHandle->RxLen == 0) {
		//1. Generate Stop condition
		if (pI2CHandle->isRestared == FALSE) {
			pI2CHandle->pI2C->CR[0] |= 1 << I2C_CR1_STOP;
		}

		//2. Close communication by clearing data
		i2c_close_reception(pI2CHandle);

		//3. Calling the application event
		I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_DATA_RX_CMPLT);
	}

}
void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle) {
	uint32_t temp1, temp2;

	// know the status ITERREN control bit in the CR2
	temp2 = (pI2CHandle->pI2C->CR[1]) & (1 << I2C_CR2_ITERREN);

	//*****check for Bus error *******
	temp1 = (pI2CHandle->pI2C->SR[0] & (1 << I2C_SR1_BERR));
	if (temp1 && temp2) {
		// This is Bus error

		// Implement the code to clear the buss erro flag
		pI2CHandle->pI2C->SR[0] &= ~(1 << I2C_SR1_BERR);

		// Imple met the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle, I2C_ER_BERR);
	}

	//*****check for arbitration lost error *******
	temp1 = (pI2CHandle->pI2C->SR[0] & (1 << I2C_SR1_ARLO));
	if (temp1 && temp2) {
		// This is arbitration lost error

		// Implement the code to clear the ARLO error flag
		pI2CHandle->pI2C->SR[0] &= ~(1 << I2C_SR1_ARLO);

		// Imple met the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle, I2C_ER_ARLO);
	}

	//*****check for ACK failure error *******
	temp1 = (pI2CHandle->pI2C->SR[0] & (1 << I2C_SR1_AF));
	if (temp1 && temp2) {
		// This is ACK failure error

		// Implement the code to clear the ACK failure error flag
		pI2CHandle->pI2C->SR[0] &= ~(1 << I2C_SR1_AF);

		// Imple met the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle, I2C_ER_AF);
	}

	//*****check for Overrun/ Underrun error *******
	temp1 = (pI2CHandle->pI2C->SR[0] & (1 << I2C_SR1_OVR));
	if (temp1 && temp2) {
		// This is Overrun/Underrun

		// Implement the code to clear the Overrun/Underun error flag
		pI2CHandle->pI2C->SR[0] &= ~(1 << I2C_SR1_OVR);

		// Imple met the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle, I2C_SR1_OVR);
	}

	//*****check for timeout error *******
	temp1 = (pI2CHandle->pI2C->SR[0] & (1 << I2C_SR1_TIMEOUT));
	if (temp1 && temp2) {
		// This is Time out error

		// Implement the code to clear the tiemout erro flag
		pI2CHandle->pI2C->SR[0] &= ~(1 << I2C_SR1_TIMEOUT);

		// Imple met the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle, I2C_SR1_TIMEOUT);
	}
}


