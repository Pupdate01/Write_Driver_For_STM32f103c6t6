/*
 * stm32f103xx_spi_driver.c
 *
 *  Created on: Apr 27, 2025
 *      Author: hiephuu2001
 */

#include<stm32f103xx_spi_driver.h>

/*
 * Peripheral Clock setup
 */

/***********************************************************************************
 * @fn							- SPI_PeriClockControl
 *
 * @brief						- This function enable or disable peripheral clock for given GPIO port
 *
 * param[in]					- Base address of the SPI peripheral
 * param[in]					- ENABLE or DISABLE macros
 * param[in]					-
 *
 * @return						- none
 *
 * @Note						- none
 */
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi) {
	if (EnorDi == ENABLE) {
		if (pSPIx == SPI1) {
			SPI1_PCLK_EN();
		} else if (pSPIx == SPI2) {
			SPI2_PCLK_EN();
		} else if (pSPIx == SPI3) {
			SPI3_PCLK_EN();
		}

	} else {
		if (pSPIx == SPI1) {
			SPI1_PCLK_DI();
		} else if (pSPIx == SPI2) {
			SPI2_PCLK_DI();
		} else if (pSPIx == SPI3) {
			SPI3_PCLK_DI();
		}

	}
}

/*
 * SPI init
 */

/***********************************************************************************
 * @fn							- SPI_Init
 *
 * @brief						- This function init SPI
 *
 * param[in]					- Base address of SPI peripheral
 * param[in]					-
 * param[in]					-
 *
 * @return						- none
 *
 * @Note						- none
 */
void SPI_Init(SPI_Handle_t *pSPIHandle) {
	//peripheral clock enable
	SPI_PeriClockControl(pSPIHandle->pSPIx, ENABLE);

	//First Lets configure the SPI_CR1 register

	uint32_t tempReg = 0;

	//1. Configure device mode
	tempReg |= (pSPIHandle->SPIConfig.SPI_DeviceMode << SPI_CR1_MSTR);

	//2. Configure Bus config
	if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD) {
		//bidi mode should be cleared
		tempReg &= ~(1 << SPI_CR1_BIDIMODE);
	} else if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD) {
		//bidi mode should be set
		tempReg |= (1 << SPI_CR1_BIDIMODE);
	} else if (pSPIHandle->SPIConfig.SPI_BusConfig
			== SPI_BUS_CONFIG_SIMPLEX_RXONLY) {
		//BIDI mode should be cleared
		tempReg &= ~(1 << SPI_CR1_BIDIMODE);

		//RXONLY bit must be set
		tempReg |= (1 << SPI_CR1_RXONLY);
	}

	//3. Configure SPI_SclkSpeed
	tempReg |= (pSPIHandle->SPIConfig.SPI_SclkSpeed << SPI_CR1_BR);

	//4. Configure DFF
	tempReg |= (pSPIHandle->SPIConfig.SPI_DFF << SPI_CR1_DFF);

	//5. Configure SPI_CPOL
	tempReg |= (pSPIHandle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL);

	//6. Configure SPI_CPHA
	tempReg |= (pSPIHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA);

	//7. Configure SPIS_SSM
	tempReg |= (pSPIHandle->SPIConfig.SPI_SSM << SPI_CR1_SSM);

	//copy tempReg to CR1
	pSPIHandle->pSPIx->CR1 = tempReg;

}

/*
 * SPI deinit
 */

/***********************************************************************************
 * @fn							- SPI_DeInit
 *
 * @brief						- This function deinit SPI
 *
 * param[in]					- Base address of the SPI peripheral
 * param[in]					-
 * param[in]					-
 *
 * @return						- none
 *
 * @Note						- none
 */
void SPI_DeInit(SPI_RegDef_t *pSPIx) {
	if (pSPIx == SPI1) {
		SPI1_REG_RESET();
	} else if (pSPIx == SPI2) {
		SPI2_REG_RESET();
	} else if (pSPIx == SPI3) {
		SPI3_REG_RESET();
	}
}
/*
 * API check Flag Status
 */

uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName) {
	if (pSPIx->SR & FlagName) {
		return FLAG_SET;
	}
	return FLAG_RESET;
}

/*
 * Data write
 */

/***********************************************************************************
 * @fn							- SPI_SendData
 *
 * @brief						- This function send data by SPI
 *
 * param[in]					- Base address of the SPI peripheral
 * param[in]					- Buffer of Data
 * param[in]					- Size of Data
 *
 * @return						- none
 *
 * @Note						- This is block call
 */
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len) {
	while (Len > 0) {
		//1. wait until TXE is set
		while (SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG) == FLAG_RESET)
			;

		//2. check DFF bit in CR1
		if (pSPIx->CR1 & (1 << SPI_CR1_DFF)) {
			//16 bit DFF
			//1. load data in to the DR
			pSPIx->DR = *((uint16_t*) pTxBuffer);
			Len -= 2;
			pTxBuffer += 2;
		} else {
			//8 bit DFF
			pSPIx->DR = *pTxBuffer;
			Len--;
			pTxBuffer++;
		}

	}
}

/*
 * SPI Data Read
 */

/***********************************************************************************
 * @fn							- SPI_ReceiveData
 *
 * @brief						- This function received Data by SPI
 *
 * param[in]					- Base address of the SPI peripheral
 * param[in]					- Buffer of Data
 * param[in]					- Size of Data
 *
 * @return						- none
 *
 * @Note						- This is block call
 */
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len) {
	while (Len > 0) {
		//1. wait until RXNE Buffet is non Empty
		while (SPI_GetFlagStatus(pSPIx, SPI_RXNE_FLAG) == FLAG_RESET)
			;

		//2. check DFF bit in CR1
		if ((pSPIx->CR1) & (1 << SPI_CR1_DFF)) {
			//16 bits DFF
			//1. read data from DR to RxBuffer Address

			*((uint16_t*) pRxBuffer) = pSPIx->DR;
			Len -= 2;
			pRxBuffer += 2;
		} else {
			// 8 bits DFF

			*pRxBuffer = pSPIx->DR;
			Len--;
			pRxBuffer++;
		}
	}
}

/*
 * IQR Configuration and ISR handling
 */

/***********************************************************************************
 * @fn							- SPI_IRQInterruptConfig
 *
 * @brief						-
 *
 * param[in]					- IQR number
 * param[in]					- ENABLE or DISABLE macros
 * param[in]					-
 *
 * @return						- none
 *
 * @Note						- none
 */
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi) {
	if (EnorDi == ENABLE) {
		if (IRQNumber <= 31) //0 to 31
				{
			// program ISER0 register
			*NVIC_ISER0 |= (1 << IRQNumber);
		} else if (IRQNumber > 31 && IRQNumber < 64) //32 to 63
				{
			//program ISER1 register
			*NVIC_ISER1 |= (1 << (IRQNumber % 32));
		} else if (IRQNumber >= 64 && IRQNumber < 68) {
			//program ISER2 register
			*NVIC_ISER2 |= (1 << (IRQNumber % 64));
		}
	} else {
		if (IRQNumber <= 31) //0 to 31
				{
			// program ISER0 register
			*NVIC_ICER0 |= (1 << IRQNumber);
		} else if (IRQNumber > 31 && IRQNumber < 64) //32 to 63
				{
			//program ISER1 register
			*NVIC_ICER1 |= (1 << (IRQNumber % 32));
		} else if (IRQNumber >= 64 && IRQNumber < 68) {
			//program ISER2 register
			*NVIC_ICER2 |= (1 << (IRQNumber % 64));
		}
	}
}

/***********************************************************************************
 * @fn							- SPI_IRQPriorityConfig
 *
 * @brief						-
 *
 * param[in]					- IQR number
 * param[in]					- ENABLE or DISABLE macros
 * param[in]					-
 *
 * @return						- none
 *
 * @Note						- none
 */
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority) {
	//1.first lets find out the ipr register
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;

	uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PRIOR_BITS_IMPLEMENTED);

	*(NVIC_PRIOR_BASE_ADDR + iprx) &= ~(0xFF << shift_amount); //clear this bit
	*(NVIC_PRIOR_BASE_ADDR + iprx) |= (IRQPriority << (8 * shift_amount));
}

/***********************************************************************************
 * @fn							- SPI_IRQHandling
 *
 * @brief						-
 *
 * param[in]					- Base address of the SPI peripheral
 * param[in]					-
 * param[in]					-
 *
 * @return						- none
 *
 * @Note						- none
 */
void SPI_IRQHandling(SPI_Handle_t *pHandle) {
	uint8_t temp1, temp2;

	//first let check for TXE
	temp1 = pHandle->pSPIx->SR & (1 << SPI_SR_TXE);
	temp2 = pHandle->pSPIx->CR2 & (1 << SPI_CR2_TXEIE);

	if (temp1 && temp2) {
		// have TXE
		spi_txe__interrupt_handle(pHandle);

	}

	//let check for RXNE
	temp1 = pHandle->pSPIx->SR & (1 << SPI_SR_RXNE);
	temp2 = pHandle->pSPIx->CR2 & (1 << SPI_CR2_RXNEIE);

	if (temp1 && temp2) {
		// have RXNE
		spi_rxne__interrupt_handle(pHandle);

	}

	//check for ovr flag
	temp1 = pHandle->pSPIx->SR & (1 << SPI_SR_OVR);
	temp2 = pHandle->pSPIx->CR2 & (1 << SPI_CR2_ERRIE);

	if (temp1 && temp2) {
		//handle ovr error
		spi_ovr_err_interrupt_handle(pHandle);
	}
}

/***********************************************************************************
 * @fn							- SPI_IRQHandling
 *
 * @brief						-
 *
 * param[in]					- Base address of the SPI peripheral
 * param[in]					-
 * param[in]					-
 *
 * @return						- none
 *
 * @Note						- none
 ************************************************************************************/

void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi) {
	if (EnorDi == ENABLE) {
		pSPIx->CR1 |= (1 << SPI_CR1_SPE);
	} else {
		pSPIx->CR1 &= ~(1 << SPI_CR1_SPE);
	}
}

/***********************************************************************************
 * @fn							- SPI_SSIConfig
 *
 * @brief						-
 *
 * param[in]					- SPI_RegDef_t
 * param[in]					- Enable or Diable
 * param[in]					-
 *
 * @return						- none
 *
 * @Note						- none
 ************************************************************************************/
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi) {
	if (EnorDi == ENABLE) {
		pSPIx->CR1 |= (1 << SPI_CR1_SSI);
	} else {
		pSPIx->CR1 &= ~(1 << SPI_CR1_SSI);
	}
}

/***********************************************************************************
 * @fn							- SPI_SSOEConfig
 *
 * @brief						-
 *
 * param[in]					- SPI_RegDef_t
 * param[in]					- Enable or Diable
 * param[in]					-
 *
 * @return						- none
 *
 * @Note						- none
 ************************************************************************************/
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi) {
	if (EnorDi == ENABLE) {
		pSPIx->CR2 |= (1 << SPI_CR2_SSOE);
	} else {
		pSPIx->CR2 &= ~(1 << SPI_CR2_SSOE);
	}

}

/***********************************************************************************
 * @fn							- SPI_SendDataIT
 *
 * @brief						-
 *
 * param[in]					- SPI_RegDef_t
 * param[in]					- Enable or Diable
 * param[in]					-
 *
 * @return						- none
 *
 * @Note						- none
 ************************************************************************************/
uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer,
		uint32_t Len) {
	uint8_t state = pSPIHandle->RxState;

	if (state != SPI_BUSY_IN_RX) {
		//1. save the Tx buffer address and length information in some global variable
		pSPIHandle->pTxBuffer = pTxBuffer;
		pSPIHandle->TxLen = Len;
		//2. Mask the SPI state as busy in transmission so that
		//no other code can take over same SPI peripheral until transmission is over

		pSPIHandle->TxState = SPI_BUSY_IN_TX;

		//3. Enable the TXEIE control bit to get interrupt whenever TXE flag is set in SR'
		pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_TXEIE);

	}
	//4. Data Transmission will be handled by the ISR code (will implement later)

	return state;
}

/***********************************************************************************
 * @fn							- SPI_SSIConfig
 *
 * @brief						-
 *
 * param[in]					- SPI_RegDef_t
 * param[in]					- Enable or Diable
 * param[in]					-
 *
 * @return						- none
 *
 * @Note						- none
 ************************************************************************************/
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer,
		uint32_t Len) {
	uint8_t state = pSPIHandle->TxState;

	if (state != SPI_BUSY_IN_TX) {
		//1. save the Tx buffer address and length information in some global variable
		pSPIHandle->pRxBuffer = pRxBuffer;
		pSPIHandle->RxLen = Len;
		//2. Mask the SPI state as busy in transmission so that
		//no other code can take over same SPI peripheral until transmission is over

		pSPIHandle->RxState = SPI_BUSY_IN_RX;

		//3. Enable the TXEIE control bit to get interrupt whenever TXE flag is set in SR'
		pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_RXNEIE);

	}
	//4. Data Transmission will be handled by the ISR code (will implement later)

	return state;
}

// Some helper function implementations

void spi_txe__interrupt_handle(SPI_Handle_t *pHandle) {
	// check DFF bit in CR1
	if (pHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF)) {
		//16 bit DFF
		//1. load data in to the DR
		pHandle->pSPIx->DR = *((uint16_t*) pHandle->pTxBuffer);
		pHandle->TxLen -= 2;
		pHandle->pTxBuffer += 2;
	} else {
		//8 bit DFF
		pHandle->pSPIx->DR = *pHandle->pTxBuffer;
		pHandle->TxLen--;
		pHandle->pTxBuffer++;
	}

	if (!pHandle->TxLen) {
		//Txlen is zero, so close the spi transmission and inform the application that
		//Tx is over.
		//this prevents interrupts from setting up of TXE flag
		spi_closetransmission(pHandle);
		SPI_ApplicationEventCallBack(pHandle, SPI_EVENT_TX_CMPLT);
	}
}

void spi_rxne__interrupt_handle(SPI_Handle_t *pHandle) {
	// check DFF bit in CR1
	if (pHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF)) {
		*((uint16_t*) pHandle->pRxBuffer) = (uint16_t) pHandle->pSPIx->DR;
		pHandle->RxLen -= 2;
		pHandle->pRxBuffer -= 2;
	} else {
		//8 bit DFF
		*(pHandle->pRxBuffer) = (uint8_t) pHandle->pSPIx->DR;
		pHandle->TxLen--;
		pHandle->pTxBuffer--;
	}

	if (!pHandle->TxLen) {
		//Txlen is zero, so close the spi transmission and inform the application that
		//Tx is over.
		//this prevents interrupts from setting up of TXE flag
		spi_closereception(pHandle);
		SPI_ApplicationEventCallBack(pHandle, SPI_EVENT_RX_CMPLT);

	}
}

void spi_ovr_err_interrupt_handle(SPI_Handle_t *pHandle) {
	uint8_t temp;

	//1. clear the ovr flag
	if (pHandle->TxState != SPI_BUSY_IN_TX) {
		temp = pHandle->pSPIx->DR;
		temp = pHandle->pSPIx->SR;
	}
	(void) temp;
	//2. inform the application
	SPI_ApplicationEventCallBack(pHandle, SPI_EVENT_OVR_ERR);
}

void spi_clearovrflag(SPI_RegDef_t *pSPIx) {
	uint8_t temp;
	temp = pSPIx->DR;
	temp = pSPIx->SR;
	(void) temp;
}

void spi_closetransmission(SPI_Handle_t *pHandle) {
	pHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_TXEIE);
	//pHandle->pTxBuffer = NULL;
	pHandle->TxLen = 0;
	pHandle->TxState = SPI_READY;
}

void spi_closereception(SPI_Handle_t *pHandle) {
	pHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_RXNEIE);
	//pHandle->pRxBuffer = NULL;
	pHandle->RxLen = 0;
	pHandle->RxState = SPI_READY;
}

__weak void SPI_ApplicationEventCallBack(SPI_Handle_t *pHanle,uint8_t AppEv)
{
	//This is a weak implementation. The application may override this function
}
