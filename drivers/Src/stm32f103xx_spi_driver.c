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
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi )
{
	if(EnorDi == ENABLE){
			if(pSPIx == SPI1){
				SPI1_PCLK_EN();
			} else if(pSPIx == SPI2){
				SPI2_PCLK_EN();
			} else if(pSPIx == SPI3){
				SPI3_PCLK_EN();
			}

		} else {
			if(pSPIx == SPI1){
				SPI1_PCLK_DI();
			} else if(pSPIx == SPI2){
				SPI2_PCLK_DI();
			} else if(pSPIx == SPI3){
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
void SPI_Init(SPI_Handle_t *pSPIHandle)
{
	//First Lets configure the SPI_CR1 register

	uint32_t tempReg = 0 ;

	//1. Configure device mode
	tempReg |= ( pSPIHandle->SPIConfig.SPI_DeviceMode  << SPI_CR1_MSTR );

	//2. Configure Bus config
	if( pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD)
	{
		//bidi mode should be cleared
		tempReg &= ~( 1 << SPI_CR1_BIDIMODE );
	} else if ( pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD)
	{
		//bidi mode should be set
		tempReg |= ( 1 << SPI_CR1_BIDIMODE );
	} else if ( pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY)
	{
		//BIDI mode should be cleared
		tempReg &= ~( 1 << SPI_CR1_BIDIMODE );

		//RXONLY bit must be set
		tempReg |= ( 1 << SPI_CR1_RXONLY );
	}

	//3. Configure SPI_SclkSpeed
	tempReg |= ( pSPIHandle->SPIConfig.SPI_SclkSpeed << SPI_CR1_BR);

	//4. Configure DFF
	tempReg |= ( pSPIHandle->SPIConfig.SPI_DFF << SPI_CR1_DFF );

	//5. Configure SPI_CPOL
	tempReg |= ( pSPIHandle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL );

	//6. Configure SPI_CPHA
	tempReg |= ( pSPIHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA );

	//7. Configure SPIS_SSM
	tempReg |= ( pSPIHandle->SPIConfig.SPI_SSM << SPI_CR1_SSM );

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
void SPI_DeInit(SPI_RegDef_t *pSPIx){
	if(pSPIx == SPI1){
		SPI1_REG_RESET();
	} else if (pSPIx == SPI2){
		SPI2_REG_RESET();
	} else if (pSPIx == SPI3){
		SPI3_REG_RESET();
	}
}


uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx , uint32_t FlagName)
{
	if(pSPIx->SR & FlagName)
	{
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
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len )
{
	while(Len >0){
		//1. wait until TXE is set
		while(SPI_GetFlagStatus(pSPIx,SPI_TXE_FLAG) == FLAG_RESET);

		//2. check DFF bit in CR1
		if(pSPIx->CR1 & (1 << SPI_CR1_DFF))
		{
			//16 bit DFF
			//1. load data in to the DR
			pSPIx->DR = *((uint16_t*)pTxBuffer);
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
 * @Note						- none
 */
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len );

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
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);

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
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);

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
void SPI_IRQHandling(SPI_Handle_t *pHandle);

