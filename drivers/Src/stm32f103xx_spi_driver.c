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
void SPI_Init(SPI_RegDef_t *pSPIHandle);

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
void SPI_DeInit(SPI_RegDef_t *pSPIx);

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
 * @Note						- none
 */
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len );

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

