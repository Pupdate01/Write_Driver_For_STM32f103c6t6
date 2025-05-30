/*
 * stm32f103xx_spi_driver.h
 *
 *  Created on: Apr 27, 2025
 *      Author: hiephuu2001
 */

#ifndef INC_STM32F103XX_SPI_DRIVER_H_
#define INC_STM32F103XX_SPI_DRIVER_H_

#include<stm32f103xx.h>

/*
 * This is a Handle structure for a SPI peripheral
 */

typedef struct
{
	uint8_t SPI_DeviceMode;		/*possible value from @SPI_DEVICEMODE*/
	uint8_t SPI_BusConfig;		/*possible value from @SPI_BUSCONFIG*/
	uint8_t SPI_SclkSpeed;		/*possible value from @SPI_SCLKSPEED*/
	uint8_t SPI_DFF;			/*possible value from @SPI_DFF*/
	uint8_t SPI_CPOL;			/*possible value from @SPI_CPOL*/
	uint8_t SPI_CPHA;			/*possible value from @SPI_CPHA*/
	uint8_t	SPI_SSM;			/*possible value from @SPI_SSM*/
}SPI_Config_t;

typedef struct
{
	SPI_RegDef_t *pSPIx;		/*This holds the base address of SPIx(x:1,2,3) peripheral*/
	SPI_Config_t SPIConfig; 	/*This holds SPI configuration settings*/
}SPI_Handle_t;

/*
 * Peripheral Clock setup
 */
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi );

/*
 * Init and De-init
 */

void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DeInit(SPI_RegDef_t *pSPIx);

/*
 * Data read and write
 */
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len );
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len );


/*
 * IQR Configuration and ISR handling
 */
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void SPI_IRQHandling(SPI_Handle_t *pHandle);

/*
 * @SPI_DEVICEMODE
 * SPI Mode possible modes
 */
#define SPI_DEVICE_MODE_MASTER 				1
#define SPI_DEVICE_MODE_SLAVE				0

/*
 * @SPI_BUSCONFIG
 * SPI Mode possible modes
 */
#define SPI_BUS_CONFIG_FD	 				1
#define SPI_BUS_CONFIG_HD					2
#define SPI_BUS_CONFIG_SIMPLEX_RXONLY		3
/*
 * @SPI_SCLKSPEED
 * SPI Mode possible modes
 */
#define SPI_FPCLK_DIV2		 				0
#define SPI_FPCLK_DIV4		 				1
#define SPI_FPCLK_DIV8		 				2
#define SPI_FPCLK_DIV16		 				3
#define SPI_FPCLK_DIV32		 				4
#define SPI_FPCLK_DIV64		 				5
#define SPI_FPCLK_DIV128	 				6
#define SPI_FPCLK_DIV256	 				7

/*
 * @SPI_DFF
 * SPI Data frame form
 */
#define SPI_DFF_8BIT		 				0
#define SPI_DFF_16BIT 						1

/*
 * @SPI_CPOL
 * SPI Clock polarity
 */
#define SPI_CPOL_LO 						0
#define SPI_CPOL_HI							1

/*
 * @SPI_SSM
 * Software slave management
 */
#define SPI_SSM_DI 							0
#define SPI_SSM_EN							1

/*
 * @SPI_CPHA
 * SPI clock phase
 */
#define SPI_CPHA_LO 						0
#define SPI_CPHA_HI							1

/*
 * SPI related status flags definitions
 */
#define SPI_TXE_FLAG						(1 << SPI_SR_TXE)
#define SPI_RXNE_FLAG						(1 << SPI_SR_RXNE)
#define SPI_BSY_FLAG						(1 << SPI_SR_BSY)

#endif /* INC_STM32F103XX_SPI_DRIVER_H_ */
