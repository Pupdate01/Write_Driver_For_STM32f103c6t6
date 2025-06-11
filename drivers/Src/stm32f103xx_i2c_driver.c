/*
 * stm32f103xx_i2c_driver.c
 *
 *  Created on: Jun 3, 2025
 *      Author: hiephuu2001
 */


#include <stm32f103xx_i2c_driver.h>

/*
 * Peripheral Clock setup
 */

/***********************************************************************************
 * @fn							- SPI_PeriClockControl
 *
 * @brief						- This function enable or disable peripheral clock for given I2C port
 *
 * param[in]					- Base address of the I2C peripheral
 * param[in]					- ENABLE or DISABLE macros
 * param[in]					-
 *
 * @return						- none
 *
 * @Note						- none
 */
void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx,uint8_t EnorDi) {
	if (EnorDi == ENABLE) {
		if (pI2Cx == I2C1) {
			I2C1_PCLK_EN();
		} else if (pI2Cx == I2C2) {
			I2C2_PCLK_EN();
		}
	} else {
		if (pI2Cx == I2C1 ) {
			I2C1_PCLK_DI();
		} else if (pI2Cx == I2C2 ) {
			I2C2_PCLK_DI();
		}
	}
}

/***********************************************************************************
 * @fn							- I2C_PeripheralControl
 *
 * @brief						-
 *
 * param[in]					- Base address of the I2C peripheral
 * param[in]					-
 * param[in]					-
 *
 * @return						- none
 *
 * @Note						- none
 ************************************************************************************/

void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi) {
	if (EnorDi == ENABLE) {
		pI2Cx->CR1 |= (1 << I2C_CR1_PE);
	} else {
		pI2Cx->CR1 &= ~(1 << I2C_CR1_PE);
	}
}

/***********************************************************************************
 * @fn							- I2C_Init
 *
 * @brief						-
 *
 * param[in]					- I2C Handle
 * param[in]					-
 * param[in]					-
 *
 * @return						- none
 *
 * @Note						- none
 ************************************************************************************/
void I2C_Init(I2C_Handle_t *pI2CHandle)
{
	uint32_t tempreg =0 ;

	//ack control bit
	tempreg |= pI2CHandle->I2C_Config.I2C_ACKControl << I2C_CR1_ACK;
	pI2CHandle->pI2Cx->CR1 = tempreg;

	//config the FREQ field field of CR2
	tempreg = 0;
	tempreg |= RCC_GetPCKL1Value()/1000000U;
	pI2CHandle->pI2Cx->CR2 = tempreg & 0x3F;

	//program the device own address
	tempreg = pI2CHandle->I2C_Config.I2C_DeviceAddress << 1;
	tempreg |= (1 << 14);
	pI2CHandle->pI2Cx->OAR1 = tempreg;

	//CCR calculations
	uint16_t ccr_value = 0;
	tempreg = 0;
	if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
	{
		//mode is standard mode
		ccr_value = ((RCC_GetPCKL1Value())/ (2*pI2CHandle->I2C_Config.I2C_SCLSpeed));
		tempreg |= (ccr_value & 0xFFF);
	} else {
		//mode is fast mode
		tempreg |= (1<<15);
		tempreg |= (pI2CHandle->I2C_Config.I2C_FMDutyCycle << 14);

		if(pI2CHandle->I2C_Config.I2C_FMDutyCycle == I2C_FM_DUTY_2)
		{
			ccr_value =  (RCC_GetPCKL1Value())/(3*pI2CHandle->I2C_Config.I2C_SCLSpeed);
		} else {
			ccr_value =  (RCC_GetPCKL1Value())/(25*pI2CHandle->I2C_Config.I2C_SCLSpeed);
		}
	}
	pI2CHandle->pI2Cx->CCR = tempreg;
}

/***********************************************************************************
 * @fn							- I2C_DeInit
 *
 * @brief						-
 *
 * param[in]					- I2C Reg
 * param[in]					-
 * param[in]					-
 *
 * @return						- none
 *
 * @Note						- none
 ************************************************************************************/
void I2C_DeInit(I2C_RegDef_t *pI2Cx);
