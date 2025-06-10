/*
 * stm32f103xx_i2c_driver.c
 *
 *  Created on: Jun 3, 2025
 *      Author: hiephuu2001
 */


#include <stm32f103xx_i2c_driver.h>

uint16_t AHB_PreScaler[8] = {2,4,8,16,32,64,128,256,512};
uint8_t  APB1_PreScaler[4] = {2,4,8,16};

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


uint32_t RCC_GetPLLOutputClock()
{
	return 0;
}

/***********************************************************************************
 * @fn							- RCC_GetPCKL1Value
 *
 * @brief						-
 *
 * param[in]					- none
 * param[in]					-
 * param[in]					-
 *
 * @return						- none
 *
 * @Note						- none
 ************************************************************************************/
uint32_t RCC_GetPCKL1Value(void)
{
	uint32_t pclk1, SystemClk;
	uint8_t clksrc, temp, ahbp, apb1p;

	clksrc = ((RCC-> CFGR >> 2) && 0x03);

	if(clksrc == 0)
	{
		SystemClk = 16000000;
	} else if(clksrc == 1)
	{
		SystemClk = 8000000;
	} else if(clksrc == 2)
	{
		SystemClk = RCC_GetPLLOutputClock();
	}

	temp = ((RCC->CFGR >> 4) && 0xF);

	if(temp < 8 )
	{
		ahbp = 1 ;
	} else {
		ahbp = AHB_PreScaler[temp-8];
	}

	//for apb1
	temp = ((RCC->CFGR >> 8) && 0x7);

	if(temp < 4 )
	{
		apb1p = 1 ;
	} else {
		apb1p = APB1_PreScaler[temp-4];
	}

	pclk1 = (SystemClk / ahbp) /apb1p;
	return pclk1;
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
