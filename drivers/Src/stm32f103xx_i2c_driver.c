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

/***********************************************************************************
 * @fn							- I2C_MasterSendData
 *
 * @brief						-
 *
 * param[in]					- I2C Handle
 * param[in]					- data buffer
 * param[in]					- len buffer
 * param[in]					- SlaveAddr
 * param[in]					- uint8_t
 *
 * @return						- none
 *
 * @Note						- none
 ************************************************************************************/
void I2C_MasterSendData(I2C_Handle_t *pI2CHandle,uint8_t *pTxbuffer, uint32_t Len, uint8_t SlaveAddr,uint8_t Sr)
{
	//1. Generation the start condition

	//2. Confirm that start generation is completed by checking the SB flag in the SR1
	//   Note: Until SB is cleared SCL will be stretched (pulled to LOW)

	//3.  Send the address of the slave with r/nw bit set to w(0) (total 8 bits )

	//4. Confirm that address phase is completed by checking the ADDR flag in SR1

	//5. clear the ADDR flag according to its software sequence
	//Note:Until ADDR is cleared SCL will be stretched (pulled to low)

	//6.send the data until len becomes 0
}


// Other Peripheral Control APIS

/***********************************************************************************
 * @fn							- I2C_GenerateStartCondition
 *
 * @brief						- Enable bit Start in CR1
 *
 * param[in]					- I2C_RegDef
 * param[in]					-
 * param[in]					-
 *
 * @return						- none
 *
 * @Note						- none
 ************************************************************************************/
void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->CR1 |= (1<<I2C_CR1_START);
}

/***********************************************************************************
 * @fn							- I2C_GenerateStopCondition
 *
 * @brief						- Enable bit Stop in CR1
 *
 * param[in]					- I2C_RegDef
 * param[in]					-
 * param[in]					-
 *
 * @return						- none
 *
 * @Note						- none
 ************************************************************************************/
void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->CR1 |= (1<<I2C_CR1_STOP);
}

/***********************************************************************************
 * @fn							- I2C_GetFlagStatus
 *
 * @brief						- Get register is set or reset
 *
 * param[in]					- I2C_RegDef
 * param[in]					-
 * param[in]					-
 *
 * @return						- none
 *
 * @Note						- none
 ************************************************************************************/
uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx , uint32_t FlagName)
{
	if(pI2Cx->CR1 & FlagName)
	{
		return FLAG_SET;
	} else {
		return FLAG_RESET;
	}
}
