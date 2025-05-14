/*
 * stm32f103xx_afio_driver.c
 *
 *  Created on: May 14, 2025
 *      Author: hiephuu2001
 */


#include<stm32f1xx_afio_driver.h>

/*
 * These API configure REMAP
 */

/***********************************************************************************
 * @fn							- SPI1_ControlRemap
 *
 * @brief						- This function enable or disable Remap of SPI1
 *
 * param[in]					- ENABLE or DISABLE macros
 * param[in]					-
 * param[in]					-
 *
 * @return						- none
 *
 * @Note						- none
 */
void SPI1_ControlRemap(uint8_t EnorDi)
{
	if(EnorDi == ENABLE){
		SPI1_REMAP_EN();
	} else
	{
		SPI1_REMAP_DI();
	}
}

/***********************************************************************************
 * @fn							- I2C1_ControlRemap
 *
 * @brief						- This function enable or disable Remap of I2C1
 *
 * param[in]					- ENABLE or DISABLE macros
 * param[in]					-
 * param[in]					-
 *
 * @return						- none
 *
 * @Note						- none
 *************************************************************************************/
void I2C1_ControlRemap(uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		I2C1_REMAP_EN();
	} else
	{
		I2C1_REMAP_DI();
	}
}

/***********************************************************************************
 * @fn							- UART1_ControlRemap
 *
 * @brief						- This function enable or disable Remap of UART1
 *
 * param[in]					- ENABLE or DISABLE macros
 * param[in]					-
 * param[in]					-
 *
 * @return						- none
 *
 * @Note						- none
 *************************************************************************************/
void UART1_ControlRemap(uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		UART1_REMAP_EN();
	} else
	{
		UART1_REMAP_DI();
	}
}

/***********************************************************************************
 * @fn							- UART2_ControlRemap
 *
 * @brief						- This function enable or disable Remap of UART2
 *
 * param[in]					- ENABLE or DISABLE macros
 * param[in]					-
 * param[in]					-
 *
 * @return						- none
 *
 * @Note						- none
 *************************************************************************************/
void UART2_ControlRemap(uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		UART2_REMAP_EN();
	} else {
		UART2_REMAP_DI();
	}
}

/***********************************************************************************
 * @fn							- UART3_ControlRemap
 *
 * @brief						- This function enable or disable Remap of UART3
 *
 * param[in]					- Value of UART3 Remap
 * param[in]					-
 * param[in]					-
 *
 * @return						- none
 *
 * @Note						- none
 *************************************************************************************/
void UART3_ControlRemap(uint8_t UART3RMVal)
{
	//clear 2 bits of UART3_REMAP
	AFIO->MAPR &= ~(0x03 << 4);

	//set 2 bits of UART3_REMAP
	AFIO->MAPR &= ~(UART3RMVal << 4);
}

/***********************************************************************************
 * @fn							- TIM1_ControlRemap
 *
 * @brief						- This function enable or disable Remap of TIM1
 *
 * param[in]					- Value of TIM1 Remap
 * param[in]					-
 * param[in]					-
 *
 * @return						- none
 *
 * @Note						- none
 *************************************************************************************/
void TIM1_ControlRemap(uint8_t TIM1RMVal){
	//clear 2 bits of TIM1_REMAP
	AFIO->MAPR &= ~(0x03 << 6);

	//set 2 bits of TIM1_REMAP
	AFIO->MAPR |= (TIM1RMVal << 6);
}

/***********************************************************************************
 * @fn							- TIM2_ControlRemap
 *
 * @brief						- This function enable or disable Remap of TIM1
 *
 * param[in]					- Value of TIM2 Remap
 * param[in]					-
 * param[in]					-
 *
 * @return						- none
 *
 * @Note						- none
 *************************************************************************************/
void TIM2_ControlRemap(uint8_t TIM2RMVal){
	//clear 2 bits of TIM2_REMAP
	AFIO->MAPR &= ~(0x03 << 8);

	//set 2 bits of TIM2_REMAP
	AFIO->MAPR |= (TIM2RMVal << 8);
}

/***********************************************************************************
 * @fn							- TIM2_ControlRemap
 *
 * @brief						- This function enable or disable Remap of TIM1
 *
 * param[in]					- Value of TIM2 Remap
 * param[in]					-
 * param[in]					-
 *
 * @return						- none
 *
 * @Note						- none
 *************************************************************************************/
void TIM3_ControlRemap(uint8_t TIM3RMVal){
	//clear 2 bits of TIM3_REMAP
	AFIO->MAPR &= ~(0x03 << 10);

	//set 2 bits of TIM3_REMAP
	AFIO->MAPR |= (TIM3RMVal << 10);
}

/***********************************************************************************
 * @fn							- TIM4_ControlRemap
 *
 * @brief						- This function enable or disable Remap of TIM4
 *
 * param[in]					- ENABLE or DISABLE macros
 * param[in]					-
 * param[in]					-
 *
 * @return						- none
 *
 * @Note						- none
 *************************************************************************************/
void TIM4_ControlRemap(uint8_t EnorDi){
	if(EnorDi == ENABLE)
	{
		TIM4_REMAP_EN();
	} else {
		TIM4_REMAP_DI();
	}
}

/***********************************************************************************
 * @fn							- CAN_ControlRemap
 *
 * @brief						- This function enable or disable Remap of CAN
 *
 * param[in]					- Value of CAN Remap
 * param[in]					-
 * param[in]					-
 *
 * @return						- none
 *
 * @Note						- none
 *************************************************************************************/
void CAN_ControlRemap(uint8_t CANRMVal){
	//clear 2 bits of CAN_REMAP
	AFIO->MAPR &= ~(0x03 << 13);

	//set 2 bits of CAN_REMAP
	AFIO->MAPR |= (CANRMVal << 13);
}

/***********************************************************************************
 * @fn							- PD01_ControlRemap
 *
 * @brief						- This function enable or disable Remap of UART3
 *
 * param[in]					- ENABLE or DISABLE macro
 * param[in]					-
 * param[in]					-
 *
 * @return						- none
 *
 * @Note						- none
 *************************************************************************************/
void PD01_ControlRemap(uint8_t EnorDi){
	if(EnorDi == ENABLE)
	{
		PD01_REMAP_EN();
	} else {
		PD01_REMAP_DI();
	}
}

/***********************************************************************************
 * @fn							- TIM5CH4_ControlRemap
 *
 * @brief						- This function enable or disable Remap of UART3
 *
 * param[in]					- ENABLE or DISABLE macros
 * param[in]					-
 * param[in]					-
 *
 * @return						- none
 *
 * @Note						- none
 *************************************************************************************/
void TIM5CH4_ControlRemap(uint8_t EnorDi){
	if(EnorDi == ENABLE)
	{
		TIM5CH4_REMAP_EN();
	} else {
		TIM5CH4_REMAP_DI();
	}
}

/***********************************************************************************
 * @fn							- ADC1_ETRGINJ_ControlRemap
 *
 * @brief						- This function enable or disable Remap of UART3
 *
 * param[in]					- ENABLE or DISABLE macros
 * param[in]					-
 * param[in]					-
 *
 * @return						- none
 *
 * @Note						- none
 *************************************************************************************/
void ADC1_ETRGINJ_ControlRemap(uint8_t EnorDi){
	if(EnorDi == ENABLE)
		{
			ADC1_ETRGINJ_REMAP_EN();
		} else {
			ADC1_ETRGINJ_REMAP_DI();
		}
}

/***********************************************************************************
 * @fn							- ADC1_ETRGREG_ControlRemap
 *
 * @brief						- This function enable or disable Remap of UART3
 *
 * param[in]					- ENABLE or DISABLE macros
 * param[in]					-
 * param[in]					-
 *
 * @return						- none
 *
 * @Note						- none
 *************************************************************************************/
void ADC1_ETRGREG_ControlRemap(uint8_t EnorDi){
	if(EnorDi == ENABLE)
	{
		ADC1_ETRGREG_REMAP_EN();
	} else {
		ADC1_ETRGREG_REMAP_DI();
	}
}

/***********************************************************************************
 * @fn							- ADC2_ETRGINJ_ControlRemap
 *
 * @brief						- This function enable or disable Remap of UART3
 *
 * param[in]					- ENABLE or DISABLE macros
 * param[in]					-
 * param[in]					-
 *
 * @return						- none
 *
 * @Note						- none
 *************************************************************************************/
void ADC2_ETRGINJ_ControlRemap(uint8_t EnorDi){
	if(EnorDi == ENABLE)
	{
		ADC2_ETRGINJ_REMAP_EN();
	} else {
		ADC2_ETRGINJ_REMAP_DI();
	}
}

/***********************************************************************************
 * @fn							- ADC2_ETRGREG_ControlRemap
 *
 * @brief						- This function enable or disable Remap of UART3
 *
 * param[in]					- ENABLE or DISABLE macros
 * param[in]					-
 * param[in]					-
 *
 * @return						- none
 *
 * @Note						- none
 *************************************************************************************/
void ADC2_ETRGREG_ControlRemap(uint8_t EnorDi){
	if(EnorDi == ENABLE)
	{
		ADC2_ETRGREG_REMAP_EN();
	} else {
		ADC2_ETRGREG_REMAP_DI();
	}
}

/***********************************************************************************
 * @fn							- SWJ_CFG
 *
 * @brief						- This function enable or disable Remap of TIM1
 *
 * param[in]					- Value of SWJCFGVal SWJ_CFG
 * param[in]					-
 * param[in]					-
 *
 * @return						- none
 *
 * @Note						- none
 *************************************************************************************/
void SWJ_CFG(uint8_t SWJCFGVal){
	//clear 3 bits of CAN_REMAP
		AFIO->MAPR &= ~(0x07 << 24);

		//set 2 bits of CAN_REMAP
		AFIO->MAPR |= (SWJCFGVal << 24);
}
