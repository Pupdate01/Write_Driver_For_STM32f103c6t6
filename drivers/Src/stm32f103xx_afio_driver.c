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

/***********************************************************************************
 * @fn							- Get_SPI1_RM
 *
 * @brief						- This get value of bit REMAP
 *
 * param[in]					- none
 * param[in]					-
 * param[in]					-
 *
 * @return						- none
 *
 * @Note						- none
 *************************************************************************************/
uint32_t Get_SPI1_RM()
{
	uint32_t Spi1rm = (AFIO -> MAPR >> 0) & 0x01;

	return Spi1rm;
}

/***********************************************************************************
 * @fn							- Get_I2C_RM
 *
 * @brief						- This get value of bit REMAP
 *
 * param[in]					- none
 * param[in]					-
 * param[in]					-
 *
 * @return						- none
 *
 * @Note						- none
 *************************************************************************************/
uint32_t Get_I2C_RM(){
	uint32_t I2crm = (AFIO -> MAPR >> 1) & 0x01;

	return I2crm;
}

/***********************************************************************************
 * @fn							- Get_UART1_RM
 *
 * @brief						- This get value of bit REMAP
 *
 * param[in]					- none
 * param[in]					-
 * param[in]					-
 *
 * @return						- none
 *
 * @Note						- none
 *************************************************************************************/
uint32_t Get_UART1_RM(){
	uint32_t Uart1rm = (AFIO -> MAPR >> 2) & 0x01;

	return Uart1rm;
}

/***********************************************************************************
 * @fn							- Get_UART2_RM
 *
 * @brief						- This get value of bit REMAP
 *
 * param[in]					- none
 * param[in]					-
 * param[in]					-
 *
 * @return						- none
 *
 * @Note						- none
 *************************************************************************************/
uint32_t Get_UART2_RM(){
	uint32_t Uart2rm = (AFIO -> MAPR >> 3) & 0x01;

	return Uart2rm;
}

/***********************************************************************************
 * @fn							- Get_UART3_RM
 *
 * @brief						- This get value of bit REMAP
 *
 * param[in]					- none
 * param[in]					-
 * param[in]					-
 *
 * @return						- none
 *
 * @Note						- none
 *************************************************************************************/
uint32_t Get_UART3_RM(){
	uint32_t Uart3rm = (AFIO -> MAPR >> 4) & 0x03;

	return Uart3rm;
}

/***********************************************************************************
 * @fn							- Get_TIM1_RM
 *
 * @brief						- This get value of bit REMAP
 *
 * param[in]					- none
 * param[in]					-
 * param[in]					-
 *
 * @return						- none
 *
 * @Note						- none
 *************************************************************************************/
uint32_t Get_TIM1_RM()
{
	uint32_t Tim1rm = (AFIO -> MAPR >> 6) & 0x03;

	return Tim1rm;
}

/***********************************************************************************
 * @fn							- Get_TIM2_RM
 *
 * @brief						- This get value of bit REMAP
 *
 * param[in]					- none
 * param[in]					-
 * param[in]					-
 *
 * @return						- none
 *
 * @Note						- none
 *************************************************************************************/
uint32_t Get_TIM2_RM()
{
	uint32_t Tim2rm = (AFIO -> MAPR >> 8) & 0x03;

	return Tim2rm;
}

/***********************************************************************************
 * @fn							- Get_TIM3_RM
 *
 * @brief						- This get value of bit REMAP
 *
 * param[in]					- none
 * param[in]					-
 * param[in]					-
 *
 * @return						- none
 *
 * @Note						- none
 *************************************************************************************/
uint32_t Get_TIM3_RM()
{
	uint32_t Tim3rm = (AFIO -> MAPR >> 10) & 0x03;

	return Tim3rm;
}

/***********************************************************************************
 * @fn							- Get_CAN_RM
 *
 * @brief						- This get value of bit REMAP
 *
 * param[in]					- none
 * param[in]					-
 * param[in]					-
 *
 * @return						- none
 *
 * @Note						- none
 *************************************************************************************/
uint32_t Get_TIM4_RM()
{
	uint32_t Tim4rm = (AFIO -> MAPR >> 12) & 0x01;

	return Tim4rm;
}

/***********************************************************************************
 * @fn							- Get_UART3_RM
 *
 * @brief						- This get value of bit REMAP
 *
 * param[in]					- none
 * param[in]					-
 * param[in]					-
 *
 * @return						- none
 *
 * @Note						- none
 *************************************************************************************/
uint32_t Get_CAN_RM()
{
	uint32_t Canrm = (AFIO -> MAPR >> 13) & 0x03;

	return Canrm;
}

/***********************************************************************************
 * @fn							- Get_PD01_RM
 *
 * @brief						- This get value of bit REMAP
 *
 * param[in]					- none
 * param[in]					-
 * param[in]					-
 *
 * @return						- none
 *
 * @Note						- none
 *************************************************************************************/
uint32_t Get_PD01_RM()
{
	uint32_t Pd01rm = (AFIO -> MAPR >> 15) & 0x01;

	return Pd01rm;
}

/***********************************************************************************
 * @fn							- Get_TIM5CH4_RM
 *
 * @brief						- This get value of bit REMAP
 *
 * param[in]					- none
 * param[in]					-
 * param[in]					-
 *
 * @return						- none
 *
 * @Note						- none
 *************************************************************************************/
uint32_t Get_TIM5CH4_RM()
{
	uint32_t Tim5ch4rm = (AFIO -> MAPR >> 16) & 0x01;

	return Tim5ch4rm;
}

/***********************************************************************************
 * @fn							- Get_ADC1_ETRGINJ_RM
 *
 * @brief						- This get value of bit REMAP
 *
 * param[in]					- none
 * param[in]					-
 * param[in]					-
 *
 * @return						- none
 *
 * @Note						- none
 *************************************************************************************/
uint32_t Get_ADC1_ETRGINJ_RM()
{
	uint32_t Adc1injrm = (AFIO -> MAPR >> 17) & 0x01;

	return Adc1injrm;
}

/***********************************************************************************
 * @fn							- Get_ADC1_ETRGREG_RM
 *
 * @brief						- This get value of bit REMAP
 *
 * param[in]					- none
 * param[in]					-
 * param[in]					-
 *
 * @return						- none
 *
 * @Note						- none
 *************************************************************************************/
uint32_t Get_ADC1_ETRGREG_RM()
{
	uint32_t adc1regrm = (AFIO -> MAPR >> 18) & 0x01;

	return adc1regrm;
}

/***********************************************************************************
 * @fn							- Get_ADC2_ETRGINJ_RM
 *
 * @brief						- This get value of bit REMAP
 *
 * param[in]					- none
 * param[in]					-
 * param[in]					-
 *
 * @return						- none
 *
 * @Note						- none
 *************************************************************************************/
uint32_t Get_ADC2_ETRGINJ_RM()
{
	uint32_t Adc2injrm = (AFIO -> MAPR >> 19) & 0x01;

	return Adc2injrm;
}

/***********************************************************************************
 * @fn							- Get_ADC2_ETRGREG_RM
 *
 * @brief						- This get value of bit REMAP
 *
 * param[in]					- none
 * param[in]					-
 * param[in]					-
 *
 * @return						- none
 *
 * @Note						- none
 *************************************************************************************/
uint32_t Get_ADC2_ETRGREG_RM()
{
	uint32_t Adc2regrm = (AFIO -> MAPR >> 20) & 0x01;

	return Adc2regrm;
}

/***********************************************************************************
 * @fn							- Get_SWJ_CFG
 *
 * @brief						- This get value of bit REMAP
 *
 * param[in]					- none
 * param[in]					-
 * param[in]					-
 *
 * @return						- none
 *
 * @Note						- none
 *************************************************************************************/
uint32_t Get_SWJ_CFG()
{
	uint32_t Swjcfg = (AFIO -> MAPR >> 24) & 0x07;

	return Swjcfg;
}
