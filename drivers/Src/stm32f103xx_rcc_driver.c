/*
 * stm32f103xx_rcc_driver.c
 *
 *  Created on: Jun 11, 2025
 *      Author: BIOS
 */


#include"stm32f103xx_rcc_driver.h"

uint16_t AHB_PreScaler[8] = {2,4,8,16,64,128,256,512};
uint8_t  APB1_PreScaler[4] = {2,4,8,16};

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
uint32_t RCC_GetPCKL2Value(void)
{
	uint32_t pclk2, SystemClk = 0;
	uint8_t clksrc, temp, ahbp, apb2p;

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
	temp = ((RCC->CFGR >> 11) && 0x7);

	if(temp < 4 )
	{
		apb2p = 1 ;
	} else {
		apb2p = APB1_PreScaler[temp-4];
	}

	pclk2 = (SystemClk / ahbp) /apb2p;
	return pclk2;
}

