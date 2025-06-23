/*
 * stm32f103xx_usart_driver.c
 *
 *  Created on: Jun 23, 2025
 *      Author: hiephuu2001
 */

#include<stm32f103xx_usart_driver.h>

/*
 * Peripheral Clock setup
 */
/***********************************************************************************
 * @fn							- USART_PeriClockControl
 *
 * @brief						-
 *
 * param[in]					-
 * param[in]					-
 * param[in]					-
 *
 * @return						- none
 *
 * @Note						- none
 ************************************************************************************/

void USART_PeriClockControl(USART_RegDef_t *pUSARTx, uint8_t EnorDi){
	if(EnorDi == ENABLE){
		if(pUSARTx == USART1)
		{
			USART1_PCLK_EN();
		} else if(pUSARTx == USART2)
		{
			USART2_PCLK_EN();
		} else if(pUSARTx == USART3)
		{
			USART3_PCLK_EN();
		} else if(pUSARTx == USART4)
		{
			USART4_PCLK_EN();
		} else if(pUSARTx == USART5)
		{
			USART5_PCLK_EN();
		}
	} else {
		if(pUSARTx == USART1)
		{
			USART1_PCLK_DI();
		} else if(pUSARTx == USART2)
		{
			USART2_PCLK_DI();
		} else if(pUSARTx == USART3)
		{
			USART3_PCLK_DI();
		} else if(pUSARTx == USART4)
		{
			USART4_PCLK_DI();
		} else if(pUSARTx == USART5)
		{
			USART5_PCLK_DI();
		}
	}
}

/*
 * Init and De-init
 */
void USART_Init(USART_Handle_t *pUSARTHandle)
{
	//Temporary variable
	uint32_t tempreg = 0;

/******************************** Configuration of CR1******************************************/

	//Implement the code to enable the Clock for given USART peripheral
	USART_PeriClockControl(pUSARTHandle->pUSARTx,ENABLE);

	//Enable USART Tx and Rx engines according to the USART_Mode configuration item
	if(pUSARTHandle->USART_Config.USART_Mode == USART_MODE_ONLY_RX)
	{
		//Implement the code to enable the Receiver bit field
		tempreg |= (1 << USART_CR1_RE);
	} else if(pUSARTHandle->USART_Config.USART_Mode == USART_MODE_ONLY_TX)
	{
		//Implement the code to enable the Tranmitter bit field
		tempreg |= (1 << USART_CR1_TE);
	} else if(pUSARTHandle->USART_Config.USART_Mode == USART_MODE_ONLY_TXRX)
	{
		//Implement the code to enable the Transmitter and Receiver bit field
		tempreg |= (1 << USART_CR1_RE) | (1<< USART_CR1_TE);
	}
	//Implement the code to configure the Word length configuration item
	tempreg |= pUSARTHandle->USART_Config.USART_WordLength << USART_CR1_M ;

	//Configuration of parity control bit fields
	if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_EN_EVEN){
		//Implement the code to enale the parity control
		tempreg |= (1 << USART_CR1_PCE);

		//Implement the code to enable EVEN parity
		//Not required because by default EVEN parity will be selected once you enable the parity control
	} else if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_EN_EVEN){
		//Implement the code to enale the parity control
		tempreg |= (1 << USART_CR1_PCE);

		 //Implement the code to enable ODD parity
		tempreg |= (1 << USART_CR1_PS );
	}
	 //Program the CR1 register
	pUSARTHandle->pUSARTx -> CR1 = tempreg;
/******************************** Configuration of CR2******************************************/

	tempreg = 0;
	//Implement the code to configure the number of stop bits inserted during USART frame transmission
	tempreg |= pUSARTHandle->USART_Config.USART_NoOfStopBits << USART_CR2_STOP ;

	//Program the CR2 register
	pUSARTHandle->pUSARTx->CR2 = tempreg;

/******************************** Configuration of CR3******************************************/
	tempreg = 0;

	//Configuration of USART hardware flow control
	if(pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_CTS)
	{
		//Implement the code to enable CTS flow control
		tempreg |= (1 << USART_CR3_CTSE);
	} else if(pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_RTS)
	{
		//Implement the code to enable RTS flow control
		tempreg |= (1 << USART_CR3_RTSE);
	} else if(pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_CTS_RTS)
	{
		//Implement the code to enable CTS flow control
		tempreg |= (1 << USART_CR3_CTSE) | (1<<USART_CR3_RTSE);
	}

	//Program the CR3 register
	pUSARTHandle->pUSARTx->CR3 = tempreg;

/******************************** Configuration of BRR(Baudrate register)******************************************/

	//Implement the code to configure the baud rate

}

void USART_DeInit(USART_RegDef_t *pUSARTx);


/*
 * Data Send and Receive
 */
/***********************************************************************************
 * @fn							- USART_SendData
 *
 * @brief						-
 *
 * param[in]					-
 * param[in]					-
 * param[in]					-
 *
 * @return						- none
 *
 * @Note						- none
 ************************************************************************************/
void USART_SendData(USART_Handle_t *pUSARTHandle,uint8_t *pTxBuffer, uint32_t Len)
{
	uint16_t *pdata;

	//Loop over until "Len" number of bytes are transferred
	for(uint32_t i = 0; i < Len; i++)
	{
		//Implement the code to wait until TXE flag is set in the SR
		while(!USART_GetFlagStatus(pUSARTHandle->pUSARTx, USART_FLAG_TXE));

		 //Check the USART_WordLength item for 9BIT or 8BIT in a frame
		if(pUSARTHandle->USART_Config.USART_HWFlowControl == USART_WORDLEN_9BITS)
		{
			//if 9BIT, load the DR with 2bytes masking the bits other than first 9 bits
			pdata = (uint16_t*)pTxBuffer;
			pUSARTHandle->pUSARTx->DR = (*pdata&(uint16_t)0x01FF);

			//check for USART_ParityControl
			if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
			{
				//No parity is used in this transfer. so, 9bits of user data will be sent
				//Implement the code to increment pTxBuffer twice
				pTxBuffer++;
				pTxBuffer++;
			}
			else
			{
				//Parity bit is used in this transfer . so , 8bits of user data will be sent
				//The 9th bit will be replaced by parity bit by the hardware
				pTxBuffer++;
			}
		} else {
			//This is 8bits data transfer
			pUSARTHandle -> pUSARTx -> DR = (*pTxBuffer&(uint8_t)0xFF);

			//Implement the code to increment the buffer address
			pTxBuffer++;
		}
	}

	//Implement the code to wait till TC flag is set in the SR
	while(!USART_GetFlagStatus(pUSARTHandle->pUSARTx, USART_FLAG_TC));
}


void USART_ReceiveData(USART_Handle_t *pUSARTx, uint8_t *pRxBuffer, uint32_t Len);
uint8_t USART_SendDataIT(USART_Handle_t *pUSARTHandle,uint8_t *pTxBuffer, uint32_t Len);
uint8_t USART_ReceiveDataIT(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len);

/*
 * IRQ Configuration and ISR handling
 */
void USART_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void USART_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void USART_IRQHandling(USART_Handle_t *pHandle);

/*
 * Other Peripheral Control APIs
 */
void USART_PeripheralControl(USART_RegDef_t *pUSARTx, uint8_t EnOrDi);

/***********************************************************************************
 * @fn							- USART_GetFlagStatus
 *
 * @brief						-
 *
 * param[in]					-
 * param[in]					-
 * param[in]					-
 *
 * @return						- none
 *
 * @Note						- none
 ************************************************************************************/
uint8_t USART_GetFlagStatus(USART_RegDef_t *pUSARTx , uint32_t FlagName){
	if (pUSARTx ->SR & FlagName )
	{
		return SET;
	}
	return RESET;
}

void USART_ClearFlag(USART_RegDef_t *pUSARTx, uint16_t StatusFlagName);

/***********************************************************************************
 * @fn							- USART_SetBaudRate
 *
 * @brief						-
 *
 * param[in]					-
 * param[in]					-
 * param[in]					-
 *
 * @return						- none
 *
 * @Note						- none
 ************************************************************************************/
void USART_SetBaudRate(USART_RegDef_t *pUSARTx,uint32_t BaudRate)
{

}


/*
 * Application callback
 */
void USART_ApplicationEventCallback(USART_Handle_t *pUSARTHandle,uint8_t AppEv);
