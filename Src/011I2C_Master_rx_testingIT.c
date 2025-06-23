/*
 * 011I2C_Master_rx_testingIT.c
 *
 *  Created on: Jun 21, 2025
 *      Author: hiephuu2001
 */

#include<stdio.h>
#include<string.h>
#include"stm32f103xx.h"

//flag variable
uint8_t rxComplt = RESET;

#define MY_ADDR 	0x61
#define SLAVE_ADDR	0x68

I2C_Handle_t I2C1Handle;

//rcv buffer
uint8_t rcv_buf[32];

void delay(void) {
	for (uint32_t i = 0; i < 500000 / 2; i++)
		;
}

/*
 * PB6 -> SCL
 * PB7 -> SDA
 */

void I2C1_GPIOInits(void) {
	uint32_t I2C1rmval = Get_I2C_RM();
	GPIO_Handle_t I2CPins;
	if (I2C1rmval == 0) {
		I2CPins.pGPIOx = GPIOB;
		I2CPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT_2MHz;
		I2CPins.GPIO_PinConfig.GPIO_CNF = GPIO_ALTFN_OP_OD;

		//Scl
		I2CPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_6;
		GPIO_Init(&I2CPins);

		//sda
		I2CPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_7;
		GPIO_Init(&I2CPins);
	} else {
		I2CPins.pGPIOx = GPIOB;
		I2CPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT_2MHz;
		I2CPins.GPIO_PinConfig.GPIO_CNF = GPIO_ALTFN_OP_OD;

		//Scl
		I2CPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_8;
		GPIO_Init(&I2CPins);

		//sda
		I2CPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_9;
		GPIO_Init(&I2CPins);
	}
}

void I2C1_Inits(void) {
	I2C1Handle.pI2Cx = I2C1;
	I2C1Handle.I2C_Config.I2C_ACKControl = I2C_ACK_ENABLE;
	I2C1Handle.I2C_Config.I2C_DeviceAddress = MY_ADDR;
	I2C1Handle.I2C_Config.I2C_FMDutyCycle = I2C_FM_DUTY_2;
	I2C1Handle.I2C_Config.I2C_SCLSpeed = I2C_SCL_SPEED_SM;

	I2C_Init(&I2C1Handle);
}

void GPIO_ButtonInit(void) {
	GPIO_Handle_t GPIOBtn;

	GPIOBtn.pGPIOx = GPIOA;
	GPIOBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GPIOBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GPIOBtn.GPIO_PinConfig.GPIO_CNF = GPIO_IP_PUPD;

	GPIO_Init(&GPIOBtn);
}

int main(void) {

	uint8_t commandcode, Len;
	//Config GPIO pin button
	GPIO_ButtonInit();

	//enable AFIO
	AFIO_PCLK_EN();

	//This function is used to initialize the GPIO pins to behave as SPI1
	I2C1_GPIOInits();

	//I2C1 peripheral configuration
	I2C1_Inits();

	//I2C_IRQ configuration
	I2C_IRQInterruptConfig(IRQ_NO_I2C1_EV, ENABLE);
	I2C_IRQInterruptConfig(IRQ_NO_I2C1_ER, ENABLE);
	//enable the I2C peripheral
	I2C_PeripheralControl(I2C1, ENABLE);

	//ack bit is made 1 after PE=1
	I2C_ManageAcking(I2C1, I2C_ACK_ENABLE);

	while (1) {
		//wait till button is pressed
		while (!GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_12))
			;

		//to avoid button de-bouncing related issue 200ms delay
		delay();

		commandcode = 0x51;

		while (I2C_MasterSendDataIT(&I2C1Handle, &commandcode, 1, SLAVE_ADDR,I2C_ENABLE_SR) != I2C_READY);

		while (I2C_MasterReceiveDataIT(&I2C1Handle, &Len, 1, SLAVE_ADDR, I2C_ENABLE_SR) != I2C_READY);

		commandcode = 0x52;
		while (I2C_MasterSendDataIT(&I2C1Handle, &commandcode, 1, SLAVE_ADDR,
		I2C_ENABLE_SR) != I2C_READY)
			;

		while (I2C_MasterReceiveDataIT(&I2C1Handle, rcv_buf, Len, SLAVE_ADDR,
		I2C_DISABLE_SR) != I2C_READY)
			;
		//wait Rx complete
		while (rxComplt != SET);
		rcv_buf[Len + 1] = '\0';

		rxComplt = RESET;
	}

}

void I2C1_EV_IRQHandler(void) {
	I2C_EV_IRQHandling(&I2C1Handle);
}

void I2C1_ER_IRQHandler(void) {
	I2C_ER_IRQHandling(&I2C1Handle);
}

void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t AppEv) {
	if (AppEv == I2C_EV_TX_CMPLT) {
		printf("Tx is completed \n");
	} else if (AppEv == I2C_EV_RX_CMPLT) {
		printf("Rx is completed \n");
		rxComplt = SET;
	} else if (AppEv == I2C_ERROR_AF) {
		printf("Error: ACK failure \n");
		//in master ack failure happens when slave fails to send ack for byte
		//send from the master
		I2C_CloseSendData(pI2CHandle);

		//generate the stop condition to release the bus
		I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

		//Hang in infinite loop
		while (1)
			;
	}
}
