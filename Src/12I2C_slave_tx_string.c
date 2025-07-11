/*
 * 12I2C_slave_tx_string.c
 *
 *  Created on: Jun 22, 2025
 *      Author: hiephuu2001
 */


#include<stdio.h>
#include<string.h>
#include"stm32f103xx.h"

#define SLAVE_ADDR	0x68
#define MY_ADDR	SLAVE_ADDR

I2C_Handle_t I2C1Handle;

//tx buffer
uint8_t Tx_buf[32] = "STM32 slave mode testing..";

void delay(void) {
	for (uint32_t i = 0; i < 500000 / 2; i++);
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

	I2C_SlaveEnableDisableCallbackEvents(I2C1,ENABLE);

	//enable the I2C peripheral
	I2C_PeripheralControl(I2C1, ENABLE);

	//ack bit is made 1 after PE=1
	I2C_ManageAcking(I2C1, I2C_ACK_ENABLE);

	while(1);
}

void I2C1_EV_IRQHandler(void) {
	I2C_EV_IRQHandling(&I2C1Handle);
}

void I2C1_ER_IRQHandler(void) {
	I2C_ER_IRQHandling(&I2C1Handle);
}

void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t AppEv) {

	static uint8_t commandCode =0;;
	static uint8_t Cnt = 0;

	if(AppEv == I2C_EV_DATA_REQ){
		//master wants some data. slave has to send it
		if(commandCode == 0x51)
		{
			//send the length information to the master
			I2C_SlaveSendData(pI2CHandle->pI2Cx, strlen((char*)Tx_buf));
		} else if(commandCode == 0x52)
		{
			//send the contents of Tx_buf
			I2C_SlaveSendData(pI2CHandle->pI2Cx, Tx_buf[Cnt++]);
		}
	} else if (AppEv == I2C_EV_DATA_RCV){
		//Data if waiting for the slave to read. slave has to read it
		commandCode  = I2C_SlaveReceiveData(pI2CHandle->pI2Cx);
	} else if (AppEv == I2C_ERROR_AF){
		//This happens only during slave txing
		//master has sent the NACK. so slave should understand that master doesn't need more data.
		commandCode = 0xff;
		Cnt = 0;

	} else if (AppEv == I2C_EV_STOP){
		//This happens only during slave reception
		//master has ended the I2C communication with the slave.

	}
}
