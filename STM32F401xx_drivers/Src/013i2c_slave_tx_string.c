/*
 * 013i2c_slave_tx_string.c
 *
 *  Created on: 12 Eyl 2023
 *      Author: alprn
 */

#include "stm32f401xx.h"
#include <string.h>
#include <stdio.h>

#define SLAVE_ADDR 		0x68

#define MY_ADDR			SLAVE_ADDR

void delay(void)
{
	for(uint32_t i = 0 ; i < 500000/2 ; i++);
}

I2C_Handle_t I2C1Handle ;

//Some data
uint8_t Tx_buf[32] = "STM32 Slave Mode Testing.."; //Make sure you don't send more then 32 bytes, that is arduino wire lib. limit.

/*
 * PB9 -> SDA
 * PB8 -> SCL
 */

void I2C1_GPIOInits(void)
{
	GPIO_Handle_t I2C1Pins;

	I2C1Pins.pGPIOx = GPIOB;

	I2C1Pins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN ;
	I2C1Pins.GPIO_PinConfig.GPIO_PinAltFunMode = 4 ;
	I2C1Pins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD ; //Need for open drain, I2C will need it.
	I2C1Pins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD ;
	I2C1Pins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	//SCLK
	I2C1Pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_8 ;
	GPIO_Init(&I2C1Pins);

	//SDA
	I2C1Pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_9 ;
	GPIO_Init(&I2C1Pins);

}

void I2C1_Inits(void)
{
	I2C1Handle.pI2Cx = I2C1;
	I2C1Handle.I2C_Config.I2C_ACKControl = I2C_ACK_ENABLE;
	I2C1Handle.I2C_Config.I2C_DeviceAddress = MY_ADDR;
	I2C1Handle.I2C_Config.I2C_FMDutyCycle = I2C_FM_DUTY_2 ;
	I2C1Handle.I2C_Config.I2C_SCLSpeed = I2C_SCL_SPEED_SM ;

	I2C_Init(&I2C1Handle);
}

int main()
{

	//I2C pin Init.
	I2C1_GPIOInits();

	//I2C peripheral configuration
	I2C1_Inits();

	//I2C IRQ Configuration
	I2C_IRQInterruptConfig(IRQ_NO_I2C1_EV, ENABLE);
	I2C_IRQInterruptConfig(IRQ_NO_I2C1_ER, ENABLE);

	I2C_SlaveEnableDisableCallbackEvents(I2C1, ENABLE);

	//Enable the i2c peripheral

	I2C_ManageAcking(I2C1, I2C_ACK_ENABLE);

	while(1);
}



void I2C1_ER_IRQHandler(void)
{
	I2C_ER_IRQHandling(&I2C1Handle);
}

void I2C1_EV_IRQHandler(void)
{
	I2C_EV_IRQHandling(&I2C1Handle);
}

void I2C_ApplicationEventCallBack(I2C_Handle_t *pI2CHandle, uint8_t AppEv)
{
	static uint8_t commandCode = 0;
	static uint8_t Cnt = 0;

	if(AppEv == I2C_EV_DATA_REQ)
	{
		//Master wants some data.
		//Slave has to send it.
		if(commandCode == 0x51)
		{
			//Send the length info to the master.
			I2C_SlaveSendData(pI2CHandle->pI2Cx, strlen((char*)Tx_buf));
		}else if(commandCode == 0x52)
		{
			//Send the contents of Tx_buf
			I2C_SlaveSendData(pI2CHandle->pI2Cx, Tx_buf[Cnt++]);
		}
	}else if(AppEv == I2C_EV_DATA_RCV)
	{
		//Data is waiting for the slave to read.
		//Slave has to read it.
		commandCode = I2C_SlaveReceiveData(pI2CHandle->pI2Cx);
	}else if(AppEv == I2C_ERROR_AF)
	{
		//This happens only during the slave txing(Transmission).
		//Master has sent the NACK.
		//So slave should understand that master doesn't need more data.
		commandCode = 0xFF;
		Cnt = 0 ;
	}else if(AppEv == I2C_EV_STOP)
	{
		//This happens only during slave reception.
		//Master has ended the I2C communication with the slave.
	}

}
