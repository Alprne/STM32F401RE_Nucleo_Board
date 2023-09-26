/*
 * 006spi_tx_testing.c
 *
 *  Created on: 24 Tem 2023
 *      Author: alprn
 */

/*
*PB15 -> SPI2_MOSI
*PB14 -> SPI2_MISO
*PB13 -> SPI2_SCLK
*PB12 -> SPI2_NSS
*
*ALT function mode : AF05
*/

#include "stm32f401xx.h"
#include <string.h>

void SPI2_GPIOInits(void)
{
	GPIO_Handle_t SPIPins;

	SPIPins.pGPIOx = GPIOB;

	SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN ;
	SPIPins.GPIO_PinConfig.GPIO_PinAltFunMode = 5 ;
	SPIPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP ; //No need for open drain, I2C will need it.
	SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD ;
	SPIPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	//SCLK
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_13 ;
	GPIO_Init(&SPIPins);

	//NSS
	//SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_12 ;
	//GPIO_Init(&SPIPins);
//*********************For this application we don't want to use MISO and NSS because there is no slave.
//*********************So, don't configure those pins and they are free for other uses.
	//MISO
	//SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_14 ;
	//GPIO_Init(&SPIPins);

	//MOSI
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_15 ;
	GPIO_Init(&SPIPins);

}

void SPI2_Inits(void)
{
	SPI_Handle_t SPI2Handle;

	SPI2Handle.pSPIx = SPI2;
	SPI2Handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD ;
	SPI2Handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER ;
	SPI2Handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV4 ; //8MHz
	SPI2Handle.SPIConfig.SPI_DFF = SPI_DFF_8BITS ;
	SPI2Handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW ; //DEFAULT
	SPI2Handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW ; //DEFAULT
	SPI2Handle.SPIConfig.SPI_SSM = SPI_SSM_EN ; //Software Slave management for NSS pin.

	SPI_Init(&SPI2Handle);

}

int main()
{
	char user_data[] = "Hello World" ;

	//this function is used to initialize the GPIO pins to behave as SPI2 pins.
	SPI2_GPIOInits();

	SPI2_Inits();

	//This makes NSS signal internally high and avoids MODF error.
	SPI_SSIConfig(SPI2, ENABLE);

	//enable the spı2 peripheral.
	SPI_PeripheralControl(SPI2, ENABLE);

	SPI_SendData(SPI2, (uint8_t*)user_data, strlen(user_data));

	while (SPI_GetFlagStatus(SPI2, SPI_BUSY_FLAG));

	//Disable the SPI2 peripheral.
	SPI_PeripheralControl(SPI2, DISABLE);

	while(1);

	return 0 ;
}



















