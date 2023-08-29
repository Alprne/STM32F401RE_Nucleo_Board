/*
 * stm32f401xx_spi_driver.c
 *
 *  Created on: 20 Tem 2023
 *      Author: alprn
 */
#include "stm32f401xx_spi_driver.h"

static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle);

// Peripheral Clock setup
/*****************************************************************
* @fn 				- SPI_PeriClockControl
*
* @brief 			- This function enables or disables peripheral clock for the given SPI periperal.
*
* @param[in] 		- Base address of the SPI peripheral
* @param[in] 		- Enable or Disable macros
* @param[in] 		-
*
* @return 			- None
*
* @Note 			- None
*/
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
		{
			if(pSPIx == SPI1)
			{
				SPI1_PCLK_EN();
			}
			else if(pSPIx == SPI2)
			{
				SPI2_PCLK_EN();
			}
			else if(pSPIx == SPI3)
			{
				SPI3_PCLK_EN();
			}
			else if(pSPIx == SPI4)
			{
				SPI4_PCLK_EN();
			}
		}
		else
		{
			if(pSPIx == SPI1)
			{
				SPI1_PCLK_DI();
			}
			else if(pSPIx == SPI2)
			{
				SPI2_PCLK_DI();
			}
			else if(pSPIx == SPI3)
			{
				SPI3_PCLK_DI();
			}
			else if(pSPIx == SPI4)
			{
				SPI4_PCLK_DI();
			}
		}
}


//Init and De-init
/*****************************************************************
* @fn 				- SPI_Init
*
* @brief 			- This function initialize the given SPI.
*
* @param[in] 		- Base address of the SPI peripheral
* @param[in] 		-
* @param[in] 		-
*
* @return 			- None
*
* @Note 			- None
*/
void SPI_Init(SPI_Handle_t *pSPIHandle)
{
	//Enable the peripheral clock.

	SPI_PeriClockControl(pSPIHandle->pSPIx, ENABLE);

	//first lets configure the SPI_CR1 register

	uint32_t tempreg = 0 ;

	//1. Configure the device mode.
	tempreg |= pSPIHandle->SPIConfig.SPI_DeviceMode << SPI_CR1_MSTR ;

	//2. Configure the bus config.
	if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD)
	{
		//bidi mode should be cleared
		tempreg &= ~(1 << SPI_CR1_BIDIMODE);
	}else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD)
	{
		//bidi mode should be set
		tempreg |= (1 << SPI_CR1_BIDIMODE);
	}else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY)
	{
		//bidi mode should be cleared
		tempreg &= ~(1 << SPI_CR1_BIDIMODE);
		//RXONLY bit must be set
		tempreg |= (1 << SPI_CR1_RXONLY);
	}
	//3. Configure the spi serial clock speed (baud rate)
	tempreg |= pSPIHandle->SPIConfig.SPI_SclkSpeed << SPI_CR1_BR0 ;

	//4. Configure the DFF
	tempreg |= pSPIHandle->SPIConfig.SPI_DFF << SPI_CR1_DFF ;

	//5. Configure the CPOL
	tempreg |= pSPIHandle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL ;

	//5. Configure the CPHA
	tempreg |= pSPIHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA ;

	//6. Configure the SSM
	tempreg |= pSPIHandle->SPIConfig.SPI_SSM << SPI_CR1_SSM ;

	pSPIHandle->pSPIx->CR1 = tempreg ; // When freshly initializing tempreg we can use assignment operator "="
}

/*****************************************************************
* @fn 				- SPI_Deinit
*
* @brief 			- This function deinitialize the given SPI.
*
* @param[in] 		- Base address of the SPI peripheral
* @param[in] 		-
* @param[in] 		-
*
* @return 			- None
*
* @Note 			- None
*/
void SPI_DeInit(SPI_RegDef_t *pSPIx)
{
	if(pSPIx == SPI1)
	{
		SPI1_REG_RESET();
	}
	else if(pSPIx == SPI2)
	{
		SPI2_REG_RESET();
	}
	else if(pSPIx == SPI3)
	{
		SPI3_REG_RESET();
	}
	else if(pSPIx == SPI4)
	{
		SPI4_REG_RESET();
	}
}


/*****************************************************************
* @fn 				- SPI_GetFlagStatus
*
* @brief 			- This function used to get flag status data.
*
* @param[in] 		- Base address of the SPI peripheral
* @param[in] 		- FlagName
* @param[in] 		-
*
* @return 			- Flag Status
*
* @Note 			- None
*/
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx , uint32_t FlagName)
{
	if(pSPIx->SR & FlagName)
	{
		return FLAG_SET;
	}
	return FLAG_RESET ;
}


// Data Send and Receive
/*****************************************************************
* @fn 				- SPI_SendData
*
* @brief 			- This function used to send data from master or slave.
*
* @param[in] 		- Base address of the SPI peripheral
* @param[in] 		- Data
* @param[in] 		- Data Length
*
* @return 			- None
*
* @Note 			- This is blocking call
*/
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTXBuffer, uint32_t Len) //Blocking API, because the function will wait until all the bytes are transmitted.
{
	while(Len > 0)
	{
		//1. Wait for until the TXE is set.
		while(SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG) == FLAG_RESET );

		//2. Check the DFF bit in CR1
		if(pSPIx->CR1 & (1 << SPI_CR1_DFF))
		{
			//16bit DFF
			//1. Load the data in to the DR
			pSPIx->DR =  *((uint16_t*)pTXBuffer);
			Len--;
			Len--;
			(uint16_t*)pTXBuffer++;
		}else
		{
			//8 bit DFF
			//1. Load the data in to the DR
			pSPIx->DR =  *pTXBuffer;
			Len--;
			pTXBuffer++;
		}
	}
}

/*****************************************************************
* @fn 				- SPI_ReceiveData
*
* @brief 			- This function used to receive data from the master or slave.
*
* @param[in] 		- Base address of the SPI peripheral
* @param[in] 		- Data Length
* @param[in] 		- Data
*
* @return 			- None
*
* @Note 			- None
*/
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRXBuffer, uint32_t Len)
{
	while(Len > 0)
		{
			//1. Wait for until the RXNE is set.
			while(SPI_GetFlagStatus(pSPIx, SPI_RXNE_FLAG) == FLAG_RESET );

			//2. Check the DFF bit in CR1
			if(pSPIx->CR1 & (1 << SPI_CR1_DFF))
			{
				//16bit DFF
				//1. Load the data from DR to RxBuffer Address.
				*((uint16_t*)pRXBuffer) = pSPIx->DR;
				Len--;
				Len--;
				(uint16_t*)pRXBuffer++;
			}else
			{
				//8 bit DFF
				//1. Load the data in to the DR
				*(pRXBuffer) = pSPIx->DR;
				Len--;
				pRXBuffer++;
			}
		}
}

/*****************************************************************
* @fn 				- SPI_SendDataIT
*
* @brief 			- This function used to send data from master or slave.
*
* @param[in] 		- Base address of the SPI peripheral
* @param[in] 		- Data
* @param[in] 		- Data Length
*
* @return 			- The SPI variables for store.
*
* @Note 			- This is non-blocking call
*/
uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTXBuffer, uint32_t Len)
{
	uint8_t state = pSPIHandle->TxState;

	if(state != SPI_BUSY_IN_TX)
	{
		//1. Save the Tx buffer address and Len information in some global variables.
		pSPIHandle->pTxBuffer = pTXBuffer ;
		pSPIHandle->TxLen = Len ;

		//2. Mark the SPI state as busy in transmission so that
		//	 no other code can take over same SPI peripheral until transmission is over
		pSPIHandle->TxState = SPI_BUSY_IN_TX ;

		//3. Enable the TXEIE control hit to get interrupt whenever TXE flag is set in SR
		pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_TXNEIE) ;

		//4. Data Transmission will handled by the ISR code (will implement later)

	}

	return state ;
}

/*****************************************************************
* @fn 				- SPI_ReceiveDataIT
*
* @brief 			- This function used to receive data from the master or slave.
*
* @param[in] 		- Base address of the SPI peripheral
* @param[in] 		- Data Length
* @param[in] 		- Data
*
* @return 			- None
*
* @Note 			- None
*/
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRXBuffer, uint32_t Len)
{
	uint8_t state = pSPIHandle->RxState;

	if(state != SPI_BUSY_IN_RX)
	{
		//1. Save the Rx buffer address and Len information in some global variables.
		pSPIHandle->pRxBuffer = pRXBuffer ;
		pSPIHandle->RxLen = Len ;

		//2. Mark the SPI state as busy in transmission so that
		//	 no other code can take over same SPI peripheral until transmission is over
		pSPIHandle->RxState = SPI_BUSY_IN_RX ;

		//3. Enable the TXEIE control hit to get interrupt whenever TXE flag is set in SR
		pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_RXNEIE) ;

		//4. Data Transmission will handled by the ISR code (will implement later)

	}

	return state ;
}

/*****************************************************************
* @fn 				- SPI_PeripheralControl
*
* @brief 			- This function is Enable or Disable the SPI_Peripheral in the ARM M4 processor
*
* @param[in] 		- Base address of the SPI peripheral
* @param[in] 		- Enable or Disable
* @param[in] 		-
*
* @return 			- None
*
* @Note 			- None
*/
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		pSPIx->CR1 |= (1 << SPI_CR1_SPE) ;
	}else
	{
		pSPIx->CR1 &= ~(1 << SPI_CR1_SPE) ;
	}
}

/*****************************************************************
* @fn 				- SPI_SSIConfig
*
* @brief 			- This function is Enable or Disable the SPI SSI bit.
*
* @param[in] 		- Base address of the SPI peripheral
* @param[in] 		- Enable or Disable
* @param[in] 		-
*
* @return 			- None
*
* @Note 			- None
*/
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		pSPIx->CR1 |= (1 << SPI_CR1_SSI) ;
	}else
	{
		pSPIx->CR1 &= ~(1 << SPI_CR1_SSI) ;
	}
}

/*****************************************************************
* @fn 				- SPI_SSIConfig
*
* @brief 			- This function is Enable or Disable the SPI SSI bit.
*
* @param[in] 		- Base address of the SPI peripheral
* @param[in] 		- Enable or Disable
* @param[in] 		-
*
* @return 			- None
*
* @Note 			- None
*/
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		pSPIx->CR2 |= (1 << SPI_CR2_SSOE) ;
	}else
	{
		pSPIx->CR2 &= ~(1 << SPI_CR2_SSOE) ;
	}
}

//IRQ Configuration and ISR Handling
/*****************************************************************
* @fn 				- SPI_IRQInterruptConfig
*
* @brief 			- This function is enable or disable the interrupt in the ARM M4 processor
*
* @param[in] 		- IRQ Number
* @param[in] 		- Enable or Disable
* @param[in] 		-
*
* @return 			- None
*
* @Note 			- None
*/
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(IRQNumber <= 31)
		{
			//Program ISER0 register
			*NVIC_ISER0 |= (1 << IRQNumber);

		}else if(IRQNumber > 31 && IRQNumber < 64) //32 to 63
		{
			//Program ISER1 register
			*NVIC_ISER1 |= (1 << (IRQNumber % 32) );

		}else if (IRQNumber >= 64 && IRQNumber < 96) //64 to 96
		{
			//Program ISER2 register
			*NVIC_ISER2 |= (1 << (IRQNumber % 64) );

		}
	}
	else
	{
		if(IRQNumber <= 31)
		{
			//Program ICER0 register
			*NVIC_ICER0 |= (1 << IRQNumber);

		}else if(IRQNumber > 31 && IRQNumber < 64)
		{
			//Program ICER1 register
			*NVIC_ICER1 |= (1 << (IRQNumber % 32) );

		}else if(IRQNumber >= 64 && IRQNumber < 96)
		{
			//Program ICER2 register
			*NVIC_ICER2 |= (1 << (IRQNumber % 64) );
		}
	}

}

/*****************************************************************
* @fn 				- SPI_IRQPriorityConfig
*
* @brief 			- This function is configure the Priority of the SPI Interrupt in other Interrupts
*
* @param[in] 		- IRQPriority
* @param[in] 		- IRQNumber
* @param[in] 		-
*
* @return 			- None
*
* @Note 			- None
*/

void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	//1. First Let's find out the irp register.
	uint8_t irpx = IRQNumber / 4 ;
	uint8_t irpx_section = IRQNumber % 4 ;

	uint8_t shift_amount = (8 * irpx_section) + (8 - NO_PR_BITS_IMPLEMENTED);
	*(NVIC_PR_BASE_ADDR + irpx ) |= (IRQPriority << shift_amount);
}

/*****************************************************************
* @fn 				- SPI_IRQHandling
*
* @brief 			- Interrupt handle function
*
* @param[in] 		- SPI_Handle_t *pHandle
* @param[in] 		-
* @param[in] 		-
*
* @return 			- None
*
* @Note 			- None
*/
void SPI_IRQHandling(SPI_Handle_t *pHandle)
{
	uint8_t temp1, temp2;
	// First lets check for TXE
	temp1 = pHandle->pSPIx->SR & (1 << SPI_SR_TXE);
	temp2 = pHandle->pSPIx->CR2 & (1 << SPI_CR2_TXNEIE);

	if( temp1 && temp2 )
	{
		//handle TXE
		spi_txe_interrupt_handle(pHandle);
	}


	temp1 = pHandle->pSPIx->SR & (1 << SPI_SR_TXE);
	temp2 = pHandle->pSPIx->CR2 & (1 << SPI_CR2_TXNEIE);

	if( temp1 && temp2 )
	{
		//handle RXNE
		spi_rxne_interrupt_handle(pHandle);
	}

	//Lets check for OVR
	temp1 = pHandle->pSPIx->SR & (1 << SPI_SR_OVR);
	temp2 = pHandle->pSPIx->CR2 & (1 << SPI_CR2_ERRIE);

	if( temp1 && temp2 )
	{
		//handle ovr error
		spi_ovr_err_interrupt_handle(pHandle);
	}

}


/*
 * Some Helper Function İmplementations
 */

static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	//2. Check the DFF bit in CR1
	if(pSPIHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF))
	{
		//16bit DFF
		//1. Load the data in to the DR
		pSPIHandle->pSPIx->DR =  *((uint16_t*)pSPIHandle->pTxBuffer);
		pSPIHandle->TxLen--;
		pSPIHandle->TxLen--;
		(uint16_t*)pSPIHandle->pTxBuffer++;
	}else
	{
		//8 bit DFF
		//1. Load the data in to the DR
		pSPIHandle->pSPIx->DR =  *pSPIHandle->pTxBuffer;
		pSPIHandle->TxLen--;
		pSPIHandle->pTxBuffer++;
	}

	if( !pSPIHandle->TxLen-- )
	{
		//TxLen is zero, so close the SPI Transmission and inform the application that
		//Tx is over.

		//This prevents interrupts from setting up of TXE flag
		SPI_CloseTransmission(pSPIHandle);
		SPI_ApplicationEventCallBack(pSPIHandle, SPI_EVENT_TX_CMPLT);
	}
}

static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	//do rxing as per the dff
	if(pSPIHandle->pSPIx->CR1 & ( 1 << 11))
	{
		//16 bit
		*((uint16_t*)pSPIHandle->pRxBuffer) = (uint16_t) pSPIHandle->pSPIx->DR;
		pSPIHandle->RxLen -= 2;
		pSPIHandle->pRxBuffer++;
		pSPIHandle->pRxBuffer++;

	}else
	{
		//8 bit
		*(pSPIHandle->pRxBuffer) = (uint8_t) pSPIHandle->pSPIx->DR;
		pSPIHandle->RxLen--;
		pSPIHandle->pRxBuffer++;
	}

	if(! pSPIHandle->RxLen)
	{
		//reception is complete
		SPI_CloseReception(pSPIHandle);
		SPI_ApplicationEventCallBack(pSPIHandle,SPI_EVENT_RX_CMPLT);
	}
}

static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	uint8_t temp;
	//1. Clear the ovr flag.
	if(pSPIHandle->TxState != SPI_BUSY_IN_TX)
	{
		temp = pSPIHandle->pSPIx->DR ;
		temp = pSPIHandle->pSPIx->SR ;
	}
	(void)temp;
	//2. Inform the application.
	SPI_ApplicationEventCallBack(pSPIHandle,SPI_EVENT_OVR_ERR);
}

void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->pSPIx->CR2 &= ~( 1 << SPI_CR2_TXNEIE );
	pSPIHandle->pTxBuffer = NULL ;
	pSPIHandle->TxLen = 0 ;
	pSPIHandle->TxState = SPI_READY ;
}

void SPI_CloseReception(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->pSPIx->CR2 &= ~( 1 << SPI_CR2_RXNEIE);
	pSPIHandle->pRxBuffer = NULL;
	pSPIHandle->RxLen = 0;
	pSPIHandle->RxState = SPI_READY;
}

void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx)
{
	uint8_t temp;
	temp = pSPIx->DR;
	temp = pSPIx->SR;
	(void)temp;
}


__weak void SPI_ApplicationEventCallBack(SPI_Handle_t *pSPIHandle, uint8_t AppEv)
{
	//If application doesn't implement this callback function, then this callback function
	//will be called.

	//This is a weak implementation, the application may override this function.
}





