/*
 * stm32f401xx_i2c_driver.c
 *
 *  Created on: 10 Ağu 2023
 *      Author: alprn
 */
#include "stm32f401xx_i2c_driver.h"

static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx);
static void I2C_ExcecuteAddressPhaseWrite(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr);
static void I2C_ExcecuteAddressPhaseRead(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr);
static void I2C_ClearADDRFlag(I2C_Handle_t *pI2CHandle);
static void I2C_MasterHandleRXNEInterrupt(I2C_Handle_t *pI2CHandle);
static void I2C_MasterHandleTXEInterrupt(I2C_Handle_t *pI2CHandle);


/*****************************************************************
* @fn 				- I2C_GenerateStartCondition
*
* @brief 			- This function used to configure and initiate start condition.
*
* @param[in] 		- Base address of the I2C peripheral
* @param[in] 		-
* @param[in] 		-

*
* @return 			- None
*
* @Note 			- This is a internal private function so I don't mention it in header file.
*/
static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->CR1 |= (1 << I2C_CR1_START);
}

/*****************************************************************
* @fn 				- I2C_ExcecuteAddressPhaseWrite
*
* @brief 			- This function used to configure and initiate Address Phase meanwhile master is configured as write data to slave.
*
* @param[in] 		- Base address of the I2C peripheral
* @param[in] 		- Slave Address
* @param[in] 		-

*
* @return 			- None
*
* @Note 			- This is a internal private function so I don't mention it in header file.
*/
static void I2C_ExcecuteAddressPhaseWrite(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr)
{
	SlaveAddr = SlaveAddr << 1 ;
	SlaveAddr &= ~(1);  //The LSB is R/nW bit which must be set 0 for WRITE. This line also makes "SlaveAddr = SlaveAddr + R/nW".
	pI2Cx->DR = SlaveAddr;
}

/*****************************************************************
* @fn 				- I2C_ExcecuteAddressPhaseRead
*
* @brief 			- This function used to configure and initiate Address Phase meanwhile master is configured as read data from slave.
*
* @param[in] 		- Base address of the I2C peripheral
* @param[in] 		- Slave Address
* @param[in] 		-

*
* @return 			- None
*
* @Note 			- This is a internal private function so I don't mention it in header file.
*/
static void I2C_ExcecuteAddressPhaseRead(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr)
{
	SlaveAddr = SlaveAddr << 1 ;
	SlaveAddr |= 1 ;  //The LSB is R/nW bit which must be set 1 for READ. This line also makes "SlaveAddr = SlaveAddr + R/nW".
	pI2Cx->DR = SlaveAddr;
}

/*****************************************************************
* @fn 				- I2C_ClearADDRFlag
*
* @brief 			- This function used to clear ADDR flag by reading SR1 then SR2 registers.
*
* @param[in] 		- Base address of the I2C peripheral
* @param[in] 		-
* @param[in] 		-

*
* @return 			- None
*
* @Note 			- This is a internal private function so I don't mention it in header file.
*/
static void I2C_ClearADDRFlag(I2C_Handle_t *pI2CHandle)
{
	uint32_t dummy_read;
	//Check for device mode
	if(pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_MSL))
	{
		//Device is in master mode.
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
		{
			if(pI2CHandle->RxSize == 1)
			{
				//First disable the ack.
				I2C_ManageAcking(pI2CHandle->pI2Cx, DISABLE);

				//Clear the ADDR flag (read SR1, read SR2)
				dummy_read = pI2CHandle->pI2Cx->SR1 ;
				dummy_read = pI2CHandle->pI2Cx->SR2 ;
				(void)dummy_read;
			}

		}else
		{
			//Clear the ADDR flag (read SR1, read SR2)
			dummy_read = pI2CHandle->pI2Cx->SR1 ;
			dummy_read = pI2CHandle->pI2Cx->SR2 ;
			(void)dummy_read;
		}

	}else
	{
		//Device is in slave mode.
		//Clear the ADDR flag (read SR1, read SR2)
		dummy_read = pI2CHandle->pI2Cx->SR1 ;
		dummy_read = pI2CHandle->pI2Cx->SR2 ;
		(void)dummy_read;
	}
}

/*****************************************************************
* @fn 				- I2C_GenerateStopCondition
*
* @brief 			- This function used to generate a stop condition to stop the I2C transmission.
*
* @param[in] 		- Base address of the I2C peripheral
* @param[in] 		-
* @param[in] 		-

*
* @return 			- None
*
* @Note 			- None
*/
void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->CR1 |= (1 << I2C_CR1_STOP);
}

/*****************************************************************
* @fn 				- I2C_SlaveEnableDisableCallbackEvents
*
* @brief 			- This function used to Enable or Disable interrupt callback events on slave devices.
*
* @param[in] 		- Base address of the I2C peripheral
* @param[in] 		- EnOrDi
* @param[in] 		-

*
* @return 			- None
*
* @Note 			- None
*/
void I2C_SlaveEnableDisableCallbackEvents(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		pI2Cx->CR2 |= ( 1 << I2C_CR2_ITEVTEN);
		pI2Cx->CR2 |= ( 1 << I2C_CR2_ITBUFEN);
		pI2Cx->CR2 |= ( 1 << I2C_CR2_ITERREN);
	}else
	{
		pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITEVTEN);
		pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITBUFEN);
		pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITERREN);
	}
}

/*****************************************************************
* @fn 				- I2C_MasterHandleTXEInterrupt
*
* @brief 			- This function used to handle Master TXE Interrupt.
*
* @param[in] 		- The I2C peripheral handle
* @param[in] 		-
* @param[in] 		-

*
* @return 			- None
*
* @Note 			- This is a internal private function so I don't mention it in header file.
*/
static void I2C_MasterHandleTXEInterrupt(I2C_Handle_t *pI2CHandle)
{
	if(pI2CHandle->TxLen > 0)
	{
		//1.Load the data in the DR.
		pI2CHandle->pI2Cx->DR |= *(pI2CHandle->pTxBuffer);

		//2.Decrement the TxLen.
		pI2CHandle->TxLen--;

		//3.Increment the buffer address.
		pI2CHandle->pTxBuffer++;

	}
}

/*****************************************************************
* @fn 				- I2C_MasterHandleRXNEInterrupt
*
* @brief 			- This function used to handle Master RXNE Interrupt.
*
* @param[in] 		- The I2C peripheral handle
* @param[in] 		-
* @param[in] 		-

*
* @return 			- None
*
* @Note 			- This is a internal private function so I don't mention it in header file.
*/
static void I2C_MasterHandleRXNEInterrupt(I2C_Handle_t *pI2CHandle)
{
	//The device is master.
	//TXE flag is set.
	//We have to do the data reception.
	if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
	{
		if(pI2CHandle->RxLen == 1)
		{
			//1.Read the data from the DR.
			*pI2CHandle->pRxBuffer |= pI2CHandle->pI2Cx->DR;

			//2.Decrement the RxLen.
			pI2CHandle->RxLen--;
		}
		if(pI2CHandle->RxLen > 1)
		{
			if(pI2CHandle->RxLen == 2)
			{
				//1.Clear the ack bit.
				I2C_ManageAcking(pI2CHandle->pI2Cx, DISABLE);
			}

			//2.Read the data from the DR.
			*pI2CHandle->pRxBuffer |= pI2CHandle->pI2Cx->DR;

			//3.Decrement the RxLen.
			pI2CHandle->RxLen--;

			//4.Increment the buffer address.
			pI2CHandle->pRxBuffer++;

		}

		if(pI2CHandle->RxLen == 0)
		{
			//Close the I2C data reception and notify the application.

			//1.Generate the stop condition.
			if(pI2CHandle->Sr == I2C_DISABLE_SR)
			{
				I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
			}

			//2.Close the I2C Rx.
			I2C_CloseReceiveData(pI2CHandle);

			//3.Notify the application about the end of communication.
			I2C_ApplicationEventCallBack(pI2CHandle,I2C_EV_RX_CMPLT);
		}
	}
}


/*
 *  **********************************Peripheral Control********************************************
 */

/*****************************************************************
* @fn 				- I2C_PeripheralControl
*
* @brief 			- This function is used to Enable or Disable the I2C_Peripheral.
*
* @param[in] 		- Base address of the I2C peripheral
* @param[in] 		- Enable or Disable
* @param[in] 		-
*
* @return 			- None
*
* @Note 			- None
*/
void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		pI2Cx->CR1 |= (1 << I2C_CR1_PE) ;
	}else
	{
		pI2Cx->CR1 &= ~(1 << I2C_CR1_PE) ;
	}
}

// Peripheral Clock setup
/*****************************************************************
* @fn 				- I2C_PeriClockControl
*
* @brief 			- This function enables or disables peripheral clock for the given I2C periperal.
*
* @param[in] 		- Base address of the I2C peripheral
* @param[in] 		- Enable or Disable macros
* @param[in] 		-
*
* @return 			- None
*
* @Note 			- None
*/
void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(pI2Cx == I2C1)
		{
			I2C1_PCLK_EN();
		}
		else if(pI2Cx == I2C2)
		{
			I2C2_PCLK_EN();
		}
		else if(pI2Cx == I2C3)
		{
			I2C3_PCLK_EN();
		}
	}
	else
	{
		if(pI2Cx == I2C1)
		{
			I2C1_PCLK_DI();
		}
		else if(pI2Cx == I2C2)
		{
			I2C2_PCLK_DI();
		}
		else if(pI2Cx == I2C3)
		{
			I2C3_PCLK_DI();
		}
	}
}

//Init and De-init

/*****************************************************************
* @fn 				- I2C_Init
*
* @brief 			- This function initialize the given I2C.
*
* @param[in] 		- Peripheral handle of the I2C peripheral
* @param[in] 		-
* @param[in] 		-
*
* @return 			- None
*
* @Note 			- None
*/
void I2C_Init(I2C_Handle_t *pI2CHandle)
{
	uint32_t tempreg = 0 ;

	//Enable the peripheral clock for the I2Cx peripheral.
	I2C_PeriClockControl(pI2CHandle->pI2Cx, ENABLE);

	//ack control bit
	tempreg |= pI2CHandle->I2C_Config.I2C_ACKControl << 10 ;
	pI2CHandle->pI2Cx->CR1 |= tempreg ;

	//Configure the FREQ field of CR2
	tempreg = 0 ;
	tempreg |= RCC_GetPCLK1Value() / 1000000U ;
	pI2CHandle->pI2Cx->CR2 |= (tempreg & 0x3F);

	//Program the device own address (OAR1)
	tempreg = 0;
	tempreg |= pI2CHandle->I2C_Config.I2C_DeviceAddress << 1;
	tempreg |= (1 << 14); //Because of the rm say so
	pI2CHandle->pI2Cx->OAR1 |= tempreg ;

	//Calculate and configure the CCR register field.
	uint16_t ccr_value = 0;
	tempreg = 0 ;
	if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
	{
		//Mode is standard mode.
		ccr_value = (RCC_GetPCLK1Value() / (2 * pI2CHandle->I2C_Config.I2C_SCLSpeed));
		tempreg |= (ccr_value & 0xFFF);
	}else
	{
		//Mode is fast mode.
		tempreg |= (1 << 15);
		tempreg |= (pI2CHandle->I2C_Config.I2C_FMDutyCycle << 14);
		if(pI2CHandle->I2C_Config.I2C_FMDutyCycle == I2C_FM_DUTY_2)
		{
		ccr_value = (RCC_GetPCLK1Value() / (3 * pI2CHandle->I2C_Config.I2C_SCLSpeed));
		}else
		{
		ccr_value = (RCC_GetPCLK1Value() / (25 * pI2CHandle->I2C_Config.I2C_SCLSpeed));
		}
		tempreg |= (ccr_value & 0xFFF);
	}
	pI2CHandle->pI2Cx->CCR |= tempreg ;

	if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM )
	{
		//Mode is standart mode

		tempreg = (RCC_GetPCLK1Value() /1000000U) + 1;
	}else
	{
		//Mode is fast mode
		tempreg = ((RCC_GetPCLK1Value() * 300) /1000000000U) + 1;
	}
	pI2CHandle->pI2Cx->TRISE |= (tempreg & 0x3F);
}

/*****************************************************************
* @fn 				- I2C_Deinit
*
* @brief 			- This function deinitialize the given I2C.
*
* @param[in] 		- Base address of the I2C peripheral
* @param[in] 		-
* @param[in] 		-
*
* @return 			- None
*
* @Note 			- None
*/
void I2C_DeInit(I2C_RegDef_t *pI2Cx)
{
	if(pI2Cx == I2C1)
	{
		I2C1_REG_RESET();
	}
	else if(pI2Cx == I2C2)
	{
		I2C2_REG_RESET();
	}
	else if(pI2Cx == I2C3)
	{
		I2C3_REG_RESET();
	}
}

/*****************************************************************
* @fn 				- I2C_GetFlagStatus
*
* @brief 			- This function used to get flag status data.
*
* @param[in] 		- Base address of the I2C peripheral
* @param[in] 		- FlagName
* @param[in] 		-
*
* @return 			- Flag Status
*
* @Note 			- None
*/
uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx , uint32_t FlagName)
{
	if(pI2Cx->SR1 & FlagName)
	{
		return FLAG_SET;
	}
	return FLAG_RESET ;
}

/*****************************************************************
* @fn 				- I2C_ManageAcking
*
* @brief 			- This function used to configure ACK.
*
* @param[in] 		- Base address of the I2C peripheral
* @param[in] 		- Enable or Disable
* @param[in] 		-
*
* @return 			- None
*
* @Note 			- None
*/
void I2C_ManageAcking(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi)
{
	if(EnOrDi == I2C_ACK_ENABLE)
	{
	pI2Cx->CR1 |= (1 << I2C_CR1_ACK);
	}else
	{
	pI2Cx->CR1 |= (~(1) << I2C_CR1_ACK);
	}
}

// Data Send and Receive
/*****************************************************************
* @fn 				- I2C_MasterSendData
*
* @brief 			- This function used to send data from master.
*
* @param[in] 		- Base address of the I2C peripheral
* @param[in] 		- Data
* @param[in] 		- Data Length
* @param[in] 		- Slave Addr
* @param[in] 		- Sr is Repeated Start parameter.
*
* @return 			- None
*
* @Note 			- None
*/
void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint8_t Len, uint8_t SlaveAddr, uint8_t Sr)
{
	//1. Generate the START condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	//2. Confirm that start generation is completed by checking the SB flag in the SR1
	// Note: Until SB is cleared SCL will be stretched (pulled to LOW)
	while(	! (I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SB)) );

	//3. Send the address of the slave with r/nw bit set to w(0) (total 8 bits )
	I2C_ExcecuteAddressPhaseWrite(pI2CHandle->pI2Cx, SlaveAddr);

	//4. Confirm that address phase is completed by checking the ADDR flag in the SR1
	while(	! (I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR)) );

	//5. Clear the ADDR flag according to its software sequence
	// Note: Until ADDR is cleared SCL will be stretched (pulled to LOW)
	I2C_ClearADDRFlag(pI2CHandle);

	//6. Send the data until Len becomes 0
	while(Len > 0)
	{
		while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_TXE))
		{
			pI2CHandle->pI2Cx->DR = *pTxBuffer;
			pTxBuffer++;
			Len--;
		}
	}

	//7. When Len becomes zero wait for TXE=1 and BTF=1 before generating the STOP condition
	// Note: TXE=1 , BTF=1 , means that both SR and DR are empty and next transmission should begin
	// when BTF=1 SCL will be stretched (pulled to LOW)
	while(	! (I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TXE)) );

	while(	! (I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_BTF)) );

	//8. Generate STOP condition and master need not to wait for the completion of stop condition.
	// Note: generating STOP, automatically clears the BTF
	if(Sr == I2C_DISABLE_SR)
	{
	I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
	}
}

/*****************************************************************
* @fn 				- I2C_MasterReceiveData
*
* @brief 			- This function used to receive data from master.
*
* @param[in] 		- Base address of the I2C peripheral
* @param[in] 		- Data
* @param[in] 		- Data Length
* @param[in] 		- Slave Addr
* @param[in] 		- Sr is Repeated Start parameter.
*
* @return 			- None
*
* @Note 			- None
*/
void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint8_t Len, uint8_t SlaveAddr, uint8_t Sr)
{

	//1. Generate the START condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	//2. confirm that start generation is completed by checking the SB flag in the SR1
	//   Note: Until SB is cleared SCL will be stretched (pulled to LOW)
	while(	! (I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SB)) );

	//3. Send the address of the slave with r/nw bit set to R(1) (total 8 bits )
	I2C_ExcecuteAddressPhaseRead(pI2CHandle->pI2Cx, SlaveAddr);

	//4. wait until address phase is completed by checking the ADDR flag in the SR1
	while(	! (I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR)) );

	//procedure to read only 1 byte from slave
	if(Len == 1)
	{
		//Disable Acking
		I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_DISABLE);

		//clear the ADDR flag
		I2C_ClearADDRFlag(pI2CHandle);

		//wait until  RXNE becomes 1
		while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_RXNE));

		if(Sr == I2C_DISABLE_SR)
		{
		//generate STOP condition
		I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
		}

		//read data in to buffer
		*pRxBuffer |=  pI2CHandle->pI2Cx->DR;
	}

    //procedure to read data from slave when Len > 1
	if(Len > 1)
	{
		//clear the ADDR flag
		I2C_ClearADDRFlag(pI2CHandle);

		//read the data until Len becomes zero
		for ( uint32_t i = Len ; i > 0 ; i--)
		{
			//wait until RXNE becomes 1
			while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_RXNE));

			if(i == 2) //if last 2 bytes are remaining
			{
				//Disable Acking
				I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_DISABLE);

				if(Sr == I2C_DISABLE_SR)
				{
				//generate STOP condition
				I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
				}
			}

			//read the data from data register in to buffer
			*pRxBuffer |=  pI2CHandle->pI2Cx->DR;
			//increment the buffer address
			pRxBuffer++;
			Len--;
		}

	}
	//re-enable ACKing
	if(pI2CHandle->I2C_Config.I2C_ACKControl == I2C_ACK_ENABLE)
	{
	I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_ENABLE);
	}
}

/*****************************************************************
* @fn 				- I2C_MasterSendDataIT
*
* @brief 			- This function used to send data to slave with interrupt configuration.
*
* @param[in] 		- Base address of the I2C peripheral
* @param[in] 		- Data
* @param[in] 		- Data Length
* @param[in] 		- Slave Addr
*
* @return 			- None
*
* @Note 			- None
*/
uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle,uint8_t *pTxBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr)
{

	uint8_t busystate = pI2CHandle->TxRxState;

	if( (busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX))
	{
		pI2CHandle->pTxBuffer = pTxBuffer;
		pI2CHandle->TxLen = Len;
		pI2CHandle->TxRxState = I2C_BUSY_IN_TX;
		pI2CHandle->DevAddr = SlaveAddr;
		pI2CHandle->Sr = Sr;

		//Implement code to Generate START Condition
		I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

		//Implement the code to enable ITBUFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITBUFEN);

		//Implement the code to enable ITEVTEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITEVTEN);

		//Implement the code to enable ITERREN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITERREN);

	}

	return busystate;

}

/*****************************************************************
* @fn 				- I2C_MasterReceiveDataIT
*
* @brief 			- This function used to receive data from slave with interrupt configuration.
*
* @param[in] 		- Base address of the I2C peripheral
* @param[in] 		- Data
* @param[in] 		- Data Length
* @param[in] 		- Slave Addr
*
* @return 			- None
*
* @Note 			- None
*/
uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle,uint8_t *pRxBuffer, uint32_t Len, uint8_t SlaveAddr,uint8_t Sr)
{

	uint8_t busystate = pI2CHandle->TxRxState;

	if( (busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX))
	{
		pI2CHandle->pRxBuffer = pRxBuffer;
		pI2CHandle->RxLen = Len;
		pI2CHandle->TxRxState = I2C_BUSY_IN_RX;
		pI2CHandle->RxSize = Len; //Rxsize is used in the ISR code to manage the data reception
		pI2CHandle->DevAddr = SlaveAddr;
		pI2CHandle->Sr = Sr;

		//Implement code to Generate START Condition
		I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

		//Implement the code to enable ITBUFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITBUFEN);

		//Implement the code to enable ITEVTEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITEVTEN);

		//Implement the code to enable ITERREN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITERREN);
	}

	return busystate;
}

/*****************************************************************
* @fn 				- I2C_CloseReceiveData
*
* @brief 			- This function used to close receive data communication from slave.
*
* @param[in] 		- I2C peripheral handle
* @param[in] 		-
* @param[in] 		-
*
* @return 			- None
*
* @Note 			- None
*/
void I2C_CloseReceiveData(I2C_Handle_t *pI2CHandle)
{
	//Implement the code to disable ITBUFEN Control Bit
	pI2CHandle->pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITBUFEN);

	//Implement the code to disable ITEVFEN Control Bit
	pI2CHandle->pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITEVTEN);

	pI2CHandle->TxRxState = I2C_READY;
	pI2CHandle->pRxBuffer = NULL;
	pI2CHandle->RxLen = 0;
	pI2CHandle->RxSize = 0;

	if(pI2CHandle->I2C_Config.I2C_ACKControl == I2C_ACK_ENABLE)
	{
		I2C_ManageAcking(pI2CHandle->pI2Cx,ENABLE);
	}
}

/*****************************************************************
* @fn 				- I2C_CloseSendData
*
* @brief 			- This function used to close send data communication to slave.
*
* @param[in] 		- I2C peripheral handle
* @param[in] 		-
* @param[in] 		-
*
* @return 			- None
*
* @Note 			- None
*/
void I2C_CloseSendData(I2C_Handle_t *pI2CHandle)
{
	//Implement the code to disable ITBUFEN Control Bit
	pI2CHandle->pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITBUFEN);

	//Implement the code to disable ITEVFEN Control Bit
	pI2CHandle->pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITEVTEN);


	pI2CHandle->TxRxState = I2C_READY;
	pI2CHandle->pTxBuffer = NULL;
	pI2CHandle->TxLen = 0;
}

/*****************************************************************
* @fn 				- I2C_SlaveSendData
*
* @brief 			- This function used to send data as slave device.
*
* @param[in] 		- I2C register definition
* @param[in] 		- Data
* @param[in] 		-
*
* @return 			- None
*
* @Note 			- None
*/
void I2C_SlaveSendData(I2C_RegDef_t *pI2C, uint8_t data)
{
	pI2C->DR |= data;
}

/*****************************************************************
* @fn 				- I2C_SlaveReceiveData
*
* @brief 			- This function used to receive data as slave device.
*
* @param[in] 		- I2C register definition
* @param[in] 		-
* @param[in] 		-
*
* @return 			- None
*
* @Note 			- None
*/
uint8_t I2C_SlaveReceiveData(I2C_RegDef_t *pI2C)
{
	return (uint8_t) pI2C->DR;
}

//IRQ Configuration and ISR Handling
/*****************************************************************
* @fn 				- I2C_IRQInterruptConfig
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
void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
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
* @fn 				- I2C_IRQPriorityConfig
*
* @brief 			- This function is configure the Priority of the I2C Interrupt in other Interrupts.
*
* @param[in] 		- IRQPriority
* @param[in] 		- IRQNumber
* @param[in] 		-
*
* @return 			- None
*
* @Note 			- None
*/
void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	//1. First Let's find out the irp register.
	uint8_t irpx = IRQNumber / 4 ;
	uint8_t irpx_section = IRQNumber % 4 ;

	uint8_t shift_amount = (8 * irpx_section) + (8 - NO_PR_BITS_IMPLEMENTED);
	*(NVIC_PR_BASE_ADDR + irpx ) |= (IRQPriority << shift_amount);
}

/*****************************************************************
* @fn 				- I2C_EV_IRQHandling
*
* @brief 			- This function is configure Event Interrupt Handling.
*
* @param[in] 		- pI2CHandle
* @param[in] 		-
* @param[in] 		-
*
* @return 			- None
*
* @Note 			- None
*/
void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle)
{
	//Interrupt handling for both master and slave mode of a device

	uint32_t temp1, temp2, temp3 ;

	temp1 = pI2CHandle->pI2Cx->CR2 & ( 1 << I2C_CR2_ITEVTEN);
	temp2 = pI2CHandle->pI2Cx->CR2 & ( 1 << I2C_CR2_ITBUFEN);

	temp3 = pI2CHandle->pI2Cx->SR1 & ( 1 << I2C_SR1_SB);

	//1. Handle for interrupt generated by SB event.
	//Note : SB flag is only applicable in Master Mode
	if(temp1 && temp3)
	{
		// The interrupt is generated because of SB event.
		// This block will not be executed in slave mode because for slave SB always zero.
		// In this block let's execute the address phase

		if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
		{
			I2C_ExcecuteAddressPhaseWrite(pI2CHandle->pI2Cx, pI2CHandle->DevAddr);
		}else if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
		{
			I2C_ExcecuteAddressPhaseRead(pI2CHandle->pI2Cx, pI2CHandle->DevAddr);
		}
	}

	temp3 = pI2CHandle->pI2Cx->SR1 & ( 1 << I2C_SR1_ADDR);
	//2. Handle for interrupt generated by ADDR event
	//Note : When master mode : Address is sent
	// 	   : When slave mode  : Address matched with own address.
	if(temp1 && temp3)
	{
		//ADDR flag is set.
		I2C_ClearADDRFlag(pI2CHandle);
	}

	temp3 = pI2CHandle->pI2Cx->SR1 & ( 1 << I2C_SR1_BTF);
	//3. Handle for interrupt generated by BTF(Byte Transfer Finished) event
	if(temp1 && temp3)
	{
		//BTF flag is set.
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
		{
			//Make sure that TXE is also set.
			if(pI2CHandle->pI2Cx->SR1 & ( 1 << I2C_SR1_BTF))
			{
				//BTF, TXE = 1
				if(pI2CHandle->TxLen == 0)
				{
					//1. Generate Stop condition.
					if(pI2CHandle->Sr == I2C_DISABLE_SR)
					{
						I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
					}

					//2. Reset all the member elements of the handle structure.
					I2C_CloseSendData(pI2CHandle);

					//3. Notify the application about transmission complete.
					I2C_ApplicationEventCallBack(pI2CHandle, I2C_EV_TX_CMPLT);
				}
			}
		}else if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
		{
				;
		}
	}

	temp3 = pI2CHandle->pI2Cx->SR1 & ( 1 << I2C_SR1_STOPF);
	//4. Handle for interrupt generated by STOPF event
	//Note : Stop detection flag is applicable only slave mode. For master this flag will never be set
	//The below code block will not be executed by the master since STOPF will not set in master mode
	if(temp1 && temp3)
	{
		//STOPF flag is set.
		//Clear the STOPF (i.e. 1 -> read SR1 / 2 -> write to CR1)
		pI2CHandle->pI2Cx->CR1 |= 0x0000;

		//Notify the application that STOP is detected.
		I2C_ApplicationEventCallBack(pI2CHandle, I2C_EV_STOP);
	}

	temp3 = pI2CHandle->pI2Cx->SR1 & ( 1 << I2C_SR1_TXE);
	//5. Handle For interrupt generated by TXE event
	if(temp1 && temp2 && temp3)
	{
		//Check for the device mode.
		if(pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_MSL))
		{
			//Master
			//TXE flag is set.
			//We have to do the data transmission.
			if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
			{
				I2C_MasterHandleTXEInterrupt(pI2CHandle);
			}
		}else
		{
			//Slave
			//Make sure that the slave is really in transmitter mode
			if(pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_TRA))
			{
				I2C_ApplicationEventCallBack(pI2CHandle, I2C_EV_DATA_REQ);
			}
		}
	}

	temp3 = pI2CHandle->pI2Cx->SR1 & ( 1 << I2C_SR1_RXNE);
	//6. Handle For interrupt generated by RXNE event
	if(temp1 && temp2 && temp3)
	{
		//RXNE flag is set.
		//Check for the device mode.
		if(pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_MSL))
		{
			//Master
			if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
			{
				I2C_MasterHandleRXNEInterrupt(pI2CHandle);
			}
		}else
		{
			//Slave
			//Make sure that the slave is really in receiver mode.
			if(!(pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_TRA)))
			{
				I2C_ApplicationEventCallBack(pI2CHandle, I2C_EV_DATA_RCV);
			}
		}
	}
}

/*****************************************************************
* @fn 				- I2C_ER_IRQHandling
*
* @brief 			- This function is configure Error Interrupt Handling.
*
* @param[in] 		- pI2CHandle
* @param[in] 		-
* @param[in] 		-
*
* @return 			- None
*
* @Note 			- None
*/
void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle)
{

	uint32_t temp1,temp2;

    //Know the status of  ITERREN control bit in the CR2
	temp2 = (pI2CHandle->pI2Cx->CR2) & ( 1 << I2C_CR2_ITERREN);


/***********************Check for Bus error************************************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1<< I2C_SR1_BERR);
	if(temp1  && temp2 )
	{
		//This is Bus error

		//Implement the code to clear the buss error flag
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_BERR);

		//Implement the code to notify the application about the error
	   I2C_ApplicationEventCallBack(pI2CHandle,I2C_ERROR_BERR);
	}

/***********************Check for arbitration lost error************************************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_ARLO );
	if(temp1  && temp2)
	{
		//This is arbitration lost error

		//Implement the code to clear the arbitration lost error flag
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_ARLO);

		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallBack(pI2CHandle, I2C_ERROR_ARLO);
	}

/***********************Check for ACK failure  error************************************/

	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_AF);
	if(temp1  && temp2)
	{
		//This is ACK failure error

	    //Implement the code to clear the ACK failure error flag
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_AF);

		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallBack(pI2CHandle, I2C_ERROR_AF);
	}

/***********************Check for Overrun/underrun error************************************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_OVR);
	if(temp1  && temp2)
	{
		//This is Overrun/underrun

	    //Implement the code to clear the Overrun/underrun error flag
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_OVR);

		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallBack(pI2CHandle, I2C_ERROR_OVR);
	}

/***********************Check for Time out error************************************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_TIMEOUT);
	if(temp1  && temp2)
	{
		//This is Time out error

	    //Implement the code to clear the Time out error flag
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_TIMEOUT);

		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallBack(pI2CHandle, I2C_ERROR_TIMEOUT);
	}

}

__weak void I2C_ApplicationEventCallBack(I2C_Handle_t *pI2CHandle, uint8_t AppEv)
{
	//If application doesn't implement this callback function, then this callback function
	//will be called.

	//This is a weak implementation, the application may override this function.
}
