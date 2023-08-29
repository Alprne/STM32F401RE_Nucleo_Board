/*
 * stm32f401xx_i2c_driver.c
 *
 *  Created on: 10 Ağu 2023
 *      Author: alprn
 */
#include "stm32f401xx_spi_driver.h"

uint16_t AHB_Prescaler[9] = {2,4,6,8,16,64,128,256,512};
uint16_t APB1_Prescaler[4] = {2, 4, 8, 16};

static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx);
static void I2C_ExcecuteAddressPhase(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr);
static void I2C_ClearADDRFlag(I2C_RegDef_t *pI2Cx);
static void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx);

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
* @fn 				- I2C_ExcecuteAddressPhase
*
* @brief 			- This function used to configure and initiate Address Phase.
*
* @param[in] 		- Base address of the I2C peripheral
* @param[in] 		- Slave Address
* @param[in] 		-

*
* @return 			- None
*
* @Note 			- This is a internal private function so I don't mention it in header file.
*/
static void I2C_ExcecuteAddressPhase(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr)
{
	SlaveAddr = SlaveAddr << 1 ;
	SlaveAddr &= ~(1);  //The LSB is R/nW bit which must be set 0 for WRITE. This line also makes "SlaveAddr = SlaveAddr + R/nW".
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
static void I2C_ClearADDRFlag(I2C_RegDef_t *pI2Cx)
{
	uint32_t dummyRead = pI2Cx->SR1;
	dummyRead = pI2Cx->SR2;
	(void)dummyRead ;
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
* @Note 			- This is a internal private function so I don't mention it in header file.
*/
static void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->CR1 |= (1 << I2C_CR1_STOP);
}




/*
 *  **********************************Peripheral Control********************************************
 */

/*****************************************************************
* @fn 				- I2C_PeripheralControl
*
* @brief 			- This function is Enable or Disable the I2C_Peripheral in the ARM M4 processor
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

uint32_t RCC_GetPLLOutputClock() //Not implemented.
{
	return 0;
}

/*****************************************************************
* @fn 				- RCC_GetPCLK1Value
*
* @brief 			- This function returns the PCLK1 value.
*
* @param[in] 		- void
* @param[in] 		-
* @param[in] 		-
*
* @return 			- uint32_t
*
* @Note 			- None
*/
uint32_t RCC_GetPCLK1Value(void)
{
	uint32_t pclk1, SystemClk;

	uint8_t clksrc, temp, ahbp, apb1p;

	clksrc = (RCC->CFGR >> 2 & 0x3) ;

	if(clksrc == 0)
	{
		SystemClk = 16000000; //HSI 16MHz CLK
	}else if (clksrc == 1)
	{
		SystemClk = 8000000; //HSE 8MHz CLK pulled from ST-LINK circuitry
	}else if(clksrc == 2)
	{
		SystemClk = RCC_GetPLLOutputClock();
	}

	temp = ((RCC->CFGR >> 4 ) & 0xF); //Reading the HPRE register to get AHB prescaler data.

	if(temp < 8)
	{
		ahbp = 1 ;
	}else
	{
		ahbp = AHB_Prescaler[temp - 8] ;
	}

	temp = ((RCC->CFGR >> 10 ) & 0x7); //Reading the PPRE1 register to get APB1 prescaler data.
									   //Prescaler =  Clock Division Factor
	if(temp < 4)
	{
		apb1p = 1 ;
	}else
	{
		apb1p = APB1_Prescaler[temp - 4] ;
	}

	pclk1 = (SystemClk / ahbp) / apb1p ;

	return pclk1;
}
/*****************************************************************
* @fn 				- I2C_Init
*
* @brief 			- This function initialize the given I2C.
*
* @param[in] 		- Base address of the I2C peripheral
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
	pI2CHandle->pI2Cx->CR1 = tempreg ;

	//Configure the FREQ field of CR2
	tempreg = 0 ;
	tempreg |= RCC_GetPCLK1Value() / 1000000U ;
	pI2CHandle->pI2Cx->CR2 = tempreg & 0x3F;

	//Program the device own address (OAR1)
	tempreg |= pI2CHandle->I2C_Config.I2C_DeviceAddress << 1;
	tempreg |= (1 << 14); //Because of the rm say so
	pI2CHandle->pI2Cx->OAR1 = tempreg ;

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
	pI2CHandle->pI2Cx->CCR = tempreg ;

	if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM )
	{
		//Mode is standart mode

		tempreg = (RCC_GetPCLK1Value() /1000000U) + 1;
	}else
	{
		//Mode is fast mode
		tempreg = ((RCC_GetPCLK1Value() * 300) /1000000000U) + 1;
	}
	pI2CHandle->pI2Cx->TRISE = tempreg & 0x3F;
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
*
* @return 			- None
*
* @Note 			- None
*/
void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint8_t Len, uint8_t SlaveAddr)
{
	//1. Generate the START condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	//2. Confirm that start generation is completed by checking the SB flag in the SR1
	// Note: Until SB is cleared SCL will be stretched (pulled to LOW)
	while(	! (I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SB)) );

	//3. Send the address of the slave with r/nw bit set to w(0) (total 8 bits )
	I2C_ExcecuteAddressPhase(pI2CHandle->pI2Cx, SlaveAddr);

	//4. Confirm that address phase is completed by checking the ADDR flag in the SR1
	while(	! (I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR)) );

	//5. Clear the ADDR flag according to its software sequence
	// Note: Until ADDR is cleared SCL will be stretched (pulled to LOW)
	I2C_ClearADDRFlag(pI2CHandle->pI2Cx);

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
	I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
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
