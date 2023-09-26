/*
 * stm32f401xx_rcc_driver.c
 *
 *  Created on: 18 Eyl 2023
 *      Author: alprn
 */

#include "stm32f401xx_rcc_driver.h"

uint16_t AHB_Prescaler[9] = {2,4,6,8,16,64,128,256,512};
uint16_t APB1_Prescaler[4] = {2, 4, 8, 16};
uint16_t APB2_Prescaler[4] = {2, 4, 8, 16};

uint32_t RCC_GetPLLOutputClock() //Not implemented.
{
	return 0;
}

/*****************************************************************
* @fn 				- RCC_GetPCLK1Value
*
* @brief 			- This function returns the PCLK1(APB1 Clock) value.
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

	clksrc = ((RCC->CFGR >> 2) & 0x3) ;

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
* @fn 				- RCC_GetPCLK1Value
*
* @brief 			- This function returns the PCLK2(APB2 Clock) value.
*
* @param[in] 		- void
* @param[in] 		-
* @param[in] 		-
*
* @return 			- uint32_t
*
* @Note 			- None
*/
uint32_t RCC_GetPCLK2Value(void)
{
	uint32_t pclk2, SystemClk;

	uint8_t clksrc, temp, ahbp, apb2p;

	clksrc = ((RCC->CFGR >> 2) & 0x3) ;

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

	temp = ((RCC->CFGR >> 13 ) & 0x7); //Reading the PPRE2 register to get APB2 prescaler data.
									   //Prescaler =  Clock Division Factor
	if(temp < 4)
	{
		apb2p = 1 ;
	}else
	{
		apb2p = APB2_Prescaler[temp - 4] ;
	}

	pclk2 = (SystemClk / ahbp) / apb2p ;

	return pclk2;
}

