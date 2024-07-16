/*
 * stm32f103xx_gpio_driver.c
 *
 *  Created on: Jan 5, 2024
 *      Author: burak
 */

/*
 * stm32f103xx_gpio_driver.c
 *
 *  Created on: 15 Ara 2023
 *      Author: burak
 */


#include "stm32f103xx_gpio_driver.h"


/*
 * Peripheral Clock Setup
 */

/****************************************************************************
 * @fn						- GPIO_PeriClockControl
 *
 * @brief					- This function enables or disables peripheral clock for the given GPIO port.
 *
 * @param[in]				- Base address of the GPIO peripheral
 * @param[in]				- ENABLE or DISABLE macros
 * @param[in]				-
 *
 * @return					- none
 *
 * @Note					- none
 */


void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_EN();
		}else if(pGPIOx == GPIOB)
		{
			GPIOB_PCLK_EN();
		}else if(pGPIOx == GPIOC)
		{
			GPIOC_PCLK_EN();
		}else if(pGPIOx == GPIOD)
		{
			GPIOD_PCLK_EN();
		}else if(pGPIOx == GPIOE)
		{
			GPIOE_PCLK_EN();
		}else if(pGPIOx == GPIOF)
		{
			GPIOF_PCLK_EN();
		}else if(pGPIOx == GPIOG)
		{
			GPIOG_PCLK_EN();
		}
	}else
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_DI();
		}else if(pGPIOx == GPIOB)
		{
			GPIOB_PCLK_DI();
		}else if(pGPIOx == GPIOC)
		{
			GPIOC_PCLK_DI();
		}else if(pGPIOx == GPIOD)
		{
			GPIOD_PCLK_DI();
		}else if(pGPIOx == GPIOE)
		{
			GPIOE_PCLK_DI();
		}else if(pGPIOx == GPIOF)
		{
			GPIOF_PCLK_DI();
		}else if(pGPIOx == GPIOG)
		{
			GPIOG_PCLK_DI();
		}
	}
}

/*
 * Init and De-init
 */

/****************************************************************************
 * @fn						- GPIO_Init
 *
 * @brief					- This function initializes the given GPIO port.
 *
 * @param[in]				- Handle address of GPIO
 * @param[in]				-
 * @param[in]				-
 *
 * @return					- none
 *
 * @Note					- none
 */

void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{

	uint32_t temp = 0; //temp. register
	uint8_t temp1 = 0;
	uint8_t temp2 = 0;

	//enable peripheral clock
	GPIO_PeriClockControl(pGPIOHandle->pGPIOx, ENABLE);

	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_MAXSPD_50MHZ)
	{

		temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8; // 7/8 = 0
		temp2 = ( pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8 ) ; // 7%8 = 7

		if(temp1 == 0)
		{
			// Pin mode settings : Input mode (reset state), Output mode max speed 10 MHz. , Output mode max speed 2 MHz. , Output mode max speed 50 MHz
			temp |= ( pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << ( 4 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber ) );

			// Output mode settings : General purpose output push-pull, General purpose output Open-drain,  Alternate function output Push-pull, Alternate function output Open-drain
			if(pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType <= GPIO_MODE_OP_OD)
			{
				temp |= ( pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << ( ( 4 * (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) ) + 2 ) );
			}else
			{
				// we need to configure alternative function registers.

			}

			// Input mode settings : Analog mode, Floating input (reset state), Input with pull-up / pull-down, Reserved
			temp |= ( pGPIOHandle->GPIO_PinConfig.GPIO_PinIPType << ( ( 4 * (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) ) + 2 ) );

			pGPIOHandle->pGPIOx->CRL = temp;
			temp = 0;
		}else if(temp1 == 1){

			// Pin mode settings : Input mode (reset state), Output mode max speed 10 MHz. , Output mode max speed 2 MHz. , Output mode max speed 50 MHz
			temp |= ( pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << ( 4 * temp2 ) );

			// Output mode settings : General purpose output push-pull, General purpose output Open-drain,  Alternate function output Push-pull, Alternate function output Open-drain
			if(pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType <= GPIO_MODE_OP_OD)
			{
				temp |= ( pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << ( ( 4 * temp2 ) + 2 ) );
			}else
			{
				// we need to configure alternative function registers.

			}

			// Input mode settings : Analog mode, Floating input (reset state), Input with pull-up / pull-down, Reserved
			temp |= ( pGPIOHandle->GPIO_PinConfig.GPIO_PinIPType << ( ( 4 * temp2 ) + 2 ) );

			pGPIOHandle->pGPIOx->CRH = temp;
			temp = 0;
		}

	}else
	{
		// This part will code later for interrupt mode.
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT)
		{
			//1.configure the FTSR
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			//clear the corresponding RTSR bit
			EXTI->RTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

		}else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT)
		{
			//1.configure the RTSR
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			//clear the corresponding RTSR bit
			EXTI->FTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT)

		{
			//configure both FTSR and RTSR
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}

		//2.configure the GPIO port selection in SYSCFG_EXTICR
		uint8_t temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4;
		uint8_t temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4;
		uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);

		AFIO_PCLK_EN();
		AFIO->EXTICR[temp1] = portcode << ( temp2 * 4 );


		//3.enable the EXTI interrupt delivery using IMR
		EXTI->IMR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

	}

}


/****************************************************************************
 * @fn						- GPIO_DeInit
 *
 * @brief					- This function deinitializes the given GPIO port.
 *
 * @param[in]				- Handle address of GPIO
 * @param[in]				-
 * @param[in]				-
 *
 * @return					- none
 *
 * @Note					- none
 */
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{
	if(pGPIOx == GPIOA)
			{
				GPIOA_REG_RESET();
			}else if(pGPIOx == GPIOB)
			{
				GPIOB_REG_RESET();
			}else if(pGPIOx == GPIOC)
			{
				GPIOC_REG_RESET();
			}else if(pGPIOx == GPIOD)
			{
				GPIOD_REG_RESET();
			}else if(pGPIOx == GPIOE)
			{
				GPIOE_REG_RESET();
			}else if(pGPIOx == GPIOF)
			{
				GPIOF_REG_RESET();
			}else if(pGPIOx == GPIOG)
			{
				GPIOG_REG_RESET();
			}
}

/*
 * Data Read and Write
 */

/****************************************************************************
 * @fn						- GPIO_ReadFromInputPin
 *
 * @brief					- This function reads from given input pin
 *
 * @param[in]				- Base address of the GPIO peripheral
 * @param[in]				- Pin number
 * @param[in]				-
 *
 * @return					- uint8_t
 *
 * @Note					- return 0 or 1
 * @Note					- input pin degerini dondurdugu icin tek deger aliyoruz.
 *  * @Note					- degerler 0 ya da 1 olabilecegi icin uint8_t sectik.
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	uint8_t value;

	value = ( (pGPIOx->IDR >> PinNumber) & 0x1 );

	return value;
}

/****************************************************************************
 * @fn						- GPIO_ReadFromInputPort
 *
 * @brief					- This function reads from input port
 *
 * @param[in]				- Base address of the GPIO peripheral
 * @param[in]				-
 * @param[in]				-
 *
 * @return					- uint16_t
 *
 * @Note					- output port degerini dondurdugu icin butun registeri aliyoruz.
 *  * @Note					- 16 bitlik oldugu icin yetmesi icin uint16_t sectik.
 */
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
	uint16_t value;

	value = (uint16_t)pGPIOx->IDR;

	return value;
}

/****************************************************************************
 * @fn						- GPIO_WriteToOutputPin
 *
 * @brief					- This function writes to output pin
 *
 * @param[in]				- Base address of the GPIO peripheral
 * @param[in]				- Pin number
 * @param[in]				- Pin value
 *
 * @return					- none
 *
 * @Note					- none
 */
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value)
{
	if(Value == GPIO_PIN_SET)
	{
		pGPIOx->ODR |= (1<<PinNumber);
	}else
	{
		pGPIOx->ODR &= ~(1<<PinNumber);
	}
}

/****************************************************************************
 * @fn						- GPIO_WriteToOutputPort
 *
 * @brief					- This function writes to output port
 *
 * @param[in]				- Base address of the GPIO peripheral
 * @param[in]				- Pin value
 * @param[in]				-
 *
 * @return					- none
 *
 * @Note					- none
 */
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value)
{
	pGPIOx->ODR = Value;
}

/****************************************************************************
 * @fn						- GPIO_ToggleOutputPin
 *
 * @brief					- This function toggles output pin
 *
 * @param[in]				- Base address of the GPIO peripheral
 * @param[in]				- Pin number
 * @param[in]				-
 *
 * @return					- none
 *
 * @Note					- none
 */
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	pGPIOx->ODR ^= (1<<PinNumber);
}

/*
 * IRQ COnfiguration and ISR Handling
 */

/****************************************************************************
 * @fn						- GPIO_IRQConfig
 *
 * @brief					- This function configures IRQ
 *
 * @param[in]				-
 * @param[in]				-
 * @param[in]				-
 *
 * @return					- none
 *
 * @Note					- none
 */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(IRQNumber <= 31)
		{
			//program ISER0 register
			*NVIC_ISER0 |= (1 << IRQNumber );
		}else if(IRQNumber > 31 && IRQNumber < 64)
		{
			//program ISER1 register
			*NVIC_ISER1 |= (1 << IRQNumber % 32 );
		}else if(IRQNumber >= 64 && IRQNumber < 96)
		{
			//program ISER2 register // 64 to 95
			*NVIC_ISER2 |= (1 << IRQNumber % 64 );
		}
	}else
	{
		if(IRQNumber <= 31)
		{
			//program ICER0 register
			*NVIC_ICER0 |= (1 << IRQNumber );
		}else if(IRQNumber > 31 && IRQNumber < 64)
		{
			//program ICER1 register
			*NVIC_ICER1 |= (1 << IRQNumber % 32 );

		}else if(IRQNumber >= 64 && IRQNumber < 96)
		{
			//program ICER2 register
			*NVIC_ICER2 |= (1 << IRQNumber % 64 );
		}
	}
}

void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority)
{
	//1.First lets find out the IPR register
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;

	uint8_t shift_amount = (8 * iprx_section) + (8 - NO_IPR_BITS_IMPLEMENTED);
	*(NVIC_IPR_BASE_ADDR + (iprx * 4)) |= (IRQPriority << shift_amount);
}

/****************************************************************************
 * @fn						- GPIO_IRQHandling
 *
 * @brief					-
 *
 * @param[in]				- Pin number
 * @param[in]				-
 * @param[in]				-
 *
 * @return					- none
 *
 * @Note					- none
 */
void GPIO_IRQHandling(uint8_t PinNumber)
{
	//clear the exti pr register corresponding to the pin number
	if(EXTI->PR & (1 << PinNumber))
	{
		//clear
		EXTI->PR |= (1 << PinNumber);
	}
}



