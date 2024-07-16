/*
 * stm32f103xx_gpio_driver.h
 *
 *  Created on: Jan 5, 2024
 *      Author: burak
 */

#ifndef INC_STM32F103XX_GPIO_DRIVER_H_
#define INC_STM32F103XX_GPIO_DRIVER_H_


/*
 * stm32f103xx_gpio_driver.h
 *
 *  Created on: 15 Ara 2023
 *      Author: burak
 */


#include "stm32f103xx.h"




typedef struct
{
	uint8_t GPIO_PinNumber;				/*!< possible values from @GPIO_PIN_NUMBERS >*/
	uint8_t GPIO_PinMode;				/*!< possible values from @GPIO_PIN_MODES >*/
	uint8_t GPIO_PinOPType;				/*!< possible values from @GPIO_OP_TYPES >*/
	uint8_t GPIO_PinIPType;				/*!< possible values from @GPIO_IP_TYPES >*/

}GPIO_PinConfig_t;


typedef struct
{
	GPIO_RegDef_t		*pGPIOx;				/* This holds the base address of the GPIO port to which the pin belongs. */
	GPIO_PinConfig_t	GPIO_PinConfig;			/* This holds GPIO pin configuration settings. */
}GPIO_Handle_t;


/*
 * Init and De-init
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


/*
 * @GPIO_PIN_MODES
 * GPIO pin possible modes
 * Port configuration register low (GPIOx_CRL) and Port configuration register high (GPIOx_CRH)
 * CNFy[1:0]: Port x configuration bits (y= 0 .. 7)
 */

/*
 * @GPIO_IP_TYPES
 * In input mode (MODE[1:0]=00):
 */
#define GPIO_MODE_ANALOG			0				// Analog mode
#define GPIO_MODE_FLOATING_IP		1				// Floating input (reset state)
#define GPIO_MODE_IP_PU_PD			2				// Input with pull-up / pull-down
#define GPIO_MODE_RESERVED			3				// Reserved

/*
 * @GPIO_OP_TYPES
 * In output mode (MODE[1:0] > 00):
 */
#define GPIO_MODE_OP_PP				0				// General purpose output push-pull
#define GPIO_MODE_OP_OD				1				// General purpose output Open-drain
#define GPIO_MODE_AF_PP				2				// Alternate function output Push-pull
#define GPIO_MODE_AF_OD				3				// Alternate function output Open-drain

/*
 * @GPIO_PIN_SPEEDS (for outputs) | MODES
 * MODEy[1:0]: Port x mode bits (y= 0 .. 7)
 */
#define GPIO_MODE_INPUT				0	// Reset state
#define GPIO_MODE_MAXSPD_10MHZ		1
#define GPIO_MODE_MAXSPD_2MHZ		2
#define GPIO_MODE_MAXSPD_50MHZ		3

/*
 * Interrupt rising-falling-triggers (94.)
 */
#define GPIO_MODE_IT_RT				4 // input rising edge trigger
#define GPIO_MODE_IT_FT				5
#define GPIO_MODE_IT_RFT			6


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/*
 * @GPIO_PIN_NUMBERS
 * GPIO pin numbers
 */
#define GPIO_PIN_0				0
#define GPIO_PIN_1				1
#define GPIO_PIN_2				2
#define GPIO_PIN_3				3
#define GPIO_PIN_4				4
#define GPIO_PIN_5				5
#define GPIO_PIN_6				6
#define GPIO_PIN_7				7
#define GPIO_PIN_8				8
#define GPIO_PIN_9				9
#define GPIO_PIN_10				10
#define GPIO_PIN_11				11
#define GPIO_PIN_12				12
#define GPIO_PIN_13				13
#define GPIO_PIN_14				14
#define GPIO_PIN_15				15
/*
 * Peripheral Clock Setup
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi);	// enable or disable


/*
 * Data Read and Write
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value); // portta 16 pin oldugu icin uint16_t dedik
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

/*
 * IRQ COnfiguration and ISR Handling
 */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void GPIO_IRQHandling(uint8_t PinNumber);
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);

///
/// Bu kismi gpio interrupt izledigimde dolduracagim
///


#endif /* INC_STM32F103XX_GPIO_DRIVER_H_ */
