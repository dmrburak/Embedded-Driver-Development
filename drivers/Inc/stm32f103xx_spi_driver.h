/*
 * stm32f103xx_spi_driver.h
 *
 *  Created on: Jan 5, 2024
 *      Author: burak
 */

#ifndef INC_STM32F103XX_SPI_DRIVER_H_
#define INC_STM32F103XX_SPI_DRIVER_H_

#include "stm32f103xx.h"


typedef struct
{
	uint8_t SPI_DeviceMode;
	uint8_t SPI_BusConfig;
	uint8_t SPI_SclkSpeed;				// Baud rate control
	uint8_t SPI_DFF;					// Data frame format
	uint8_t SPI_CPOL;
	uint8_t SPI_CPHA;
	uint8_t SPI_SSM;
}SPI_Config_t;

typedef struct
{
	SPI_RegDef_t	*pSPIx;
	SPI_Config_t	SPIConfig;
}SPI_Handle_t;


/*
 * @SPI_DeviceMode
 */
#define SPI_DEVICE_MODE_MASTER		1
#define SPI_DEVICE_MODE_SLAVE		0

/*
 * @SPI_BusConfig
 */
#define SPI_BUS_CONFIG_HD								1
#define SPI_BUS_CONFIG_FD								2
#define SPI_BUS_CONFIG_SIMPLEX_TXONLY					3
#define SPI_BUS_CONFIG_SIMPLEX_RXONLY					4

/*
 * @SPI_SclkSpeed
 */
#define SPI_SCLK_SPEED_DIV2 							0
#define SPI_SCLK_SPEED_DIV4 							1
#define SPI_SCLK_SPEED_DIV8 							2
#define SPI_SCLK_SPEED_DIV16 							3
#define SPI_SCLK_SPEED_DIV32 							4
#define SPI_SCLK_SPEED_DIV64 							5
#define SPI_SCLK_SPEED_DIV128 							6
#define SPI_SCLK_SPEED_DIV256 							7

/*
 * @SPI_DFF
 */
#define SPI_DFF_8BITS									0
#define SPI_DFF_16BITS									1

/*
 * @SPI_CPOL
 */
#define SPI_CPOL_HIGH									1
#define SPI_CPOL_LOW									0 //default


/*
 * @SPI_CPHA
 */
#define SPI_CPHA_HIGH									1
#define SPI_CPHA_LOW									0

/*
 * @SPI_SSM
 */
#define SPI_SSM_DI										0
#define SPI_SSM_EN										1

/*
 * SPI related status flag definition
 */
#define SPI_RXNE_FLAG		(1<<SPI_SR_RXNE)
#define SPI_TXE_FLAG		(1<<SPI_SR_TXE)
#define SPI_CHSIDE_FLAG		(1<<SPI_SR_CHSIDE)
#define SPI_UDR_FLAG		(1<<SPI_SR_UDR)
#define SPI_CRCERR_FLAG		(1<<SPI_SR_CRCERR)
#define SPI_MODF_FLAG		(1<<SPI_SR_MODF)
#define SPI_OVR_FLAG		(1<<SPI_SR_OVR)
#define SPI_BSY_FLAG		(1<<SPI_SR_BSY)

/*
 * Init and De-init
 */
void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DeInit(SPI_RegDef_t *pSPIx);

/*
 * Peripheral Clock setup
 */
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi);	// enable or disable

/*
 * Peripheral control on CR
 */
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi);
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi);

/*
 * Data Send and Receive
 */
void SPI_SendData(SPI_RegDef_t	*pSPIx, uint8_t *pTxBuffer, uint32_t Len);
void SPI_ReceiveData(SPI_RegDef_t	*pSPIx, uint8_t *pRxBuffer, uint32_t Len);

/*
 * IRQ Configuration and ISR handling
 */
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void SPI_IRQ_PriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void SPI_IRQHandling(SPI_Handle_t *pHandle);


uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName); // it will return 0 or 1

/*
 * Application callback
 */
void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t AppEv);





#endif /* INC_STM32F103XX_SPI_DRIVER_H_ */