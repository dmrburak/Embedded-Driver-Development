/*
 * stm32f103xx_spi_driver.c
 *
 *  Created on: Jan 5, 2024
 *      Author: burak
 */


#include "stm32f103xx_spi_driver.h"

/*
 * SPI Peripheral Control
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
	}else
	{
		if(pSPIx == SPI1)
		{
			SPI1_PCLK_DI();
		}else if(pSPIx == SPI2)
		{
			SPI2_PCLK_DI();
		}else if(pSPIx == SPI3)
		{
			SPI3_PCLK_DI();
		}
	}
}

/*
 * SPI Init Function
 */
void SPI_Init(SPI_Handle_t *pSPIHandle)
{
	uint16_t tempreg=0;

	//peripheral clock enable
	SPI_PeriClockControl(pSPIHandle->pSPIx, ENABLE);

	//1.configure the device mode
	tempreg |= pSPIHandle->SPIConfig.SPI_DeviceMode << SPI_CR1_MSTR;

	//2.configure the bus config
	if(pSPIHandle->SPIConfig.SPI_BusConfig==SPI_BUS_CONFIG_FD)
	{
		//bidi mode should be cleared
		tempreg &= ~(1<<SPI_CR1_BIDIMODE);
	}else if(pSPIHandle->SPIConfig.SPI_BusConfig==SPI_BUS_CONFIG_HD)
	{
		//bidi mode should be set
		tempreg |= (1<<SPI_CR1_BIDIMODE);
	}else if(pSPIHandle->SPIConfig.SPI_BusConfig==SPI_BUS_CONFIG_SIMPLEX_RXONLY)
	{
		//bidi mode should be cleared
		tempreg &= ~(1<<SPI_CR1_BIDIMODE);
		//RXONLY bit must be set
		tempreg |= (1<<SPI_CR1_RXONLY);
	}

	//3.configure the serial clock speed
	tempreg |= pSPIHandle->SPIConfig.SPI_SclkSpeed << SPI_CR1_BR;

	//4.configure the data frame format
	tempreg |= pSPIHandle->SPIConfig.SPI_DFF << SPI_CR1_DFF;

	//5.configure the CPOL
	tempreg |= pSPIHandle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL;

	//6.configure the CPHA
	tempreg |= pSPIHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA;

	//7.configure the SSM
	tempreg |= pSPIHandle->SPIConfig.SPI_SSM << SPI_CR1_SSM;


	pSPIHandle->pSPIx->CR1 = tempreg;

}



/*
 * SPI DeInit Function
 */
void SPI_DeInit(SPI_RegDef_t *pSPIHandle)
{
	if(pSPIHandle == SPI1)
	{
		SPI1_REG_RESET();
	}else if(pSPIHandle == SPI2)
	{
		SPI2_REG_RESET();
	}else if(pSPIHandle == SPI3)
	{
		SPI3_REG_RESET();
	}else
	{

	}
}

uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName)
{
	if(pSPIx->SR & FlagName)
	{
		return FLAG_SET;
	}
	return FLAG_RESET; // SPI veri gönderme işlemcisi boş olduğunda FLAG_RESET değerine sahip olur.
}

void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len)
{
	while(Len > 0)
	{
		//1. wait until TXE is set
		while( SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG) == FLAG_RESET );
		//Bu satır, TXE bayrağının FLAG_RESET olmadığı sürece döngünün devam etmesini sağlar.
		//bu döngü içerisine girer ve bayrak set edilene (veri gönderme tamponu boşalana) kadar bekler.

		//2. check the DFF bit in CR1
		if(pSPIx->CR1 & ( 1 << SPI_CR1_DFF ))
		{
			//16 bit DFF
			//load the data into the DR
			pSPIx->DR = *((uint16_t*)pTxBuffer); //16bit = 2byte'lik veri yazdigimiz icin 2 kez Len--
			Len--;
			Len--;
			(uint16_t*)pTxBuffer++; // 2 byte transfer ettigimiz icin 2 artmali
		}else
		{
			//8 bit DFF
			pSPIx->DR = *(pTxBuffer); //8bit = 1byte'lik veri yazdigimiz icin 1 kez Len--
			Len--;
			pTxBuffer++;

		}

		// Check for overrun error
//		        if (pSPIx->SR & (1 << SPI_SR_OVR))
//		        {
//		            // Clear the OVR flag by reading DR and SR
//		            uint8_t temp = pSPIx->DR;
//		            temp = pSPIx->SR;
//		            (void)temp; // To suppress unused variable warning
//		        }
	}

	// Wait until TXE is set and BSY flag is reset
//	    while (!(pSPIx->SR & (1 << SPI_SR_TXE)));
//	    while (pSPIx->SR & (1 << SPI_SR_BSY));
}


void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len)
{
	while(Len > 0)
		{
			//1. wait until RXE is set
			while( SPI_GetFlagStatus(pSPIx, SPI_RXNE_FLAG) == FLAG_RESET );
			//bu döngü içerisine girer ve bayrak reset edilene (veri alma tamponu boşalana) kadar bekler.

			//2. check the DFF bit in CR1
			if(pSPIx->CR1 & ( 1 << SPI_CR1_DFF ))
			{
				//16 bit DFF
				//load the data from DR to RxBuffer address.
				*((uint16_t*)pRxBuffer) = pSPIx->DR; //16bit = 2byte'lik veri yazdigimiz icin 2 kez Len--
				Len--;
				Len--;
				(uint16_t*)pRxBuffer++; // 2 byte transfer ettigimiz icin 2 artmali
			}else
			{
				//8 bit DFF
				*(pRxBuffer) = pSPIx->DR; //8bit = 1byte'lik veri yazdigimiz icin 1 kez Len--
				Len--;
				pRxBuffer++;

			}

			// Check for overrun error
	//		        if (pSPIx->SR & (1 << SPI_SR_OVR))
	//		        {
	//		            // Clear the OVR flag by reading DR and SR
	//		            uint8_t temp = pSPIx->DR;
	//		            temp = pSPIx->SR;
	//		            (void)temp; // To suppress unused variable warning
	//		        }
		}
}



void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		pSPIx->CR1 |= (1<<SPI_CR1_SPE);
	}else
	{
		pSPIx->CR1 &= ~(1<<SPI_CR1_SPE);
	}
}


void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		pSPIx->CR1 |= (1<<SPI_CR1_SSI);
	}else
	{
		pSPIx->CR1 &= ~(1<<SPI_CR1_SSI);
	}
}
