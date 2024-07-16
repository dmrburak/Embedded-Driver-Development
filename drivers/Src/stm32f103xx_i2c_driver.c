/*
 * stm32f103xx_i2c_driver.c
 *
 *  Created on: 3 Tem 2024
 *      Author: B U R A K
 */

#include <stm32f103xx_i2c_driver.h>

uint16_t AHB_PreScaler[8] = {2,4,8,16,64,128,256,512};
uint8_t APB_PreScaler[4] = {2,4,8,16};

//private functions
static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx);
static void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx);
static void I2C_ExecuteAddressPhaseWrite(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr);
static void I2C_ExecuteAddressPhaseRead(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr);
static void I2C_ClearADDRFlag(I2C_RegDef_t *pI2Cx);


static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->CR1 |= (1 << I2C_CR1_START);
}

static void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->CR1 |= (1 << I2C_CR1_STOP);
}

/*
 * @I2C_ExecuteAddressPhase(I2C_RegDef_t *pI2Cx, uint8_t  SlaveAddr)
 * we are going to send the seven bit slave address and one bit of a read and write information.
 */
static void I2C_ExecuteAddressPhaseWrite(I2C_RegDef_t *pI2Cx, uint8_t  SlaveAddr)
{
	//first let's shift the slave address by one bit making space for the read write bit.
	SlaveAddr = SlaveAddr << 1;
	SlaveAddr &= ~(1); // clear the 0th bit. SlaveAddr is Slave address + r/nw(write) bit = 0.
	pI2Cx->DR = SlaveAddr;
}

static void I2C_ExecuteAddressPhaseRead(I2C_RegDef_t *pI2Cx, uint8_t  SlaveAddr)
{
	//first let's shift the slave address by one bit making space for the read write bit.
	SlaveAddr = SlaveAddr << 1;
	SlaveAddr |= (1); // clear the 0th bit. SlaveAddr is Slave address + r/nw(write) bit = 1.
	pI2Cx->DR = SlaveAddr;
}

static void I2C_ClearADDRFlag(I2C_RegDef_t *pI2Cx)
{
	uint32_t dummyRead;
	dummyRead = pI2Cx->SR1;
	dummyRead = pI2Cx->SR2;
	(void)dummyRead;
}


/*
 * I2C Peripheral Control
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
	}else
	{
		if(pI2Cx == I2C1)
		{
			I2C1_PCLK_DI();
		}else if(pI2Cx == I2C2)
		{
			I2C2_PCLK_DI();
		}
	}
}

uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t FlagName)
{
	if(pI2Cx->SR1 & FlagName)
	{
		return FLAG_SET;
	}
	return FLAG_RESET; // I2C veri gönderme işlemcisi boş olduğunda FLAG_RESET değerine sahip olur.
}

/*
 * I2C Init Function
 */
void I2C_Init(I2C_Handle_t *pI2CHandle)
{
	uint32_t tempreg=0;

	//enable the clock for the i2c peripheral
	I2C_PeriClockControl(pI2CHandle->pI2Cx, ENABLE);

	//ack control bit
	tempreg |= pI2CHandle->I2CConfig.I2C_AckControl << I2C_CR1_ACK;
	pI2CHandle->pI2Cx->CR1 = tempreg;

	//configure the FREQ field of CR2
	tempreg = 0;
	tempreg |= RCC_GetPCLK1Value() / 1000000U;
	pI2CHandle->pI2Cx->CR2 = (tempreg & 0x3F); // 0x3F = 63

	//program the device own address
	tempreg = 0;
	tempreg |= pI2CHandle->I2CConfig.I2C_DeviceAddress << 1;
	tempreg |= (1<<14);
	pI2CHandle->pI2Cx->OAR1 = tempreg;

	//CCR calculations
	uint16_t ccr_value = 0;
	tempreg = 0;
	if(pI2CHandle->I2CConfig.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
	{
		//mode is standard mode
		ccr_value = RCC_GetPCLK1Value() / ( 2 * pI2CHandle->I2CConfig.I2C_SCLSpeed );
		tempreg |= (ccr_value & 0xFFF); // ccr_value 16 bit ama biz 12 bitini kullanacağız. diğer bitleri maskeliyoruz.
	}else
	{
		//mode is fast mode
		tempreg |= (1<< I2C_FS);
		tempreg |= (pI2CHandle->I2CConfig.I2C_FMDutyCycle << 14);

		if(pI2CHandle->I2CConfig.I2C_FMDutyCycle == I2C_FM_DUTY_2)
		{
			ccr_value = RCC_GetPCLK1Value() / ( 3 * pI2CHandle->I2CConfig.I2C_SCLSpeed );
		}else
		{
			ccr_value = RCC_GetPCLK1Value() / ( 25 * pI2CHandle->I2CConfig.I2C_SCLSpeed );
		}
		tempreg |= (ccr_value & 0xFFF);
	}
	pI2CHandle->pI2Cx->CCR = tempreg;


	//TRISE configuration
	if(pI2CHandle->I2CConfig.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
	{
		//mode is standard mode
		tempreg |= (RCC_GetPCLK1Value() / 1000000000U) + 1;
	}else
	{
		tempreg |= ((RCC_GetPCLK1Value() * 300) / 1000000000U ) + 1;
	}

	pI2CHandle->pI2Cx->TRISE = (tempreg & 0x3F);
}

void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
		pI2Cx->CR1 |= (1 << I2C_CR1_PE);
	else{
		pI2Cx->CR1 &= ~(1 << I2C_CR1_PE);
	}
}




uint32_t RCC_GetPCLK1Value(void)
{
	uint32_t pclk1, SystemClk;

	uint8_t clksrc, temp, ahbp, apb1p;

	clksrc = ((RCC->CFGR >> 2) & 0x3);

	if(clksrc == 0)
	{
		SystemClk = 8000000; // HSI is 8MHz
	}else if(clksrc == 1)
	{
		SystemClk = 8000000; // HSE 3 to 25MHz
	}else if(clksrc == 2)
	{
		SystemClk = RCC_GetPLLOutputClock();
	}

	//for ahb
	temp = ((RCC->CFGR >> 4  & 0xF));	// 1111'i maskelemk için 0xF = 15
	if(temp<8)
	{
		ahbp = 1;
	}else
	{
		ahbp = AHB_PreScaler[temp - 8];
	}

	//for apb
	temp = ((RCC->CFGR >> 8  & 0x7)); // 111 'i maskelemek için
	if(temp<4)
	{
		apb1p = 1;
	}else
	{
		apb1p = AHB_PreScaler[temp - 4];
	}

	pclk1 = (SystemClk / ahbp) / apb1p;

	return pclk1;
}




uint32_t RCC_GetPLLOutputClock()
{

}


void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t Len, uint8_t SlaveAddr)
{

	//1. Generate the START condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	//2. confirm that generation is completed by checking the SB flag in SR1
	//   Note: Until SB is cleared SCL will be stretched (pulled to LOW)
	while(  !I2C_GetFlagStatus( pI2CHandle->pI2Cx, I2C_SB_FLAG )  ); // SB=1 olana kadar burada bekler.

	//3. Send the address of the slave with r/nw bit set to w(0) (total 8 bits)
	I2C_ExecuteAddressPhaseWrite(pI2CHandle->pI2Cx, SlaveAddr);

	//4. Confirm that address phase is completed by checking ADDR flag in the SR1
	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_ADDR_FLAG));

	//5. clear the ADDR flag according to its software sequence
	//	 Note: Until ADDR is cleared SCL will be stretched (pulled to LOW)
	I2C_ClearADDRFlag(pI2CHandle->pI2Cx);

	//6. send the data until Len becomes 0
	while(Len > 0)
	{
		while( !I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_TXE_FLAG) );
		pI2CHandle->pI2Cx->DR = *pTxBuffer;
		pTxBuffer++;
		Len--;
	}

	//7. when Len becomes zero wait for TXE=1 and BTF=1 before generating the STOP condition.
	//	 Note: TXE=1 , BTF=1 , means that both SR and DR are empty and next transmission should begin
	//   when BTF=1 SCL will be stretched (pulled to LOW)
	while( !I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_TXE_FLAG) );
	while( !I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_BTF_FLAG) );

	//8. Generate STOP condition and master need not to wait for the completion of stop condition.
	//	 Note: generatin STOP, automatically clears the BTF.
	I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
}


void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint32_t Len, uint8_t SlaveAddr)
{
	//1. Generate the START condition.
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	//2. Confirm that start generation is completed by checking the SB flag in the SR1
	//	 Note: Until SB is cleared SCL will be stretched (pulled to LOW)
	while(!(I2C_GetFlagStatus(pI2CHandle->pI2Cx->SR1, I2C_SB_FLAG)));

	//3. Send the address of the slave with r/nw bit set to R(1) (total 8 bits)
	I2C_ExecuteAddressPhaseRead(pI2CHandle->pI2Cx, SlaveAddr);

	//4. Wait until address phase is completed by checking the ADDR flag in the SR1
	while(!(I2C_GetFlagStatus(pI2CHandle->pI2Cx->SR1, I2C_ADDR_FLAG)));

	//procedure to read only 1 byte from slave
	if(Len == 1)
	{
		//Disable ACKing
		I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_DISABLE);

		//Generate STOP condition
		I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

		//Clear ADDR flag
		I2C_ClearADDRFlag(pI2CHandle->pI2Cx);

		//Wait until RXNE becomes 1
		while(!(I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_RXNE_FLAG)));

		//Read data in to buffer
		*pRxBuffer = pI2CHandle->pI2Cx->DR;

		return;
	}

	//procedure to read data from slave when Len > 1
	if(Len > 1)
	{
		//clear ADDR flag
		I2C_ClearADDRFlag(pI2CHandle->pI2Cx);

		//read the data until Len becomes zero
		for(uint32_t i = Len; i > 0 ; i--)
		{
			///wait until RXNE becomes 1
			while(!(I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_RXNE_FLAG)));

			if(i==2)
			{
				//clear the ACK bit
				pI2CHandle->I2CConfig.I2C_AckControl = I2C_ACK_DISABLE;

				//generate STOP condition
				I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
			}

			//read the data from data register in to buffer
			*pRxBuffer = pI2CHandle->pI2Cx->DR;

			//increment the buffer address
			pRxBuffer++;
		}
	}

	//re-enable ACKing	// bu fonksiyona girdiğinde zaten enable idi. çıkarken aynı hale getiriyoruz.
	if(pI2CHandle->I2CConfig.I2C_AckControl == I2C_ACK_ENABLE)
	{
		I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_ENABLE);
	}

}


I2C_ManageAcking(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
{
	if(EnorDi == I2C_ACK_ENABLE)
		{
		//enable the ack
		pI2Cx->CR1 |= (1<<I2C_CR1_ACK);
		}else
		{
			//disable the ack
			pI2Cx->CR1 &= ~(1<<I2C_CR1_ACK);
		}
}

uint8_t  I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle,uint8_t *pTxBuffer, uint32_t Len,uint8_t SlaveAddr,uint8_t Sr)
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


uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr)
{
	uint8_t busystate = pI2CHandle->TxRxState;

	if( (busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX))
	{
		pI2CHandle->pTxBuffer = pRxBuffer;
		pI2CHandle->TxLen = Len;
		pI2CHandle->TxRxState = I2C_BUSY_IN_RX;
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


void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle)
{
	//Interrupt handling for both master and slave mode of a device
	uint32_t temp;
	//1.Handle for interrupt generated by SM event
	//Note: SB flag is only applicable in Master mode

	//2.Handle for interrupt generated by ADDR  event
	//Note: When master mode: Address is sent
	//		When slave  mode: Address matched with own address

	//3.Handle for interrupt generated by BTF(Byte Transfer Finished) event

	//4.Handle for interrupt generated by STOPF event
	//Note: Stop detection flag is applicable only slave mode. For master this flag will

	//5.Handle for interrupt generated by TXE event

	//6.Handle for interrupt generated by RXNE event
}


void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle)
{

}


void I2C_SlaveSendData(I2C_RegDef_t *pI2C, uint8_t data)
{
	pI2C->DR = data;
}

uint8_t I2C_SlaveReceiveData(I2C_RegDef_t *pI2C)
{
	return (uint8_t) pI2C->DR;
}
