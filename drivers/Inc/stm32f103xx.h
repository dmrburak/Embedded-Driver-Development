/*
 * stm32f103xx.h
 *
 *  Created on: Jan 5, 2024
 *      Author: burak
 */

#ifndef INC_STM32F103XX_H_
#define INC_STM32F103XX_H_


#include <stdint.h>
#define __vo	volatile		// Registerler'de volatile kullanımına dikkat.




/************************START: Processor Specific Details******************************************/
/*
 * ARM Cortex M3 Processor NVIC ISERx register addresses
 */
#define NVIC_ISER0					((__vo uint32_t*)0xE000E100)
#define NVIC_ISER1					((__vo uint32_t*)0xE000E104)
#define NVIC_ISER2					((__vo uint32_t*)0xE000E108)
#define NVIC_ISER3					((__vo uint32_t*)0xE000E10C)

/*
 * ARM Cortex M3 Processor NVIC ICERx register addresses
 */
#define NVIC_ICER0					((__vo uint32_t*)0XE000E180)
#define NVIC_ICER1					((__vo uint32_t*)0XE000E184)
#define NVIC_ICER2					((__vo uint32_t*)0XE000E188)
#define NVIC_ICER3					((__vo uint32_t*)0XE000E18C)

/*
 * ARM Cortex M3 Processor NVIC IPRxx register addresses
 */
#define NVIC_IPR_BASE_ADDR					((__vo uint32_t*)0xE000E400)
//#define NVIC_IPR1					((__vo uint32_t*)0xE000E404)
//#define NVIC_IPR2					((__vo uint32_t*)0xE000E408)
//#define NVIC_IPR3					((__vo uint32_t*)0xE000E40C)
// base adrese ekleme yaparak secip kullanacagız. bu yuzden digerlerini yazmadik.

#define NO_IPR_BITS_IMPLEMENTED				4

/*
 * Base addresses of Flash and SRAM memories
 */

#define SRAM_BASE_ADDR							0x20000000UL
#define FLASH_BASE_ADDR							0x08000000UL

/*
 * AHBx and APBx Bus peripheral base addresses
 */

#define AHB_BASE_ADDR							0x40018000UL
#define APB2_BASE_ADDR							0x40010000UL
#define APB1_BASE_ADDR							0x40000000UL


typedef struct
{
	__vo uint32_t CR1;
	__vo uint32_t CR2;
	__vo uint32_t OAR1;
	__vo uint32_t OAR2;
	__vo uint32_t DR;
	__vo uint32_t SR1;
	__vo uint32_t SR2;
	__vo uint32_t CCR;
	__vo uint32_t TRISE;
}I2C_RegDef_t;


typedef struct
{
	__vo uint32_t CR1;				// SPI control register 1							Address offset:		0x00
	__vo uint32_t CR2;				// SPI control register 2							Address offset:		0x04
	__vo uint32_t SR;				// SPI status register								Address offset:		0x08
	__vo uint32_t DR;				// SPI data register								Address offset:		0x0C
	__vo uint32_t CRCPR;			// SPI CRC polynomial register						Address offset:		0x10
	__vo uint32_t RX_CRCR;			// SPI RX CRC register								Address offset:		0x14
	__vo uint32_t TX_CRCR;			// SPI TX CRC register								Address offset:		0x18
	__vo uint32_t I2SCFGR;			// SPI_I2S configuration register					Address offset:		0x1C
	__vo uint32_t I2SPR;			// SPI_I2S prescaler register						Address offset:		0x20

}SPI_RegDef_t;

typedef struct
{
	__vo uint32_t SR;
	__vo uint32_t DR;
	__vo uint32_t BRR;
	__vo uint32_t CR1;
	__vo uint32_t CR2;
	__vo uint32_t CR3;
	__vo uint32_t GTPR;	//Guard time and prescaler register

}USART_RegDef_t;

typedef struct
{
	__vo uint32_t	CR;				// Clock control register								Address offset: 0x00
	__vo uint32_t	CFGR;			// Clock configuration register							Address offset: 0x04
	__vo uint32_t	CIR;			// Clock interrupt register								Address offset: 0x08
	__vo uint32_t	APB2RSTR;		// APB2 peripheral reset register						Address offset: 0x0C
	__vo uint32_t	APB1RSTR;		// APB1 peripheral reset register						Address offset: 0x10
	__vo uint32_t	AHBENR;			// AHB peripheral clock enable register					Address offset: 0x14
	__vo uint32_t	APB2ENR;		// APB2 peripheral clock enable register				Address offset: 0x18
	__vo uint32_t	APB1ENR;		// APB1 peripheral clock enable register				Address offset: 0x1C
	__vo uint32_t	BDCR;			// Backup domain control register						Address offset: 0x20
	__vo uint32_t	CSR;			// Control/status register								Address offset: 0x24
	__vo uint32_t	AHBSTR;			// Peripheral clock reset register						Address offset: 0x28
	__vo uint32_t 	CFGR2;			// Clcok configuration register2						Address offset: 0x2C
}RCC_RegDef_t;

typedef struct
{

	__vo uint32_t CRL;		//  Port configuration register low						Address offset:		0x00
	__vo uint32_t CRH;		//  Port configuration register high					Address offset:		0x04
	__vo uint32_t IDR;		//	Port input data registe								Address offset:		0x08h
	__vo uint32_t ODR;		//	Port output data register							Address offset:		0x0C
	__vo uint32_t BSRR;		//	Port bit set/reset register							Address offset:		0x10
	__vo uint32_t BRR;		//	Port bit reset register								Address offset:		0x14
	__vo uint32_t LCKR;		//	Port configuration lock register					Address offset:		0x18
}GPIO_RegDef_t;

typedef struct
{
	__vo uint32_t IMR;
	__vo uint32_t EMR;
	__vo uint32_t RTSR;
	__vo uint32_t FTSR;
	__vo uint32_t SWIER;
	__vo uint32_t PR;
}EXTI_RegDef_t;

typedef struct
{
	__vo uint32_t EVCR;
	__vo uint32_t MAPR[3];
	__vo uint32_t EXTICR[4]; // exticr1-2-3-4
	__vo uint32_t MAPR2[2];
}AFIO_RegDef_t;

#define AFIO_BASE_ADDR					( APB2_BASE_ADDR + 0x00 )
#define AFIO							((AFIO_RegDef_t*)AFIO_BASE_ADDR)

#define EXTI_BASE_ADDR					( APB2_BASE_ADDR + 0x0400 )
#define EXTI							((EXTI_RegDef_t*)EXTI_BASE_ADDR)

#define SPI1_BASE_ADDR					( APB2_BASE_ADDR + 0x3000 )
#define SPI2_BASE_ADDR					( APB1_BASE_ADDR + 0x3800 )
#define SPI3_BASE_ADDR					( APB1_BASE_ADDR + 0x3C00 )
#define SPI1							((SPI_RegDef_t*)SPI1_BASE_ADDR)
#define SPI2							((SPI_RegDef_t*)SPI2_BASE_ADDR)
#define SPI3							((SPI_RegDef_t*)SPI3_BASE_ADDR)


#define I2C1_BASE_ADDR					( APB1_BASE_ADDR + 0x5400)
#define I2C2_BASE_ADDR					( APB1_BASE_ADDR + 0x5800)
#define I2C1							((I2C_RegDef_t*)I2C1_BASE_ADDR)	//peripheral definition macro
#define I2C2 							((I2C_RegDef_t*)I2C2_BASE_ADDR)


#define USART1_BASE_ADDR				(APB2_BASE_ADDR + 0X3800)
#define USART2_BASE_ADDR				(APB1_BASE_ADDR + 0X4400)
#define USART3_BASE_ADDR				(APB1_BASE_ADDR + 0X4800)
#define UART4_BASE_ADDR					(APB1_BASE_ADDR + 0X4C00)
#define UART5_BASE_ADDR					(APB1_BASE_ADDR + 0X5000)
#define USART1							((USART_RegDef_t*)USART1_BASE_ADDR)
#define USART2							((USART_RegDef_t*)USART2_BASE_ADDR)
#define USART3							((USART_RegDef_t*)USART3_BASE_ADDR)
#define UART4							((USART_RegDef_t*)UART4_BASE_ADDR)
#define UART5							((USART_RegDef_t*)UART5_BASE_ADDR)

#define RCC_BASE_ADDR					0x40021000UL
#define RCC								((RCC_RegDef_t*)RCC_BASE_ADDR)

#define GPIOA_BASE_ADDR					(APB2_BASE_ADDR + 0x0800)
#define GPIOB_BASE_ADDR					(APB2_BASE_ADDR + 0x0C00)
#define GPIOC_BASE_ADDR					(APB2_BASE_ADDR + 0x1000)
#define GPIOD_BASE_ADDR					(APB2_BASE_ADDR + 0x1400)
#define GPIOE_BASE_ADDR					(APB2_BASE_ADDR + 0x1800)
#define GPIOF_BASE_ADDR					(APB2_BASE_ADDR + 0x1C00)
#define GPIOG_BASE_ADDR					(APB2_BASE_ADDR + 0x2000)
#define GPIOA							((GPIO_RegDef_t*)GPIOA_BASE_ADDR)
#define GPIOB							((GPIO_RegDef_t*)GPIOB_BASE_ADDR)
#define GPIOC							((GPIO_RegDef_t*)GPIOC_BASE_ADDR)
#define GPIOD							((GPIO_RegDef_t*)GPIOD_BASE_ADDR)
#define GPIOE							((GPIO_RegDef_t*)GPIOE_BASE_ADDR)
#define GPIOF							((GPIO_RegDef_t*)GPIOF_BASE_ADDR)
#define GPIOG							((GPIO_RegDef_t*)GPIOG_BASE_ADDR)

//GPIO_RegDef_t *pGPIOA = GPIOA; // GPIOA base adresini GPIO_RegDef_t türünden bir işaretçiye dönüştürür.

/*
 * Clock Enable Macros for AFIO and EXTI
 */
#define AFIO_PCLK_EN()						( RCC->APB2ENR |= (1<<0) )
/*
 * Clock Enable Macros for GPIOx peripherals
 */
#define GPIOA_PCLK_EN()						( RCC->APB2ENR |= (1<<2) )
#define GPIOB_PCLK_EN()						( RCC->APB2ENR |= (1<<3) )
#define GPIOC_PCLK_EN()						( RCC->APB2ENR |= (1<<4) )
#define GPIOD_PCLK_EN()						( RCC->APB2ENR |= (1<<5) )
#define GPIOE_PCLK_EN()						( RCC->APB2ENR |= (1<<6) )
#define GPIOF_PCLK_EN()						( RCC->APB2ENR |= (1<<7) )
#define GPIOG_PCLK_EN()						( RCC->APB2ENR |= (1<<8) )

/*
 * Clock Enable Macros for I2Cx peripherals
 */
#define I2C1_PCLK_EN()							( RCC->APB1ENR |= (1<<21) )
#define I2C2_PCLK_EN()							( RCC->APB1ENR |= (1<<22) )
// Ikisini ayri ayri tanimlamak yerine (3<<21) yapabilirdik.

/*
 * Clock Enable Macros for SPIx peripherals
 */
#define SPI1_PCLK_EN()							( RCC->APB2ENR |= (1<<12) )
#define SPI2_PCLK_EN()							( RCC->APB1ENR |= (1<<14) )
#define SPI3_PCLK_EN()							( RCC->APB1ENR |= (1<<15) )

/*
 * Clock Enable Macros for USARTx peripherals
 */
#define USART1_PCLK_EN()							( RCC->APB2ENR |= (1<<14) )
#define USART2_PCLK_EN()							( RCC->APB1ENR |= (1<<17) )
#define USART3_PCLK_EN()							( RCC->APB1ENR |= (1<<18) )
#define UART4_PCLK_EN()								( RCC->APB1ENR |= (1<<19) )
#define UART5_PCLK_EN()								( RCC->APB1ENR |= (1<<20) )

/*
 * Clock Disable Macros for GPIOx peripherals
 */
#define GPIOA_PCLK_DI()						( RCC->APB2ENR &= ~(1<<2) )
#define GPIOB_PCLK_DI()						( RCC->APB2ENR &= ~(1<<3) )
#define GPIOC_PCLK_DI()						( RCC->APB2ENR &= ~(1<<4) )
#define GPIOD_PCLK_DI()						( RCC->APB2ENR &= ~(1<<5) )
#define GPIOE_PCLK_DI()						( RCC->APB2ENR &= ~(1<<6) )
#define GPIOF_PCLK_DI()						( RCC->APB2ENR &= ~(1<<7) )
#define GPIOG_PCLK_DI()						( RCC->APB2ENR &= ~(1<<8) )

/*
 * Clock Disable Macros for I2Cx peripherals
 */
#define I2C1_PCLK_DI()							( RCC->APB1ENR &= ~(1<<21) )
#define I2C2_PCLK_DI()							( RCC->APB1ENR &= ~(1<<22) )

/*
 * Clock Disable Macros for SPIx peripherals
 */
#define SPI1_PCLK_DI()							( RCC->APB2ENR &= ~(1<<12) )
#define SPI2_PCLK_DI()							( RCC->APB1ENR &= ~(1<<14) )
#define SPI3_PCLK_DI()							( RCC->APB1ENR &= ~(1<<15) )

/*
 * Clock Disable Macros for USARTx peripherals
 */
#define USART1_PCLK_DI()							( RCC->APB2ENR &= ~(1<<14) )
#define USART2_PCLK_DI()							( RCC->APB1ENR &= ~(1<<17) )
#define USART3_PCLK_DI()							( RCC->APB1ENR &= ~(1<<18) )
#define USART4_PCLK_DI()							( RCC->APB1ENR &= ~(1<<19) )
#define USART5_PCLK_DI()							( RCC->APB1ENR &= ~(1<<20) )


/*
 * Macros to reset GPIOx peripherals
 * Reset icin acip kapamamiz gerektigi icin, acip kapatiyoruz.
 * 1 durumunda reset oldugu icin 1 yaptiktan sonra 0 yapiyoruz.
 */
#define GPIOA_REG_RESET()					do{ (RCC->APB2RSTR |= (1<<2)); (RCC->APB2RSTR &= ~(1<<2)); }while(0)
#define GPIOB_REG_RESET()					do{ (RCC->APB2RSTR |= (1<<3)); (RCC->APB2RSTR &= ~(1<<3)); }while(0)
#define GPIOC_REG_RESET()					do{ (RCC->APB2RSTR |= (1<<4)); (RCC->APB2RSTR &= ~(1<<4)); }while(0)
#define GPIOD_REG_RESET()					do{ (RCC->APB2RSTR |= (1<<5)); (RCC->APB2RSTR &= ~(1<<5)); }while(0)
#define GPIOE_REG_RESET()					do{ (RCC->APB2RSTR |= (1<<6)); (RCC->APB2RSTR &= ~(1<<6)); }while(0)
#define GPIOF_REG_RESET()					do{ (RCC->APB2RSTR |= (1<<7)); (RCC->APB2RSTR &= ~(1<<7)); }while(0)
#define GPIOG_REG_RESET()					do{ (RCC->APB2RSTR |= (1<<8)); (RCC->APB2RSTR &= ~(1<<8)); }while(0)

/*
 * Macros to reset SPIx
 */
#define SPI1_REG_RESET()	do{ RCC->APB2RSTR |= (1<<12); RCC->APB2RSTR &= ~(1<<12); } while(0)
#define SPI2_REG_RESET()    do{ RCC->APB1RSTR |= (1<<14); RCC->APB1RSTR &= ~(1<<14); } while(0)
#define SPI3_REG_RESET()    do{ RCC->APB1RSTR |= (1<<15); RCC->APB1RSTR &= ~(1<<15); } while(0)

/*
 * Macros to reset USARTx
 */
#define USART1_REG_RESET()	do{ RCC->APB2RSTR |= (1<<14); RCC->APB2RSTR &= ~(1<<14); } while(0)
#define USART2_REG_RESET()	do{ RCC->APB1RSTR |= (1<<17); RCC->APB1RSTR &= ~(1<<17); } while(0)
#define USART3_REG_RESET()	do{ RCC->APB1RSTR |= (1<<18); RCC->APB1RSTR &= ~(1<<18); } while(0)
#define USART4_REG_RESET()	do{ RCC->APB1RSTR |= (1<<19); RCC->APB1RSTR &= ~(1<<19); } while(0)
#define USART5_REG_RESET()	do{ RCC->APB1RSTR |= (1<<20); RCC->APB1RSTR &= ~(1<<20); } while(0)

/*
 * Macros to reset I2Cx
 */
#define I2C1_REG_RESET()	do{ RCC->APB1RSTR |= (1<<21); RCC->APB1RSTR &= ~(1<<21); } while(0)
#define I2C2_REG_RESET()	do{ RCC->APB1RSTR |= (1<<22); RCC->APB1RSTR &= ~(1<<22); } while(0)


// some generic macros

#define ENABLE 				1
#define DISABLE 			0
#define SET					ENABLE
#define RESET				DISABLE
#define GPIO_PIN_SET		ENABLE
#define GPIO_PIN_RESET		DISABLE
#define FLAG_RESET			RESET
#define FLAG_SET			SET

/*
 * IRQ(Interrupt Request) Numbers of STM32F103x MCU
 */
#define IRQ_NO_EXTI0			6
#define IRQ_NO_EXTI1			7
#define IRQ_NO_EXTI2			8
#define IRQ_NO_EXTI3			9
#define IRQ_NO_EXTI4			10
#define IRQ_NO_EXTI9_5			23
#define IRQ_NO_EXTI15_10		40
#define IRQ_NO_I2C1_EV			31
#define IRQ_NO_I2C1_ER			32
#define IRQ_NO_I2C2_EV			33
#define IRQ_NO_I2C2_ER			34
#define IRQ_NO_SPI1				35
#define IRQ_NO_SPI2				36
#define IRQ_NO_SPI3				51
#define IRQ_NO_USART1			37
#define IRQ_NO_USART2			38
#define IRQ_NO_USART3			39
#define IRQ_NO_UART4			52
#define IRQ_NO_UART5			53

/*
 * macros for all the possible priority levels
 */
#define NVIC_IRQ_PRI0  			0
#define NVIC_IRQ_PRI15   		15

/*
 * Bit position definitions SPI_CR1
 */
#define SPI_CR1_CPHA			0
#define SPI_CR1_CPOL			1
#define SPI_CR1_MSTR			2
#define SPI_CR1_BR				3
#define SPI_CR1_SPE				6
#define SPI_CR1_LSBFIRST		7
#define SPI_CR1_SSI				8
#define SPI_CR1_SSM				9
#define SPI_CR1_RXONLY			10
#define SPI_CR1_DFF				11
#define SPI_CR1_CRCNEXT			12
#define SPI_CR1_CRCEN			13
#define SPI_CR1_BIDIOE			14
#define SPI_CR1_BIDIMODE		15

/*
 * Bit position definitions SPI_CR2
 */
#define SPI_CR2_RXDMAEN			0
#define SPI_CR2_TXDMAEN			1
#define SPI_CR2_SSOE			2
#define SPI_CR2_ERRIE			5
#define SPI_CR2_RXNEIE			6
#define SPI_CR2_TXEIE			7

/*
 * Bit position definitions SPI_SR
 */
#define SPI_SR_RXNE			0
#define SPI_SR_TXE			1
#define SPI_SR_CHSIDE		2
#define SPI_SR_UDR			3
#define SPI_SR_CRCERR		4
#define SPI_SR_MODF			5
#define SPI_SR_OVR			6
#define SPI_SR_BSY			7

/*
 * Bit position definitions I2C_CR1
 */
#define I2C_CR1_PE			0
#define I2C_CR1_SMBUS		1
#define I2C_CR1_SMBTYPE		3
#define I2C_CR1_ENARP		4
#define I2C_CR1_ENPEC		5
#define I2C_CR1_ENGC		6
#define I2C_CR1_NOSTRETCH	7
#define I2C_CR1_START		8
#define I2C_CR1_STOP		9
#define I2C_CR1_ACK			10
#define I2C_CR1_POS			11
#define I2C_CR1_PEC			12
#define I2C_CR1_ALERT		13
#define I2C_CR1_SWRST		15

/*
 * Bit position definitions I2C_CR2
 */
#define I2C_CR2_fREQ		5
#define I2C_CR2_ITERREN		8
#define I2C_CR2_ITEVTEN		9
#define I2C_CR2_ITBUFEN		10
#define I2C_CR2_DMAEN		11
#define I2C_CR2_LAST		12

/*
 * Bit position definitions I2C_SR1
 */
#define I2C_SR1_SB			0
#define I2C_SR1_ADDR		1
#define I2C_SR1_BTF			2
#define I2C_SR1_ADD10		3
#define I2C_SR1_STOPF		4
#define I2C_SR1_RxNE		6
#define I2C_SR1_TxE			7
#define I2C_SR1_BERR		8
#define I2C_SR1_ARL			9
#define I2C_SR1_AF			10
#define I2C_SR1_OVR			11
#define I2C_SR1_PECERR 		12
#define I2C_SR1_TIMEOUT		14
#define I2C_SR1_SMBALERT	15

/*
 * Bit position definitions I2C_SR2
 */
#define I2C_SR2_MSL			0
#define I2C_SR2_BUSY		1
#define I2C_SR2_TRA			2
#define I2C_SR2_GENCALL		4
#define I2C_SR2_SMBDEFAULT	5
#define I2C_SR2_SMBHOST		6
#define I2C_SR2_DUALF		7
#define I2C_SR2_PEC			8

/*
 * Bit position definitions I2C_CCR. Sm:Standard Mode, Fm:Fast Mode.
 */
#define I2C_CCRR				11	//Clock control register in Fm/Sm mode(master mode)
#define I2C_DUTY			14	//Fm mode duty cycle
#define I2C_FS				15	//I2C master mode selection.(Sm or Fm)

/*
 * Bit position definitions I2C_TRISE
 */
#define I2C_TRISEE			5	//Maximum rise time in Fm/Sm mode (master mode)

/*
 * Bit position definitions USART_CR1
 */
#define USART_CR1_SBK					0
#define USART_CR1_RWU 					1
#define USART_CR1_RE  					2
#define USART_CR1_TE 					3
#define USART_CR1_IDLEIE 				4
#define USART_CR1_RXNEIE  				5
#define USART_CR1_TCIE					6
#define USART_CR1_TXEIE					7
#define USART_CR1_PEIE 					8
#define USART_CR1_PS 					9
#define USART_CR1_PCE 					10
#define USART_CR1_WAKE  				11
#define USART_CR1_M 					12
#define USART_CR1_UE 					13
#define USART_CR1_OVER8  				15



/*
 * Bit position definitions USART_CR2
 */
#define USART_CR2_ADD   				0
#define USART_CR2_LBDL   				5
#define USART_CR2_LBDIE  				6
#define USART_CR2_LBCL   				8
#define USART_CR2_CPHA   				9
#define USART_CR2_CPOL   				10
#define USART_CR2_CLKEN   				11
#define USART_CR2_STOP   				13
#define USART_CR2_LINEN   				14


/*
 * Bit position definitions USART_CR3
 */
#define USART_CR3_EIE   				0
#define USART_CR3_IREN   				1
#define USART_CR3_IRLP  				2
#define USART_CR3_HDSEL   				3
#define USART_CR3_NACK   				4
#define USART_CR3_SCEN   				5
#define USART_CR3_DMAR  				6
#define USART_CR3_DMAT   				7
#define USART_CR3_RTSE   				8
#define USART_CR3_CTSE   				9
#define USART_CR3_CTSIE   				10

/*
 * Bit position definitions USART_SR
 */
#define USART_SR_PE        				0
#define USART_SR_FE        				1
#define USART_SR_NE        				2
#define USART_SR_ORE       				3
#define USART_SR_IDLE       			4
#define USART_SR_RXNE        			5
#define USART_SR_TC        				6
#define USART_SR_TXE        			7
#define USART_SR_LBD        			8
#define USART_SR_CTS        			9


#define GPIO_BASEADDR_TO_CODE(x)      ( (x == GPIOA)?0:\
										(x == GPIOB)?1:\
										(x == GPIOC)?2:\
										(x == GPIOD)?3:\
								        (x == GPIOE)?4:\
								        (x == GPIOF)?5:\
								        (x == GPIOG)?6:0)


#include "stm32f103xx_gpio_driver.h"
#include "stm32f103xx_spi_driver.h"
#include "stm32f103xx_i2c_driver.h"
#include "stm32f103xx_usart_driver.h"

#endif /* INC_STM32F103XX_H_ */
