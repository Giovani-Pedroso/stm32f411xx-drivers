/*
 * stm32f411xx.h
 *
 *  Created on: Oct 12, 2022
 *      Author: santana
 */

#ifndef INC_STM32F411XX_H_
#define INC_STM32F411XX_H_

//include the types uint32_t
#include <stdint.h>

#define _vo volatile
/*
 * Base memory of the flash and the sram
 */
#define FLASH_BASEADDR 		0x08000000U
#define SRAM1_BASEADDR 		0x20000000U
#define SRAM 				SRAM1_BASE_ADDR

/*
 * AHBx and APBx bus peripheral base address
 */
#define PERIPH_BASE 		0x40000000U
#define APB1PERIPH_BASEADDR PERIPH_BASE
#define APB2PERIPH_BASEADDR 0x40010000U
#define AHB1PERIPH_BASEADDR 0x40020000U
#define AHB2PERIPH_BASEADDR 0x50000000U

/*
 * The base addres of the peripherals in the APB1
 */
#define TIMER2_BASEADDR		(APB1PERIPH_BASEADDR + 0x0000)
#define TIMER3_BASEADDR 	(APB1PERIPH_BASEADDR + 0x0400)
#define TIMER4_BASEADDR 	(APB1PERIPH_BASEADDR + 0x0800)
#define TIMER5_BASEADDR 	(APB1PERIPH_BASEADDR + 0x0c00)

#define RTC_BASEADDR 		(APB1PERIPH_BASEADDR + 0x2800)
#define WWDG_BASEADDR 		(APB1PERIPH_BASEADDR + 0x2c00)
#define IWDG_BASEADDR 		(APB1PERIPH_BASEADDR + 0x3000)

#define I2S2EXT_BASEADDR 	(APB1PERIPH_BASEADDR + 0x3400)

#define SPI2_I2S2_BASEADDR 	(APB1PERIPH_BASEADDR + 0x3800)
#define SPI3_I2S3_BASEADDR 	(APB1PERIPH_BASEADDR + 0x3c00)

#define I2S3EXT_BASEADDR 	(APB1PERIPH_BASEADDR + 0x4000)

#define USART2_BASEADDR 	(APB1PERIPH_BASEADDR + 0x4400)

#define I2C1_BASEADDR 		(APB1PERIPH_BASEADDR + 0x5400)
#define I2C2_BASEADDR 		(APB1PERIPH_BASEADDR + 0x5800)
#define I2C3_BASEADDR 		(APB1PERIPH_BASEADDR + 0x5c00)

#define PWR_BASEADDR 		(APB1PERIPH_BASEADDR + 0x6fff)

/*
 * base address for the apb2
 */
#define TIM1_BASEADDR 		(APB2PERIPH_BASEADDR + 0x0000)

#define USART1_BASEADDR 	(APB2PERIPH_BASEADDR + 0x1000)
#define USART6_BASEADDR 	(APB2PERIPH_BASEADDR + 0x1400)

#define ADC1_BASEADDR 		(APB2PERIPH_BASEADDR + 0x2000)
#define SDIO_BASEADDR 		(APB2PERIPH_BASEADDR + 0x2c00)

#define SPI1_I2S1_BASEADDR 	(APB2PERIPH_BASEADDR + 0x3000)
#define SPI4_I2S4_BASEADDR 	(APB2PERIPH_BASEADDR + 0x3400)

#define SYSCFG_BASEADDR 	(APB2PERIPH_BASEADDR + 0x3800)
#define EXTI_BASEADDR 		(APB2PERIPH_BASEADDR + 0x3c00)

#define TIM9_BASEADDR 		(APB2PERIPH_BASEADDR + 0x4000)
#define TIM10_BASEADDR 		(APB2PERIPH_BASEADDR + 0x4400)
#define TIM11_BASEADDR 		(APB2PERIPH_BASEADDR + 0x4800)

#define SPI5_I2SS5_BASEADDR (APB2PERIPH_BASEADDR + 0x5000)

/*
 * base address for the ahb1
 */
#define GPIOA_BASEADDR 		(AHB1PERIPH_BASEADDR + 0x0000)
#define GPIOB_BASEADDR 		(AHB1PERIPH_BASEADDR + 0x0400)
#define GPIOC_BASEADDR 		(AHB1PERIPH_BASEADDR + 0x0800)
#define GPIOD_BASEADDR 		(AHB1PERIPH_BASEADDR + 0x0c00)
#define GPIOE_BASEADDR 		(AHB1PERIPH_BASEADDR + 0x1000)
#define GPIOH_BASEADDR 		(AHB1PERIPH_BASEADDR + 0x1c00)
#define CRC_BASEADDR 		(AHB1PERIPH_BASEADDR + 0x3000)
#define RCC_BASEADDR	 	(AHB1PERIPH_BASEADDR + 0x3800)
#define FLASH_INTR_BASEADDR (APB2PERIPH_BASEADDR + 0x3c00)
#define DMA1_BASEADDR 		(AHB1PERIPH_BASEADDR + 0x6000)
#define DMA2_BASEADDR 		(AHB1PERIPH_BASEADDR + 0x6400)

/*
 * base for address for the AHB2
 */
#define USB_OTG_FS_BASEADDR (AHB2PERIPH_BASEADDR + 0x0000)


/*
 * Definition of the sructures of each peripheeral
 */
typedef struct{
	//The variables were define as uint32 because the architcture
	//of the mcu is 32 while work with embedded systems is a good
	// pratice to use the volatile, _vo is a define that i creaated
	// to avoid wirte volatile

	//the definitions MUST be in same sequency as the register map
	_vo uint32_t MODER;		// mode register
	_vo uint32_t OTYPER;	// output type register
	_vo uint32_t OSPEEDR;	// output speed register
	_vo uint32_t PUPDR;		// pull-up/pull-down register
	_vo uint32_t IDR;		// input data register
	_vo	uint32_t ODR;		// output data registe
	_vo	uint32_t BSRR;		// bit set/reset register
	_vo	uint32_t LCKR;		// configuration lock register
	_vo	uint32_t AFRL;		// alternate function low register
	_vo	uint32_t AFRH;		// alternate function high register
}GPIO_RegDef_t;//GPIO registers definition

typedef struct{
	_vo uint32_t CR;		// clock control register
	_vo uint32_t PLLCFGR ;	// PLL configuration register
	_vo uint32_t CFGR;		// clock configuration registe
	_vo uint32_t CIR;		// clock interrupt register
	_vo uint32_t AHB1RSTR ;	// AHB1 peripheral reset register
	_vo uint32_t AHB2RSTR;	// AHB2 peripheral reset register
	uint32_t RESERVED0[2];	//there are two possitions reverveted
	_vo uint32_t APB1RSTR;	// APB1 peripheral reset register for
	_vo uint32_t APB2RSTR ;	// APB2 peripheral reset register
	uint32_t RESERVED1[2];	//there are two possitions reverveted
	_vo uint32_t AHB1ENR ;	// AHB1 peripheral clock enable register
	_vo uint32_t AHB2ENR ;	// AHB2 peripheral clock enable register
	uint32_t RESERVED2[2];	//there are two possitions reverveted
	_vo uint32_t APB1ENR;	// APB1 peripheral clock enable register
	_vo uint32_t APB2ENR;	// APB2 peripheral clock enable register
	uint32_t RESERVED3[2];	//there are two possitions reverveted
	_vo uint32_t AHB1LPENR;	// AHB1 peripheral clock enable in low power mode register
	_vo uint32_t AHB2LPENR;	// AHB2 peripheral clock enable in low power mode register
	uint32_t RESERVED4[2];	//there are two possitions reverveted
	_vo uint32_t APB1LPENR;	// APB1 peripheral clock enable in low power mode register
	_vo uint32_t APB2LPENR;	// APB2 peripheral clock enabled in low power mode register
	uint32_t RESERVED5[2];	//there are two possitions reverveted
	_vo uint32_t BDCR;		// Backup domain control register
	_vo uint32_t CSR;		// clock control & status register
	uint32_t RESERVED6[2];	//there are two possitions reverveted
	_vo uint32_t SSCGR;		// spread spectrum clock generation registe
	_vo uint32_t PLLI2SCFGR;// PLLI2S configuration registe
	uint32_t RESERVED7;	//there are two possitions reverveted
	_vo uint32_t DCKCFGR;	// Dedicated Clocks Configuration Register

}RCC_RegDef_t;

/*
 * Peripheral definitions (Peripheral base addresses typecasted to xxx_RegDef_t
 */
#define GPIOA 	((GPIO_RegDef_t*) GPIOA_BASEADDR)
#define GPIOB	((GPIO_RegDef_t*) GPIOB_BASEADDR)
#define GPIOC	((GPIO_RegDef_t*) GPIOC_BASEADDR)
#define GPIOD 	((GPIO_RegDef_t*) GPIOD_BASEADDR)
#define GPIOE 	((GPIO_RegDef_t*) GPIOE_BASEADDR)
#define GPIOH 	((GPIO_RegDef_t*) GPIOH_BASEADDR)

#define RCC 	((RCC_RegDef_t*) RCC_BASEADDR)

/*
 * clock enable macors for the GPIOs
 */
#define GPIOA_PCLK_EN() (RCC->AHB1ENR |= (1<<0))
#define GPIOB_PCLK_EN() (RCC->AHB1ENR |= (1<<1))
#define GPIOC_PCLK_EN() (RCC->AHB1ENR |= (1<<2))
#define GPIOD_PCLK_EN() (RCC->AHB1ENR |= (1<<3))
#define GPIOE_PCLK_EN() (RCC->AHB1ENR |= (1<<4))
#define GPIOH_PCLK_EN() (RCC->AHB1ENR |= (1<<7))

/*
 * Clock enable macros for the i2c
 */
#define I2C1_PCLK_EN() (RCC->APB1ENR |= (1<<21))
#define I2C2_PCLK_EN() (RCC->APB1ENR |= (1<<22))
#define I2C3_PCLK_EN() (RCC->APB1ENR |= (1<<23))


/*
 * Clock enable for the spi
#define SPI2_PCLK_EN() (RCC->APB1ENR |= (1<<14))
 */
#define SPI0_PCLK_EN() (RCC->APB2ENR |= (1<<12))
#define SPI2_PCLK_EN() (RCC->APB1ENR |= (1<<14))
#define SPI3_PCLK_EN() (RCC->APB1ENR |= (1<<15))








#endif /* INC_STM32F411XX_H_ */
