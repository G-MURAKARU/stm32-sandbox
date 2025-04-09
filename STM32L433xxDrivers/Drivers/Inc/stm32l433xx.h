/*
 * stm32l433xx.h
 *
 *  Created on: Mar 31, 2025
 *      Author: MURAKARU
 */

#ifndef INC_STM32L433XX_H_
#define INC_STM32L433XX_H_


#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>


/**********************************START:Processor Specific Details **********************************/

/*
 * ARM Cortex-Mx Processor NVIC ISERx register Addresses, from user guide
 */
#define NVIC_ISERx_BASE_ADDR			0xE000E100UL
#define NVIC_ISER0						( (__RW uint32_t *)(NVIC_ISERx_BASE_ADDR + 0x0UL) )
#define NVIC_ISER1						( (__RW uint32_t *)(NVIC_ISERx_BASE_ADDR + 0x4UL) )
#define NVIC_ISER2						( (__RW uint32_t *)(NVIC_ISERx_BASE_ADDR + 0x8UL) )
#define NVIC_ISER3						( (__RW uint32_t *)(NVIC_ISERx_BASE_ADDR + 0xCUL) )


#define NVIC_ICERx_BASE_ADDR			0xE000E180UL
#define NVIC_ICER0						( (__RW uint32_t *)(NVIC_ICERx_BASE_ADDR + 0x0UL) )
#define NVIC_ICER1						( (__RW uint32_t *)(NVIC_ICERx_BASE_ADDR + 0x4UL) )
#define NVIC_ICER2						( (__RW uint32_t *)(NVIC_ICERx_BASE_ADDR + 0x8UL) )
#define NVIC_ICER3						( (__RW uint32_t *)(NVIC_ICERx_BASE_ADDR + 0xCUL) )

#define NVIC_IPRx_BASE_ADDR			0xE000E400UL
#define NO_PR_BITS_IMPLEMENTED		(uint8_t) 4


#define __RW							volatile			/* volatile, readable and writable */
#define __R							const				/* read only */
#define __W							volatile			/* volatile, write only */


/* Base Addresses of the MCU Internal Memories */
#define SRAM1_BASE_ADDR 				0x20000000UL
#define SRAM2_BASE_ADDR 				0x2000C000UL

#define SRAM_BASE_ADDR 				SRAM1_BASE_ADDR

#define ROM_BASE_ADDR 				0x1FFF0000UL


/* Base Addresses of the MCU Bus Domains */
#define PERIPHERAL_BASE_ADDR 			0x40000000U

#define APB1_PERIPHERAL_OFFSET		0x0UL
#define APB1_BASE_ADDR				(PERIPHERAL_BASE_ADDR + APB1_PERIPHERAL_OFFSET)

#define APB2_PERIPHERAL_OFFSET		0x10000UL
#define APB2_BASE_ADDR				(PERIPHERAL_BASE_ADDR + APB2_PERIPHERAL_OFFSET)

#define AHB1_PERIPHERAL_OFFSET		0x20000UL
#define AHB1_BASE_ADDR				(PERIPHERAL_BASE_ADDR + AHB1_PERIPHERAL_OFFSET)

#define AHB2_PERIPHERAL_OFFSET		0x8000000UL
#define AHB2_BASE_ADDR				(PERIPHERAL_BASE_ADDR + AHB2_PERIPHERAL_OFFSET)

/* Base Addresses of Peripherals on each Bus Domain, AHBx and APBx */

/* Peripherals on AHB1 */
#define DMA1_AHB1_OFFSET 				0x0UL
#define DMA1_BASE_ADDR				(AHB1_BASE_ADDR + DMA1_AHB1_OFFSET)

#define DMA2_AHB1_OFFSET				0x400UL
#define DMA2_BASE_ADDR				(AHB1_BASE_ADDR + DMA2_AHB1_OFFSET)

#define RCC_AHB1_OFFSET				0x1000UL
#define RCC_BASE_ADDR					(AHB1_BASE_ADDR + RCC_AHB1_OFFSET)

#define FLASH_REG_AHB1_OFFSET			0x2000UL
#define FLASH_REG_BASE_ADDR			(AHB1_BASE_ADDR + FLASH_REG_AHB1_OFFSET)

#define CRC_AHB1_OFFSET				0x3000UL
#define CRC_BASE_ADDR					(AHB1_BASE_ADDR + CRC_AHB1_OFFSET)

#define TSC_AHB1_OFFSET				0x4000UL
#define TSC_BASE_ADDR					(AHB1_BASE_ADDR + TSC_AHB1_OFFSET)

/* Peripherals on AHB2 */
#define GPIOA_AHB2_OFFSET				0x0UL
#define GPIOA_BASE_ADDR				(AHB2_BASE_ADDR + GPIOA_AHB2_OFFSET)

#define GPIOB_AHB2_OFFSET				0x400UL
#define GPIOB_BASE_ADDR				(AHB2_BASE_ADDR + GPIOB_AHB2_OFFSET)

#define GPIOC_AHB2_OFFSET				0x800UL
#define GPIOC_BASE_ADDR				(AHB2_BASE_ADDR + GPIOC_AHB2_OFFSET)

#define ADC_AHB2_OFFSET				0x8040000UL
#define ADC_BASE_ADDR					(AHB2_BASE_ADDR + ADC_AHB2_OFFSET)

#define RNG_AHB2_OFFSET				0x8060800UL
#define RNG_BASE_ADDR					(AHB2_BASE_ADDR + RNG_AHB2_OFFSET)

/* Peripherals on APB1 */
#define TIM2_APB1_OFFSET				0x0UL
#define TIM2_BASE_ADDR				(APB1_BASE_ADDR + TIM2_APB1_OFFSET)

#define TIM6_APB1_OFFSET				0x1000UL
#define TIM6_BASE_ADDR				(APB1_BASE_ADDR + TIM6_APB1_OFFSET)

#define TIM7_APB1_OFFSET				0x1400UL
#define TIM7_BASE_ADDR				(APB1_BASE_ADDR + TIM7_APB1_OFFSET)

#define LCD_APB1_OFFSET				0x2400UL
#define LCD_BASE_ADDR					(APB1_BASE_ADDR + LCD_APB1_OFFSET)

#define RTC_APB1_OFFSET				0x2800UL
#define RTC_BASE_ADDR					(APB1_BASE_ADDR + RTC_APB1_OFFSET)

#define WWDG_APB1_OFFSET				0x2C00UL
#define WWDG_BASE_ADDR				(APB1_BASE_ADDR + WWDG_APB1_OFFSET)

#define IWDG_APB1_OFFSET				0x3000UL
#define IWDG_BASE_ADDR				(APB1_BASE_ADDR + IWDG_APB1_OFFSET)

#define SPI2_APB1_OFFSET				0x3800UL
#define SPI2_BASE_ADDR				(APB1_BASE_ADDR + SPI2_APB1_OFFSET)

#define SPI3_APB1_OFFSET				0x3C00UL
#define SPI3_BASE_ADDR				(APB1_BASE_ADDR + SPI3_APB1_OFFSET)

#define USART2_APB1_OFFSET			0x4400UL
#define USART2_BASE_ADDR				(APB1_BASE_ADDR + USART2_APB1_OFFSET)

#define USART3_APB1_OFFSET			0x4800UL
#define USART3_BASE_ADDR				(APB1_BASE_ADDR + USART3_APB1_OFFSET)

#define I2C1_APB1_OFFSET				0x5400UL
#define I2C1_BASE_ADDR				(APB1_BASE_ADDR + I2C1_APB1_OFFSET)

#define I2C2_APB1_OFFSET				0x5800UL
#define I2C2_BASE_ADDR				(APB1_BASE_ADDR + I2C2_APB1_OFFSET)

#define I2C3_APB1_OFFSET				0x5C00UL
#define I2C3_BASE_ADDR				(APB1_BASE_ADDR + I2C3_APB1_OFFSET)

#define CRS_APB1_OFFSET				0x6000UL
#define CRS_BASE_ADDR					(APB1_BASE_ADDR + CRS_APB1_OFFSET)

#define CAN1_APB1_OFFSET				0x6400UL
#define CAN1_BASE_ADDR				(APB1_BASE_ADDR + CAN1_APB1_OFFSET)

#define USB_FS_APB1_OFFSET			0x6800UL
#define USB_FS_BASE_ADDR				(APB1_BASE_ADDR + USB_FS_APB1_OFFSET)

#define USB_SRAM_APB1_OFFSET			0x6C00UL
#define USB_SRAM_BASE_ADDR			(APB1_BASE_ADDR + USB_SRAM_APB1_OFFSET)

#define PWR_REG_APB1_OFFSET			0x7000UL
#define PWR_REG_BASE_ADDR				(APB1_BASE_ADDR + PWR_REG_APB1_OFFSET)

#define DAC1_APB1_OFFSET				0x7400UL
#define DAC1_BASE_ADDR				(APB1_BASE_ADDR + DAC1_APB1_OFFSET)

#define OPAMP_APB1_OFFSET				0x7800UL
#define OPAMP_BASE_ADDR				(APB1_BASE_ADDR + OPAMP_APB1_OFFSET)

#define LPTIM1_APB1_OFFSET			0x7C00UL
#define LPTIM1_BASE_ADDR				(APB1_BASE_ADDR + LPTIM1_APB1_OFFSET)

#define LPUART_APB1_OFFSET			0x8000UL
#define LPUART_BASE_ADDR				(APB1_BASE_ADDR + LPUART_APB1_OFFSET)

#define SWPMI1_APB1_OFFSET			0x8800UL
#define SWPMI1_BASE_ADDR				(APB1_BASE_ADDR + SWPMI1_APB1_OFFSET)

#define LPTIM2_APB1_OFFSET			0x9400UL
#define LPTIM2_BASE_ADDR				(APB1_BASE_ADDR + LPTIM2_APB1_OFFSET)

/* Peripherals on APB2 */
#define SYSCFG_APB2_OFFSET			0x0UL
#define SYSCFG_BASE_ADDR				(APB2_BASE_ADDR + SYSCFG_APB2_OFFSET)

#define VREFBUF_APB2_OFFSET			0x30UL
#define VREFBUF_BASE_ADDR				(APB2_BASE_ADDR + VREFBUF_APB2_OFFSET)

#define COMP_APB2_OFFSET				0x200UL
#define COMP_BASE_ADDR				(APB2_BASE_ADDR + COMP_APB2_OFFSET)

#define EXTI_APB2_OFFSET				0x400UL
#define EXTI_BASE_ADDR				(APB2_BASE_ADDR + EXTI_APB2_OFFSET)

#define FIREWALL_APB2_OFFSET			0x1C00UL
#define FIREWALL_BASE_ADDR			(APB2_BASE_ADDR + FIREWALL_APB2_OFFSET)

#define SDMMC1_APB2_OFFSET			0x2800UL
#define SDMMC1_BASE_ADDR				(APB2_BASE_ADDR + SDMMC1_APB2_OFFSET)

#define TIM1_APB2_OFFSET				0x2C00UL
#define TIM1_BASE_ADDR				(APB2_BASE_ADDR + TIM1_APB2_OFFSET)

#define SPI1_APB2_OFFSET				0x3000UL
#define SPI1_BASE_ADDR				(APB2_BASE_ADDR + SPI1_APB2_OFFSET)

#define USART1_APB2_OFFSET			0x3800UL
#define USART1_BASE_ADDR				(APB2_BASE_ADDR + USART1_APB2_OFFSET)

#define TIM15_APB2_OFFSET				0x4000UL
#define TIM15_BASE_ADDR				(APB2_BASE_ADDR + TIM15_APB2_OFFSET)

#define TIM16_APB2_OFFSET				0x4400UL
#define TIM16_BASE_ADDR				(APB2_BASE_ADDR + TIM16_APB2_OFFSET)

#define SAI1_APB2_OFFSET				0x5400UL
#define SAI1_BASE_ADDR				(APB2_BASE_ADDR + SAI1_APB2_OFFSET)


/********************************** PERIPHERAL REGISTER STRUCTURE DEFINITIONS **********************************/

/*
 * @GPIO_PERIPHERAL_REGISTERS
 * Initializer for a GPIO peripheral instance, containing all memory-mapped registers
 */
typedef struct GPIO_PeripheralRegisters
{
	__RW uint32_t MODER;					/* GPIO Port Mode Register */
	__RW uint32_t OTYPER;					/* GPIO Port Output Type Register */
	__RW uint32_t OSPEEDR;					/* GPIO Port Output Speed Register */
	__RW uint32_t PUPDR;					/* GPIO Port Pull-Up/Pull-Down Register */
	__R volatile uint32_t IDR;				/* GPIO Port Input Data Register */
	__RW uint32_t ODR;						/* GPIO Port Output Data Register */
	__W  uint32_t BSRR;						/* GPIO Port Bit Set-Reset Register */
	__RW uint32_t LCKR;						/* GPIO Port Configuration Lock Register */
	__RW uint32_t AFRL;						/* GPIO Port Alternate Function Low Register */
	__RW uint32_t AFRH;						/* GPIO Port Alternate Function High Register */
	__W  uint32_t BRR;						/* GPIO Port Bit Reset Register */
} GPIOx_Reg_t;

/*
 * @RCC_PERIPHERAL_REGISTERS
 * Initializer for an RCC peripheral instance, containing all memory-mapped registers
 */
typedef struct RCC_PeripheralRegisters
{
	__RW uint32_t CR;						/* RCC Clock Control Register */
	__RW uint32_t ICSCR;					/* RCC Internal Clock Sources Calibration Register - [0:7, 16:23] - R, [8:15, 24:31] - RW */
	__RW uint32_t CFGR;						/* RCC Clock Configuration Register */
	__RW uint32_t PLLCFGR;					/* RCC PLL Configuration Register */
	__RW uint32_t PLLSAI1CFGR;				/* RCC PLLSAI1 Configuration Register */
	uint32_t RESERVED1;						/* Reserved Offset 0x14 - 0x17 */
	__RW uint32_t CIER;						/* RCC Clock Interrupt Enable Register */
	__R volatile uint32_t CIFR;			/* RCC Clock Interrupt Flag Register */
	__W  uint32_t CICR;						/* RCC Clock Interrupt Clear Register */
	uint32_t RESERVED2;						/* Reserved Offset 0x24 - 0x27 */
	__RW uint32_t AHB1RSTR;					/* RCC AHB1 Peripheral Reset Register */
	__RW uint32_t AHB2RSTR;					/* RCC AHB2 Peripheral Reset Register */
	__RW uint32_t AHB3RSTR;					/* RCC AHB3 Peripheral Reset Register - QuadSPI */
	uint32_t RESERVED3;						/* Reserved Offset 0x34 - 0x37 */
	__RW uint32_t APB1RSTR1;				/* RCC APB1 Peripheral Reset Register 1 */
	__RW uint32_t APB1RSTR2;				/* RCC APB1 Peripheral Reset Register 2 */
	__RW uint32_t APB2RSTR;					/* RCC APB2 Peripheral Reset Register */
	uint32_t RESERVED4;						/* Reserved Offset 0x44 - 0x47 */
	__RW uint32_t AHB1ENR;					/* RCC AHB1 Peripheral Clock Enable Register */
	__RW uint32_t AHB2ENR;					/* RCC AHB2 Peripheral Clock Enable Register */
	__RW uint32_t AHB3ENR;					/* RCC AHB3 Peripheral Clock Enable Register - QuadSPI */
	uint32_t RESERVED5;						/* Reserved Offset 0x54 - 0x57 */
	__RW uint32_t APB1ENR1;					/* RCC APB1 Peripheral Clock Enable Register 1 */
	__RW uint32_t APB1ENR2;					/* RCC APB1 Peripheral Clock Enable Register 2 */
	__RW uint32_t APB2ENR;					/* RCC APB2 Peripheral Clock Enable Register */
	uint32_t RESERVED6;						/* Reserved Offset 0x64 - 0x67 */
	__RW uint32_t AHB1SMENR;				/* RCC AHB1 Peripheral Clocks Enable in Sleep and Stop Modes Register */
	__RW uint32_t AHB2SMENR;				/* RCC AHB2 Peripheral Clocks Enable in Sleep and Stop Modes Register */
	__RW uint32_t AHB3SMENR;				/* RCC AHB3 Peripheral Clocks Enable in Sleep and Stop Modes Register */
	uint32_t RESERVED7;						/* Reserved Offset 0x74 - 0x77 */
	__RW uint32_t APB1SMENR1;				/* RCC APB1 Peripheral Clocks Enable in Sleep and Stop Modes Register 1 */
	__RW uint32_t APB1SMENR2;				/* RCC APB1 Peripheral Clocks Enable in Sleep and Stop Modes Register 2 */
	__RW uint32_t APB2SMENR;				/* RCC APB2 Peripheral Clocks Enable in Sleep and Stop Modes Register */
	uint32_t RESERVED8;						/* Reserved Offset 0x84 - 0x87 */
	__RW uint32_t CCIPR;					/* RCC Peripherals Independent Clock Configuration Register */
	uint32_t RESERVED9;						/* Reserved Offset 0x8C - 0x8F */
	__RW uint32_t BDCR;						/* RCC Backup Domain Control Register */
	__RW uint32_t CSR;						/* RCC Control/Status Register, [1] - R */
	__R volatile uint32_t CRRCR;			/* RCC Clock Recovery RC Register, [0] - RW*/
	__RW uint32_t CCIPR2;					/* RCC Peripherals Independent Clock Configuration Register */
} RCC_Reg_t;

/*
 * @SYSCFG_PERIPHERAL_REGISTERS
 * Initializer for a SYSCFG peripheral instance, containing all memory-mapped registers
 */
typedef struct SYSCFG_PeripheralRegisters
{
	__RW uint32_t MEMRMP;					/* SYSCFG memory remap register */
	__RW uint32_t CFGR1;					/* SYSCFG configuration register 1 */
	__RW uint32_t EXTICR[4];				/* SYSCFG external interrupt configuration registers */
	__RW uint32_t SCSR;						/* SYSCFG SRAM2 control and status Register */
	__RW uint32_t CFGR2;					/* SYSCFG  configuration register 2*/
	__RW uint32_t SWPR;						/* SYSCFG SRAM2 write protection register */
	__RW uint32_t SKR;						/* SYSCFG SRAM2 key register */
} SYSCFG_Reg_t;

/*
 * @EXTI_PERIPHERAL_REGISTERS
 * Initializer for an EXTI peripheral instance, containing all memory-mapped registers
 */
typedef struct EXTI_PeripheralRegisters
{
	__RW uint32_t IMR1;						/* EXTI Interrupt Mask Register 1 */
	__RW uint32_t EMR1;						/* EXTI Event Mask Register 1 */
	__RW uint32_t RTSR1;					/* EXTI Rising Trigger Selection Register 1 */
	__RW uint32_t FTSR1;					/* EXTI Falling Trigger Selection Register 1 */
	__RW uint32_t SWIER1;					/* EXTI Software Interrupt Event Register 1 */
	__RW uint32_t PR1;						/* EXTI Pending Register 1 */
	uint64_t RESERVED1;						/* Reserved Offset 0x18 - 0x1F */
	__RW uint32_t IMR2;						/* EXTI Interrupt Mask Register 2 */
	__RW uint32_t EMR2;						/* EXTI Event Mask Register 2 */
	__RW uint32_t RTSR2;					/* EXTI Rising Trigger Selection Register 2 */
	__RW uint32_t FTSR2;					/* EXTI Falling Trigger Selection Register 2 */
	__RW uint32_t SWIER2;					/* EXTI Software Interrupt Event Register 2 */
	__RW uint32_t PR2;						/* EXTI Pending Register 2 */
} EXTI_Reg_t;

/*
 * @SPI_PERIPHERAL_REGISTERS
 * Initializer for an SPI peripheral instance, containing all memory-mapped registers
 */
typedef struct SPI_PeripheralRegisters
{
	__RW uint32_t CR1;						/* SPI control register 1 */
	__RW uint32_t CR2;						/* SPI control register 2 */
	__R volatile uint32_t SR;				/* SPI status register, see CRCERR flag */
	__RW uint32_t CRCPR;					/* SPI CRC polynomial register */
	__R volatile uint32_t RXCRCR;			/* SPI Rx CRC register */
	__R volatile uint32_t TXCRCR;			/* SPI Tx CRC register */
} SPIx_Reg_t;

/*
 * @NVIC_IRQ_NUMBERS
 * NVIC possible interrupt request numbers, based on MCU NVIC vector table
 */
typedef enum NVIC_IRQ_Numbers
{
	EXTI0 = 6, EXTI1, EXTI2, EXTI3, EXTI4, EXTI9_5 = 23, EXTI15_10 = 40,
} IRQNum_e;


#define GPIOA 							( (GPIOx_Reg_t *)GPIOA_BASE_ADDR )
#define GPIOB 							( (GPIOx_Reg_t *)GPIOB_BASE_ADDR )
#define GPIOC 							( (GPIOx_Reg_t *)GPIOC_BASE_ADDR )

#define RCC							( (RCC_Reg_t *)RCC_BASE_ADDR )

#define EXTI							( (EXTI_Reg_t *)EXTI_BASE_ADDR )

#define SYSCFG							( (SYSCFG_Reg_t *)SYSCFG_BASE_ADDR )

#define NVIC							( (uint32_t *)NVIC_IPRx_BASE_ADDR )

/* Peripheral Clock Enable/Disable Macro Definitions */

/* Clock Enable for GPIOx */
#define GPIOA_CLK_EN() 				( (RCC->AHB2ENR) |= (1 << 0) )
#define GPIOB_CLK_EN() 				( (RCC->AHB2ENR) |= (1 << 1) )
#define GPIOC_CLK_EN() 				( (RCC->AHB2ENR) |= (1 << 2) )

/* Clock Disable for GPIOx */
#define GPIOA_CLK_DI() 				( (RCC->AHB2ENR) &= ~(1 << 0) )
#define GPIOB_CLK_DI() 				( (RCC->AHB2ENR) &= ~(1 << 1) )
#define GPIOC_CLK_DI() 				( (RCC->AHB2ENR) &= ~(1 << 2) )

/* Clock Enable for I2Cx */
#define I2C1_CLK_EN()					( (RCC->APB1ENR1) |= (1 << 21) )
#define I2C2_CLK_EN()					( (RCC->APB1ENR1) |= (1 << 22) )
#define I2C3_CLK_EN()					( (RCC->APB1ENR1) |= (1 << 23) )

/* Clock Disable for I2Cx */
#define I2C1_CLK_DI()					( (RCC->APB1ENR1) &= ~(1 << 21) )
#define I2C2_CLK_DI()					( (RCC->APB1ENR1) &= ~(1 << 22) )
#define I2C3_CLK_DI()					( (RCC->APB1ENR1) &= ~(1 << 23) )

/* Clock Enable for SPIx */
#define SPI1_CLK_EN()					( (RCC->APB2ENR) |= (1 << 12) )
#define SPI2_CLK_EN()					( (RCC->APB1ENR1) |= (1 << 14) )
#define SPI3_CLK_EN()					( (RCC->APB1ENR1) |= (1 << 15) )

/* Clock Disable for SPIx */
#define SPI1_CLK_DI()					( (RCC->APB2ENR) &= ~(1 << 12) )
#define SPI2_CLK_DI()					( (RCC->APB1ENR1) &= ~(1 << 14) )
#define SPI3_CLK_DI()					( (RCC->APB1ENR1) &= ~(1 << 15) )

/* Clock Enable for U(S)ARTx */
#define USART1_CLK_EN()				( (RCC->APB2ENR) |= (1 << 21) )
#define USART2_CLK_EN()				( (RCC->APB1ENR1) |= (1 << 17) )
#define USART3_CLK_EN()				( (RCC->APB1ENR1) |= (1 << 18) )
#define LPUART_CLK_EN()				( (RCC->APB1ENR2) |= (1 << 0) )

/* Clock Disable for U(S)ARTx */
#define USART1_CLK_DI()				( (RCC->APB2ENR) &= ~(1 << 21) )
#define USART2_CLK_DI()				( (RCC->APB1ENR1) &= ~(1 << 17) )
#define USART3_CLK_DI()				( (RCC->APB1ENR1) &= ~(1 << 18) )
#define LPUART_CLK_DI()				( (RCC->APB1ENR2) &= ~(1 << 0) )

/* Clock Enable for SYSCFG */
#define SYSCFG_CLK_EN()				( (RCC->APB2ENR) |= (1 << 0) )
/* Clock Disable for SYSCFG */
#define SYSCFG_CLK_DI()				( (RCC->APB2ENR) &= ~(1 << 0) )

/* GPIO Register Reset Macro Definitions */
#define GPIOA_REG_RESET()				do { RCC->AHB2RSTR |= (1 << 0); RCC->AHB2RSTR &= ~(1 << 0); } while (0)
#define GPIOB_REG_RESET()				do { RCC->AHB2RSTR |= (1 << 1); RCC->AHB2RSTR &= ~(1 << 1); } while (0)
#define GPIOC_REG_RESET()				do { RCC->AHB2RSTR |= (1 << 2); RCC->AHB2RSTR &= ~(1 << 2); } while (0)

/* Miscellaneous macros */
#define ENABLE							(bool) 1
#define DISABLE						(bool) 0
#define SET							ENABLE
#define RESET							DISABLE
#define CLEAR_ONE_BITMASK				0x1UL
#define CLEAR_TWO_BITMASK				0x3UL
#define CLEAR_FOUR_BITMASK			0xFUL
#define SET_ONE_BITMASK				CLEAR_ONE_BITMASK
#define GENERIC_OFFSET				(uint8_t) 4

uint8_t SYSCFG_EXTICR_helper_func(__R GPIOx_Reg_t *);


#endif /* INC_STM32L433XX_H_ */
