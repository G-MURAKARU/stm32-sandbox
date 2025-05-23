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
#include <stdlib.h>
#include <stdbool.h>


/**********************************START:Processor Specific Details **********************************/

/*
 * ARM Cortex-Mx Processor Nested Vector Interrupt Controller (NVIC) Register Addresses, from user guide
 */

/* NVIC Interrupt Set-enable Register memory locations */
// 7 available but using 4 because 64 < IRQ positions < 96
#define NVIC_ISERx_BASE_ADDR				0xE000E100UL
#define NVIC_ISER0							( (__RW uint32_t *)(NVIC_ISERx_BASE_ADDR + 0x0UL) )
#define NVIC_ISER1							( (__RW uint32_t *)(NVIC_ISERx_BASE_ADDR + 0x4UL) )
#define NVIC_ISER2							( (__RW uint32_t *)(NVIC_ISERx_BASE_ADDR + 0x8UL) )
#define NVIC_ISER3							( (__RW uint32_t *)(NVIC_ISERx_BASE_ADDR + 0xCUL) )

/* NVIC Interrupt Clear-enable Register memory locations */
#define NVIC_ICERx_BASE_ADDR				0xE000E180UL
#define NVIC_ICER0							( (__RW uint32_t *)(NVIC_ICERx_BASE_ADDR + 0x0UL) )
#define NVIC_ICER1							( (__RW uint32_t *)(NVIC_ICERx_BASE_ADDR + 0x4UL) )
#define NVIC_ICER2							( (__RW uint32_t *)(NVIC_ICERx_BASE_ADDR + 0x8UL) )
#define NVIC_ICER3							( (__RW uint32_t *)(NVIC_ICERx_BASE_ADDR + 0xCUL) )

/* NVIC Interrupt Priority Register memory locations */
#define NVIC_IPRx_BASE_ADDR				0xE000E400UL
#define NO_PR_BITS_IMPLEMENTED			(uint8_t) 4				/* Number of implemented/usable IPR IRQ priority bits */


/* Macros to be used with registers denoting contextual read/write access */
#define __RW								volatile				/* software readable and writable */
#define __R								const					/* software read only, will not change */
#define __RH								volatile const			/* software read only, may be written by hardware */
#define __W								volatile				/* software write only */


/* Base Addresses of the MCU Internal Memories */
#define SRAM1_BASE_ADDR 					0x20000000UL
#define SRAM2_BASE_ADDR 					0x2000C000UL

#define SRAM_BASE_ADDR 					SRAM1_BASE_ADDR

#define ROM_BASE_ADDR 					0x1FFF0000UL


/* Base Addresses of the MCU Bus Domains */
#define PERIPHERAL_BASE_ADDR 				0x40000000UL

#define APB1_PERIPHERAL_OFFSET			0x0UL
#define APB1_BASE_ADDR					(PERIPHERAL_BASE_ADDR + APB1_PERIPHERAL_OFFSET)

#define APB2_PERIPHERAL_OFFSET			0x10000UL
#define APB2_BASE_ADDR					(PERIPHERAL_BASE_ADDR + APB2_PERIPHERAL_OFFSET)

#define AHB1_PERIPHERAL_OFFSET			0x20000UL
#define AHB1_BASE_ADDR					(PERIPHERAL_BASE_ADDR + AHB1_PERIPHERAL_OFFSET)

#define AHB2_PERIPHERAL_OFFSET			0x8000000UL
#define AHB2_BASE_ADDR					(PERIPHERAL_BASE_ADDR + AHB2_PERIPHERAL_OFFSET)

/* Base Addresses of Peripherals on each Bus Domain, AHBx and APBx */

/* Peripherals on AHB1 */
#define DMA1_AHB1_OFFSET 					0x0UL
#define DMA1_BASE_ADDR					(AHB1_BASE_ADDR + DMA1_AHB1_OFFSET)

#define DMA2_AHB1_OFFSET					0x400UL
#define DMA2_BASE_ADDR					(AHB1_BASE_ADDR + DMA2_AHB1_OFFSET)

#define RCC_AHB1_OFFSET					0x1000UL
#define RCC_BASE_ADDR						(AHB1_BASE_ADDR + RCC_AHB1_OFFSET)

#define FLASH_REG_AHB1_OFFSET				0x2000UL
#define FLASH_REG_BASE_ADDR				(AHB1_BASE_ADDR + FLASH_REG_AHB1_OFFSET)

#define CRC_AHB1_OFFSET					0x3000UL
#define CRC_BASE_ADDR						(AHB1_BASE_ADDR + CRC_AHB1_OFFSET)

#define TSC_AHB1_OFFSET					0x4000UL
#define TSC_BASE_ADDR						(AHB1_BASE_ADDR + TSC_AHB1_OFFSET)

/* Peripherals on AHB2 */
#define GPIOA_AHB2_OFFSET					0x0UL
#define GPIOA_BASE_ADDR					(AHB2_BASE_ADDR + GPIOA_AHB2_OFFSET)

#define GPIOB_AHB2_OFFSET					0x400UL
#define GPIOB_BASE_ADDR					(AHB2_BASE_ADDR + GPIOB_AHB2_OFFSET)

#define GPIOC_AHB2_OFFSET					0x800UL
#define GPIOC_BASE_ADDR					(AHB2_BASE_ADDR + GPIOC_AHB2_OFFSET)

#define ADC_AHB2_OFFSET					0x8040000UL
#define ADC_BASE_ADDR						(AHB2_BASE_ADDR + ADC_AHB2_OFFSET)

#define RNG_AHB2_OFFSET					0x8060800UL
#define RNG_BASE_ADDR						(AHB2_BASE_ADDR + RNG_AHB2_OFFSET)

/* Peripherals on APB1 */
#define TIM2_APB1_OFFSET					0x0UL
#define TIM2_BASE_ADDR					(APB1_BASE_ADDR + TIM2_APB1_OFFSET)

#define TIM6_APB1_OFFSET					0x1000UL
#define TIM6_BASE_ADDR					(APB1_BASE_ADDR + TIM6_APB1_OFFSET)

#define TIM7_APB1_OFFSET					0x1400UL
#define TIM7_BASE_ADDR					(APB1_BASE_ADDR + TIM7_APB1_OFFSET)

#define LCD_APB1_OFFSET					0x2400UL
#define LCD_BASE_ADDR						(APB1_BASE_ADDR + LCD_APB1_OFFSET)

#define RTC_APB1_OFFSET					0x2800UL
#define RTC_BASE_ADDR						(APB1_BASE_ADDR + RTC_APB1_OFFSET)

#define WWDG_APB1_OFFSET					0x2C00UL
#define WWDG_BASE_ADDR					(APB1_BASE_ADDR + WWDG_APB1_OFFSET)

#define IWDG_APB1_OFFSET					0x3000UL
#define IWDG_BASE_ADDR					(APB1_BASE_ADDR + IWDG_APB1_OFFSET)

#define SPI2_APB1_OFFSET					0x3800UL
#define SPI2_BASE_ADDR					(APB1_BASE_ADDR + SPI2_APB1_OFFSET)

#define SPI3_APB1_OFFSET					0x3C00UL
#define SPI3_BASE_ADDR					(APB1_BASE_ADDR + SPI3_APB1_OFFSET)

#define USART2_APB1_OFFSET				0x4400UL
#define USART2_BASE_ADDR					(APB1_BASE_ADDR + USART2_APB1_OFFSET)

#define USART3_APB1_OFFSET				0x4800UL
#define USART3_BASE_ADDR					(APB1_BASE_ADDR + USART3_APB1_OFFSET)

#define I2C1_APB1_OFFSET					0x5400UL
#define I2C1_BASE_ADDR					(APB1_BASE_ADDR + I2C1_APB1_OFFSET)

#define I2C2_APB1_OFFSET					0x5800UL
#define I2C2_BASE_ADDR					(APB1_BASE_ADDR + I2C2_APB1_OFFSET)

#define I2C3_APB1_OFFSET					0x5C00UL
#define I2C3_BASE_ADDR					(APB1_BASE_ADDR + I2C3_APB1_OFFSET)

#define CRS_APB1_OFFSET					0x6000UL
#define CRS_BASE_ADDR						(APB1_BASE_ADDR + CRS_APB1_OFFSET)

#define CAN1_APB1_OFFSET					0x6400UL
#define CAN1_BASE_ADDR					(APB1_BASE_ADDR + CAN1_APB1_OFFSET)

#define USB_FS_APB1_OFFSET				0x6800UL
#define USB_FS_BASE_ADDR					(APB1_BASE_ADDR + USB_FS_APB1_OFFSET)

#define USB_SRAM_APB1_OFFSET				0x6C00UL
#define USB_SRAM_BASE_ADDR				(APB1_BASE_ADDR + USB_SRAM_APB1_OFFSET)

#define PWR_REG_APB1_OFFSET				0x7000UL
#define PWR_REG_BASE_ADDR					(APB1_BASE_ADDR + PWR_REG_APB1_OFFSET)

#define DAC1_APB1_OFFSET					0x7400UL
#define DAC1_BASE_ADDR					(APB1_BASE_ADDR + DAC1_APB1_OFFSET)

#define OPAMP_APB1_OFFSET					0x7800UL
#define OPAMP_BASE_ADDR					(APB1_BASE_ADDR + OPAMP_APB1_OFFSET)

#define LPTIM1_APB1_OFFSET				0x7C00UL
#define LPTIM1_BASE_ADDR					(APB1_BASE_ADDR + LPTIM1_APB1_OFFSET)

#define LPUART_APB1_OFFSET				0x8000UL
#define LPUART_BASE_ADDR					(APB1_BASE_ADDR + LPUART_APB1_OFFSET)

#define SWPMI1_APB1_OFFSET				0x8800UL
#define SWPMI1_BASE_ADDR					(APB1_BASE_ADDR + SWPMI1_APB1_OFFSET)

#define LPTIM2_APB1_OFFSET				0x9400UL
#define LPTIM2_BASE_ADDR					(APB1_BASE_ADDR + LPTIM2_APB1_OFFSET)

/* Peripherals on APB2 */
#define SYSCFG_APB2_OFFSET				0x0UL
#define SYSCFG_BASE_ADDR					(APB2_BASE_ADDR + SYSCFG_APB2_OFFSET)

#define VREFBUF_APB2_OFFSET				0x30UL
#define VREFBUF_BASE_ADDR					(APB2_BASE_ADDR + VREFBUF_APB2_OFFSET)

#define COMP_APB2_OFFSET					0x200UL
#define COMP_BASE_ADDR					(APB2_BASE_ADDR + COMP_APB2_OFFSET)

#define EXTI_APB2_OFFSET					0x400UL
#define EXTI_BASE_ADDR					(APB2_BASE_ADDR + EXTI_APB2_OFFSET)

#define FIREWALL_APB2_OFFSET				0x1C00UL
#define FIREWALL_BASE_ADDR				(APB2_BASE_ADDR + FIREWALL_APB2_OFFSET)

#define SDMMC1_APB2_OFFSET				0x2800UL
#define SDMMC1_BASE_ADDR					(APB2_BASE_ADDR + SDMMC1_APB2_OFFSET)

#define TIM1_APB2_OFFSET					0x2C00UL
#define TIM1_BASE_ADDR					(APB2_BASE_ADDR + TIM1_APB2_OFFSET)

#define SPI1_APB2_OFFSET					0x3000UL
#define SPI1_BASE_ADDR					(APB2_BASE_ADDR + SPI1_APB2_OFFSET)

#define USART1_APB2_OFFSET				0x3800UL
#define USART1_BASE_ADDR					(APB2_BASE_ADDR + USART1_APB2_OFFSET)

#define TIM15_APB2_OFFSET					0x4000UL
#define TIM15_BASE_ADDR					(APB2_BASE_ADDR + TIM15_APB2_OFFSET)

#define TIM16_APB2_OFFSET					0x4400UL
#define TIM16_BASE_ADDR					(APB2_BASE_ADDR + TIM16_APB2_OFFSET)

#define SAI1_APB2_OFFSET					0x5400UL
#define SAI1_BASE_ADDR					(APB2_BASE_ADDR + SAI1_APB2_OFFSET)

/* Miscellaneous macros */
#define ENABLE								(bool) 1
#define DISABLE							(bool) 0
#define SET								ENABLE
#define RESET								DISABLE
#define CLEAR_ONE_BITMASK					0x1UL
#define CLEAR_TWO_BITMASK					0x3UL
#define CLEAR_FOUR_BITMASK				0xFUL
#define SET_ONE_BITMASK					CLEAR_ONE_BITMASK
#define SET_TWO_BITMASK					CLEAR_TWO_BITMASK
#define SET_FOUR_BITMASK					CLEAR_FOUR_BITMASK
#define CHECK_ONE_BITMASK					CLEAR_ONE_BITMASK
#define CHECK_TWO_BITMASK					CLEAR_TWO_BITMASK
#define SINGLE_BITMASK					CLEAR_ONE_BITMASK
#define __weak								__attribute__((weak))


/********************************** PERIPHERAL REGISTER STRUCTURE DEFINITIONS **********************************/

/* GPIOx_Reg_t was here */

/*
 * @RCC_PERIPHERAL_REGISTERS
 * Reset & Clock Control; Initializer for an RCC peripheral instance, containing all memory-mapped registers
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
	__RH uint32_t CIFR;						/* RCC Clock Interrupt Flag Register */
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
	__RH uint32_t CRRCR;					/* RCC Clock Recovery RC Register, [0] - RW*/
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
	__RW uint32_t CFGR2;					/* SYSCFG  configuration register 2 */
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

/* SPIx_Reg_t was here */

/*
 * @RCC_AHB2_ENABLE_FLAGS
 * RCC AHB2 peripheral clock enable flag positions
 */
typedef enum RCC_AHB2_EnableRegister
{
	RCC_GPIOAEN, RCC_GPIOBEN, RCC_GPIOCEN, RCC_ADCEN = 13, RCC_RNGEN = 18,
} RCC_AHB2ENR_e;

/*
 * @RCC_AHB2_RESET_FLAGS
 * RCC AHB2 peripheral reset flag positions
 */
typedef enum RCC_AHB2_ResetRegister
{
	RCC_GPIOARST, RCC_GPIOBRST, RCC_GPIOCRST, RCC_ADCRST = 13, RCC_RNGRST = 18,
} RCC_AHB2RSTR_e;

/*
 * @RCC_APB1_R1_ENABLE_FLAGS
 * RCC APB1 R1 peripheral enable flag positions
 */
typedef enum RCC_APB1_R1_EnableRegister
{
	RCC_TIM2EN, RCC_TIM3EN, RCC_TIM6EN = 4, RCC_TIM7EN, RCC_LCDEN = 9, RCC_RTCAPBEN, RCC_WWDGEN, RCC_SPI2EN = 14,
	RCC_SPI3EN = 15, RCC_USART2EN = 17, RCC_USART3EN, RCC_I2C1EN = 21, RCC_I2C2EN, RCC_I2C3EN, RCC_CRSEN,
	RCC_CAN1EN, RCC_USBFSEN, RCC_PWREN = 28, RCC_DAC1EN, RCC_OPAMPEN, RCC_LPTIM1EN,
} RCC_APB1ENR1_e;

/*
 * @RCC_APB1_R1_RESET_FLAGS
 * RCC APB1 R1 peripheral reset flag positions
 */
typedef enum RCC_APB1_R1_ResetRegister
{
	RCC_TIM2RST, RCC_TIM3RST, RCC_TIM6RST = 4, RCC_TIM7RST, RCC_LCDRST = 9, RCC_RTCAPBRST, RCC_WWDGRST,
	RCC_SPI2RST = 14, RCC_SPI3RST = 15, RCC_USART2RST = 17, RCC_USART3RST, RCC_I2C1RST = 21, RCC_I2C2RST,
	RCC_I2C3RST, RCC_CRSRST, RCC_CAN1RST, RCC_USBFSRST, RCC_PWRRST = 28, RCC_DAC1RST, RCC_OPAMPRST, RCC_LPTIM1RST,
} RCC_APB1RSTR1_e;

/*
 * @RCC_APB1_R2_ENABLE_FLAGS
 * RCC APB1 R2 peripheral enable flag positions
 */
typedef enum RCC_APB1_R2_EnableRegister
{
	RCC_LPUART1EN, RCC_SWPMI1EN = 2, RCC_LPTIM2EN = 5,
} RCC_APB1ENR2_e;

/*
 * @RCC_APB1_R2_RESET_FLAGS
 * RCC APB1 R2 peripheral reset flag positions
 */
typedef enum RCC_APB1_R2_ResetRegister
{
	RCC_LPUART1RST, RCC_SWPMI1RST = 2, RCC_LPTIM2RST = 5,
} RCC_APB1RSTR2_e;

/*
 * @RCC_APB2_ENABLE_FLAGS
 * RCC APB2 peripheral enable flag positions
 */
typedef enum RCC_APB2_EnableRegister
{
	RCC_SYSCFGEN, RCC_FWEN = 7, RCC_SDMMC1EN = 10, RCC_TIM1EN, RCC_SPI1EN, RCC_USART1EN = 14, RCC_TIM15EN = 16,
	RCC_TIM16EN, RCC_SAIEN = 21, RCC_DFSDM1EN = 24,
} RCC_APB2ENR_e;

/*
 * @RCC_APB2_RESET_FLAGS
 * RCC APB2 peripheral reset flag positions
 */
typedef enum RCC_APB2_ResetRegister
{
	RCC_SYSCFGRST, RCC_FWRST = 7, RCC_SDMMC1RST = 10, RCC_TIM1RST, RCC_SPI1RST, RCC_USART1RST = 14,
	RCC_TIM15RST = 16, RCC_TIM16RST, RCC_SAIRST = 21, RCC_DFSDM1RST = 24,
} RCC_APB2RSTR_e;

/*
 * @NVIC_IRQ_NUMBERS
 * NVIC possible interrupt request numbers, based on MCU NVIC vector table
 */
typedef enum NVIC_IRQ_Numbers
{
	IRQ_EXTI0 = 6, IRQ_EXTI1, IRQ_EXTI2, IRQ_EXTI3, IRQ_EXTI4, IRQ_EXTI9_5 = 23,
	IRQ_EXTI15_10 = 40, IRQ_SPI1 = 35, IRQ_SPI2 = 36, IRQ_SPI3 = 51,
} NVIC_IRQNum_e;


/* GPIO base register pointers were here */

/* Pointers to globally-used (peripheral) registers */
#define RCC								( (__RW RCC_Reg_t *const)RCC_BASE_ADDR )

#define EXTI								( (__RW EXTI_Reg_t *const)EXTI_BASE_ADDR )

#define SYSCFG								( (__RW SYSCFG_Reg_t *const)SYSCFG_BASE_ADDR )

#define NVIC_IPRx							( (__RW uint32_t *const)NVIC_IPRx_BASE_ADDR )

/* SPI base register pointers were here */

/* Peripheral Clock Enable/Disable Macro Definitions */

/* GPIO clock enable/disable macros were here */

/* I2C clock enable/disable macros were here */

/* SPI clock enable/disable macros were here */

/* Clock Enable for U(S)ARTx */
#define USART1_CLK_EN()					( (RCC->APB2ENR) |= (SET_ONE_BITMASK << RCC_USART1EN) )
#define USART2_CLK_EN()					( (RCC->APB1ENR1) |= (SET_ONE_BITMASK << RCC_USART2EN) )
#define USART3_CLK_EN()					( (RCC->APB1ENR1) |= (SET_ONE_BITMASK << RCC_USART3EN) )
#define LPUART_CLK_EN()					( (RCC->APB1ENR2) |= (SET_ONE_BITMASK << RCC_LPUART1EN) )

/* Clock Disable for U(S)ARTx */
#define USART1_CLK_DI()					( (RCC->APB2ENR) &= ~(CLEAR_ONE_BITMASK << RCC_USART1EN) )
#define USART2_CLK_DI()					( (RCC->APB1ENR1) &= ~(CLEAR_ONE_BITMASK << RCC_USART2EN) )
#define USART3_CLK_DI()					( (RCC->APB1ENR1) &= ~(CLEAR_ONE_BITMASK << RCC_USART3EN) )
#define LPUART_CLK_DI()					( (RCC->APB1ENR2) &= ~(CLEAR_ONE_BITMASK << RCC_LPUART1EN) )

/* Clock Enable for SYSCFG */
#define SYSCFG_CLK_EN()					( (RCC->APB2ENR) |= (SET_ONE_BITMASK << RCC_SYSCFGEN) )
/* Clock Disable for SYSCFG */
#define SYSCFG_CLK_DI()					( (RCC->APB2ENR) &= ~(CLEAR_ONE_BITMASK << RCC_SYSCFGEN) )

/* GPIO register reset macros were here */

/* SPI register reset macros were here */


uint8_t MCU_GetFlagStatus(uint32_t, uint8_t, uint8_t);
void NVIC_IRQPositionConfig(uint8_t, bool);
void NVIC_IRQPriorityConfig(uint8_t, uint16_t);


#endif /* INC_STM32L433XX_H_ */
