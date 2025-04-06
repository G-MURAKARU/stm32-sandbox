/*
 * main.h
 *
 *  Created on: Mar 27, 2025
 *      Author: OMEN 16
 */

#ifndef MAIN_H_
#define MAIN_H_

#include <stdint.h>
#include <stdio.h>

/* Register memory map definitions */

/* Base address for all peripheral registers */
#define PERIPH_BASE_ADDR 0x40000000UL

/* Base address for AHB1 */
#define AHB1_PERIPH_OFFSET 0x20000UL
#define AHB1_BASE_ADDR (PERIPH_BASE_ADDR + AHB1_PERIPH_OFFSET)

/* Base address for Reset and Clock Control (RCC), found on AHB1 */
#define RCC_AHB1_OFFSET 0X1000UL
#define RCC_BASE_ADDR (AHB1_BASE_ADDR + RCC_AHB1_OFFSET)

/* Base address for AHB2 Enable Register, found on RCC, which supplies all GPIO Ports */
#define AHB2ENR_RCC_OFFSET 0X4CUL
#define AHB2ENR_BASE_ADDR (RCC_BASE_ADDR + AHB2ENR_RCC_OFFSET)

/* Base address for AHB2 */
#define AHB2_PERIPH_OFFSET 0x8000000UL
#define AHB2_BASE_ADDR (PERIPH_BASE_ADDR + AHB2_PERIPH_OFFSET)

/* Base address for desired GPIO Ports, in this case GPIOA, found on AHB2 */
#define GPIOA_AHB2_OFFSET 0x0UL
#define GPIOA_BASE_ADDR (AHB2_BASE_ADDR + GPIOA_AHB2_OFFSET)

/* Mode Register Offset on all GPIOs */
#define GPIOx_MODER_OFFSET 0x0UL

/* Base address for GPIOA mode register */
#define GPIOA_MODER_ADDR (GPIOA_BASE_ADDR + GPIOx_MODER_OFFSET)

/* Input Register Offset on all GPIOs */
#define GPIOx_IDR_OFFSET 0X10UL

/* Base address for GPIOA input register */
#define GPIOA_IDR_ADDR (GPIOA_BASE_ADDR + GPIOx_IDR_OFFSET)

/* Output Register Offset on all GPIOs */
#define GPIOx_ODR_OFFSET 0X14UL

/* Base address for GPIOA output register */
#define GPIOA_ODR_ADDR (GPIOA_BASE_ADDR + GPIOx_ODR_OFFSET)

/* Pull-Up/Pull-Down Register Offset on all GPIOs */
#define GPIOx_PUPDR_OFFSET 0X0CUL

/* Base address for GPIOA pull-up/pull-down register*/
#define GPIOA_PUPDR_ADDR (GPIOA_BASE_ADDR + GPIOx_PUPDR_OFFSET)

/* Structure definitions for Peripheral Registers */

/* Advanced High-Speed Bus 2 Enable Register layout structure */
typedef struct AHB2EnableRegister
{
	uint32_t gpioa_en : 1;
	uint32_t gpiob_en : 1;
	uint32_t gpioc_en : 1;
	uint32_t gpiod_en : 1;
	uint32_t gpioe_en : 1;
	uint32_t reserved_1 : 2;
	uint32_t gpioh_en : 1;
	uint32_t reserved_2 : 5;
	uint32_t adc_en : 1;
	uint32_t reserved_3 : 2;
	uint32_t aes_en : 1;
	uint32_t reserved_4 : 1;
	uint32_t rng_en : 1;
	uint32_t reserved_5 : 13;
} RCC_AHB2ENR_t;

/* GPIO Mode Register layout structure */
typedef struct GPIOModeRegister
{
	uint32_t pin_0 : 2;
	uint32_t pin_1 : 2;
	uint32_t pin_2 : 2;
	uint32_t pin_3 : 2;
	uint32_t pin_4 : 2;
	uint32_t pin_5 : 2;
	uint32_t pin_6 : 2;
	uint32_t pin_7 : 2;
	uint32_t pin_8 : 2;
	uint32_t pin_9 : 2;
	uint32_t pin_10 : 2;
	uint32_t pin_11 : 2;
	uint32_t pin_12 : 2;
	uint32_t pin_13 : 2;
	uint32_t pin_14 : 2;
	uint32_t pin_15 : 2;
} GPIOx_MODER_t;

/* GPIO Output Register layout structure */
typedef struct GPIOOutputDataRegister
{
	uint32_t pin_0 : 1;
	uint32_t pin_1 : 1;
	uint32_t pin_2 : 1;
	uint32_t pin_3 : 1;
	uint32_t pin_4 : 1;
	uint32_t pin_5 : 1;
	uint32_t pin_6 : 1;
	uint32_t pin_7 : 1;
	uint32_t pin_8 : 1;
	uint32_t pin_9 : 1;
	uint32_t pin_10 : 1;
	uint32_t pin_11 : 1;
	uint32_t pin_12 : 1;
	uint32_t pin_13 : 1;
	uint32_t pin_14 : 1;
	uint32_t pin_15 : 1;
	uint32_t reserved : 16;
} GPIOx_ODR_t;

/* GPIO Input Register layout structure */
typedef struct GPIOInputDataRegister
{
	uint32_t pin_0 : 1;
	uint32_t pin_1 : 1;
	uint32_t pin_2 : 1;
	uint32_t pin_3 : 1;
	uint32_t pin_4 : 1;
	uint32_t pin_5 : 1;
	uint32_t pin_6 : 1;
	uint32_t pin_7 : 1;
	uint32_t pin_8 : 1;
	uint32_t pin_9 : 1;
	uint32_t pin_10 : 1;
	uint32_t pin_11 : 1;
	uint32_t pin_12 : 1;
	uint32_t pin_13 : 1;
	uint32_t pin_14 : 1;
	uint32_t pin_15 : 1;
	uint32_t reserved : 16;
} GPIOx_IDR_t;

/* GPIO Pull-Up/Pull-Down Register layout structure */
typedef struct GPIOPullUpDownRegister
{
	uint32_t pin_0 : 2;
	uint32_t pin_1 : 2;
	uint32_t pin_2 : 2;
	uint32_t pin_3 : 2;
	uint32_t pin_4 : 2;
	uint32_t pin_5 : 2;
	uint32_t pin_6 : 2;
	uint32_t pin_7 : 2;
	uint32_t pin_8 : 2;
	uint32_t pin_9 : 2;
	uint32_t pin_10 : 2;
	uint32_t pin_11 : 2;
	uint32_t pin_12 : 2;
	uint32_t pin_13 : 2;
	uint32_t pin_14 : 2;
	uint32_t pin_15 : 2;
} GPIOx_PUPDR_t;

/* some function definitions */
extern void initialise_monitor_handles(void);
void delay(void);

#endif /* MAIN_H_ */
