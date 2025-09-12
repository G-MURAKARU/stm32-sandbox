/*
 * x_irq.h
 *
 *  Created on: Sep 12, 2025
 *      Author: MURAKARU
 */

#ifndef INC_X_NVIC_H_
#define INC_X_NVIC_H_


#include "x_gpio_defs.h"

/*
 * ARM Cortex-Mx Processor Nested Vector Interrupt Controller (NVIC) Register Addresses, from user guide
 */

/* NVIC Interrupt Set-Enable Register memory locations */
// 7 available but using 4 because 64 < IRQ positions < 96
#define NVIC_ISERx_BASE_ADDR				0xE000E100UL
#define NVIC_ISER0							( (__RW uint32_t *const)(NVIC_ISERx_BASE_ADDR + 0x0UL) )
#define NVIC_ISER1							( (__RW uint32_t *const)(NVIC_ISERx_BASE_ADDR + 0x4UL) )
#define NVIC_ISER2							( (__RW uint32_t *const)(NVIC_ISERx_BASE_ADDR + 0x8UL) )
#define NVIC_ISER3							( (__RW uint32_t *const)(NVIC_ISERx_BASE_ADDR + 0xCUL) )

/* NVIC Interrupt Clear-Enable Register memory locations */
#define NVIC_ICERx_BASE_ADDR				0xE000E180UL
#define NVIC_ICER0							( (__RW uint32_t *const)(NVIC_ICERx_BASE_ADDR + 0x0UL) )
#define NVIC_ICER1							( (__RW uint32_t *const)(NVIC_ICERx_BASE_ADDR + 0x4UL) )
#define NVIC_ICER2							( (__RW uint32_t *const)(NVIC_ICERx_BASE_ADDR + 0x8UL) )
#define NVIC_ICER3							( (__RW uint32_t *const)(NVIC_ICERx_BASE_ADDR + 0xCUL) )

/* NVIC Interrupt Priority Register memory locations */
#define NVIC_IPRx_BASE_ADDR				0xE000E400UL

/* Nester Vector Interrupt Controller */
#define NVIC_IPRx							( (__RW uint32_t *const)NVIC_IPRx_BASE_ADDR )

#define NUM_PR_BITS_IMPLEMENTED			(uint8_t) 4				/* Number of implemented/usable IPR IRQ priority bits */


/*
 * @NVIC_IRQ_NUMBERS
 * NVIC possible interrupt request numbers, based on MCU NVIC vector table
 */
typedef enum NVIC_IRQ_Numbers
{
	IRQ_EXTI0 = 6, IRQ_EXTI1, IRQ_EXTI2, IRQ_EXTI3, IRQ_EXTI4, IRQ_EXTI9_5 = 23,
	IRQ_SPI1 = 35, IRQ_SPI2 = 36, IRQ_EXTI15_10 = 40, IRQ_SPI3 = 51,
} NVIC_IRQNum_e;

/* Function prototypes */

void EnableIRQ(void *const peripheral, uint8_t id, uint16_t priority);
void DisableIRQ(void *const peripheral, uint8_t id);

#endif /* INC_X_IRQ_H_ */
