/*
 * x_nvic.c
 *
 *  Created on: Sep 12, 2025
 *      Author: MURAKARU
 */

#include "x_nvic.h"
#include "x_gpio_defs.h"

static int8_t NVIC_ResolveIRQ(void *const, uint8_t);
static __always_inline void NVIC_EnableIRQ(__RW uint32_t *const, uint8_t);
static __always_inline void NVIC_DisableIRQ(__RW uint32_t *const, uint8_t);

extern int8_t GPIO_GetIRQn(uint8_t);

/*

 * @fn							- MCU_IRQPositionConfig
 *
 * @brief						- this function configures the IRQ position for an interrupt event - enables or disables interrupt actuation
 *
 * @param[IRQPosition]			- (pre-)configured IRQ position on NVIC for the given interrupt event
 * @param[en_di]				- action to take: ENABLE(1) or DISABLE(0)
 *
 * @return						- none
 *
 * @Note						- none

 */
static void NVIC_ConfigIRQn(uint8_t irq_position, bool en_di)
{
	/* Each bit in an ISER register represents an IRQ position,
	 * Check data sheet for number of available/pre-configured IRQ positions */

	/* 1. Find the memory location for the desired IRQ position in the Interrupt Set-Enable Register */

	// Find which ISER the IRQ position belongs to
	const uint8_t iser_reg_x = irq_position / 32;

	// Find which bit in the derived ISER corresponds to desired IRQ position
	const uint8_t iser_irq_offset = irq_position % 32;

	// Logic to manipulate the flag of the desired IRQ position (enables/disables the interrupt event)
	// set = write 1 to ISER, clear = write 1 to ICER
	switch (iser_reg_x)
	{
		case 0:
//			(en_di) ? ( *NVIC_ISER0 |= (SET_ONE_BITMASK << iser_irq_offset) ) : ( *NVIC_ICER0 |= (SET_ONE_BITMASK << iser_irq_offset) );
			if (en_di) NVIC_EnableIRQ(NVIC_ISER0, iser_irq_offset);
			else NVIC_DisableIRQ(NVIC_ICER0, iser_irq_offset);
			break;

		case 1:
//			(en_di) ? ( *NVIC_ISER1 |= (SET_ONE_BITMASK << iser_irq_offset) ) : ( *NVIC_ICER1 |= (SET_ONE_BITMASK << iser_irq_offset) );
			if (en_di) NVIC_EnableIRQ(NVIC_ISER1, iser_irq_offset);
			else NVIC_DisableIRQ(NVIC_ICER1, iser_irq_offset);
			break;

		case 2:
//			(en_di) ? ( *NVIC_ISER2 |= (SET_ONE_BITMASK << iser_irq_offset) ) : ( *NVIC_ICER2 |= (SET_ONE_BITMASK << iser_irq_offset) );
			if (en_di) NVIC_EnableIRQ(NVIC_ISER2, iser_irq_offset);
			else NVIC_DisableIRQ(NVIC_ICER2, iser_irq_offset);
			break;

		case 3:
//			(en_di) ? ( *NVIC_ISER3 |= (SET_ONE_BITMASK << iser_irq_offset) ) : ( *NVIC_ICER3 |= (SET_ONE_BITMASK << iser_irq_offset) );
			if (en_di) NVIC_EnableIRQ(NVIC_ISER3, iser_irq_offset);
			else NVIC_DisableIRQ(NVIC_ICER3, iser_irq_offset);
			break;

		default:
			break;
	}
}

/*

 * @fn							- MCU_IRQPriorityConfig
 *
 * @brief						- this function configures the IRQ priority for a given interrupt event (IRQ position) - lower number = higher priority
 *
 * @param[IRQPosition]			- (pre-)configured IRQ position on NVIC for the given interrupt event
 * @param[IRQPriority]			- priority to assign to the given interrupt event, depending on priority grouping
 *
 * @return						- none
 *
 * @Note						- MCU allows configuration of only the 4 MSBs of an interrupt's priority

 */
static void NVIC_ConfigIRQ_Priority(uint8_t irq_position, uint16_t irq_priority)
{
	/* Each IRQ position's priority setting takes 1 byte in Interrupt Priority Register (IPR) - 60 IPRs */

	/* 1. Find the memory location for the desired IRQ position's priority in IPR */

	// Find which IPR the IRQ position belongs to
	const uint8_t priority_reg_x = irq_position / 4;

	// Find which byte in the derived IPR corresponds to desired IRQ position
	const uint8_t priority_irq_offset = irq_position % 4;

	// Define a shift offset, considering each priority takes 1 byte, and only the 4 MSBs are implemented
	const uint8_t shift_amount = (uint8_t)(priority_irq_offset * 8) + NUM_PR_BITS_IMPLEMENTED;

	/* Remember underlying working of pointer arithmetic, NVIC is already a pointer to 4-byte data
	 * hence no need multiply by 4 */
	*(NVIC_IPRx + priority_reg_x) |= (irq_priority << shift_amount);
}


void EnableIRQ(void *const peripheral, uint8_t id, uint16_t priority)
{
	int8_t position = NVIC_ResolveIRQ(peripheral, id);

	if (position != -1)
	{
		NVIC_ConfigIRQ_Priority((uint8_t)position, priority);
		NVIC_ConfigIRQn((uint8_t)position, ENABLE);
	}
}


void DisableIRQ(void *const peripheral, uint8_t id)
{
	int8_t position = NVIC_ResolveIRQ(peripheral, id);

	if (position != -1) NVIC_ConfigIRQn((uint8_t)position, DISABLE);
}


static int8_t NVIC_ResolveIRQ(void *const peripheral, uint8_t id)
{
	if ( (peripheral == GPIOA) || (peripheral == GPIOB) || (peripheral == GPIOC) ) return GPIO_GetIRQn(id);
	else return (-1);
}

static __always_inline void NVIC_EnableIRQ(__RW uint32_t *const nvic_iser, uint8_t offset)
{
	*(nvic_iser) |= (SET_ONE_BITMASK << offset);
}

static __always_inline void NVIC_DisableIRQ(__RW uint32_t *const nvic_icer, uint8_t offset)
{
	*(nvic_icer) |= (SET_ONE_BITMASK << offset);
}
