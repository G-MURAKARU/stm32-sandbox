/*
 * stm32l433xx.c
 *
 *  Created on: Apr 5, 2025
 *      Author: MURAKARU
 */


#include "stm32l433xx.h"


/*

 * @fn							- MCU_GetFlagStatus
 *
 * @brief						- this function gets the status of a register flag
 *
 * @param[register_contents]	- the contents of the register containing the flag whose status is needed
 * @param[flag_length]			- the number of bits that make up the register flag
 *
 * @return						- flag status, either SET(1) or RESET(0)
 *
 * @Note						- none

 */
uint8_t MCU_GetFlagStatus(uint32_t register_contents, uint8_t flag_length, uint8_t offset)
{
	if (flag_length == 1) return ( (register_contents >> offset) & CHECK_ONE_BITMASK );
	else if (flag_length == 2) return ( (register_contents >> offset) & CHECK_TWO_BITMASK );
	return (-1);
}

/* Interrupt Configuration/Handling */

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
void NVIC_IRQPositionConfig(uint8_t IRQPosition, bool en_di)
{
	/* Each bit in an ISER register represents an IRQ position,
	 * Check data sheet for number of available/pre-configured IRQ positions */

	/* 1. Find the memory location for the desired IRQ position in the Interrupt Set-Enable Register */

	// Find which ISER the IRQ position belongs to
	const uint8_t iser_reg_x = IRQPosition / 32;

	// Find which bit in the derived ISER corresponds to desired IRQ position
	const uint8_t iser_irq_offset = IRQPosition % 32;

	// Logic to manipulate the flag of the desired IRQ position (enables/disables the interrupt event)
	// set = write 1 to ISER, clear = write 1 to ICER
	switch (iser_reg_x)
	{
		case 0:
			(en_di) ? ( *NVIC_ISER0 |= (SET_ONE_BITMASK << iser_irq_offset) ) : ( *NVIC_ICER0 |= (SET_ONE_BITMASK << iser_irq_offset) );
			break;

		case 1:
			(en_di) ? ( *NVIC_ISER1 |= (SET_ONE_BITMASK << iser_irq_offset) ) : ( *NVIC_ICER1 |= (SET_ONE_BITMASK << iser_irq_offset) );
			break;

		case 2:
			(en_di) ? ( *NVIC_ISER2 |= (SET_ONE_BITMASK << iser_irq_offset) ) : ( *NVIC_ICER2 |= (SET_ONE_BITMASK << iser_irq_offset) );
			break;

		case 3:
			(en_di) ? ( *NVIC_ISER3 |= (SET_ONE_BITMASK << iser_irq_offset) ) : ( *NVIC_ICER3 |= (SET_ONE_BITMASK << iser_irq_offset) );
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
void NVIC_IRQPriorityConfig(uint8_t IRQPosition, uint16_t IRQPriority)
{
	/* Each IRQ position's priority setting takes 1 byte in Interrupt Priority Register (IPR) - 60 IPRs */

	/* 1. Find the memory location for the desired IRQ position's priority in IPR */

	// Find which IPR the IRQ position belongs to
	const uint8_t priority_reg_x = IRQPosition / 4;

	// Find which byte in the derived IPR corresponds to desired IRQ position
	const uint8_t priority_irq_offset = IRQPosition % 4;

	// Define a shift offset, considering each priority takes 1 byte, and only the 4 MSBs are implemented
	const uint8_t shift_amount = (priority_irq_offset * 8) + NO_PR_BITS_IMPLEMENTED;

	/* Remember underlying working of pointer arithmetic, NVIC is already a pointer to 4-byte data
	 * hence no need multiply by 4 */
	*(NVIC_IPRx + priority_reg_x) |= (IRQPriority << shift_amount);
}
