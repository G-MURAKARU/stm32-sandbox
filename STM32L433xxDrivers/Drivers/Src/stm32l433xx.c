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
int8_t MCU_GetFlagStatus(uint32_t register_contents, uint8_t flag_length, uint8_t offset)
{
	if (flag_length == 1) return ( (register_contents >> offset) & CHECK_ONE_BITMASK );
	else if (flag_length == 2) return ( (register_contents >> offset) & CHECK_TWO_BITMASK );
	return (-1);
}
