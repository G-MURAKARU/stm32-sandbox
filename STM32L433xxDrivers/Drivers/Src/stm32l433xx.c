/*
 * stm32l433xx.c
 *
 *  Created on: Apr 5, 2025
 *      Author: OMEN 16
 */


#include "stm32l433xx.h"


uint8_t get_flag_status(uint32_t register_contents, uint8_t flag_length, uint8_t offset)
{
	if (flag_length == 1) return ( (register_contents >> offset) & CHECK_ONE_BITMASK );
	else if (flag_length == 2) return ( (register_contents >> offset) & CHECK_TWO_BITMASK );
	return (-1);
}
