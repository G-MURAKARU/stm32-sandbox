/*
 * x_i2c.c
 *
 *  Created on: Apr 21, 2025
 *      Author: MURAKARU
 */


#include "stm32l433xx.h"
#include "x_i2c.h"

/*

 * @fn						- I2C_PeriphClkCtrl
 *
 * @brief					- This function enables or disables the peripheral clock for the given I2C port, through the RCC register
 *
 * @param[ptr_I2Cx]			- base address of the I2C peripheral
 * @param[en_di]			- action to take: ENABLE(1) or DISABLE(0)
 *
 * @return					- none
 *
 * @Note					- none

 */
void I2C_PeriphClkCtrl(__R I2Cx_Reg_t *const ptr_I2Cx, bool en_di)
{
	if (ptr_I2Cx == I2C1) (en_di) ? PER_EnableClock(RCC_MAP_I2C1) : PER_DisableClock(RCC_MAP_I2C1);

	else if (ptr_I2Cx == I2C2) (en_di) ? PER_EnableClock(RCC_MAP_I2C2) : PER_DisableClock(RCC_MAP_I2C2);

	else if (ptr_I2Cx == I2C3) (en_di) ? PER_EnableClock(RCC_MAP_I2C3) : PER_DisableClock(RCC_MAP_I2C3);

	else {};
}

/*

 * @fn						- I2C_Init
 *
 * @brief					- this function initializes the given I2C port
 *
 * @param[ptr_I2CHandle]	- pointer to the base address of the I2C Handle structure
 *
 * @return					- none
 *
 * @Note					- none

 */
void I2C_Init(__RH I2C_Handle_t *const ptr_I2CHandle);

/*

 * @fn						- I2C_DeInit
 *
 * @brief					- this function resets all registers of a given I2C port through the RCC reset register
 *
 * @param[ptr_I2Cx]			- pointer to the base address of the I2C peripheral
 *
 * @return					- none
 *
 * @Note					- none

 */
void I2C_DeInit(__RH I2Cx_Reg_t *const ptr_I2Cx)
{
	if (ptr_I2Cx == I2C1) PER_ResetPeripheral(RCC_MAP_I2C1);

	else if (ptr_I2Cx == I2C2) PER_ResetPeripheral(RCC_MAP_I2C2);

	else if (ptr_I2Cx == I2C3) PER_ResetPeripheral(RCC_MAP_I2C3);

	else {};
}

/*

 * @fn						- I2C_Control
 *
 * @brief					- this function enables or disables I2Cx communication, through configuring PE flag
 *
 * @param[ptr_I2Cx]			- pointer to the I2Cx port to enable/disable
 * @param[en_di]			- action to take: ENABLE(1) or DISABLE(0)
 *
 * @return					- none
 *
 * @Note					- none

 */
void I2C_Control(I2Cx_Reg_t *const ptr_I2Cx, bool en_di)
{
	(en_di) ? ( ptr_I2Cx->CR1 |= (SET_ONE_BITMASK << I2C_ENABLE) ) : ( ptr_I2Cx->CR1 &= ~(CLEAR_ONE_BITMASK << I2C_ENABLE) );
}

