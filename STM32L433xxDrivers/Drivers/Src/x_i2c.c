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
	if (ptr_I2Cx == I2C1) (en_di) ? I2C1_CLK_EN() : I2C1_CLK_DI();

	else if (ptr_I2Cx == I2C2) (en_di) ? I2C2_CLK_EN() : I2C2_CLK_DI();

	else if (ptr_I2Cx == I2C3) (en_di) ? I2C3_CLK_EN() : I2C3_CLK_DI();

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
	if (ptr_I2Cx == I2C1) I2C1_REG_RESET();

	else if (ptr_I2Cx == I2C2) I2C2_REG_RESET();

	else if (ptr_I2Cx == I2C3) I2C3_REG_RESET();

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
void I2C_Control(__W I2Cx_Reg_t *const ptr_I2Cx, bool en_di)
{
	(en_di) ? ( ptr_I2Cx->CR1 |= (SET_ONE_BITMASK << I2C_ENABLE) ) : ( ptr_I2Cx->CR1 &= ~(CLEAR_ONE_BITMASK << I2C_ENABLE) );
}

