/*
 * x_spi.c
 *
 *  Created on: Apr 9, 2025
 *      Author: MURAKARU
 */


#include "stm32l433xx.h"
#include "x_spi.h"


/*

 * @fn						- SPI_PeriphClkCtrl
 *
 * @brief					- This function enables or disables the peripheral clock for the given SPI port
 *
 * @param[ptr_SPIx]			- base address of the SPI peripheral
 * @param[en_di]			- ENABLE or DISABLE macros, ENABLE=1, DISABLE=0
 *
 * @return					- none
 *
 * @Note					- none

 */
void SPI_PeriphClkCtrl(__R SPIx_Reg_t *const ptr_SPIx, uint8_t en_di)
{
	if (ptr_SPIx == SPI1) (en_di) ? SPI1_CLK_EN() : SPI1_CLK_DI();
	else if (ptr_SPIx == SPI2) (en_di) ? SPI2_CLK_EN() : SPI2_CLK_DI();
	else if (ptr_SPIx == SPI3) (en_di) ? SPI3_CLK_EN() : SPI3_CLK_DI();
	else;
}


/*

 * @fn						- SPI_Init
 *
 * @brief					- This function initializes the given SPI port
 *
 * @param[ptr_SPIHandle]	- base address of the SPI Handle structure
 *
 * @return					- none
 *
 * @Note					- none

 */
void SPI_Init(__W SPI_Handle_t *const ptr_SPIHandle)
{
	//
}


/*

 * @fn						- SPI_DeInit
 *
 * @brief					- This function resets all registers of a given SPI port through the RCC reset register
 *
 * @param[ptr_SPIx]			- base address of the SPI peripheral
 *
 * @return					- none
 *
 * @Note					- none

 */
void SPI_DeInit(__R SPIx_Reg_t *const ptr_SPIx)
{
	if (ptr_SPIx == SPI1) SPI1_REG_RESET();

	else if (ptr_SPIx == SPI2) SPI2_REG_RESET();

	else if (ptr_SPIx == SPI3) SPI3_REG_RESET();

	else;
}


/* SPI Data Read and Write */

// blocking APIs
void SPI_DataSend(__RW SPIx_Reg_t *const, __W uint8_t *const, uint32_t);
void SPI_DataReceive(__RW SPIx_Reg_t *const, __R uint8_t *const, uint32_t);

/* SPI Interrupt Configuration/Handling */
void SPI_IRQNumberConfig(uint8_t, uint8_t);
void SPI_IRQPriorityConfig(uint8_t, uint16_t);
void SPI_IRQHandler(__R SPI_Handle_t *const);



