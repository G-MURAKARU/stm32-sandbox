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
void SPI_Init(__R SPI_Handle_t *const ptr_SPIHandle)
{
	/* set up for CR1 register contents */
	volatile uint32_t temp_register = 0;

	/* 1. Configure the device mode */
	/* Set the MSTR bit in SPI CR1 if mode = master		!< Already cleared if slave > */
	if (ptr_SPIHandle->SPI_Config.SPI_DeviceMode) temp_register |= (SET_ONE_BITMASK << MSTR);

	/* 2. Configure the SPI Bus Mode */
	switch (ptr_SPIHandle->SPI_Config.SPI_BusCommsConfig)
	{
		case FULL_DUPLEX:
			// BIDI Mode should be cleared
			/* temp_register &= ~(CLEAR_ONE_BITMASK << BIDIMODE);		!< Already cleared > */

			// RXONLY should be cleared (FD enabled)
			/* temp_register &= ~(CLEAR_ONE_BITMASK << RXONLY);			!< Already cleared > */

			break;
		case HALF_DUPLEX:
			// BIDI Mode should be set
			temp_register |= (SET_ONE_BITMASK << BIDIMODE);

			// BIDI Output should be enabled
			temp_register |= (SET_ONE_BITMASK << BIDIOE);

			break;
		case SIMPLEX_RX:
			// BIDI Mode should be cleared
			/* temp_register &= ~(CLEAR_ONE_BITMASK << BIDIMODE);		!< Already cleared > */

			// RXONLY should be set
			temp_register |= (SET_ONE_BITMASK << RXONLY);

			break;

		default:
			break;
	}

	/* 3. Configure (pre-scale) the SPI bus baud rate */
	temp_register |= (ptr_SPIHandle->SPI_Config.SPI_ClockConfig << BAUD);

	/* 4. Configure the SPI data frame format */
	/* Found in SPI CR2 */
	ptr_SPIHandle->ptr_SPIx->CR2 |= (ptr_SPIHandle->SPI_Config.SPI_DataFrameFormat << DS);

	/* 5. Configure the SPI clock polarity */
	temp_register |= (ptr_SPIHandle->SPI_Config.SPI_ClockPolarity << CPOL);

	/* 6. Configure the SPI clock phase */
	temp_register |= (ptr_SPIHandle->SPI_Config.SPI_ClockPhase << CPHA);

	/* 7. SPI CRC configurations */
	if (ptr_SPIHandle->SPI_Config.SPI_CRCEnable)
	{
		/* Set CRC enabled bit */
		temp_register |= (SET_ONE_BITMASK << CRCEN);
		/* Select CRC length, set if 16-bit, leave cleared if 8-bit */
		if (ptr_SPIHandle->SPI_Config.SPI_CRCLength) temp_register |= (SET_ONE_BITMASK << CRCL);
	}

	/* 8. Write configurations to SPI CR1 */
	ptr_SPIHandle->ptr_SPIx->CR1 = temp_register;
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



