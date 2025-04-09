/*
 * x_spi.h
 *
 *  Created on: Apr 9, 2025
 *      Author: MURAKARU
 */

#ifndef INC_X_SPI_H_
#define INC_X_SPI_H_


#include "stm32l433xx.h"


/* Structure forward declaration */
typedef struct SPIPeripheralConfiguration SPI_Config_t;
typedef struct SPIHandleStructure SPI_Handle_t;

/*
 * @SPI_PORT_CONFIGURATION
 * Initializer for a SPI port's configuration structure, containing all port settings
 */
typedef struct SPIPeripheralConfiguration
{
	uint8_t SPI_DeviceMode;										/* Master or slave mode */
	uint8_t SPI_BusCommsConfig;									/* Full-duplex, half-dduplex or simplex */
	uint8_t SPI_ClockConfig;									/* Baud rate/Clock speed (Hz) */
	uint8_t SPI_DataFrameFormat;								/* 8-bit or 16-bit frames */
	uint8_t SPI_ClockPolarity;									/* SPI bus idle state: LOW or HIGH */
	uint8_t SPI_ClockPhase;										/*  */
	uint8_t SPI_SSM;											/* SSM flag configuration */
} SPI_Config_t;

/*
 * @SPI_HANDLER
 * Initializer for a SPI handler structure, that manages a SPI instance's configuration and properties
 */
typedef struct SPIHandleStructure
{
	SPIx_Reg_t *ptr_SPIx;
	SPI_Config_t SPI_Config;
}  SPI_Handle_t;

/*

 *****************************************************************************************
 *								APIs supported by this driver
 *		 For more information about the APIs check the function definitions
 *****************************************************************************************

*/

/* SPI Clock Configuration */
void SPI_PeriphClkCtrl(__R SPIx_Reg_t *const, uint8_t);

/* SPI Initialization */
void SPI_Init(__W SPI_Handle_t *const);
void SPI_DeInit(__R SPIx_Reg_t *const);					/* See RCC Peripheral Reset Register - Resets all registers */

/* SPI Data Read and Write */

// blocking APIs
void SPI_DataSend(__RW SPIx_Reg_t *const, __W uint8_t *const, uint32_t);
void SPI_DataReceive(__RW SPIx_Reg_t *const, __R uint8_t *const, uint32_t);

/* SPI Interrupt Configuration/Handling */
void SPI_IRQNumberConfig(uint8_t, uint8_t);
void SPI_IRQPriorityConfig(uint8_t, uint16_t);
void SPI_IRQHandler(__R SPI_Handle_t *const);

/* Other SPI Control Peripherals */


#endif /* INC_X_SPI_H_ */
