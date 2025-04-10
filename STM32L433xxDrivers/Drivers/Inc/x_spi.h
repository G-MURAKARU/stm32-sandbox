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
	uint8_t SPI_ClockPhase;										/* Clock transition on first data capture edge */
	uint8_t SPI_SSM;											/* SSM flag configuration */
	uint8_t SPI_CRCEnable;										/* SSM flag configuration */
	uint8_t SPI_CRCLength;										/* SSM flag configuration */
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
 * @SPI_CR1_CONTENTS
 * SPI port control register bits, for shifting
 */
typedef enum SPIControlRegister1
{
	CPHA, CPOL, MSTR, BAUD, SPE=6, LSB_FIRST, SSI, SSM, RXONLY, CRCL, CRCNEXT, CRCEN, BIDIOE, BIDIMODE,
} SPI_CR1_e;

/*
 * @SPI_CR2_CONTENTS
 * SPI port control register bits, for shifting
 */
typedef enum SPIControlRegister2
{
	RXDMAEN, TXDMAEN, SSOE, NSSP, FRF, ERRIE, RXNEIE, TXEIE, DS, FRXTH = 12, LDMA_RX, LDMA_TX,
} SPI_CR2_e;

/*
 * @SPI_DEVICE_MODES
 * SPI port possible device modes
 */
typedef enum SPIDeviceModes
{
	SLAVE, MASTER,
} SPI_Mode_e;

/*
 * @SPI_BUS_MODES
 * SPI port possible bus communication configuration modes
 */
typedef enum SPIBusCommsConfigModes
{
	FULL_DUPLEX, HALF_DUPLEX, SIMPLEX_TX, SIMPLEX_RX,
} SPI_Bus_e;

/*
 * @SPI_CLOCK_MODES
 * SPI port possible serial clock speed modes (prescalers)
 */
typedef enum SPIClockPrescalers
{
	PRE_2, PRE_4, PRE_8, PRE_16, PRE_32, PRE_64, PRE_128, PRE_256,
} SPI_Clock_e;

/*
 * @SPI_DATA_FRAME_FORMATS
 * SPI port possible data frame formats/sizes
 */
typedef enum SPIDataFrameFormats
{
	FR_4BIT, FR_5BIT, FR_6BIT, FR_7BIT, FR_8BIT, FR_9BIT, FR_10BIT,
	FR_11BIT, FR_12BIT, FR_13BIT, FR_14BIT, FR_15BIT, FR_16BIT,
} SPI_Frame_e;

/*
 * @SPI_CLOCK_POLARITY
 * SPI port possible clock polarity states (idle)
 */
typedef enum SPIClockPolarity
{
	CK_LOW, CK_HIGH
} SPI_CPOL_e;

/*
 * @SPI_CLOCK_PHASE
 * SPI port possible clock phases
 */
typedef enum SPIClockPhase
{
	CP_LOW, CP_HIGH
} SPI_CPHA_e;

/*
 * @SPI_SSM
 * SPI port possible slave management modes
 */
typedef enum SPISlaveManagement
{
	SSM_DISABLE, SSM_ENABLE,
} SPI_SSM_e;

/*
 * @SPI_CRC_ENABLE
 * SPI port possible CRC enable states
 */
typedef enum SPICRCEnable
{
	CRC_DISABLE, CRC_ENABLE,
} SPI_CRCEn_e;

/*
 * @SPI_CRC_LENGTH
 * SPI port possible CRC polynomial length
 */
typedef enum SPICRCLength
{
	CRC_8BIT, CRC_16BIT,
} SPI_CRCLen_e;

/*

 *****************************************************************************************
 *								APIs supported by this driver
 *		 For more information about the APIs check the function definitions
 *****************************************************************************************

*/

/* SPI Clock Configuration */
void SPI_PeriphClkCtrl(__R SPIx_Reg_t *const, uint8_t);

/* SPI Initialization */
void SPI_Init(__R SPI_Handle_t *const);
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
