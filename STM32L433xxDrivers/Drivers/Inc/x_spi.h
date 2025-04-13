/*
 * x_spi.h
 *
 *  Created on: Apr 9, 2025
 *      Author: MURAKARU
 */

#ifndef INC_X_SPI_H_
#define INC_X_SPI_H_


#include "stm32l433xx.h"


/* Structure forward declarations */
typedef struct SPI_PeripheralRegisters SPIx_Reg_t;

/* Pointers to SPI base addresses */
#define SPI1							( (__RW SPIx_Reg_t *const)SPI1_BASE_ADDR )
#define SPI2							( (__RW SPIx_Reg_t *const)SPI2_BASE_ADDR )
#define SPI3							( (__RW SPIx_Reg_t *const)SPI3_BASE_ADDR )

/* Clock Enable for SPIx */
#define SPI1_CLK_EN()					( (RCC->APB2ENR) |= (1 << 12) )
#define SPI2_CLK_EN()					( (RCC->APB1ENR1) |= (1 << 14) )
#define SPI3_CLK_EN()					( (RCC->APB1ENR1) |= (1 << 15) )

/* Clock Disable for SPIx */
#define SPI1_CLK_DI()					( (RCC->APB2ENR) &= ~(1 << 12) )
#define SPI2_CLK_DI()					( (RCC->APB1ENR1) &= ~(1 << 14) )
#define SPI3_CLK_DI()					( (RCC->APB1ENR1) &= ~(1 << 15) )

/* SPI Register Reset Macro Definitions */
#define SPI1_REG_RESET()				do { RCC->APB2RSTR |= (1 << 12); RCC->APB2RSTR &= ~(1 << 12); } while (0)
#define SPI2_REG_RESET()				do { RCC->APB1RSTR1 |= (1 << 14); RCC->APB1RSTR1 &= ~(1 << 14); } while (0)
#define SPI3_REG_RESET()				do { RCC->APB1RSTR1 |= (1 << 15); RCC->APB1RSTR1 &= ~(1 << 15); } while (0)

/*
 * @SPI_PERIPHERAL_REGISTERS
 * Initializer for an SPI peripheral instance, containing all memory-mapped registers
 */
typedef struct SPI_PeripheralRegisters
{
	__RW uint32_t CR1;											/* SPI control register 1 */
	__RW uint32_t CR2;											/* SPI control register 2 */
	__RH uint32_t SR;									/* SPI status register, see CRCERR flag */
	__RW uint32_t DR;											/* SPI data register */
	__RW uint32_t CRCPR;										/* SPI CRC polynomial register */
	__RH uint32_t RXCRCR;								/* SPI Rx CRC register */
	__RH uint32_t TXCRCR;								/* SPI Tx CRC register */
} SPIx_Reg_t;

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
	uint8_t SPI_CRCEnable;										/* CRC flag configuration */
	uint8_t SPI_CRCLength;										/* CRC polynomial length */
	uint8_t SPI_SSI;											/* SSI flag configuration */
	uint8_t SPI_SSOE;											/* SSOE flag configuration, CR2 */
	uint8_t SPI_NSSP;											/* NSSP flag configuration, CR2 */
} SPI_Config_t;

/*
 * @SPI_HANDLER
 * Initializer for a SPI handler structure, that manages a SPI instance's configuration and properties
 */
typedef struct SPIHandleStructure
{
	__RW SPIx_Reg_t *ptr_SPIx;
	SPI_Config_t SPI_Config;
}  SPI_Handle_t;

/*
 * @SPI_CR1_CONTENTS
 * SPI port control register bits, for shifting
 */
typedef enum SPIControlRegister1
{
	CPHA, CPOL, MSTR, BAUD, SPE = 6, LSB_FIRST, SSI, SSM, RXONLY, CRCL, CRCNEXT, CRCEN, BIDIOE, BIDIMODE,
} SPI_CR1_e;

/*
 * @SPI_CR2_CONTENTS
 * SPI port control register bits, for shifting
 */
typedef enum SPIControlRegister2
{
	RXDMAEN, TXDMAEN, SSOE, NSSP, FRF, ERRIE, RXNEIE, TXEIE, DS, FRXTH = 12, LDMA_RX, LDMA_TX,
} SPI_CR2_e;

typedef enum SPIStatusRegister
{
	RXNE, TXE, CRCERR = 4, MODF, OVR, BSY, FRE, FRLVL, FTLVL = 11,
} SPI_SR_e;

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
	FULL_DUPLEX, HALF_DUPLEX, SIMPLEX_RX, SIMPLEX_TX,
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
	FR_4BIT = 3, FR_5BIT, FR_6BIT, FR_7BIT, FR_8BIT, FR_9BIT, FR_10BIT,
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
 * @SPI_SSI
 * SPI port possible internal slave select modes
 */
typedef enum SPIInternalSlaveSelect
{
	SSI_DISABLE, SSI_ENABLE,
} SPI_SSI_e;

/*
 * @SPI_SSOE
 * SPI port possible slave select output enable modes (NSS)
 */
typedef enum SPISlaveSelectOutputEnable
{
	SSOE_DISABLE, SSOE_ENABLE,
} SPI_SSOE_e;

/*
 * @SPI_NSSP
 * SPI port possible NSS pulse management modes
 */
typedef enum SPINSSPulseManagement
{
	NSSP_DISABLE, NSSP_ENABLE,
} SPI_NSSP_e;

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
void SPI_Init(__RH SPI_Handle_t *const);
void SPI_DeInit(__RH SPIx_Reg_t *const);					/* See RCC Peripheral Reset Register - Resets all registers */

/* SPI Data Read and Write */

// Blocking APIs
void SPI_DataSend(__RW SPIx_Reg_t *const, const uint8_t *, int32_t, uint8_t);
void SPI_DataReceive(__RW SPIx_Reg_t *const, const uint8_t *, uint32_t);

/* SPI Interrupt Configuration/Handling */
void SPI_IRQNumberConfig(uint8_t, uint8_t);
void SPI_IRQPriorityConfig(uint8_t, uint16_t);
void SPI_IRQHandler(__R SPI_Handle_t *const);

/* Other SPI Control Peripherals */
void SPI_Control(__W SPIx_Reg_t *const, uint8_t);


#endif /* INC_X_SPI_H_ */
