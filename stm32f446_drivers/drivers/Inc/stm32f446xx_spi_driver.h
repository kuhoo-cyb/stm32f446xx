/*
 * stm32f446xx_spi_driver.h
 *
 *  Created on: Jul 2, 2024
 *      Author: cwc
 */

#ifndef INC_STM32F446XX_SPI_DRIVER_H_
#define INC_STM32F446XX_SPI_DRIVER_H_
#include<stddef.h>
#include "stm32f446xx.h"
/*
 * configuration structure for SPIx peripherals
 */

typedef struct{
	uint8_t SPI_DeviceMode;
	uint8_t SPI_BusConfig;
	uint8_t SPI_SclkSpeed;
	uint8_t SPI_DFF;
	uint8_t SPI_CPOL;
	uint8_t SPI_CPHA;
	uint8_t SPI_SSM;

}SPI_Config_t;

/*
 * handle structure for SPIx peripherals
 */

typedef struct{
	SPI_RegDef_t  *pSPIx;
	SPI_Config_t   SPIConfig;
	uint8_t        *pTxBuffer;/* to store the appl. Tx buffer address*/
	uint8_t        *pRxBuffer;/* to store the appl.Rx buffer address*/
	uint32_t       TxLen;
	uint32_t       RxLen;
	uint8_t        TxState;
	uint8_t        RxState;
}SPI_Handle_t;

/*
 * SPI application status
 */

#define SPI_READY              0
#define SPI_BUSY_IN_RX         1
#define SPI_BUSY_IN_TX         2

/*
 * Possible SPI application events
 */

#define SPI_EVENT_TX_CMPLT     1
#define SPI_EVENT_RX_CMPLT     2
#define SPI_EVENT_OVR_ERR      3
#define SPI_EVENT_CRC_ERR      4


/*
 * @DeviceMode
 */

#define SPI_DEVICE_MODE_MASTER  1
#define SPI_DEVICE_MODE_SLAVE   0

/*
 * @SPI_BusConfig
 */

#define SPI_BUS_CONFIG_FD               1  //full duplex
#define SPI_BUS_CONFIG_HD               2
//#define SPI_BUS_CONFIG_SIMPLEX_TXONLY   3//this is same as full duplex ie transmit and recieve both
#define SPI_BUS_CONFIG_SIMPLEX_RXONLY   3


/*
 * @SPI_SclkSpeed
 */

#define SPI_SCLK_SPEED_DIV2            0
#define SPI_SCLK_SPEED_DIV4            1
#define SPI_SCLK_SPEED_DIV8            2
#define SPI_SCLK_SPEED_DIV16           3
#define SPI_SCLK_SPEED_DIV32           4
#define SPI_SCLK_SPEED_DIV64           5
#define SPI_SCLK_SPEED_DIV128          6
#define SPI_SCLK_SPEED_DIV256          7

/*
 * @SPI_DFF
 */

#define SPI_DFF_8BITS  0
#define SPI_DFF_16BITS 1

/*
 * @SPI_CPOL
 */

#define SPI_CPOL_HIGH  1
#define SPI_CPOL_LOW   0

/*
 * @SPI_CPHA
 */

#define SPI_CPHA_HIGH  1
#define SPI_CPHA_LOW   0

/*
 * @SSM
 */

#define SPI_SSM_EN   1
#define SPI_SSM_DI   0


/*
 * SPI related flag definitions
 */

#define SPI_TXE_FLAG     (1<< SPI_SR_TXE)
#define SPI_RXNE_FLAG    (1<< SPI_SR_RXNE)
#define SPI_CHSIDE_FLAG  (1<< SPI_SR_CHSIDE)
#define SPI_UDR_FLAG     (1<< SPI_SR_UDR)
#define SPI_CRCERR_FLAG  (1<< SPI_SR_CRCERR)
#define SPI_MODF_FLAG    (1<< SPI_SR_MODF)
#define SPI_OVR_FLAG     (1<< SPI_SR_OVR)
#define SPI_FRE_FLAG     (1<< SPI_SR_FRE)
#define SPI_BUSY_FLAG    (1<< SPI_SR_BSY)

/*
 *************************************************************************************
 APIs supported by this driver
 *************************************************************************************

 */

//peripheral clock control
void SPI_PClkCntrl(SPI_RegDef_t *pSPIx,uint8_t EnorDi);//by using this u can en or dis the pclk for the given gpio base addr


//Init and DeInit

void SPI_Init(SPI_Handle_t *pSPIHandle);//to initalise gpio port and pin
void SPI_DeInit(SPI_RegDef_t *pSPIx);

/*
 * Data Send and Recieve
 */

void SPI_SendData(SPI_RegDef_t *pSPIx,uint8_t *pTxBuffer,uint32_t Len);
void SPI_ReceiveData(SPI_RegDef_t *pSPIx,uint8_t *pRxBuffer,uint32_t Len);

uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIx,uint8_t *pTxBuffer,uint32_t Len);
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIx,uint8_t *pRxBuffer,uint32_t Len);

/*IRQ*/

void SPI_IRQInterruptConfig(uint8_t IRQNumber,uint8_t EnorDi);
void SPI_IRQPriorityConfig(uint8_t IRQNumber,uint32_t IRQPriority);
void SPI_IRQHandling(SPI_Handle_t *pHandle);


/*
 * other peripheral control APIs
 */

void SPI_PeripheralControl(SPI_RegDef_t *pSPIx,uint8_t EnOrDi);
void SPI_SSIConfig(SPI_RegDef_t *pSPIx,uint8_t EnOrDi);
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx,uint8_t EnOrDi);
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName);

void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx);
void SPI_CloseTransmission(SPI_Handle_t *pHandle);
void SPI_CloseReception(SPI_Handle_t *pHandle);

/* application callback
 */

void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHanle,uint8_t AppEv);


#endif /* INC_STM32F446XX_SPI_DRIVER_H_ */
