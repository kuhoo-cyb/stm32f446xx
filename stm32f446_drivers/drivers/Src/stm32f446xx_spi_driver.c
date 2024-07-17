/*
 * stm32f446xx_spi_driver.c
 *
 *  Created on: Jul 2, 2024
 *      Author: cwc
 */

#include "stm32f446xx_spi_driver.h"
#include "stm32f446xx.h"

//spi is full duplex ie both txn and rx can happen simultaneusly
//helper fxn

static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle){
	//2. check DFF bit in cr1
	if (pSPIHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF)) {
		//16 bit
		//1.load data into the DR

		pSPIHandle->pSPIx->DR = *((uint16_t*) pSPIHandle->pTxBuffer);			//typecast to 16 bit
		pSPIHandle->TxLen--;
		pSPIHandle->TxLen--;
		(uint16_t*) pSPIHandle->pTxBuffer++;			//to point to the next data item
	} else {
		//8 bit dff
		pSPIHandle->pSPIx->DR = *(pSPIHandle->pTxBuffer);
		pSPIHandle->TxLen--;
		pSPIHandle->pTxBuffer++;
	}
	//if len=0 close txn
	if(! pSPIHandle->TxLen){//means txlen =0
		//Tx len zero so close spi communication and inform the application that Tx is over
		//this prevents intrr from setting up of txe flag
		SPI_CloseTransmission(pSPIHandle);
		SPI_ApplicationEventCallback(pSPIHandle,SPI_EVENT_TX_CMPLT);
	}

}
static void spi_rxe_interrupt_handle(SPI_Handle_t *pSPIHandle){

	if (pSPIHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF)) {
		//16 bit
		//1.load data into the DR

		*((uint16_t*)pSPIHandle->pRxBuffer)=(uint16_t) pSPIHandle->pSPIx->DR;
		pSPIHandle->RxLen-=2;
		pSPIHandle->RxLen++;
		pSPIHandle->RxLen++;
	} else {
		//8 bit dff
		*(pSPIHandle->pRxBuffer)=(uint8_t) pSPIHandle->pSPIx->DR;
		pSPIHandle->RxLen--;
		pSPIHandle->pRxBuffer++;
	}

	if(! pSPIHandle->RxLen){
		SPI_CloseReception(pSPIHandle);
		SPI_ApplicationEventCallback(pSPIHandle,SPI_EVENT_RX_CMPLT);
	}
}

static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle){

	uint8_t temp;
	//1. clear ovr flag;
	if(pSPIHandle->TxState != SPI_BUSY_IN_TX){
		temp=pSPIHandle->pSPIx->DR;
		temp=pSPIHandle->pSPIx->SR;
	}
	(void) temp;

	//2. inform application
	SPI_ApplicationEventCallback(pSPIHandle,SPI_EVENT_OVR_ERR);

}

void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle){
	pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_TXEIE);
	pSPIHandle->pTxBuffer = NULL;
	pSPIHandle->TxState = SPI_READY;
	pSPIHandle->TxLen = 0;

}
void SPI_CloseReception(SPI_Handle_t *pSPIHandle){
	pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_RXNEIE);
	pSPIHandle->pRxBuffer = NULL;
	pSPIHandle->RxState = SPI_READY;
	pSPIHandle->RxLen = 0;
}

void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx){
	uint8_t temp;
	temp=pSPIx->DR;
	temp=pSPIx->SR;
	(void) temp;
}

__attribute__((weak)) void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHanle,uint8_t AppEv){
	//this is a weak implementation . the application may override this fxn

}






/*
 ********************************************************************
 *@fn       -   GPIO_PClkCntrl
 *
 *@brief    -   this fxn enables or disables peri clock for the given GPIO port
 *
 *@param[in]-   base address of the GPIO peripheral
 *@param[in]-   ENABLE or DISABLE macros
 *
 *
 *@return   -   none
 *
 *@NOte     -   none
 *
 */

void SPI_PClkCntrl(SPI_RegDef_t *pSPIx, uint8_t EnorDi) {
	if (EnorDi == ENABLE) {
		if (pSPIx == SPI1) {
			SPI1_PCLK_EN();
		} else if (pSPIx == SPI2) {
			SPI2_PCLK_EN();
		} else if (pSPIx == SPI3) {
			SPI3_PCLK_EN();
		} else if (pSPIx == SPI4) {
			SPI4_PCLK_EN();
		}

	} else {

		if (pSPIx == SPI1) {
			SPI1_PCLK_DI();
		} else if (pSPIx == SPI2) {
			SPI2_PCLK_DI();
		} else if (pSPIx == SPI3) {
			SPI3_PCLK_DI();
		} else if (pSPIx == SPI4) {
			SPI4_PCLK_DI();
		}
	}
}

/*
 ********************************************************************
 *@fn       -   GPIO_PClkCntrl
 *
 *@brief    -   this fxn enables or disables peri clock for the given GPIO port
 *
 *@param[in]-   base address of the GPIO peripheral
 *@param[in]-   ENABLE or DISABLE macros
 *
 *
 *@return   -   none
 *
 *@NOte     -   none
 *
 */

void SPI_Init(SPI_Handle_t *pSPIHandle) {

	SPI_PClkCntrl(pSPIHandle->pSPIx, ENABLE);
	//first lets configure SPI_CR1 reg

	uint32_t tempreg = 0;

	//1. configure device mode

	tempreg |= pSPIHandle->SPIConfig.SPI_DeviceMode << SPI_CR1_MSTR;;

	//2.configure bus config

	if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD) {
		//bidi mode to be cleared
		tempreg &= ~(1 << SPI_CR1_BIDIMODE);

	} else if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD) {

		//bidi mode to be set
		tempreg |= (1 << SPI_CR1_BIDIMODE);

	} else if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY) {
		//bidi mode to be cleared AND RXONLY bit must be set
		tempreg &= ~(1 << SPI_CR1_BIDIMODE);
		tempreg |= (1 << SPI_CR1_RXONLY);

	}

	//3. configure Sclk speed(baud rate)

	tempreg |= pSPIHandle->SPIConfig.SPI_SclkSpeed << SPI_CR1_BR;

	//4. configure DFF

	tempreg |= pSPIHandle->SPIConfig.SPI_DFF << SPI_CR1_DFF;

	//5.configure cpol

	tempreg |= pSPIHandle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL;

	//6. configure CPHA

	tempreg |= pSPIHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA;
	tempreg |= pSPIHandle->SPIConfig.SPI_SSM << SPI_CR1_SSM;


	pSPIHandle->pSPIx->CR1 = tempreg;

}

/*
 ********************************************************************
 *@fn       -   GPIO_PClkCntrl
 *
 *@brief    -   this fxn enables or disables peri clock for the given GPIO port
 *
 *@param[in]-   base address of the GPIO peripheral
 *@param[in]-   ENABLE or DISABLE macros
 *
 *
 *@return   -   none
 *
 *@NOte     -   none
 *
 */

void SPI_DeInit(SPI_RegDef_t *pSPIx) {
	if (pSPIx == SPI1) {
		SPI1_REG_RESET();
	} else if (pSPIx == SPI2) {
		SPI2_REG_RESET();
	} else if (pSPIx == SPI3) {
		SPI3_REG_RESET();
	} else if (pSPIx == SPI4) {
		SPI4_REG_RESET();
	}
}

/*
 ********************************************************************
 *@fn       -   GPIO_PClkCntrl
 *
 *@brief    -   this fxn enables or disables peri clock for the given GPIO port
 *
 *@param[in]-   base address of the GPIO peripheral
 *@param[in]-   ENABLE or DISABLE macros
 *
 *
 *@return   -   none
 *
 *@NOte     -  This is a blocking call
 *
 */

uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName) {

	if (pSPIx->SR & FlagName) {
		return FLAG_SET;
	}

	return FLAG_RESET;
}

void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len) {

	while (Len > 0) {
		//1.wait until TXE is set
		while (SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG) == FLAG_RESET);

		//2. check DFF bit in cr1
		if (pSPIx->CR1 & (1 << SPI_CR1_DFF)) {
			//16 bit
			//1.load data into the DR

			pSPIx->DR = *((uint16_t*) pTxBuffer);			//typecast to 16 bit
			Len--;
			Len--;
			(uint16_t*) pTxBuffer++;			//to point to the next data item
		} else {
			//8 bit dff
			pSPIx->DR = *(pTxBuffer);
			Len--;
			pTxBuffer++;
		}
	}
}

/*
 ********************************************************************
 *@fn       -   GPIO_PClkCntrl
 *
 *@brief    -   this fxn enables or disables peri clock for the given GPIO port
 *
 *@param[in]-   base address of the GPIO peripheral
 *@param[in]-   ENABLE or DISABLE macros
 *
 *
 *@return   -   none
 *
 *@NOte     -   none
 *
 */

void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len) {
	while (Len > 0) {
		//1.wait until RXNE is set
		while (SPI_GetFlagStatus(pSPIx, SPI_RXNE_FLAG) == (uint8_t)FLAG_RESET);

		//2. check DFF(data frame format) bit in cr1
		if (pSPIx->CR1 & (1 << SPI_CR1_DFF)) {
			//16 bit
			//1.load data from DR to Rxbuffer address

			*((uint16_t*)pRxBuffer) = pSPIx->DR ;		//typecast to 16 bit
			Len--;
			Len--;
			(uint16_t*) pRxBuffer++;			//to point to the next data item
		} else {
			//8 bit dff
			*(pRxBuffer)=pSPIx->DR;
			Len--;
			pRxBuffer++;
		}
	}
}

/*
 ********************************************************************
 *@fn       -   GPIO_PClkCntrl
 *
 *@brief    -   this fxn enables or disables peri clock for the given GPIO port
 *
 *@param[in]-   base address of the GPIO peripheral
 *@param[in]-   ENABLE or DISABLE macros
 *
 *
 *@return   -   none
 *
 *@NOte     -   none
 *
 */

void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi) {

	if (EnOrDi == ENABLE) {
		pSPIx->CR1 |= (1 << SPI_CR1_SPE);
	} else {
		pSPIx->CR1 &= ~(1 << SPI_CR1_SPE);

	}
}

void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi) {
	if (EnOrDi == ENABLE) {
		pSPIx->CR1 |= (1 << SPI_CR1_SSI);
	} else {
		pSPIx->CR1 &= ~(1 << SPI_CR1_SSI);

	}
}

void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi) {
	if (EnOrDi == ENABLE) {
		pSPIx->CR2 |= (1 << SPI_CR2_SSOE);
	} else {
		pSPIx->CR2 &= ~(1 << SPI_CR2_SSOE);

	}
}


uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle,uint8_t *pTxBuffer,uint32_t Len){

	uint8_t state=pSPIHandle->TxState;

	if(state!= SPI_BUSY_IN_TX){
		//1. save the Tx buffer and Len info in some global variable

		pSPIHandle->pTxBuffer = pTxBuffer;
		pSPIHandle->TxLen = Len;

		//2. Mark the spi state as busy in txn so that no other code can take over SPI
		//peripheral until txn is over

		pSPIHandle->TxState = SPI_BUSY_IN_TX;

		//3.Enable the TXEIE control bit to get interrupt whenever TXE flag is set in SR
		//TXEIE-Txn buffer empty interrupt enable
		pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_TXEIE);
	}

	return state;
}


uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle,uint8_t *pRxBuffer,uint32_t Len){
	uint8_t state = pSPIHandle->RxState;

	if (state != SPI_BUSY_IN_RX) {
		//1. save the Tx buffer and Len info in some global variable

		pSPIHandle->pRxBuffer = pRxBuffer;
		pSPIHandle->RxLen = Len;

		//2. Mark the spi state as busy in txn so that no other code can take over SPI
		//peripheral until txn is over

		pSPIHandle->TxState = SPI_BUSY_IN_RX;

		//3.Enable the TXEIE control bit to get interrupt whenever TXE flag is set in SR(status reg)
		//RXEIE-Rxn buffer empty interrupt enable
		pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_RXNEIE);
	}

	return state;
}


void SPI_IRQHandling(SPI_Handle_t *pHandle){

	uint8_t temp1,temp2;
	//1. first lets check for TXE(ie why interrupt happened)
	temp1=pHandle->pSPIx->SR & (1<< SPI_SR_TXE);//if txe set temp1=1
	temp2=pHandle->pSPIx->CR2 & (1<<SPI_CR2_TXEIE);

	if(temp1 & temp2){
		//handle TXE
		spi_txe_interrupt_handle(pHandle);
	}

	//2. check for rxne
	temp1 = pHandle->pSPIx->SR & (1 << SPI_SR_RXNE);		//if txe set temp1=1
	temp2 = pHandle->pSPIx->CR2 & (1 << SPI_CR2_RXNEIE);//if txe flag set have to set it 1 to generate an inrr req.

	if (temp1 & temp2) {
		//handle TXE
		spi_rxe_interrupt_handle(pHandle);
	}

	//3. check for overrun flag
	temp1 = pHandle->pSPIx->SR & (1 << SPI_SR_OVR);		//if txe set temp1=1
	temp2 = pHandle->pSPIx->CR2 & (1 << SPI_CR2_ERRIE);

	if (temp1 & temp2) {
		//handle TXE
		spi_ovr_err_interrupt_handle(pHandle);
	}


}

