/*
 * 007_txonly_arduino.c
 *
 *  Created on: Jul 3, 2024
 *      Author: cwc
 */

/*
 * 006_spi_tx_testing.c
 *
 *  Created on: Jul 2, 2024
 *      Author: cwc
 */

#include <string.h>
#include "stm32f446xx.h"
#include "stm32f446xx_spi_driver.h"
#include "stm32f446xx_gpio_driver.h"

//PB15->SPI2_MOSI
//PB14->SPI2_MISO
//PB13->SPI2_SCLK
//PB12->SPI2_NSS ---as input avoid multi master bus collision ie SPI can have multiple masters
//AF MODE=5

void delay(void) {
	for (uint32_t i = 0; i < 500000; i++);
}

void SPI2_GPIOInits(void) {

	GPIO_Handle_t SPIPins;

	SPIPins.pGPIOx = GPIOB;
	SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_AF;
	SPIPins.GPIO_PinConfig.GPIO_PinAltFxnMode = 5;
	SPIPins.GPIO_PinConfig.GPIO_PinOpType = GPIO_OP_TYPE_PP;
	SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	SPIPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPD_FAST;

	//SCLK
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GPIO_Init(&SPIPins);

	//MOSI
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;
	GPIO_Init(&SPIPins);

	//mISO
	//SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
	//GPIO_Init(&SPIPins);

	//not needed as no slave

	//NSS
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GPIO_Init(&SPIPins);

}

void SPI2_Inits(void) {
	SPI_Handle_t SPI2handle;

	SPI2handle.pSPIx = SPI2;
	SPI2handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI2handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV32;//generate sclk of 8 MHX
	SPI2handle.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
	SPI2handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
	SPI2handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
	SPI2handle.SPIConfig.SPI_SSM = SPI_SSM_DI;	//hardware slave management

	SPI_Init(&SPI2handle);
}

void GPIO_ButtonInit(void) {

	GPIO_Handle_t GpioBtn;

	GpioBtn.pGPIOx = GPIOC;
	GpioBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GpioBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GpioBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPD_FAST;
	GpioBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;//toggle with very les intensity as R=40KOHM
	// so disable pu resistor and connect external resistance of 320-470 ohm:::between PD12-GND

	GPIO_Init(&GpioBtn);

}

int main(void) {

	char user_data[] = "Hello world";

	GPIO_ButtonInit();

	//this fxn used to initialize GPIO pins to behave as SPI2 pins
	SPI2_GPIOInits();

	//this fxn used to initialize the SPI2 peripheral parameters
	SPI2_Inits();

	SPI_SSOEConfig(SPI2, ENABLE);

	/*
	 * making SSOE 1 does NSS op enable
	 * the NSS pin is automatically managed by the hardware
	 * i.e. when SPE=1 --> NSS=0
	 * and  when SPE=0 --> NSS=1
	 */

	while (1) {

		//wait till the button is pressed
		while (!GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13));

		delay();
		//enable SPI2 peripheral
		SPI_PeripheralControl(SPI2, ENABLE);

		//first send length info to slave
		uint8_t datalen=strlen(user_data);
		SPI_SendData(SPI2,&datalen,1);

		//to send data
		SPI_SendData(SPI2, (uint8_t*) user_data, strlen(user_data));

		//lets confirm spi is not busy
		while( SPI_GetFlagStatus(SPI2,SPI_BUSY_FLAG) );

		//disable the SPI2 peripheral wait until last bit sent
		SPI_PeripheralControl(SPI2, DISABLE);
	}


	return 0;
}

