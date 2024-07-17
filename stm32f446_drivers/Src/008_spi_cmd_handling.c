/*
 * 008_spi_cmd_handling.c
 *
 *  Created on: Jul 3, 2024
 *      Author: cwc
 */

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

#include <stdio.h>
#include <string.h>
#include "stm32f446xx.h"
#include "stm32f446xx_spi_driver.h"
#include "stm32f446xx_gpio_driver.h"

//eg  CMS_LED_CTRL  <pin no>  <value>
//pin no- digital pin no of the arduino board (0-9) 1 byte where led connected//so need ext led
//value -- if 1: do led on else off

#define COMMAND_LED_CTRL    0x50
#define COMMAND_SENSOR_READ 0x51
#define COMMAND_LED_READ    0x52
#define COMMAND_PRINT       0x53
#define COMMAND_ID_READ     0x54

#define LED_ON    1
#define LED_OFF   0

//arduino analog pins

#define ANALOG_PIN0    0
#define ANALOG_PIN1    1
#define ANALOG_PIN2    2
#define ANALOG_PIN3    3
#define ANALOG_PIN4    4

#define LED_PIN   9

//PB15->SPI2_MOSI
//PB14->SPI2_MISO
//PB13->SPI2_SCLK
//PB12->SPI2_NSS ---as input avoid multi master bus collision ie SPI can have multiple masters
//AF MODE=5

void delay(void) {
	for (uint32_t i = 0; i < 500000 / 2; i++)
		;
}

void SPI2_GPIOInits(void) {

	GPIO_Handle_t SPIPins;

	SPIPins.pGPIOx = GPIOB;
	SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_AF;
	SPIPins.GPIO_PinConfig.GPIO_PinAltFxnMode = 5;
	SPIPins.GPIO_PinConfig.GPIO_PinOpType = GPIO_OP_TYPE_PP;
	SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	SPIPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPD_FAST;

	//SCLK-seriak clock--dlk sent from master
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GPIO_Init(&SPIPins);

	//MOSI- master out slave in--- data sent from mastr to slave
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;
	GPIO_Init(&SPIPins);

	//mISO- master in slave out--- data sent from slave to master
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
	GPIO_Init(&SPIPins);

	//NSS- slave select
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GPIO_Init(&SPIPins);

}

void SPI2_Inits(void) {
	SPI_Handle_t SPI2handle;

	SPI2handle.pSPIx = SPI2;
	SPI2handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI2handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV8; //generate sclk of 8 MHX
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

uint8_t SPI_VerifyResponse(uint8_t ackbyte) {
	if (ackbyte == 0xF5) {
		return 1;
	} else {
		return 0;
	}
}

int main(void) {

	uint8_t dummy_write = 0xff;
	uint8_t dummy_read;

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
		while (!GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13))
			;

		delay();
		//enable SPI2 peripheral
		SPI_PeripheralControl(SPI2, ENABLE);

		//1.CMD_LED_CNTRL  <pin no(1)>   <value(1)>

		uint8_t ackbyte;
		uint8_t args[2];

		uint8_t commndcode = COMMAND_LED_CTRL;
		SPI_SendData(SPI2, &commndcode, 1);

		//do dummy read  to cear off the RXNE
		SPI_ReceiveData(SPI2, &dummy_read, 1);

		//send some dummy bits (1 byte) to fetch response from slave
		SPI_SendData(SPI2, &dummy_write, 1);

		//read the ack byte recieved
		SPI_ReceiveData(SPI2, &ackbyte, 1);

		if (SPI_VerifyResponse(ackbyte)) {

			//send othe arguments
			args[0] = LED_PIN;
			args[1] = LED_ON;
			SPI_SendData(SPI2, args, 2);

			//dummy read
			SPI_ReceiveData(SPI2, args, 2);
			printf("COMMAND_LED_CTRL Executed\n");
		}

		//end of command_led _contrl

		//2.CMD_SENSOR_READ <analog pin no(1)>
		//wait till the button is pressed
		while (!GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13))
			;

		delay();

		commndcode = COMMAND_SENSOR_READ;

		SPI_SendData(SPI2, &commndcode, 1);

		//do dummy read  to cear off the RXNE
		SPI_ReceiveData(SPI2, &dummy_read, 1);

		//send some dummy bits (1 byte) to fetch response from slave
		SPI_SendData(SPI2, &dummy_write, 1);

		//read the ack byte recieved
		SPI_ReceiveData(SPI2, &ackbyte, 1);

		if (SPI_VerifyResponse(ackbyte)) {

			//send othe arguments
			args[0] = ANALOG_PIN0;
			SPI_SendData(SPI2, args, 1);
			SPI_ReceiveData(SPI2, &dummy_read, 1);

			//insert delay so tha slave ready with data adc convrersion
			delay();

			//send some dummy bits (1 byte) to fetch response from slave
			SPI_SendData(SPI2, &dummy_write, 1);

			uint8_t analog_read;

			SPI_ReceiveData(SPI2, &analog_read, 1);
			printf("COMMAND_SENSOR_READ %d\n", analog_read);
		}

		//3.  CMD_LED_READ 	 <pin no(1) >

		//wait till button is pressed
		while (!GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0))
			;

		//to avoid button de-bouncing related issues 200ms of delay
		delay();

		commndcode = COMMAND_LED_READ;

		//send command
		SPI_SendData(SPI2, &commndcode, 1);

		//do dummy read to clear off the RXNE
		SPI_ReceiveData(SPI2, &dummy_read, 1);

		//Send some dummy byte to fetch the response from the slave
		SPI_SendData(SPI2, &dummy_write, 1);

		//read the ack byte received
		SPI_ReceiveData(SPI2, &ackbyte, 1);

		if (SPI_VerifyResponse(ackbyte)) {
			args[0] = LED_PIN;

			//send arguments
			SPI_SendData(SPI2, args, 1); //sending one byte of

			//do dummy read to clear off the RXNE
			SPI_ReceiveData(SPI2, &dummy_read, 1);

			//insert some delay so that slave can ready with the data
			delay();

			//Send some dummy bits (1 byte) fetch the response from the slave
			SPI_SendData(SPI2, &dummy_write, 1);

			uint8_t led_status;
			SPI_ReceiveData(SPI2, &led_status, 1);
			printf("COMMAND_READ_LED %d\n", led_status);

		}

		//4. CMD_PRINT 		<len(2)>  <message(len) >

		//wait till button is pressed
		while (!GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0))
			;

		//to avoid button de-bouncing related issues 200ms of delay
		delay();

		commndcode = COMMAND_PRINT;

		//send command
		SPI_SendData(SPI2, &commndcode, 1);

		//do dummy read to clear off the RXNE
		SPI_ReceiveData(SPI2, &dummy_read, 1);

		//Send some dummy byte to fetch the response from the slave
		SPI_SendData(SPI2, &dummy_write, 1);

		//read the ack byte received
		SPI_ReceiveData(SPI2, &ackbyte, 1);

		uint8_t message[] = "Hello ! How are you ??";
		if (SPI_VerifyResponse(ackbyte)) {
			args[0] = strlen((char*) message);

			//send arguments
			SPI_SendData(SPI2, args, 1); //sending length

			//do dummy read to clear off the RXNE
			SPI_ReceiveData(SPI2, &dummy_read, 1);

			delay();

			//send message
			for (int i = 0; i < args[0]; i++) {
				SPI_SendData(SPI2, &message[i], 1);
				SPI_ReceiveData(SPI2, &dummy_read, 1);
			}

			printf("COMMAND_PRINT Executed \n");

		}

		//5. CMD_ID_READ
		//wait till button is pressed
		while (!GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0))
			;

		//to avoid button de-bouncing related issues 200ms of delay
		delay();

		commndcode = COMMAND_ID_READ;

		//send command
		SPI_SendData(SPI2, &commndcode, 1);

		//do dummy read to clear off the RXNE
		SPI_ReceiveData(SPI2, &dummy_read, 1);

		//Send some dummy byte to fetch the response from the slave
		SPI_SendData(SPI2, &dummy_write, 1);

		//read the ack byte received
		SPI_ReceiveData(SPI2, &ackbyte, 1);

		uint8_t id[11];
		uint32_t i = 0;
		if (SPI_VerifyResponse(ackbyte)) {
			//read 10 bytes id from the slave
			for (i = 0; i < 10; i++) {
				//send dummy byte to fetch data from slave
				SPI_SendData(SPI2, &dummy_write, 1);
				SPI_ReceiveData(SPI2, &id[i], 1);
			}

			id[10] = '\0';

			printf("COMMAND_ID : %s \n", id);

		}

		//lets confirm SPI is not busy
		while (SPI_GetFlagStatus(SPI2, SPI_BUSY_FLAG))
			;

		//Disable the SPI2 peripheral
		SPI_PeripheralControl(SPI2, DISABLE);

		printf("SPI Communication Closed\n");
	}

	return 0;
}

