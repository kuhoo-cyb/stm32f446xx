/*
 * 010_i2c_master_tx_testing.c
 *
 *  Created on: Jul 17, 2024
 *      Author: cwc
 */

#include <stdio.h>
#include <string.h>
#include "stm32f446xx.h"
#include "stm32f446xx_i2c_driver.h"
#include "stm32f446xx_gpio_driver.h"

#define SLAVE_ADDR    0x68

#define LOW  0
#define BTN_PRESSED  LOW

I2C_Handle_t I2C1Handle;

uint8_t some_data[]="we are testing data";

void delay(void) {
	for (uint32_t i = 0; i < 500000 / 2; i++);
}

/*
 * PB6->SCL
 * PB9->SDA
 */
void I2C1_GPIOInits(void) {

	GPIO_Handle_t I2CPins;

	I2CPins.pGPIOx = GPIOB;
	I2CPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_AF;
	I2CPins.GPIO_PinConfig.GPIO_PinAltFxnMode = 4;
	I2CPins.GPIO_PinConfig.GPIO_PinOpType = GPIO_OP_TYPE_OD;
	I2CPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
	I2CPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPD_FAST;

	//SCLK
	I2CPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_6;
	GPIO_Init(&I2CPins);

	//SDA
	I2CPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_9;
	GPIO_Init(&I2CPins);


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

void I2C1_Inits(void) {

	I2C1Handle.pI2Cx = I2C1;
	I2C1Handle.I2C_Config.I2C_AckControl = I2C_ACK_ENABLE;
	I2C1Handle.I2C_Config.I2C_DeviceAddress = 0x61;
	I2C1Handle.I2C_Config.I2C_FMDutyCycle = I2C_FM_DUTY_2; //generate sclk of 8 MHX
	I2C1Handle.I2C_Config.I2C_SCLSpeed=I2C_SCL_SPEED_SM;

	I2C_Init(&I2C1Handle);
}

int main(void){

	GPIO_ButtonInit();

	//I2C pin inits
	I2C1_GPIOInits();

	//I2C peripheral config
	I2C1_Inits();

	//enable i2c peripheral
	I2C_PeripheralControl(I2C1,ENABLE);

	//wait for button press
	while (1) {
		while(GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13));
		delay();

		//send some data to slave
		I2C_MasterSendData(&I2C1Handle,some_data,strlen((char*)some_data),SLAVE_ADDR);

    }



}
