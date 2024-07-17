/*
 * 003LedToggle-Button.c
 *
 *  Created on: Jun 29, 2024
 *      Author: cwc
 */

//BUTTON -PC13
//LED= PA5
//in pull up config ie if button pressed->LED OFF||LOW::::button not pressed->LED ON||HIGH
#include "stm32f446xx.h"
#include "stm32f446xx_gpio_driver.h"

#define LOW  0
#define BTN_PRESSED  LOW

void delay(void){
	for(uint32_t i=0;i<500000;i++);
}

int main(void){


	GPIO_Handle_t GpioLed,GpioBtn;

	GpioLed.pGPIOx=GPIOA;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_5;
	GpioLed.GPIO_PinConfig.GPIO_PinMode= GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed= GPIO_SPD_FAST;
	GpioLed.GPIO_PinConfig.GPIO_PinOpType=GPIO_OP_TYPE_OD;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl=GPIO_PIN_PU;//toggle with very les intensity as R=40KOHM
	// so disable pu resistor and connect external resistance of 320-470 ohm:::between PD12-GND


	GPIO_PClkCntrl(GPIOA,ENABLE);
	GPIO_Init(&GpioLed);

	GpioLed.pGPIOx=GPIOC;
		GpioBtn.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_13;
		GpioBtn.GPIO_PinConfig.GPIO_PinMode= GPIO_MODE_IN;
		GpioBtn.GPIO_PinConfig.GPIO_PinSpeed= GPIO_SPD_FAST;
		GpioBtn.GPIO_PinConfig.GPIO_PinPuPdControl=GPIO_NO_PUPD;//toggle with very les intensity as R=40KOHM
		// so disable pu resistor and connect external resistance of 320-470 ohm:::between PD12-GND


		GPIO_PClkCntrl(GPIOC,ENABLE);
		GPIO_Init(&GpioBtn);

	while(1){
		if(GPIO_ReadFromInputPin(GPIOC,GPIO_PIN_NO_13)==BTN_PRESSED){
			delay();
			GPIO_ToggleOutputPin(GPIOA,GPIO_PIN_NO_5);
		}


	}
	return 0;
}