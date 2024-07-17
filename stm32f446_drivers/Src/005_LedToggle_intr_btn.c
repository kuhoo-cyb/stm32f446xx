/*
 * 005_LedToggle_intr_btn.c
 *
 *  Created on: Jul 1, 2024
 *      Author: cwc
 */

/*
 * 004LedToggle-extBtn.c
 *
 *  Created on: Jun 29, 2024
 *      Author: cwc
 */

//LED IP=PD12
//BUTTON IP=PD5
#include<string.h>
#include "stm32f446xx.h"
#include "stm32f446xx_gpio_driver.h"

#define LOW  0
#define BTN_PRESSED  LOW

void delay(void) {
	for (uint32_t i = 0; i < 500000/2; i++);
}

int main(void) {

	GPIO_Handle_t GpioLed, GpioBtn;
	memset(&GpioLed,0,sizeof(GpioLed));//to clear all
	memset(&GpioBtn,0,sizeof(GpioBtn));


	GpioLed.pGPIOx = GPIOD;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPD_LOW;
	GpioLed.GPIO_PinConfig.GPIO_PinOpType = GPIO_OP_TYPE_PP;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD; //toggle with very les intensity as R=40KOHM
	// so disable pu resistor and connect external resistance of 320-470 ohm:::between PD12-GND

	GPIO_PClkCntrl(GPIOD, ENABLE);
	GPIO_Init(&GpioLed);

	GpioLed.pGPIOx = GPIOD;
	GpioBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
	GpioBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;
	GpioBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPD_FAST;
	GpioBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;//toggle with very les intensity as R=40KOHM
	// so disable pu resistor and connect external resistance of 320-470 ohm:::between PD12-GND

	GPIO_PClkCntrl(GPIOD, ENABLE);
	GPIO_Init(&GpioBtn);

	//INTERRUPT CONFIGURATION
	GPIO_IRQPriorityConfig(IRQ_NO_EXTI9_5,NVIC_IRQ_PRI15);	//can proritize from 0-15 any
	GPIO_IRQInterruptConfig(IRQ_NO_EXTI9_5,ENABLE);

	while(1);

	return 0;
}
////isr implementation

void EXTI9_5_IRQHandler(void) {

	delay();
	GPIO_IRQHandling(GPIO_PIN_NO_5);
	GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_12);
}
