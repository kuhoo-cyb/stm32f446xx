
#include "stm32f446xx.h"

#include "stm32f446xx_gpio_driver.h"

void delay(void){
	for(uint32_t i=0;i<500000;i++);
}
int main(void){


	GPIO_Handle_t GpioLed;

	GpioLed.pGPIOx=GPIOA;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_5;
	GpioLed.GPIO_PinConfig.GPIO_PinMode= GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed= GPIO_SPD_FAST;
	GpioLed.GPIO_PinConfig.GPIO_PinOpType=GPIO_OP_TYPE_OD;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl=GPIO_PIN_PU;//toggle with very les intensity as R=40KOHM
	// so disable pu resistor and connect external resistance of 320-470 ohm:::between PD12-GND


	GPIO_PClkCntrl(GPIOA,ENABLE);
	GPIO_Init(&GpioLed);

	while(1){
		GPIO_ToggleOutputPin(GPIOA,GPIO_PIN_NO_5);
		delay();
	}
	return 0;
}
