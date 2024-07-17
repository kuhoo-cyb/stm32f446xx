/*
 * 011_i2c_master_rx_testing.c
 *
 *  Created on: Jul 17, 2024
 *      Author: cwc
 */

//R/W==1
//SCL clock=100Khz(sm)
//ext pull up resistor=3.3kohm for SDA and SCL line
//ext pull up calc :: R(max)=T(rise)/(0.8473*Cb(max))----Cb(max)=400,,,,T(rise)=
//look up logic analyser connection in vid 58-1

//SDA=PB7
//SCL=PB6

//first master send cmnd code0x51 to read the length of data from slave
//and then master sends cmnd code 0x52 to read the complete data from slave

//1 : data write - mstr send command code 0x51 to slave to read length
//2 : data read - master reading response from slave about length of string
//3 : data write - master sending cmmnd code 0x52 to read cmplt data
//4 : data read - master read complete data


#include <stdio.h>
#include <string.h>
#include "stm32f446xx.h"
#include "stm32f446xx_i2c_driver.h"
#include "stm32f446xx_gpio_driver.h"

//extern void initialise_monitor_handles();

#define SLAVE_ADDR    0x68

I2C_Handle_t I2C1Handle;

uint8_t rcv_buf[32];

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



	uint8_t commndcode;

	uint8_t len;

	//intialise_monitor_handles();
	//printf("Application is running\n");

	//go to debug config and in run cmmnd in startup - monitor arm semihosting enable
	GPIO_ButtonInit();

	//I2C pin inits
	I2C1_GPIOInits();

	//I2C peripheral config
	I2C1_Inits();

	//enable i2c peripheral
	I2C_PeripheralControl(I2C1,ENABLE);
	//here PE becomes 1

	//ack bit made 1 after pe=1
	I2C_ManageAcking(I2C1,ENABLE);

	//wait for button press
	while (1) {
		while (GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13));
		delay();

		commndcode=0x51;
		I2C_MasterSendData(&I2C1Handle,&commndcode,1,SLAVE_ADDR,I2C_ENABLE_SR);//length = 1byte

		//receive length information from slave
		I2C_MasterRecieveData(&I2C1Handle,&len,1,SLAVE_ADDR,I2C_ENABLE_SR);

		commndcode=0x52;
		I2C_MasterSendData(&I2C1Handle,&commndcode,1,SLAVE_ADDR,I2C_ENABLE_SR);//length = 1byte

		I2C_MasterRecieveData(&I2C1Handle,rcv_buf,len,SLAVE_ADDR,I2C_DISABLE_SR);//as want to end transaction


		//rcv_buf[len+1]='\0';
		//printf("data: %s",rcv_buf);
	}


}
