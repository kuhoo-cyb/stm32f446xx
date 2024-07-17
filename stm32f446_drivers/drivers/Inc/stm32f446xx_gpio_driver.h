/*
 * stm32f446xx_grio_driver.h
 *
 *  Created on: Jun 24, 2024
 *      Author: cwc
 */

#include <stdint.h>
#include "stm32f446xx.h"

//********** handle structure for gpio pin**********8

typedef struct{
	uint8_t GPIO_PinNumber;//one byte is enough (8 bit) as range =0-15 only
	                        /*!<POSSIBLE VALUES FROM @GPIO_PIN_NUMBER>*/
	uint8_t GPIO_PinMode;  /*!<POSSIBLE VALUES FROM @GPIO_PIN_MODES>*/
	uint8_t GPIO_PinSpeed; /*!<POSSIBLE VALUES FROM @GPIO_PIN_SPEED>*/
	uint8_t GPIO_PinPuPdControl; /*!<POSSIBLE VALUES FROM @GPIO_PIN_PUPDR>*/
	uint8_t GPIO_PinOpType; /*!<POSSIBLE VALUES FROM @GPIO_PIN_OUTPUT_TYPES>*/
	uint8_t GPIO_PinAltFxnMode; /*!<POSSIBLE VALUES FROM @GPIO_PIN_ALTFXN>*/

}GPIO_PinConfig_t;

typedef struct{

	GPIO_RegDef_t *pGPIOx;//this holds the base address of the gpio port to which the port belongs
	GPIO_PinConfig_t GPIO_PinConfig;//this holds gpio pin configuration settingd
}GPIO_Handle_t;

/*
 * @GPIO_PIN_NUMBER
 */

#define GPIO_PIN_NO_0    0
#define GPIO_PIN_NO_1    1
#define GPIO_PIN_NO_2    2
#define GPIO_PIN_NO_3    3
#define GPIO_PIN_NO_4    4
#define GPIO_PIN_NO_5    5
#define GPIO_PIN_NO_6    6
#define GPIO_PIN_NO_7    7
#define GPIO_PIN_NO_8    8
#define GPIO_PIN_NO_9    9
#define GPIO_PIN_NO_10   10
#define GPIO_PIN_NO_11   11
#define GPIO_PIN_NO_12   12
#define GPIO_PIN_NO_13   13
#define GPIO_PIN_NO_14   14
#define GPIO_PIN_NO_15   15

/*
 * @GPIO_PIN_MODES
 * GPIO PIN POSSIBLE MODES
 */
//  <=3 = NON_INTERRUPT MODES
//  >3  = INTERRUPT MODES
#define GPIO_MODE_IN     0
#define GPIO_MODE_OUT    1
#define GPIO_MODE_AF     2
#define GPIO_MODE_ANALOG 3
#define GPIO_MODE_IT_FT  4   //input falling edge trigger       |
#define GPIO_MODE_IT_RT  5   //rising edge trigger              |all 3 interrupt modes
#define GPIO_MODE_IT_RFT 6   ///rising edge falling edge trigger|

/*
 * @GPIO_PIN_OUTPUT_TYPES
 * GPIO output types
 */

#define GPIO_OP_TYPE_PP       0
#define GPIO_OP_TYPE_OD       1

/*
 * @GPIO_PIN_SPEED
 * GPIO op speed
 */

#define GPIO_SPD_LOW    0
#define GPIO_SPD_MED    0
#define GPIO_SPD_FAST   0
#define GPIO_SPD_HIGH   0

/*
 * GPIO PORT PUPDR
 * @GPIO_PIN_PUPDR
 */

#define GPIO_NO_PUPD    0
#define GPIO_PIN_PU     1
#define GPIO_PIN_PD     2

/*
 *************************************************************************************
 APIs supported by this driver
 *************************************************************************************

 */

//peripheral clock control
void GPIO_PClkCntrl(GPIO_RegDef_t *pGPIOx,uint8_t EnorDi);//by using this u can en or dis the pclk for the given gpio base addr


//Init ans DeInit

void GPIO_Init(GPIO_Handle_t *pGPIOHandle);//to initalise gpio port and pin
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);//deinitialise the reg of given gpio peripheral,,,,,ie reset them
                                        //just takes base add of peripheral to reset all its reg

//Data Read and Write

uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx,uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx,uint8_t PinNumber,uint8_t Value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx,uint16_t Value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx,uint8_t PinNumber);

//IRQ configuration and ISR handle

void GPIO_IRQInterruptConfig(uint8_t IRQNumber,uint8_t EnorDi);
void GPIO_IRQPriorityConfig(uint8_t IRQNumber,uint32_t IRQPriority);
void GPIO_IRQHandling(uint8_t PinNumber);//when interrupt triggers the user application call this fxn to process the interrupt
                                         //as IRQ handling fxn should know from which pin number interrupt is triggered


