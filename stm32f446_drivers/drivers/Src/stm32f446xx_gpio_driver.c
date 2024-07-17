/*
 * stm32f446xx_gpio_driver.c
 *
 *  Created on: Jun 24, 2024
 *      Author: cwc
 */

#include <stdint.h>
#include "stm32f446xx_gpio_driver.h"

//peripheral clock control
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
void GPIO_PClkCntrl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi) {
	if (EnorDi == ENABLE) {
		if (pGPIOx == GPIOA) {
			GPIOA_PCLK_EN();
		} else if (pGPIOx == GPIOB) {
			GPIOB_PCLK_EN();
		} else if (pGPIOx == GPIOC) {
			GPIOC_PCLK_EN();
		} else if (pGPIOx == GPIOD) {
			GPIOD_PCLK_EN();
		} else if (pGPIOx == GPIOE) {
			GPIOE_PCLK_EN();
		} else if (pGPIOx == GPIOF) {
			GPIOF_PCLK_EN();
		} else if (pGPIOx == GPIOG) {
			GPIOG_PCLK_EN();
		} else if (pGPIOx == GPIOH) {
			GPIOH_PCLK_EN();
		}

	} else {

		if (pGPIOx == GPIOA) {
			GPIOA_PCLK_DI();
		} else if (pGPIOx == GPIOB) {
			GPIOB_PCLK_DI();
		} else if (pGPIOx == GPIOC) {
			GPIOC_PCLK_DI();
		} else if (pGPIOx == GPIOD) {
			GPIOD_PCLK_DI();
		} else if (pGPIOx == GPIOE) {
			GPIOE_PCLK_DI();
		} else if (pGPIOx == GPIOF) {
			GPIOF_PCLK_DI();
		} else if (pGPIOx == GPIOG) {
			GPIOG_PCLK_DI();
		} else if (pGPIOx == GPIOH) {
			GPIOH_PCLK_DI();
		}
	}
}

//Init ans DeInit

/*
 ********************************************************************
 *@fn       -   GPIO_Init
 *
 *@brief    -   this fxn initialise GPIO port and pin
 *
 *@param[in]-   base address of the GPIO peripheral+GPIO PIN CONFIGURATION
 *
 *@return   -   none
 *
 *@NOte     -   none
 *
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle) {

	GPIO_PClkCntrl(pGPIOHandle->pGPIOx,ENABLE);

	//1.CONFIGURE MODE OF GPIO PIN

	uint32_t temp = 0;

	if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG) {

		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode
				<< (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		//each pin take 2 bit so mul by 2.....eg.pin no=0 store in 0-1;no=1 stored in 2-3

		pGPIOHandle->pGPIOx->MODER &= ~(0x3
				<< pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		//to clear ie put 00 in bits that need to be modified
		pGPIOHandle->pGPIOx->MODER |= temp; //setting moder with temp val ie mode
		//only change the reqd bit *********
	}
	/*
	 * STEPS FOR GPIO PIN INTERRUPT CONFIG:
	 * 1. PIN MUST BE IN INPUT CONFIG. -as receiving interrupt
	 * 2. CONFIGURE EDGE TRIGGER(RT,FT,RFT)
	 * 3. ENABLE INT DELIVERY FROM PERIPHERAL TO PROCESSOR
	 * 4. IDENTIFY IRQ NO ON WHICH PROCESSOR ACCEPTS INTERRUPT FOR THAT PIN.(FROM VECT. TABLE)
	 * 5. CONFIGURE IRQ PRIORITY FOR IDENTIFIED IRQ NO(PROCESSOR SIDE)
	 * 6. ENAVLE IRQ RECEPTIOM ON THAT IRQ NO.
	 * 7. IMPLEMENT IRQ HANDLER
	 */
	else {
		//(interrupt mode)
		if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT) {
			//1.configure the FTSR
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			//2. clear corresponding RTSR bit
			EXTI->RTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

		}
		if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT) {
			//1.configure the RTSR
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->FTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

		}
		if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT) {
			//1.configure both RTSR & FTSR
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}

		//2. config the GPIO port selection in SYSCFG_EXTICR
		uint8_t temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4;
		uint8_t temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4;
		uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
		SYSCFG_PCLK_EN();
		SYSCFG->EXTICR[temp1] = portcode << (temp2 * 4);
		//3. enable the exti interrupt delivery using IMR
		EXTI->IMR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	}
	temp = 0;
	//2.SPEED
	temp = pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed
			<< (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3
			<< pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

	pGPIOHandle->pGPIOx->OSPEEDR |= temp;
	temp = 0;
	//3.PUPD
	temp = pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl
			<< (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

	pGPIOHandle->pGPIOx->PUPDR &= ~(0x3
			<< pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

	pGPIOHandle->pGPIOx->PUPDR |= temp;
	temp = 0;
	//4.OPTYPE
	temp = pGPIOHandle->GPIO_PinConfig.GPIO_PinOpType
			<< (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

	pGPIOHandle->pGPIOx->OTYPER &= ~(0x3
			<< pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

	pGPIOHandle->pGPIOx->OTYPER |= temp;
	temp = 0;

	//5.ALT FXNALITY
	if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_AF) {
		//configure the af reg which onle
		uint8_t temp1, temp2;

		temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;
		temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;
		temp = pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFxnMode << (4 * temp2);

		pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0x3
				<< pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

		pGPIOHandle->pGPIOx->AFR[temp1] |= temp;
	}
}

/*
 ********************************************************************
 *@fn       -   GPIO_DeInit
 *
 *@brief    -   this fxn reset all registers of the gpio port given as the argument
 *
 *@param[in]-   gives base address of GPIO port

 *
 *@return   -   none
 *
 *@NOte     -   none
 *
 */
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx) {
	if (pGPIOx == GPIOA) {
		GPIOA_REG_RESET();
	} else if (pGPIOx == GPIOB) {
		GPIOB_REG_RESET();
	} else if (pGPIOx == GPIOC) {
		GPIOC_REG_RESET();
	} else if (pGPIOx == GPIOD) {
		GPIOD_REG_RESET();
	} else if (pGPIOx == GPIOE) {
		GPIOE_REG_RESET();
	} else if (pGPIOx == GPIOF) {
		GPIOF_REG_RESET();
	} else if (pGPIOx == GPIOG) {
		GPIOG_REG_RESET();
	} else if (pGPIOx == GPIOH) {
		GPIOH_REG_RESET();
	}

}
//Data Read and Write

/*
 ********************************************************************
 *@fn       -   GPIO_ReadFromInputPin
 *
 *@brief    -   this fxn reads corresponding bit position in IDR
 *
 *@param[in]-   gives base address of GPIO port
 *@param[in]-   give pin number

 *
 *@return   -   0 or 1
 *
 *@NOte     -   none
 *
 */

uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber) {
	uint8_t value;
	value = (uint8_t) ((pGPIOx->IDR >> PinNumber) & 0x00000001);//idr value shift by pin no to get bit position to lsb pos then make every bit postion except lsb =0

	return value;
}

/*
 ********************************************************************
 *@fn       -   GPIO_ReadFromInputPort
 *
 *@brief    -   this fxn reads the IDR port ie all 16 bits
 *
 *@param[in]-   gives base address of GPIO port

 *
 *@return   -   16 bit return value whatever the value of port
 *
 *@NOte     -   none
 *
 */
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx) {
	uint16_t value;
	value = (uint16_t) pGPIOx->IDR;
	return value;
}
/*
 ********************************************************************
 *@fn       -   GPIO_WriteToOutputPin
 *
 *@brief    -   this fxn writes to thr  corresponding bit position in IDR
 *
 *@param[in]-   gives base address of GPIO port
 *@param[in]-   give pin number
 *@param[in]-   gives vaue to be written (0 or 1)

 *
 *@return   -   none
 *
 *@NOte     -   none
 *
 */

void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber,
		uint8_t Value) {
	if (Value == GPIO_PIN_SET) {
		pGPIOx->ODR |= (1 << PinNumber);
	} else {
		pGPIOx->ODR &= ~(1 << PinNumber);

	}
}

void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value) {
	pGPIOx->ODR = Value;
}

void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber) {
	pGPIOx->ODR ^= (1 << PinNumber);		//use XOR to toggle the bit field
	//0^1=1 ........1^1=0   thats how is sets low to high AND high to low
}

//IRQ configuration and ISR handle

void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi) {

	if (EnorDi == ENABLE) {
		if (IRQNumber <= 31) {
			//program ISER0 register
			*NVIC_ISER0 |= (1 << IRQNumber);
		}

		else if (IRQNumber > 31 && IRQNumber < 64) {
			//ISER1
			*NVIC_ISER1 |= (1 << (IRQNumber % 32));

		} else if (IRQNumber >= 64 && IRQNumber < 96) {
			//ISER2
			*NVIC_ISER3 |= (1 << (IRQNumber % 64));

		}
	} else {
		if (IRQNumber <= 31) {
			//program ISER0 register
			*NVIC_ICER0 |= (1 << IRQNumber);
		}

		else if (IRQNumber > 31 && IRQNumber < 64) {
			//ISER1
			*NVIC_ICER1 |= (1 << (IRQNumber % 32));

		} else if (IRQNumber >= 64 && IRQNumber < 96) {
			//ISER2
			*NVIC_ICER3 |= (1 << (IRQNumber % 64));

		}
	}
}

void GPIO_IRQPriorityConfig(uint8_t IRQNumber,uint32_t IRQPriority){
	//1.lets find ipr reg
	uint8_t iprx=IRQNumber/4;
	uint8_t iprx_section=IRQNumber%4;

	uint8_t shift_amount=(8*iprx_section)+(8- NO_PR_BITS_IMPLEMENTED);
	*(NVIC_PR_BASEADDR + iprx) |= (IRQPriority<<shift_amount);
//as if *4 increment by 20*4=80,,,,as one incrementation adds 4 so dont mul by 4
	//only hogher-4 bits implemented out of 8-bits ans lower-4 bits are not applicable
}


void GPIO_IRQHandling(uint8_t PinNumber) {
//when interrupt triggers the user application call this fxn to process the interrupt
//as IRQ handling fxn should know from which pin number interrupt is triggered

	//clr exti pr reg correspondinf to pin no
	if(EXTI->PR & (1<< PinNumber)){//means pemding is there
		//clear
		EXTI->PR |= (1<<PinNumber);//writing 1 clears regster
	}
}
