/*
 * stm32f446xx.h
 *
 *  Created on: Jun 23, 2024
 *      Author: cwc
 */

#ifndef INC_STM32F446XX_H_
#define INC_STM32F446XX_H_
#include<stdint.h>

#define __vo volatile
/*****************************START:PROCESSOR SPECIFIC DETAILS******************************
/*
 * ARM CORTEX MX PROCESSOR NVIC ISERx REGISTER ADDR
 */

#define NVIC_ISER0           ((__vo uint32_t*)0xE000E100)
#define NVIC_ISER1           ((__vo uint32_t*)0xE000E104)
#define NVIC_ISER2           ((__vo uint32_t*)0xE000E108)
#define NVIC_ISER3           ((__vo uint32_t*)0xE000E10C)

/*
 * ARM CORTEX MX PROCESSOR NVIC ICERx REGISTER ADDR
 */

#define NVIC_ICER0           ((__vo uint32_t*)0xE000E180)
#define NVIC_ICER1           ((__vo uint32_t*)0xE000E184)
#define NVIC_ICER2           ((__vo uint32_t*)0xE000E188)
#define NVIC_ICER3           ((__vo uint32_t*)0xE000E18C)

/*
 * ARM CORTEX MX PROCESSOR priority REGISTER ADDR
 */

#define NVIC_PR_BASEADDR     ((__vo uint32_t*)0xE000E400)


#define NO_PR_BITS_IMPLEMENTED     4
//base address of flash and sram

#define FLASH_BASEADDR       0x08000000U//base address of flash
#define SRAM1_BASEADDR       0x20000000U //112 kb=1COO BYTES
#define SRAM2_BASEADDR       0x2001C000U  //sram1+1C00=2001c00
#define ROM                  0x1FFF0000U//also called system memory
#define SRAM                 SRAM1_BASEADDR

//Base address of peripheral buses(AHB1,AHB2,APB1,APB2)

#define PERIPH_BASEADDR      0x40000000U
#define APB1PERI_BASEADDR    PERIPH_BASEADDR
#define APB2PERI_BASEADDR    0x40010000U
#define AHB1PERI_BASEADDR    0x40020000U
#define AHB2PERI_BASEADDR    0x50000000U

//Base addresses of peripherals of the AHB1 buses

#define GPIOA_OFFADDR         0x0000
#define GPIOA_BASEADDR        (AHB1PERI_BASEADDR+GPIOA_OFFADDR)
#define GPIOB_BASEADDR        (AHB1PERI_BASEADDR+0x0400)
#define GPIOC_BASEADDR        (AHB1PERI_BASEADDR+0x0800)
#define GPIOD_BASEADDR        (AHB1PERI_BASEADDR+0x1000)
#define GPIOE_BASEADDR        (AHB1PERI_BASEADDR+0x1400)
#define GPIOF_BASEADDR        (AHB1PERI_BASEADDR+0x1800)
#define GPIOG_BASEADDR        (AHB1PERI_BASEADDR+0x1C00)
#define GPIOH_BASEADDR        (AHB1PERI_BASEADDR+0x2000)
#define RCC_BASEADDR          (AHB1PERI_BASEADDR+0x3800)

//BASE ADDRESSES OF APB2 BUS

#define EXTI_BASEADDR         (APB2PERI_BASEADDR+0x3C00)
#define SYSCFG_BASEADDR       (APB2PERI_BASEADDR+0x3800)
#define SPI1_BASEADDR         (APB2PERI_BASEADDR+0x3000)
#define SPI4_BASEADDR         (APB2PERI_BASEADDR+0x3400)
#define USART1_BASEADDR       (APB2PERI_BASEADDR+0x1000)
#define USART6_BASEADDR       (APB2PERI_BASEADDR+0x1400)

//BASE ADDR OF APB1 BUS

#define SPI2_BASEADDR         (APB1PERI_BASEADDR+0x3800)
#define SPI3_BASEADDR         (APB1PERI_BASEADDR+0x3C00)
#define USART3_BASEADDR       (APB1PERI_BASEADDR+0x4800)
#define USART2_BASEADDR       (APB1PERI_BASEADDR+0x4400)

#define I2C1_BASEADDR         (APB1PERI_BASEADDR+0x5400)
#define I2C2_BASEADDR         (APB1PERI_BASEADDR+0x5800)
#define I2C3_BASEADDR         (APB1PERI_BASEADDR+0x5C00)

//peripheral definitions(peri base add typecasted to reg_def_t

#define GPIOA     ((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB     ((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC     ((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD     ((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE     ((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOF     ((GPIO_RegDef_t*)GPIOF_BASEADDR)
#define GPIOG     ((GPIO_RegDef_t*)GPIOG_BASEADDR)
#define GPIOH     ((GPIO_RegDef_t*)GPIOH_BASEADDR)

#define RCC       ((RCC_RegDef_t*)RCC_BASEADDR)

#define EXTI      ((EXTI_RegDef_t*)EXTI_BASEADDR)

#define SYSCFG    ((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)

#define SPI1      ((SPI_RegDef_t*)SPI1_BASEADDR)
#define SPI2      ((SPI_RegDef_t*)SPI2_BASEADDR)
#define SPI3      ((SPI_RegDef_t*)SPI3_BASEADDR)
#define SPI4      ((SPI_RegDef_t*)SPI4_BASEADDR)

#define I2C1       ((I2C_RegDef_t*)I2C1_BASEADDR)
#define I2C2       ((I2C_RegDef_t*)I2C2_BASEADDR)
#define I2C3       ((I2C_RegDef_t*)I2C3_BASEADDR)

//*********clock enable macros for GPIOx peripherals********
#define GPIOA_PCLK_EN()       (RCC->AHB1ENR |= (1<<0))
#define GPIOB_PCLK_EN()       (RCC->AHB1ENR |= (1<<1))
#define GPIOC_PCLK_EN()       (RCC->AHB1ENR |= (1<<2))
#define GPIOD_PCLK_EN()       (RCC->AHB1ENR |= (1<<3))
#define GPIOE_PCLK_EN()       (RCC->AHB1ENR |= (1<<4))
#define GPIOF_PCLK_EN()       (RCC->AHB1ENR |= (1<<5))
#define GPIOG_PCLK_EN()       (RCC->AHB1ENR |= (1<<6))
#define GPIOH_PCLK_EN()       (RCC->AHB1ENR |= (1<<7))

//********CLOCK ENABLE MACROS FOR I2CX PERIPHERALS***********
#define I2C1_PCLK_EN()        (RCC->APB1ENR |= (1<<21))
#define I2C2_PCLK_EN()        (RCC->APB1ENR |= (1<<22))
#define I2C3_PCLK_EN()        (RCC->APB1ENR |= (1<<23))

//********CLOCK ENABLE MACROS FOR SPIX PERIPHERALS***********
#define SPI1_PCLK_EN()        (RCC->APB2ENR |= (1<<12))
#define SPI2_PCLK_EN()        (RCC->APB1ENR |= (1<<14))
#define SPI3_PCLK_EN()        (RCC->APB1ENR |= (1<<15))
#define SPI4_PCLK_EN()        (RCC->APB2ENR |= (1<<13))

//********CLOCK ENABLE MACROS FOR USARTX PERIPHERALS***********
#define USART1_PCLK_EN()       (RCC->APB2ENR |= (1<<4))
#define USART2_PCLK_EN()       (RCC->APB1ENR |= (1<<17))
#define USART3_PCLK_EN()       (RCC->APB1ENR |= (1<<18))
#define UART4_PCLK_EN()        (RCC->APB1ENR |= (1<<19))
#define UART5_PCLK_EN()        (RCC->APB1ENR |= (1<<20))
#define USART6_PCLK_EN()       (RCC->APB2ENR |= (1<<5))


//********CLOCK ENABLE MACROS FOR SYSCFG PERIPHERALS***********
#define SYSCFG_PCLK_EN()       (RCC->APB2ENR |= (1<<14))

//*********clock DISable macros for GPIOx peripherals********
#define GPIOA_PCLK_DI()       (RCC->AHB1ENR &= ~(1<<0))
#define GPIOB_PCLK_DI()       (RCC->AHB1ENR &= ~(1<<1))
#define GPIOC_PCLK_DI()       (RCC->AHB1ENR &= ~(1<<2))
#define GPIOD_PCLK_DI()       (RCC->AHB1ENR &= ~(1<<3))
#define GPIOE_PCLK_DI()       (RCC->AHB1ENR &= ~(1<<4))
#define GPIOF_PCLK_DI()       (RCC->AHB1ENR &= ~(1<<5))
#define GPIOG_PCLK_DI()       (RCC->AHB1ENR &= ~(1<<6))
#define GPIOH_PCLK_DI()       (RCC->AHB1ENR &= ~(1<<7))

//********CLOCK DISABLE MACROS FOR I2CX PERIPHERALS***********
#define I2C1_PCLK_DI()        (RCC->APB1ENR &= ~(1<<21))
#define I2C2_PCLK_DI()        (RCC->APB1ENR &= ~(1<<22))
#define I2C3_PCLK_DI()        (RCC->APB1ENR &= ~(1<<23))

//********CLOCK DISABLE MACROS FOR SPIX PERIPHERALS***********
#define SPI1_PCLK_DI()        (RCC->APB2ENR &= ~(1<<12))
#define SPI2_PCLK_DI()        (RCC->APB1ENR &= ~(1<<14))
#define SPI3_PCLK_DI()        (RCC->APB1ENR &= ~(1<<15))
#define SPI4_PCLK_DI()        (RCC->APB2ENR &= ~(1<<13))

//********CLOCK DISABLE MACROS FOR USARTX PERIPHERALS***********
#define USART1_PCLK_DI()       (RCC->APB2ENR &= ~(1<<4))
#define USART2_PCLK_DI()       (RCC->APB1ENR &= ~(1<<17))
#define USART3_PCLK_DI()       (RCC->APB1ENR &= ~(1<<18))
#define UART4_PCLK_DI()        (RCC->APB1ENR &= ~(1<<19))
#define UART5_PCLK_DI()        (RCC->APB1ENR &= ~(1<<20))
#define USART6_PCLK_DI()       (RCC->APB2ENR &= ~(1<<5))


//********CLOCK DISABLE MACROS FOR SYSCFG PERIPHERALS***********
#define SYSCFG_PCLK_DI()       (RCC->APB2ENR &= ~(1<<14))

//********MACROS TO RESET GPIO FOR PERIPHERALS***********

//do....while....condn 0 loop===this is to execute 2 statements in one macro
#define GPIOA_REG_RESET()      do{(RCC->AHB1RSTR |= (1<<0)); (RCC->AHB1RSTR &= ~(1<<0));}while(0)//reset ans rem reset
#define GPIOB_REG_RESET()      do{(RCC->AHB1RSTR |= (1<<1)); (RCC->AHB1RSTR &= ~(1<<1));}while(0)//reset ans rem reset
#define GPIOC_REG_RESET()      do{(RCC->AHB1RSTR |= (1<<2)); (RCC->AHB1RSTR &= ~(1<<2));}while(0)//reset ans rem reset
#define GPIOD_REG_RESET()      do{(RCC->AHB1RSTR |= (1<<3)); (RCC->AHB1RSTR &= ~(1<<3));}while(0)//reset ans rem reset
#define GPIOE_REG_RESET()      do{(RCC->AHB1RSTR |= (1<<4)); (RCC->AHB1RSTR &= ~(1<<4));}while(0)//reset ans rem reset
#define GPIOF_REG_RESET()      do{(RCC->AHB1RSTR |= (1<<5)); (RCC->AHB1RSTR &= ~(1<<5));}while(0)//reset ans rem reset
#define GPIOG_REG_RESET()      do{(RCC->AHB1RSTR |= (1<<6)); (RCC->AHB1RSTR &= ~(1<<6));}while(0)//reset ans rem reset
#define GPIOH_REG_RESET()      do{(RCC->AHB1RSTR |= (1<<7)); (RCC->AHB1RSTR &= ~(1<<7));}while(0)//reset ans rem reset


//********MACROS TO RESET GPIO FOR PERIPHERALS***********

#define SPI1_REG_RESET()      do{(RCC->APB2RSTR |= (1<<12)); (RCC->APB2RSTR &= ~(1<<12));}while(0)//reset ans rem reset
#define SPI2_REG_RESET()      do{(RCC->APB1RSTR |= (1<<14)); (RCC->APB1RSTR &= ~(1<<14));}while(0)//reset ans rem reset
#define SPI3_REG_RESET()      do{(RCC->APB1RSTR |= (1<<15)); (RCC->APB1RSTR &= ~(1<<15));}while(0)//reset ans rem reset
#define SPI4_REG_RESET()      do{(RCC->APB2RSTR |= (1<<13)); (RCC->APB2RSTR &= ~(1<<13));}while(0)//reset ans rem reset


# define GPIO_BASEADDR_TO_CODE(x)   ((x==GPIOA)? 0 :\
		                             (x==GPIOB)? 1 :\
		                             (x==GPIOC)? 2 :\
		                             (x==GPIOD)? 3 :\
		                             (x==GPIOE)? 4 :\
		                             (x==GPIOF)? 5 :\
		                             (x==GPIOG)? 6 :\
		                             (x==GPIOH)? 7 :0 )

/*
 * IRQ number of mcu
 */

#define IRQ_NO_EXTI0     6
#define IRQ_NO_EXTI1     7
#define IRQ_NO_EXTI2     8
#define IRQ_NO_EXTI3     9
#define IRQ_NO_EXTI4     10
#define IRQ_NO_EXTI9_5   23
#define IRQ_NO_EXTI15_10 40
#define IRQ_NO_SPI1      35
#define IRQ_NO_SPI2      36
#define IRQ_NO_SPI3      51
#define IRQ_NO_SPI4      84
#define IRQ_NO_I2C1_EV   31
#define IRQ_NO_I2C1_EV   32
#define IRQ_NO_I2C2_EV   33
#define IRQ_NO_I2C2_EV   34
#define IRQ_NO_I2C3_EV   72
#define IRQ_NO_I2C3_EV   73


/*
 * MACROS FOR ALL THE PRIORITY LEVELS
 */

#define NVIC_IRQ_PRI0   0
#define NVIC_IRQ_PRI15  15

//some generic macros

#define ENABLE         1
#define DISABLE        0
#define SET            ENABLE
#define RESET          DISABLE
#define GPIO_PIN_SET   SET
#define GPIO_PIN_RESET RESET
#define FLAG_RESET     RESET
#define FLAG_SET       SET

/*
 *
 * BIT POSITION DEFINITIONS OF SPI PERIPHERALS
 */

#define SPI_CR1_CPHA       0
#define SPI_CR1_CPOL       1
#define SPI_CR1_MSTR       2
#define SPI_CR1_BR         3
#define SPI_CR1_SPE        6
#define SPI_CR1_LSBFIRST   7
#define SPI_CR1_SSI        8
#define SPI_CR1_SSM        9
#define SPI_CR1_RXONLY     10
#define SPI_CR1_DFF        11
#define SPI_CR1_CRCNEXT    12
#define SPI_CR1_CRCEN      13
#define SPI_CR1_BIDIOE     14
#define SPI_CR1_BIDIMODE   15

#define SPI_CR2_RXDMAEN       0
#define SPI_CR2_TXDMAEN       1
#define SPI_CR2_SSOE          2
#define SPI_CR2_FRF           4
#define SPI_CR2_ERRIE         5
#define SPI_CR2_RXNEIE        6
#define SPI_CR2_TXEIE         7


#define SPI_SR_RXNE         0
#define SPI_SR_TXE          1
#define SPI_SR_CHSIDE       2
#define SPI_SR_UDR          3
#define SPI_SR_CRCERR       4
#define SPI_SR_MODF         5
#define SPI_SR_OVR          6
#define SPI_SR_BSY          7
#define SPI_SR_FRE          8

/*
 *
 * BIT POSITION DEFINITIONS OF I2C PERIPHERALS
 */

#define I2C_CR1_PE          0
#define I2C_CR1_SMBUS       1
#define I2C_CR1_SMBTYPE     3
#define I2C_CR1_ENARP       4
#define I2C_CR1_ENPEC       5
#define I2C_CR1_ENGC        6
#define I2C_CR1_NOSTRECH    7
#define I2C_CR1_START       8
#define I2C_CR1_STOP        9
#define I2C_CR1_ACK         10
#define I2C_CR1_POS         11
#define I2C_CR1_PEC         12
#define I2C_CR1_ALERT       13
#define I2C_CR1_SWRST       15

#define I2C_CR2_FREQ          0
#define I2C_CR2_ITERREN       8
#define I2C_CR2_ITEVTEN       9
#define I2C_CR2_ITBUFEN       10
#define I2C_CR2_DMAEN         11
#define I2C_CR2_LAST          12

#define I2C_SR1_SB         0
#define I2C_SR1_ADDR       1
#define I2C_SR1_BTF        2
#define I2C_SR1_ADD10      3
#define I2C_SR1_STOPF      4
#define I2C_SR1_RXNE       6
#define I2C_SR1_TXE        7
#define I2C_SR1_BERR       8
#define I2C_SR1_ARLO       9
#define I2C_SR1_AF         10
#define I2C_SR1_OVR        11
#define I2C_SR1_PECERR     12
#define I2C_SR1_TIMEOUT    14
#define I2C_SR1_SMBALERT   15


#define I2C_SR2_MSL         0
#define I2C_SR2_BUSY        1
#define I2C_SR2_TRA         2
#define I2C_SR2_GENCALL     4
#define I2C_SR2_SMBDEF      5
#define I2C_SR2_SMBHOST     6
#define I2C_SR2_DUALF       7
#define I2C_SR2_PEC         8

#define I2C_CCR_CCR         0
#define I2C_CCR_DUTY        14
#define I2C_CCR_FS          15


#define I2C_TRISE_TRISE     0

//********PERIPHERAL REG DEFINITION FR GPIO*************

typedef struct {
	//ALL REG VERY VOLATILE
	//FOR EG idr changes data for every ahb1 cycle
	//so use volatile keyword
	volatile uint32_t MODER; //off add=0x00,,,add of  moder variable (&MODER) =(0x40020000+0x00)
	volatile uint32_t OTYPER; //off addr=0x04
	volatile uint32_t OSPEEDR; //off addr=0x08
	volatile uint32_t PUPDR; //off addr=0x0C
	volatile uint32_t IDR; //off addr=0x10
	volatile uint32_t ODR; //off addr=0x14
	volatile uint32_t BSRR; //off addr=0x18
	volatile uint32_t LCKR; //off addr=0x1C
	volatile uint32_t AFR[2]; //off addr=0x20-0X24....ARR OF 2 REG FORMES IE[2]

}GPIO_RegDef_t;

typedef struct{
	volatile uint32_t CR; //off add=0x00
	volatile uint32_t PLLCFGR; //off add=0x04
	volatile uint32_t CFGR; //off add=0x08
	volatile uint32_t CIR; //off add=0x0C
	volatile uint32_t AHB1RSTR; //off add=0x10
	volatile uint32_t AHB2RSTR; //off add=0x14
	volatile uint32_t AHB3RSTR; //off add=0x18
	volatile uint32_t RES0; //off add=0x1C
	volatile uint32_t APB1RSTR; //off add=0x20
	volatile uint32_t APB2RSTR; //off add=0x24
	volatile uint32_t RES1[2]; //off add=0x28-2C
	volatile uint32_t AHB1ENR; //off add=0x30
	volatile uint32_t AHB2ENR; //off add=0x34
	volatile uint32_t AHB3ENR; //off add=0x38
	volatile uint32_t RES2; ///off add=0x3C
	volatile uint32_t APB1ENR; //off add=0x40
	volatile uint32_t APB2ENR; //off add=0x44
	volatile uint32_t RES3[2]; //off add=0x48-4C
	volatile uint32_t AHB1LPENR; //off add=0x50
	volatile uint32_t AHB2LPENR; //off add=0x54
	volatile uint32_t AHB3LPENR; //off add=0x58
	volatile uint32_t RES4; //off add=0x5C
	volatile uint32_t APB1LPENR;//off add=0x60
	volatile uint32_t APB2LPENR; //off add=0x64
	volatile uint32_t RES5[2]; //off add=0x68-6C
	volatile uint32_t BDCR;//off add=0x70
	volatile uint32_t CSR; //off add=0x74
	volatile uint32_t RES6[2];//off add=0x78-7C
	volatile uint32_t SSCGR;//off add=0x80
	volatile uint32_t PLLI2SCFGR; //off add=0x84
	volatile uint32_t PLLSAICFGR; //off add=0x88
	volatile uint32_t DCKCFGR; //off add=0x8C
	volatile uint32_t CKCFGR; //off add=0x90
	volatile uint32_t DCKCFGR2; //off add=0x94
}RCC_RegDef_t;
//therefore,help programmer to easily access to various registers of peripheral
/*
 pGPIOA->MODER=25;//storing value 25 in moder register
 *(0x40020000+0x00)=25;//this is how compiler does

 */

//**********peripheral register definition for EXTI*********
typedef struct{
	volatile uint32_t IMR;  //off add=0x00
	volatile uint32_t EMR;//0x04
	volatile uint32_t RTSR;//0x08
	volatile uint32_t FTSR;//0x0C
	volatile uint32_t SWIER;//0x10
	volatile uint32_t PR;//0x14


}EXTI_RegDef_t;

typedef struct{
	volatile uint32_t MEMRMP;//0X00
	volatile uint32_t PMC;//0X04
	volatile uint32_t EXTICR[4];//0X08-0x14****each reg has 4 EXTI pins of 4 bit each
	uint32_t RES1[2];//0x14-0x20
	volatile uint32_t CMPCR;//0X20
	uint32_t RES2[2];//0x20-0x2C
	volatile uint32_t CFGR;//0x2C

}SYSCFG_RegDef_t;

//**********peripheral register definition for SPI*********
typedef struct{
	__vo uint32_t CR1;
	__vo uint32_t CR2;
	__vo uint32_t SR;
	__vo uint32_t DR;
	__vo uint32_t CRCPR;
	__vo uint32_t RXCRCR;
	__vo uint32_t TXCRCR;
	__vo uint32_t I2SCFGR;
	__vo uint32_t I2SPR;
}SPI_RegDef_t;

typedef struct{
	__vo uint32_t CR1;
	__vo uint32_t CR2;
	__vo uint32_t OAR1;
	__vo uint32_t OAR2;
	__vo uint32_t DR;
	__vo uint32_t SR1;
	__vo uint32_t SR2;
	__vo uint32_t CCR;
	__vo uint32_t TRISE;
	__vo uint32_t FLTR;

}I2C_RegDef_t;

#endif /* INC_STM32F446XX_H_ */
