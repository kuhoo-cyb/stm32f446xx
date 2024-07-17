/*
 * stm32f446xx_i2c_driver.c
 *
 *  Created on: Jul 9, 2024
 *      Author: cwc
 */


/*
 * Config struct for I2C peripheral
 */

#include "stm32f446xx.h"
#include "stm32f446xx_i2c_driver.h"

uint32_t RCC_GetPLLOutputClock(void){
	return;
}

uint16_t AHB_PreScaler[9]={2,4,8,16,32,64,128,256,512};
uint8_t APB1_PreScaler[4]={2,4,8,16};

static void I2C_GenerateStartCondn(I2C_RegDef_t *pI2Cx){
	pI2Cx->CR1 |= ( 1<< I2C_CR1_START);
}

static void I2C_GenerateStopCondn(I2C_RegDef_t *pI2Cx){
	pI2Cx->CR1 |= ( 1<< I2C_CR1_STOP);
}

uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx,uint32_t FlagName){

	if(pI2Cx->SR1 && FlagName){
		return FLAG_SET;
	}
	return FLAG_RESET;
}

static void I2C_ExecuteAddressPhasewrite(I2C_RegDef_t *pI2Cx,uint8_t SlaveAddr){
	SlaveAddr = SlaveAddr<<1;//as slave addr 7 bit and 1 r/wr bit so to make space for r/wr
	SlaveAddr &= ~(1);//to clear the 0th bit for r?wr
	pI2Cx->DR=SlaveAddr;
}

static void I2C_ExecuteAddressPhaseread(I2C_RegDef_t *pI2Cx,uint8_t SlaveAddr){
	SlaveAddr = SlaveAddr<<1;//as slave addr 7 bit and 1 r/wr bit so to make space for r/wr
	SlaveAddr |= (1);//to clear the 0th bit for r?wr
	pI2Cx->DR=SlaveAddr;
}

static void I2C_ClearADDRFlag(I2C_RegDef_t *pI2Cx){
	uint32_t dummyrd=pI2Cx->SR1;
	dummyrd=pI2Cx->SR2;
	(void)dummyrd;
}

static void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx){
	pI2Cx->CR1 |= (1<<I2C_CR1_STOP);
}
/*********************************************************************
 * @fn      		  - I2C_PeriClockControl
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -

 */
void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(pI2Cx == I2C1)
		{
			I2C1_PCLK_EN();
		}else if (pI2Cx == I2C2)
		{
			I2C2_PCLK_EN();
		}else if (pI2Cx == I2C3)
		{
			I2C3_PCLK_EN();
		}
	}
	else
	{
		if (pI2Cx == I2C1) {
			I2C1_PCLK_DI();
		} else if (pI2Cx == I2C2) {
			I2C2_PCLK_DI();
		} else if (pI2Cx == I2C3) {
			I2C3_PCLK_DI();
		}
	}

}




uint32_t RCC_GetPCLK1Value(void){
	uint32_t pclk1,Systemclk;

	uint8_t clksrc,temp,ahbp,apb1p;

	clksrc= ((RCC->CFGR >> 2) & 0x3);//move the src value to 0 and 1 position by >> and
	//then mask by 3 so that only 0 and 1 bit position we get where info lies and every bit is 0
	if(clksrc==0){
		Systemclk=16000000;
	}
	else if(clksrc==1){
		Systemclk=8000000;
	}
	else if(clksrc==2){
		Systemclk=RCC_GetPLLOutputClock();
	}

	// find val of ahb prescaler
	temp=((RCC->CFGR >> 4)& 0xF);

	if(temp<8){
		ahbp=1;
	}
	else{
		ahbp=AHB_PreScaler[temp-8];
	}

	//find apb1 prescaler rcc 10-11-12 in rcc cnfg
	temp = ((RCC->CFGR >> 10) & 0x7);

	if (temp < 4) {
		apb1p = 1;
	} else {
		ahbp=APB1_PreScaler[temp - 4];
	}

	pclk1=(Systemclk/ahbp)/apb1p;
	return pclk1;
}

/*********************************************************************
 * @fn      		  - I2C_PeriClockControl
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -

 */

void I2C_Init(I2C_Handle_t *pI2CHandle){
	//1.configure the mode(std or fast)

	//2.configure the speed of the sclk

	//3.configure device address if its in slave mode

	//4.enable the acking

	//5.configure the rise time for i2c pins(slew rate)
	//also all the config above must be done whrn the peripheral is disabled in the control reg
	//enable peripheral after these configurations

	I2C_PeripheralControl(pI2CHandle->pI2Cx,ENABLE);

	uint32_t tempreg=0;

	//if PE=0 cant make ack=1
	//so do PE=1
	tempreg |= pI2CHandle->I2C_Config.I2C_AckControl << I2C_CR1_ACK;
	pI2CHandle->pI2Cx->CR1=tempreg;

	//calculate apb1 clock freq
	tempreg=0;
	tempreg |= RCC_GetPCLK1Value()/1000000U;//only want no
	pI2CHandle->pI2Cx->CR2= (tempreg & 0x3F);

	//store slave addr ie program device own addr
	tempreg=pI2CHandle->I2C_Config.I2C_DeviceAddress <<1;
	tempreg |= (1<<14);//always 1 as stated in RM
	pI2CHandle->pI2Cx->OAR1=tempreg;

	//CCR calculation
	uint16_t ccr_val=0;
	tempreg=0;
	if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM){
		//mode is std mode
		//in std mode T(high)=T(low)
		//in SM T(high->scl)=CCR*T(pclk1) and T(low->scl)=CCR*T(pclk1)
		//so T(scl=high+low)=2*CCR*T(pclk1)
		//so CCR=T(scl)/(2*T(pclk1)) or F(pclk1)/(2*F(scl))

		ccr_val=(RCC_GetPCLK1Value()/(2* pI2CHandle->I2C_Config.I2C_SCLSpeed));
		tempreg|=(ccr_val & 0xFFF);

	}else{
		//fm
		tempreg|=(1<<15);//f?s mode to be set 1 for fm
		tempreg |= (pI2CHandle->I2C_Config.I2C_FMDutyCycle >>14);//duty cycle decided 1 or 0
		//if duty cycle=0-> CCR=F(pclk1)/(3*F(scl))
		//if duty cycle=1-> CCR=F(pclk1)/(25*F(scl))
		if(pI2CHandle->I2C_Config.I2C_FMDutyCycle == I2C_FM_DUTY_2){
			ccr_val=(RCC_GetPCLK1Value()/(3* pI2CHandle->I2C_Config.I2C_SCLSpeed));
		}else{
			ccr_val=(RCC_GetPCLK1Value()/(25* pI2CHandle->I2C_Config.I2C_SCLSpeed));
		}

		tempreg|=(ccr_val & 0xFFF);

	}
	pI2CHandle->pI2Cx->CCR=tempreg;//ccr mei tempreg daaldo

	//trise configuration
	//divide rise time by the time period of the PCLK1 +1
	// trise = ( FPCLK1 * Trise(max) )+1
	if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM){
		//sm
		tempreg=(RCC_GetPCLK1Value()/1000000U)+1;// FPCLK1* 1000ns(max trise)= /1000000

	}
	else{
		//fm
		tempreg=((RCC_GetPCLK1Value()*300)/1000000000U)+1;//300 ns is max rise time / ins
	}
	pI2CHandle->pI2Cx->TRISE=(tempreg & 0x3F);

}

/*********************************************************************
 * @fn      		  - I2C_PeriClockControl
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -

 */

void I2C_DeInit(I2C_RegDef_t *pI2Cx);



/*********************************************************************
 * @fn      		  - I2C_PeriClockControl
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -

 */



void I2C_MasterSendData(I2C_Handle_t *pI2CHandle,uint8_t *pTxbuffer,uint32_t Len,uint8_t SlaveAddr,uint8_t Sr){
	//1.generate start condition
	I2C_GenerateStartCondn(pI2CHandle->pI2Cx);

	//2.confirm that start generation is completed by checking the SB flag in the SR1
	// note : until SB is 0 SCL will be stretched(pulled to LOW)
	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_SB));
	//as when sb=1 we have to get out of the loop as start is generated

	//3. send address of slave with r/wr bit set to 0 (tot 8 bits)
	I2C_ExecuteAddressPhasewrite(pI2CHandle->pI2Cx,SlaveAddr);

	//4.check if addr phase is complete by checking the ADDR flag in the SR1
	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_ADDR));

	//5.clear the address flag acc to its software sequence as this bit is cleared by
	//software reading SR1 reg followed by reading SR2
	I2C_ClearADDRFlag(pI2CHandle->pI2Cx);

	//6. send data until length becomes 0
	while(Len>1){
		while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_TXE));//wait till txe is set
		//as if dr is emty then only push the data in it
		pI2CHandle->pI2Cx->DR= *pTxbuffer;
		pTxbuffer++;
		Len--;
	}

	//7.when len=0 wait for TXE=1 and BTF=1 before generating the STOP condn
	//as TXE=1 and BTF=1 means SR and DR are both empty and next txn shoul begin
	//when BTF=1 SCL will be stretched (pulled to low)
	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_TXE));
	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_BTF));

	//8. Generate StOP condn and master need to wait for the completion of STOP condn
	//and also generating STOP cndn automatically clear the BTF
	if(Sr==I2C_DISABLE_SR){
		I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

	}

}



void I2C_ManageAcking(I2C_RegDef_t *pI2Cx,uint8_t EnOrDi){
	if(EnOrDi==I2C_ACK_ENABLE){
		pI2Cx->CR1 |= (1<<I2C_CR1_ACK);
	}else{
		pI2Cx->CR1 &= ~(1<<I2C_CR1_ACK);

	}

}


/*********************************************************************
 * @fn      		  - I2C_PeriClockControl
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -

 */

// if last byte to be read
//addr=1 -- ack=0,stop=1 -- addr =0 then data is read and txn stopped and we can read when rxne=1
// if more bytes to be read first master recieves and reads data if rxne=1
//and then another byte is recieved or data is transfrred parallely iff rxne=1
//when length=2 ack=0 -- stop=1 -- read DR
//so for last data byte nack send to slave and data receieved and read

void I2C_MasterRecieveData(I2C_Handle_t *pI2CHandle,uint8_t *pRxbuffer,uint32_t Len,uint8_t SlaveAddr,uint8_t Sr){

	//1.Generate the START condn
	I2C_GenerateStartCondn(pI2CHandle->pI2Cx);

	//2.confirm the start generation is complete by checking the SB flag in SR1
	// alssooo until the SB is cleared SCL will be streched ie pulled to LOW

	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_SB));

	//3.send the address of the slave with R/w bit set to R(1)-tot 8 bits

	I2C_ExecuteAddressPhaseread(pI2CHandle->pI2Cx,SlaveAddr);

	//4.wait until the address phase is complete by checking ADDR flag in SR1

	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_ADDR));

	//to read only one byte from slave
	if(Len==1){

		//Disable Acking
		I2C_ManageAcking(pI2CHandle->pI2Cx,I2C_ACK_DISABLE);

		//Clear the ADDr flag

		I2C_ClearADDRFlag(pI2CHandle->pI2Cx);

		//wait till RXNE becomes 1

		while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_RXNE));

		//generate STOP condn

		if (Sr == I2C_DISABLE_SR) {
			I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

		}
		//read data in the buffer

		*pRxbuffer=pI2CHandle->pI2Cx->DR;

	}

	if(Len>1){

		//clear the ADDR flag
		I2C_ClearADDRFlag(pI2CHandle->pI2Cx);

		//read the data until Len becomed 0
		for(uint32_t i=Len;i>0;i--){
			//wait until Rxne become 1

			while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_RXNE));

			if(i==2){

				//clear the ack bit
				I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_DISABLE);

				//generate STOP condn

				if (Sr == I2C_DISABLE_SR) {
					I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

				}
			}

			//read data from register in to buffer
			*pRxbuffer=pI2CHandle->pI2Cx->DR;


			//increment the buffer address

			pRxbuffer++;
		}

	}
	//re enable acking as when enter this API acking was enable
	if(pI2CHandle->I2C_Config.I2C_AckControl==I2C_ACK_ENABLE){

		I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_DISABLE);
	}

}

/*********************************************************************
 * @fn      		  - I2C_PeriClockControl
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -

 */

uint8_t  I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle,uint8_t *pTxBuffer,uint32_t Len,uint8_t SlaveAddr,uint8_t Sr){

	uint8_t busystate = pI2CHandle->TxRxState;

	if( (busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX))
	{
		pI2CHandle->pTxBuffer = pTxBuffer;
		pI2CHandle->TxLen = Len;
		pI2CHandle->TxRxState = I2C_BUSY_IN_TX;
		pI2CHandle->DevAddr = SlaveAddr;
		pI2CHandle->Sr = Sr;

		//Implement code to Generate START Condition

		I2C_GenerateStartCondition(pI2CHandle->pI2Cx);
		//after this SB interrupt will trigger
		//so I2C_CR2_ITEVTEN is enabled beloww

		//Implement the code to enable ITBUFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITBUFEN);

		//Implement the code to enable ITEVFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITEVTEN);

		//Implement the code to enable ITERREN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITERREN);

	}

	return busystate;

}


/*********************************************************************
 * @fn      		  - I2C_PeriClockControl
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -

 */

uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle,uint8_t *pRxBuffer,uint32_t Len, uint8_t SlaveAddr,uint8_t Sr){

	uint8_t busystate = pI2CHandle->TxRxState;

	if( (busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX))
	{
		pI2CHandle->pRxBuffer = pRxBuffer;
		pI2CHandle->RxLen = Len;
		pI2CHandle->TxRxState = I2C_BUSY_IN_RX;
		pI2CHandle->RxSize = Len; //Rxsize is used in the ISR code to manage the data reception
		pI2CHandle->DevAddr = SlaveAddr;
		pI2CHandle->Sr = Sr;

		//Implement code to Generate START Condition
		I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

		//Implement the code to enable ITBUFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITBUFEN);

		//Implement the code to enable ITEVFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITEVTEN);

		//Implement the code to enable ITERREN Control Bit
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITERREN);

	}

	return busystate;
}

/*********************************************************************
 * @fn      		  - I2C_PeriClockControl
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -

 */
//void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi){


//}
//void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);

/*********************************************************************
 * @fn      		  - I2C_PeriClockControl
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -

 */
