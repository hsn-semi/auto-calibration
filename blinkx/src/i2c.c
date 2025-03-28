/******************************************************/
/***  Project:       AraController                  ***/
/***  File:          i2c.c                          ***/
/***  Created on:    22.07.2016                     ***/
/***  Author:        Andreas Stärker                ***/
/***  Company:       Blink AG                       ***/
/***  Changes:       06.09.2019: Bugfix ReadNack    ***/
/***  Changes:       01.02.2024: ClockSpeed changed ***/
/******************************************************/



#include <stdio.h>
#include <string.h>
#include <stdlib.h>

//#include "stm32f4_discovery.h"
#include "stm32f4xx_conf.h"
//#include "stm32f4xx_tim.h"
//#include "stm32f4xx_rcc.h"
#include "stm32f4xx.h"

#include "stm32f4xx_usart.h"

#include "defines.h"
#include "main.h"
#include "i2c.h"



uint32_t I2cTimeoutCount = 0;

char strDebugI2c[60];
char strTempI2c[20];
uint32_t temp32 = 0;

#define I2C_MAX_JOBS    10

TypeI2cJob I2CJobList[I2C_MAX_JOBS + 1];
uint8_t I2cJobIn = 0;
uint8_t I2cJobOut = 0;

uint8_t I2cDebugOutput = 0;


//variables:




// ********************************************** //
// ***       Function: I2cInit                *** //
// ***       Purpose: init I2C                *** //
// ***       Input: I2C_TypeDef* I2Cx         *** //
// ***       Return: none                     *** //
// ********************************************** //
void I2cInit(I2C_TypeDef* I2Cx)
{

	  GPIO_InitTypeDef 		GPIO_InitStructure;
	  I2C_InitTypeDef 	    I2C_InitStructure;
	  //NVIC_InitTypeDef 		NVIC_InitStructure;

	  GPIO_StructInit(&GPIO_InitStructure);
	  I2C_StructInit(&I2C_InitStructure);



	  if(I2Cx == I2C1)
	  {
		  	  /* I2C clock enable */
		  	  RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
		  	  /* GPIOB clock enable */
		  	  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
		  	  /* Reset I2Cx IP */
			  RCC_APB1PeriphResetCmd(RCC_APB1Periph_I2C1, ENABLE);
			  /* Release reset signal of I2Cx IP */
			  RCC_APB1PeriphResetCmd(RCC_APB1Periph_I2C1, DISABLE);
			  /* GPIOB Configuration: I2C1 SCL and SDA */
			  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
			  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
			  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
			  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
			  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
			  GPIO_Init(GPIOB, &GPIO_InitStructure);
			  /* connect pins to alternate function I2C1 */
			  GPIO_PinAFConfig(GPIOB, GPIO_PinSource8, GPIO_AF_I2C1);
			  GPIO_PinAFConfig(GPIOB, GPIO_PinSource9, GPIO_AF_I2C1);

			  I2C_InitStructure.I2C_ClockSpeed = 1000;
	  }
	  if(I2Cx == I2C2)
	  {
				/* I2C clock enable */
			  RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);
			  /* GPIOF clock enable */
			  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
			  /* Reset I2Cx IP */
			  RCC_APB1PeriphResetCmd(RCC_APB1Periph_I2C2, ENABLE);
			  /* Release reset signal of I2Cx IP */
			  RCC_APB1PeriphResetCmd(RCC_APB1Periph_I2C2, DISABLE);
			  /* GPIOF Configuration: I2C2 SCL and SDA */
			  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
			  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
			  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
			  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
			  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
			  GPIO_Init(GPIOB, &GPIO_InitStructure);
			  /* connect Therm-I2C-pins to alternate function I2C2 */
			  GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_I2C2);
			  GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_I2C2);

			  I2C_InitStructure.I2C_ClockSpeed = 1000;
	  }
	  if(I2Cx == I2C3)
	  {
				/* I2C clock enable */
			  RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C3, ENABLE);
			  /* GPIOF clock enable */
			  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOC, ENABLE);
			  /* Reset I2Cx IP */
			  RCC_APB1PeriphResetCmd(RCC_APB1Periph_I2C3, ENABLE);
			  /* Release reset signal of I2Cx IP */
			  RCC_APB1PeriphResetCmd(RCC_APB1Periph_I2C3, DISABLE);
			  /* GPIOF Configuration: I2C3 SCL */
			  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
			  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
			  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
			  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
			  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
			  GPIO_Init(GPIOA, &GPIO_InitStructure);
			  /* GPIOF Configuration: I2C3 SDA */
			  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
			  GPIO_Init(GPIOC, &GPIO_InitStructure);
			  /* connect Therm-I2C-pins to alternate function I2C2 */
			  GPIO_PinAFConfig(GPIOA, GPIO_PinSource8, GPIO_AF_I2C3);
			  GPIO_PinAFConfig(GPIOC, GPIO_PinSource9, GPIO_AF_I2C3);

			  /* slower speed Battery */
			  I2C_InitStructure.I2C_ClockSpeed = 1000;
	  }


	  I2C_DeInit(I2Cx);

	  I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
	  I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
	  I2C_InitStructure.I2C_OwnAddress1 = 0;
	  I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
	  I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;

	  I2C_Init(I2Cx, &I2C_InitStructure);

	  /* Enable the I2Cx global Interrupts */
	  I2C_ITConfig(I2Cx, I2C_IT_EVT | I2C_IT_BUF, DISABLE);


	  /* Enable the I2C global Interrupt
	  NVIC_InitStructure.NVIC_IRQChannel = I2C_IRQn;
	  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 7;
	  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	  NVIC_Init(&NVIC_InitStructure);
	  */

	  /* Enable the I2Cx */
	  I2C_Cmd(I2Cx, ENABLE);

	  memset(I2CJobList, 0, sizeof(I2CJobList));




}

// *************************************************************************** //
// ***       Function: I2cStart                                            *** //
// ***       Purpose: generates start and sends address                    *** //
// ***       Input: I2Cx - I2C-Interface                                   *** //
// ***       Input: address - slave-address 8bit (bit 0 set automatically) *** //
// ***       Input: direction -                                            *** //
// ***       Return: value of I2C_ERROR                                    *** //
// *************************************************************************** //
uint8_t I2cStart(I2C_TypeDef* I2Cx, uint8_t address, uint8_t direction)
{
	/* wait until I2Cx is not busy any more */
	I2cTimeoutCount = 0;
	while(I2C_GetFlagStatus(I2Cx, I2C_FLAG_BUSY))
	{
			I2cTimeoutCount++;
			if(I2cTimeoutCount > I2C_TIMEOUT)
			{
				//strcpy(strDebugI2c, "I2cErrorBusy");
				//SendToCommunicationPort(strDebugI2c, strlen(strDebugI2c));
				//break;
				return I2C_ERROR_BUSY;
			}
	}

	/* Send I2C1 START condition */
	I2C_GenerateSTART(I2Cx, ENABLE);

	/* wait for successful transmission of Start condition */
	I2cTimeoutCount = 0;
	while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT))
	{
		I2cTimeoutCount++;
		if(I2cTimeoutCount > I2C_TIMEOUT)
		{
			//strcpy(strDebugI2c, "I2cErrorMastermodeselect");
			//SendToCommunicationPort(strDebugI2c, strlen(strDebugI2c));
			//break;
			return I2C_ERROR_MASTER_MODE_SELECT;
		}
	}

	/* Send slave Address for write */
	I2C_Send7bitAddress(I2Cx, address, direction);

	/* wait for I2Cx EV6, check if
	 * either Slave has acknowledged Master transmitter or
	 * Master receiver mode, depending on the transmission
	 * direction
	 */
	I2cTimeoutCount = 0;
	if(direction == I2C_Direction_Transmitter)
	{
		while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
		{
				I2cTimeoutCount++;
				if(I2cTimeoutCount > I2C_TIMEOUT)
				{
					//strcpy(strDebugI2c, "I2cErrorMasterTxModeSelected");
					//SendToCommunicationPort(strDebugI2c, strlen(strDebugI2c));
					//break;
					return I2C_ERROR_MASTER_TX_MODE_SELECT;
				}
		}
	}
	else if(direction == I2C_Direction_Receiver)
	{
		while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED))
		{
				I2cTimeoutCount++;
				if(I2cTimeoutCount > I2C_TIMEOUT)
				{
					//strcpy(strDebugI2c, "I2cErrorMasterRxModeSelect");
					//SendToCommunicationPort(strDebugI2c, strlen(strDebugI2c));
					//break;
					return I2C_ERROR_MASTER_RX_MODE_SELECT;
				}
		}
	}

	return I2C_NO_ERROR;

}

// ************************************************************* //
// ***       Function: I2cRestart                            *** //
// ***       Purpose: generates Restart (reading followed)   *** //
// ***       Input: I2Cx - I2C-Interface                     *** //
// ***       Input: address - 7-bit-slave address            *** //
// ***       Input: direction -                              *** //
// ***       Return: value of I2C_ERROR                      *** //
// ************************************************************* //
uint8_t I2cRestart(I2C_TypeDef * I2Cx, uint8_t address, uint8_t direction)
{
	I2cTimeoutCount = 0;
	while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTING))
	{
		I2cTimeoutCount++;
		if(I2cTimeoutCount > I2C_TIMEOUT)
		{
			//strcpy(strDebugI2c, "I2cErrorMasterByteTransmitting");
			//SendToCommunicationPort(strDebugI2c, strlen(strDebugI2c));
			//break;
			return I2C_ERROR_MASTER_BYTE_TRANSMIT;
		}
	}


    /* Send I2C1 START condition */
    I2C_GenerateSTART(I2Cx, ENABLE);

    //while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

    I2cTimeoutCount = 0;
    while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT))
    {
    		I2cTimeoutCount++;
    		if(I2cTimeoutCount > I2C_TIMEOUT)
    		{
    			//strcpy(strDebugI2c, "I2cErrorMasterModeSelected");
    			//SendToCommunicationPort(strDebugI2c, strlen(strDebugI2c));
    			//break;
    			return I2C_ERROR_MASTER_MODE_SELECT;
    		}
	}

	// Send slave Address
	I2C_Send7bitAddress(I2Cx, address, direction);

	/* wait for I2Cx EV6, check if
	 * either Slave has acknowledged Master transmitter or
	 * Master receiver mode, depending on the transmission
	 * direction
	 */
	I2cTimeoutCount = 0;
	if(direction == I2C_Direction_Transmitter)
	{
		while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
		{
				I2cTimeoutCount++;
				if(I2cTimeoutCount > I2C_TIMEOUT)
				{
					//strcpy(strDebugI2c, "I2cErrorMasterTxModeSelected");
					//SendToCommunicationPort(strDebugI2c, strlen(strDebugI2c));
					//break;
					return I2C_ERROR_MASTER_TX_MODE_SELECT;
				}
		}
	}
	else if(direction == I2C_Direction_Receiver)
	{
		while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED))
		{
				I2cTimeoutCount++;
				if(I2cTimeoutCount > I2C_TIMEOUT)
				{
					//strcpy(strDebugI2c, "I2cErrorMasterRxModeSelect");
					//SendToCommunicationPort(strDebugI2c, strlen(strDebugI2c));
					//break;
					return I2C_ERROR_MASTER_RX_MODE_SELECT;
				}
		}
	}

	return I2C_NO_ERROR;

}

// ************************************************************* //
// ***       Function: I2cWrite                              *** //
// ***       Purpose: wartet bis zum senden des letzten Bytes und schreibt data in das Ausgangs-Shift Register   *** //
// ***       Input: I2Cx - I2C-Interface                     *** //
// ***       Input: data -    Data Byte to send              *** //
// ***       Return: value of I2C_ERROR                      *** //
// ************************************************************* //
uint8_t I2cWrite(I2C_TypeDef* I2Cx, uint8_t data)
{
		/* wait for I2C1 EV8 --> last byte is still being transmitted (last byte in SR, buffer empty), next byte can already be written */
		I2cTimeoutCount = 0;
		while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTING))
		{
				I2cTimeoutCount++;
				if(I2cTimeoutCount > I2C_TIMEOUT)
				{
					//strcpy(strDebugI2c, "I2cErrorWriteMasterByteTransmitting");
					//SendToCommunicationPort(strDebugI2c, strlen(strDebugI2c));
					//break;
					return I2C_ERROR_MASTER_BYTE_TRANSMIT;
				}
		}

		I2C_SendData(I2Cx, data);

		return I2C_NO_ERROR;
}

// ******************************************************************** //
// ***       Function: I2cReadAck                                   *** //
// ***       Purpose: liest ein Byte vom Bus und sendet ein ack,    *** //
// ***            beendet damit also nicht das Empfangen            *** //
// ***       Input: I2Cx - I2C-Interface                            *** //
// ***       Return: received Byte                                  *** //
// ******************************************************************** //
uint8_t I2cReadAck(I2C_TypeDef* I2Cx)
{
		/* enable acknowledge of received data */
		I2C_AcknowledgeConfig(I2Cx, ENABLE);

		/* wait until one byte has been received */
		I2cTimeoutCount = 0;
		while( !I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED) )
		{
				I2cTimeoutCount++;
				if(I2cTimeoutCount > I2C_TIMEOUT)
				{
					if(I2cDebugOutput > 0)
					{
						strcpy(strDebugI2c, "I2cErrorReadAckMasterByteReceived");
						SendToCommunicationPort(strDebugI2c, strlen(strDebugI2c));
					}
					break;

				}
		}

		/* read data from I2C data register and return data byte */
		uint8_t data = I2C_ReceiveData(I2Cx);
		return data;
}

// ***************************************************************** //
// ***       Function: I2cReadNack                               *** //
// ***       Purpose: liest ein Byte vom Bus und sendet Stop,    *** //
// ***           beendet damit also nicht das Empfangen          *** //
// ***       Input: I2Cx - I2C-Interface                         *** //
// ***       Return: received Byte                               *** //
// ***************************************************************** //
uint8_t I2cReadNack(I2C_TypeDef* I2Cx)
{

		// see reference manual for more info
		/* disable acknowledge of received data */
		I2C_AcknowledgeConfig(I2Cx, DISABLE);



		//debug
		//PowerLedBlue(ON);

		/* wait until one byte has been received */
		I2cTimeoutCount = 0;
		while( !I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED) )
		{
				I2cTimeoutCount++;
				if(I2cTimeoutCount > I2C_TIMEOUT)
				{
					if(I2cDebugOutput > 0)
					{
						temp32 = I2Cx->SR2;
						temp32 = temp32 << 16;
						temp32 += I2Cx->SR1;
						itoa(temp32, strTempI2c, 16);
						strcpy(strDebugI2c, "I2cErrorReadNackMasterByteReceived state 0x");
						strcat(strDebugI2c, strTempI2c);
						SendToCommunicationPort(strDebugI2c, strlen(strDebugI2c));
					}
					break;
				}
		}

		//debug
		//PowerLedBlue(OFF);

		/* read data from I2C data register and return data byte */
		uint8_t data = I2C_ReceiveData(I2Cx);

		/* generate stop condition after last byte received */
		I2C_GenerateSTOP(I2Cx, ENABLE);


		return data;



}

// *********************************************************************** //
// ***       Function: I2cStop                                         *** //
// ***       Purpose: wartet auf das Senden des letzten Bytes          *** //
// ***         und erzeugt dann die Stop-Condition, beendet damit      *** //
// ***         einen Schreibzyklus und gibt den Bus frei               *** //
// ***       Input: I2Cx - I2C-Interface                               *** //
// ***       Return: value of I2C_ERROR                                *** //
// *********************************************************************** //
uint8_t I2cStop(I2C_TypeDef* I2Cx)
{
		I2cTimeoutCount = 0;
		while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
		{
				I2cTimeoutCount++;
				if(I2cTimeoutCount > I2C_TIMEOUT)
				{
					//strcpy(strDebugI2c, "I2cErrorStopMasterByteTransmitted");
					//SendToCommunicationPort(strDebugI2c, strlen(strDebugI2c));
					//break;
					return I2C_ERROR_MASTER_BYTE_TRANSMIT;
				}
		}

		/* Send I2C1 STOP Condition after last byte has been transmitted */
		I2C_GenerateSTOP(I2Cx, ENABLE);

		/* wait for I2C1 EV8_2 --> byte has been transmitted*/
		I2cTimeoutCount = 0;
		while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
		{
				I2cTimeoutCount++;
				if(I2cTimeoutCount > I2C_TIMEOUT)
				{
					//temp32 = I2Cx->SR2;
					//temp32 = temp32 << 16;
					//temp32 += I2Cx->SR1;
					//itoa(temp32, strTempI2c, 16);
					//strcpy(strDebugI2c, "I2cErrorAfterStopMasterByteTransmitted state 0x");
					//strcat(strDebugI2c, strTempI2c);
					//SendToCommunicationPort(strDebugI2c, strlen(strDebugI2c));
					//break;
					return I2C_ERROR_MASTER_BYTE_TRANSMIT;
				}
		}


		return I2C_NO_ERROR;

}

// ************************************************************* //
// ***       Function:IsI2cAddressPresent                    *** //
// ***       Purpose: returns ADC-Value                      *** //
// ***       Input: I2Cx - I2C-Interface                     *** //
// ***              adr - 7-bit-slave address                *** //
// ***       Return: 0 - not present, 1 - present            *** //
// ************************************************************* //
uint8_t IsI2cAddressPresent(I2C_TypeDef* I2Cx, uint8_t adr)
{
	uint8_t u;

	u = I2cStart(I2Cx, adr, I2C_Direction_Receiver);
	/* end the conversation */
	I2cReadNack(I2Cx);
	I2cStop(I2Cx);
	//I2cReadNack(I2Cx);
	//I2C_SoftwareResetCmd(I2Cx, ENABLE);
	//I2C_SoftwareResetCmd(I2Cx, DISABLE);

	if(u != I2C_NO_ERROR)
	{
		return 0;
	}

	return 1;

}

// ************************************************************ //
// ***       Function:I2cTMP275Config                       *** //
// ***       Purpose: checks, if LM3509 is present          *** //
// ***       Input: resolution (0 ... 3)                    *** //
// ***              0 - 0.5°C; conv. time = 27.5ms          *** //
// ***              0 - 0.25°C; conv. time = 55ms           *** //
// ***              0 - 0.15°C; conv. time = 110ms          *** //
// ***              0 - 0.0625°C; conv. time = 220ms        *** //
// ***       Return: none                                   *** //
// ************************************************************ //
void I2cTMP275Config(uint8_t resolution)
{
	uint8_t dat;

	//LedRed(ON);

	//limit input values
	dat = resolution << 5;


	I2cStart(I2C1, I2C_ADDRESS_TMP275, I2C_Direction_Transmitter);
	I2cWrite(I2C1, 0x01);// 0x01: config register
	I2cWrite(I2C1, dat);//
	I2cStop(I2C1);

	//debug
	//LedRed(OFF);

	return;



}

// ************************************************************ //
// ***       Function:I2cTMP275Config                       *** //
// ***       Purpose: sets pointer to temperature register  *** //
// ***       Input: none                                    *** //
// ***       Return: none                                   *** //
// ************************************************************ //
void I2cTMP275SetTemperatureRegister(void)
{
	uint8_t dat;

	//LedRed(ON);



	I2cStart(I2C1, I2C_ADDRESS_TMP275, I2C_Direction_Transmitter);
	I2cWrite(I2C1, 0x00);// 0x00: temperature register
	I2cStop(I2C1);

	//debug
	//LedRed(OFF);

	return;



}

// ************************************************************* //
// ***       Function:I2cReadTMP275                          *** //
// ***       Purpose: returns temperature                    *** //
// ***       Input: none                                     *** //
// ***       Return: Temperature in 1/1000°C                 *** //
// ************************************************************* //
int32_t I2cReadTMP275(void)
{

	uint8_t temp8[2];
	uint16_t dat;
	int32_t temp;

	//debug
	//LedRed(ON);

	I2cStart(I2C1, I2C_ADDRESS_TMP275, I2C_Direction_Receiver);
	temp8[0] = I2cReadAck(I2C1);
	temp8[1] = I2cReadNack(I2C1);
	//temp8[1] = I2cReadAck(I2C1);
	//I2cStop(I2C1);

	dat = temp8[0] << 8;
	dat |= temp8[1];
	dat = dat >> 4;//temperature in 0.0625°C steps

	temp = dat * 625;
	temp = temp / 10;//temperature in 1/1000°C

	//debug
	//LedRed(OFF);

	return temp;


}

// ************************************************************* //
// ***       Function:I2cReadEepromByte                      *** //
// ***       Purpose: reads one Byte from EEPROM             *** //
// ***       Input: address - address in EEPROM-Memory       *** //
// ***       Return: Data byte                               *** //
// ************************************************************* //
uint8_t I2cReadEepromByte(uint16_t address)
{

	uint8_t data;

	//debug
	//LedYellow(ON);

	I2cStart(I2C1, I2C_ADDRESS_24LC256, I2C_Direction_Transmitter);
	I2cWrite(I2C1, (uint8_t)(address >> 8));
	I2cWrite(I2C1, (uint8_t)(address & 0xFF));
	I2cRestart(I2C1, I2C_ADDRESS_24LC256, I2C_Direction_Receiver);
	data = I2cReadNack(I2C1);

	//debug
	//LedYellow(OFF);

	return data;

}

// ************************************************************* //
// ***       Function:I2cReadEepromExtByte                   *** //
// ***       Purpose: reads one Byte from external EEPROM    *** //
// ***       Input: address - address in EEPROM-Memory       *** //
// ***       Return: Data byte                               *** //
// ************************************************************* //
uint8_t I2cReadEepromExtByte(uint16_t address)
{

	uint8_t data;

	//debug
	//LedYellow(ON);

	I2cStart(I2C1, I2C_ADDRESS_24LC256_EXT, I2C_Direction_Transmitter);
	I2cWrite(I2C1, (uint8_t)(address >> 8));
	I2cWrite(I2C1, (uint8_t)(address & 0xFF));
	I2cRestart(I2C1, I2C_ADDRESS_24LC256_EXT, I2C_Direction_Receiver);
	data = I2cReadNack(I2C1);

	//debug
	//LedYellow(OFF);

	return data;

}

// ************************************************************* //
// ***       Function:I2cReadEepromInteger                   *** //
// ***       Purpose: reads Integer from EEPROM              *** //
// ***       Input: address - start address in EEPROM-Memory *** //
// ***       Return: Data byte                               *** //
// ************************************************************* //
int32_t I2cReadEepromInteger(uint16_t address)
{

	int32_t data;

	//debug
	//LedYellow(ON);

	I2cStart(I2C1, I2C_ADDRESS_24LC256, I2C_Direction_Transmitter);
	I2cWrite(I2C1, (uint8_t)(address >> 8));
	I2cWrite(I2C1, (uint8_t)(address & 0xFF));
	I2cRestart(I2C1, I2C_ADDRESS_24LC256, I2C_Direction_Receiver);
	data = I2cReadAck(I2C1);
	data = data << 8;
	data = data | I2cReadAck(I2C1);
	data = data << 8;
	data = data | I2cReadAck(I2C1);
	data = data << 8;
	data = data | I2cReadNack(I2C1);

	//debug
	//LedYellow(OFF);

	return data;

}

// ************************************************************* //
// ***       Function:I2cReadEepromExtInteger                *** //
// ***       Purpose: reads Integer from external EEPROM     *** //
// ***       Input: address - start address in EEPROM-Memory *** //
// ***       Return: Data byte                               *** //
// ************************************************************* //
int32_t I2cReadEepromExtInteger(uint16_t address)
{

	int32_t data;

	//debug
	//LedYellow(ON);

	I2cStart(I2C1, I2C_ADDRESS_24LC256_EXT, I2C_Direction_Transmitter);
	I2cWrite(I2C1, (uint8_t)(address >> 8));
	I2cWrite(I2C1, (uint8_t)(address & 0xFF));
	I2cRestart(I2C1, I2C_ADDRESS_24LC256_EXT, I2C_Direction_Receiver);
	data = I2cReadAck(I2C1);
	data = data << 8;
	data = data | I2cReadAck(I2C1);
	data = data << 8;
	data = data | I2cReadAck(I2C1);
	data = data << 8;
	data = data | I2cReadNack(I2C1);

	//debug
	//LedYellow(OFF);

	return data;

}

// ************************************************************* //
// ***       Function:I2cReadEepromAscii                     *** //
// ***       Purpose: reads Integer from EEPROM              *** //
// ***       Input: address - start address in EEPROM-Memory *** //
// ***       Return: Data byte                               *** //
// ************************************************************* //
void I2cReadEepromAscii(uint16_t address, uint8_t len, uint8_t * buf)
{

	int32_t data;
	uint8_t i;

	//debug
	//LedYellow(ON);

	I2cStart(I2C1, I2C_ADDRESS_24LC256, I2C_Direction_Transmitter);
	I2cWrite(I2C1, (uint8_t)(address >> 8));
	I2cWrite(I2C1, (uint8_t)(address & 0xFF));
	I2cRestart(I2C1, I2C_ADDRESS_24LC256, I2C_Direction_Receiver);
	for(i = 0 ; i < (len - 1) ; i++)
	{
		buf[i] = I2cReadAck(I2C1);

	}
	buf[i] =  I2cReadNack(I2C1);

	//filter out unreadable signs
	for(i = 0 ; i < len ; i++)
	{
		if((buf[i] < 32) || (buf[i] > 126))
		{
			buf[i] = 45; // - // 32;//space
		}
	}

	//debug
	//LedYellow(OFF);

	return;

}

// ************************************************************* //
// ***       Function:I2cReadEepromExtAscii                  *** //
// ***       Purpose: reads Integer from EEPROM              *** //
// ***       Input: address - start address in EEPROM-Memory *** //
// ***       Return: Data byte                               *** //
// ************************************************************* //
void I2cReadEepromExtAscii(uint16_t address, uint8_t len, uint8_t * buf)
{

	int32_t data;
	uint8_t i;

	//debug
	//LedYellow(ON);

	I2cStart(I2C1, I2C_ADDRESS_24LC256_EXT, I2C_Direction_Transmitter);
	I2cWrite(I2C1, (uint8_t)(address >> 8));
	I2cWrite(I2C1, (uint8_t)(address & 0xFF));
	I2cRestart(I2C1, I2C_ADDRESS_24LC256_EXT, I2C_Direction_Receiver);
	for(i = 0 ; i < (len - 1) ; i++)
	{
		buf[i] = I2cReadAck(I2C1);
	}
	buf[i] =  I2cReadNack(I2C1);

	//filter out unreadable signs
	for(i = 0 ; i < len ; i++)
	{
		if((buf[i] < 32) || (buf[i] > 126))
		{
			buf[i] = 32;//space
		}
	}

	//debug
	//LedYellow(OFF);

	return;

}




// ************************************************************* //
// ***       Function:I2cWritedEepromByte                    *** //
// ***       Purpose: append data to transmit input buffer   *** //
// ***       Input: address - address in EEPROM-Memory       *** //
// ***       Input: data - Data byte                         *** //
// ***       Return: none                                    *** //
// ************************************************************* //
void I2cWriteEepromByte(uint16_t address, uint8_t data)
{

	//debug
	//LedYellow(ON);

	I2cStart(I2C1, I2C_ADDRESS_24LC256, I2C_Direction_Transmitter);
	I2cWrite(I2C1, (uint8_t)(address >> 8));
	I2cWrite(I2C1, (uint8_t)(address & 0xFF));
	I2cWrite(I2C1, data);
	I2cStop(I2C1);

	//debug
	//LedYellow(OFF);

	return;

}

// ***************************************************************** //
// ***       Function:I2cWritedEepromExtByte                     *** //
// ***       Purpose: append data to transmit input buffer       *** //
// ***       Input: address - address in  external EEPROM-Memory *** //
// ***       Input: data - Data byte                             *** //
// ***       Return: none                                        *** //
// ***************************************************************** //
void I2cWriteEepromExtByte(uint16_t address, uint8_t data)
{

	//debug
	//LedYellow(ON);

	I2cStart(I2C1, I2C_ADDRESS_24LC256_EXT, I2C_Direction_Transmitter);
	I2cWrite(I2C1, (uint8_t)(address >> 8));
	I2cWrite(I2C1, (uint8_t)(address & 0xFF));
	I2cWrite(I2C1, data);
	I2cStop(I2C1);

	//debug
	//LedYellow(OFF);

	return;

}

// ************************************************************* //
// ***       Function:I2cWritedEepromInteger                 *** //
// ***       Purpose: writes 32-bit-Integer into EEPROM      *** //
// ***       Input: address - address in EEPROM-Memory       *** //
// ***       Input: data - integer to write                  *** //
// ***       Return: none                                    *** //
// ************************************************************* //
void I2cWriteEepromInteger(uint16_t address, int32_t data)
{
	//debug
	//LedYellow(ON);

	I2cStart(I2C1, I2C_ADDRESS_24LC256, I2C_Direction_Transmitter);
	I2cWrite(I2C1, (uint8_t)(address >> 8));
	I2cWrite(I2C1, (uint8_t)(address & 0xFF));
	I2cWrite(I2C1, (uint8_t)(data >> 24));
	I2cWrite(I2C1, (uint8_t)(data >> 16));
	I2cWrite(I2C1, (uint8_t)(data >> 8));
	I2cWrite(I2C1, (uint8_t)data);
	I2cStop(I2C1);

	//debug
	//LedYellow(OFF);

	return;

}

// ****************************************************************** //
// ***       Function:I2cWritedEepromExtInteger                   *** //
// ***       Purpose: writes 32-bit-Integer into EEPROM           *** //
// ***       Input: address - address in  external EEPROM-Memory  *** //
// ***       Input: data - integer to write                       *** //
// ***       Return: none                                         *** //
// ****************************************************************** //
void I2cWriteEepromExtInteger(uint16_t address, int32_t data)
{
	//debug
	//LedYellow(ON);

	I2cStart(I2C1, I2C_ADDRESS_24LC256_EXT, I2C_Direction_Transmitter);
	I2cWrite(I2C1, (uint8_t)(address >> 8));
	I2cWrite(I2C1, (uint8_t)(address & 0xFF));
	I2cWrite(I2C1, (uint8_t)(data >> 24));
	I2cWrite(I2C1, (uint8_t)(data >> 16));
	I2cWrite(I2C1, (uint8_t)(data >> 8));
	I2cWrite(I2C1, (uint8_t)data);
	I2cStop(I2C1);

	//debug
	//LedYellow(OFF);

	return;

}

// ************************************************************* //
// ***       Function:I2cWriteEepromAscii                    *** //
// ***       Purpose: writes ASCII-bytes into EEPROM         *** //
// ***       Input: address - address in EEPROM-Memory       *** //
// ***       Input: data - integer to write                  *** //
// ***       Return: none                                    *** //
// ************************************************************* //
void I2cWriteEepromAscii(uint16_t address, uint8_t len, uint8_t * buf)
{
	//debug
	//LedYellow(ON);
	uint8_t i;

	I2cStart(I2C1, I2C_ADDRESS_24LC256, I2C_Direction_Transmitter);
	I2cWrite(I2C1, (uint8_t)(address >> 8));
	I2cWrite(I2C1, (uint8_t)(address & 0xFF));
	for(i = 0 ; i < len  ; i++)
	{
		I2cWrite(I2C1, buf[i]);
	}
	I2cStop(I2C1);

	//debug
	//LedYellow(OFF);

	return;

}

// ************************************************************* //
// ***       Function:I2cWriteEepromExtAscii                 *** //
// ***       Purpose: writes ASCII-bytes into EEPROM         *** //
// ***       Input: address - address in EEPROM-Memory       *** //
// ***       Input: data - integer to write                  *** //
// ***       Return: none                                    *** //
// ************************************************************* //
void I2cWriteEepromExtAscii(uint16_t address, uint8_t len, uint8_t * buf)
{
	//debug
	//LedYellow(ON);
	uint8_t i;

	I2cStart(I2C1, I2C_ADDRESS_24LC256_EXT, I2C_Direction_Transmitter);
	I2cWrite(I2C1, (uint8_t)(address >> 8));
	I2cWrite(I2C1, (uint8_t)(address & 0xFF));
	for(i = 0 ; i < len  ; i++)
	{
		I2cWrite(I2C1, buf[i]);
	}
	I2cStop(I2C1);

	//debug
	//LedYellow(OFF);

	return;

}





// ************************************************************* //
// ***       Function:I2cWriteLTC2943                        *** //
// ***       Purpose: writes byte into Register              *** //
// ***       Input: register - register of LTC2943           *** //
// ***       Input: data - integer to write                  *** //
// ***       Return: 0 - ok; > 0 - error                     *** //
// ************************************************************* //
uint8_t I2cWriteLTC2943(uint8_t reg, int8_t data)
{
	uint8_t error;
	//debug
	//LedYellow(ON);

	error = I2cStart(I2C3, I2C_ADDRESS_LTC2943, I2C_Direction_Transmitter);
	error |= I2cWrite(I2C3, reg);
	error |= I2cWrite(I2C3, data);
	I2cStop(I2C3);

    return error;

	//debug
	//LedYellow(OFF);

}

// ************************************************************* //
// ***       Function:I2cWriteChargeRegisterLTC2943          *** //
// ***       Purpose: reads word from register Register      *** //
// ***       Input: register - register of LTC2943           *** //
// ***       Input: data - integer to write                  *** //
// ***       Return: none                                    *** //
// ************************************************************* //
void I2cWriteChargeRegisterLTC2943(uint16_t dat)
{


	//debug
	//LedYellow(ON);

	I2cStart(I2C3, I2C_ADDRESS_LTC2943, I2C_Direction_Transmitter);
	I2cWrite(I2C3, LTC2943_ACCUMULATED_CHARGE_REGISTER);
	I2cWrite(I2C3, (uint8_t)(dat >> 8));//MSB
	I2cWrite(I2C3, (uint8_t)(dat & 0xFF));//LSB
	I2cStop(I2C3);


	//debug
	//LedYellow(OFF);

	return;

}

// ************************************************************* //
// ***       Function:I2cReadLTC2451                         *** //
// ***       Purpose: returns ADC-Value                      *** //
// ***       Input: none                                     *** //
// ***       Return: DDC-Value                               *** //
// ************************************************************* //
uint16_t I2cReadLTC2451(void)
{

	uint8_t temp8[2];
	uint16_t i;

	//debug
	//LedRed(ON);

	I2cStart(I2C1, I2C_ADDRESS_LTC2451, I2C_Direction_Receiver);
	temp8[0] = I2cReadAck(I2C1);
	temp8[1] = I2cReadNack(I2C1);

	//debug
	//LedRed(OFF);

	i = temp8[0];
	i = i << 8;
	i |= temp8[1];

	return i;


}

// ******************************************************************** //
// ***            I2cSetModeLTC2481                                 *** //
// ***       sets Mode of LTC2481                                   *** //
// ***       Purpose: sets byte to port                             *** //
// ***                                                              *** //
// ***       Input:  gain -  0 : gain = 1                           *** //
// ***                       1 : gain = 4 (2 @ double speed)        *** //
// ***                       2 : gain = 8 (4 @ double speed)        *** //
// ***                       3 : gain = 16 (8 @ double speed)       *** //
// ***                       4 : gain = 32 (16 @ double speed)      *** //
// ***                       5 : gain = 64 (32 @ double speed)      *** //
// ***                       6 : gain = 128 (64 @ double speed)     *** //
// ***                       7 : gain = 256 (128 @ double speed)    *** //
// ***               tempr - 0: read extern Voltage                 *** //
// ***                       1: read temperature                    *** //
// ***               rej -   0: 50/60Hz rejection                   *** //
// ***                       1: 50Hz rejection                      *** //
// ***                       2: 60Hz rejection                      *** //
// ***              speed -  0: normal speed                        *** //
// ***                       1: double speed                        *** //
// ***       Return: 0 - ok, 0xFF - busy                            *** //
// ******************************************************************** //
void I2cSetModeLTC2481(uint8_t gain, uint8_t tempr, uint8_t rej, uint8_t speed)
{
    uint8_t ctl = 0;
    uint8_t  t = 0;
    uint8_t adr;
    uint8_t ret = 0;



	 //gain
	 ctl = gain << 5;
	 //temperature
	 t = tempr & 0x01;
	 t = t << 3;
	 ctl |= t;
	 //rejection
	 t = rej & 0x03;
	 t = t << 1;
	 ctl |= t;
	 //speed
	 t = speed & 0x01;
	 ctl |= t;



     //debug
	//LedRed(ON);

	adr = I2C_ADDRESS_LTC2481;


	ret = I2cStart(I2C2, adr, I2C_Direction_Transmitter);
	ret |= I2cWrite(I2C2, ctl);
	ret |= I2cStop(I2C2);


	//debug
	//LedRed(OFF);

	return;

}

// ************************************************************* //
// ***       Function:I2cReadLTC2481                         *** //
// ***       Purpose: returns ADC-Value                      *** //
// ***       Input: none                                     *** //
// ***       Return: ADC-Value                               *** //
// ************************************************************* //
uint32_t I2cReadLTC2481(void)
{

	uint8_t temp8[3];
	uint32_t i;
	uint8_t adr;
	uint8_t ret;

	//debug
	//LedRed(ON);

	adr = I2C_ADDRESS_LTC2481;


	ret = I2cStart(I2C2, adr, I2C_Direction_Receiver);
	temp8[0] = I2cReadAck(I2C2);
	temp8[1] = I2cReadAck(I2C2);
	temp8[2] = I2cReadNack(I2C2);


	i = temp8[0];
	i = i << 8;
	i |= temp8[1];
	i = i << 8;
	i |= temp8[2];
	i = i >> 6;
	i = i & 0xFFFF;

	//debug
	//LedRed(OFF);

	return i;


}

// ************************************************************* //
// ***       Function:IsI2cLTC2481Present                    *** //
// ***       Purpose: checks, if LTC2481 is present          *** //
// ***       Return: 0 - not present, 1 - present            *** //
// ************************************************************* //
uint8_t IsI2cLTC2481Present(void)
{
	uint8_t u,v, adr;

	//LedRed(ON);


	adr = I2C_ADDRESS_LTC2481;



	u = I2cStart(I2C2, adr, I2C_Direction_Receiver);
	v = I2cReadAck(I2C2);
	v = I2cReadAck(I2C2);
	v = I2cReadNack(I2C2);

	//debug
	//LedRed(OFF);

	if(u != I2C_NO_ERROR)
	{
		return 0;
	}

	return 1;

}


// ************************************************************* //
// ***       Function:I2cReadWordLTC2943                     *** //
// ***       Purpose: reads word from register Register      *** //
// ***       Input: register - register of LTC2943           *** //
// ***       Input: data - integer to write                  *** //
// ***       Return: none                                    *** //
// ************************************************************* //
uint16_t I2cReadWordLTC2943(uint8_t reg)
{

	int16_t data;

	//debug
	//LedYellow(ON);

	I2cStart(I2C3, I2C_ADDRESS_LTC2943, I2C_Direction_Transmitter);
	I2cWrite(I2C3, reg);
	I2cRestart(I2C3, I2C_ADDRESS_LTC2943, I2C_Direction_Receiver);
	data = I2cReadAck(I2C3);
	data = data << 8;
	data = data | I2cReadNack(I2C3);

	//debug
	//LedYellow(OFF);

	return data;

}

// ************************************************************* //
// ***       Function:I2cReadWordLTC2943                     *** //
// ***       Purpose: reads word from register Register      *** //
// ***       Input: register - register of LTC2943           *** //
// ***       Input: data - integer to write                  *** //
// ***       Return: none                                    *** //
// ************************************************************* //
uint8_t I2cReadStatusLTC2943(void)
{

	int8_t data;

	//debug
	//LedYellow(ON);

	I2cStart(I2C3, I2C_ADDRESS_LTC2943, I2C_Direction_Transmitter);
	I2cWrite(I2C3, LTC2943_STATUS_REGISTER);//Status-Register: 0x00
	I2cRestart(I2C3, I2C_ADDRESS_LTC2943, I2C_Direction_Receiver);
	data = I2cReadNack(I2C3);

	//debug
	//LedYellow(OFF);

	return data;

}

// ************************************************************* //
// ***       Function:IsI2cLTC2943Present                    *** //
// ***       Purpose: checks, if LTC2481 is present          *** //
// ***       Input: none                                     *** //
// ***       Return: 0 - not present, 1 - present            *** //
// ************************************************************* //
uint8_t IsI2cLTC2943Present(void)
{
	uint8_t u,v;

	//LedRed(ON);


	u = I2cStart(I2C3, I2C_ADDRESS_LTC2943, I2C_Direction_Receiver);
	v = I2cReadAck(I2C3);
	v = I2cReadAck(I2C3);
	v = I2cReadNack(I2C3);

	//debug
	//LedRed(OFF);

	if(u != I2C_NO_ERROR)
	{
		return 0;
	}

	return 1;

}

// ************************************************************* //
// ***       Function:I2cWriteRegisterL3GD20H                *** //
// ***       Purpose: reads word from register Register      *** //
// ***       Input: register - register of LTC2943           *** //
// ***       Input: data - integer to write                  *** //
// ***       Return: none                                    *** //
// ************************************************************* //
void I2cWriteRegisterL3GD20H(uint8_t reg, uint8_t dat)
{


	//debug
	//LedYellow(ON);

	I2cStart(I2C1, I2C_ADDRESS_L3GD20H, I2C_Direction_Transmitter);
	I2cWrite(I2C1, reg);
	I2cWrite(I2C1, dat);//MSB
	I2cStop(I2C1);


	//debug
	//LedYellow(OFF);

	return;

}

// ************************************************************* //
// ***       Function:I2cReadWordL3GD20H                     *** //
// ***       Purpose: reads word from register Register      *** //
// ***       Input: register - register of L3GD20H           *** //
// ***       Input: data - integer to write                  *** //
// ***       Return: none                                    *** //
// ************************************************************* //
uint16_t I2cReadWordL3GD20H(uint8_t reg)
{

	int16_t data;

	//debug
	//LedYellow(ON);

	I2cStart(I2C1, I2C_ADDRESS_L3GD20H, I2C_Direction_Transmitter);
	I2cWrite(I2C1, reg | 0x80);//0x80: set the auto increment flag
	I2cRestart(I2C1, I2C_ADDRESS_L3GD20H, I2C_Direction_Receiver);
	data = I2cReadAck(I2C1);
	data = data << 8;
	data = data | I2cReadNack(I2C1);

	//debug
	//LedYellow(OFF);

	return data;

}

// ************************************************************* //
// ***       Function:I2cReadMultipleBytesL3GD20H            *** //
// ***       Purpose: reads bytes from register Register     *** //
// ***       Input: register - register of L3GD20H           *** //
// ***              buf - pointer to received data           *** //
// ***              num - number of bytes to receive         *** //
// ***       Return: none                                    *** //
// ************************************************************* //
void I2cReadMultipleBytesL3GD20H(uint8_t reg, uint8_t * buf, uint8_t num)
{


	uint8_t i;

	//debug
	//LedYellow(ON);

	I2cStart(I2C1, I2C_ADDRESS_L3GD20H, I2C_Direction_Transmitter);
	I2cWrite(I2C1, reg | 0x80);//0x80: set the auto increment flag
	I2cRestart(I2C1, I2C_ADDRESS_L3GD20H, I2C_Direction_Receiver);
	for(i = 0; i < (num - 1); i++)
	{
		buf[i] = I2cReadAck(I2C1);
	}
	buf[i] = I2cReadNack(I2C1);

	//debug
	//LedYellow(OFF);

	return;

}

// ************************************************************* //
// ***       Function:I2cWriteRegisterBMP280                 *** //
// ***       Purpose: reads word from register Register      *** //
// ***       Input: register - register of BMP280            *** //
// ***       Input: data - integer to write                  *** //
// ***       Return: none                                    *** //
// ************************************************************* //
void I2cWriteRegisterBMP280(uint8_t reg, uint8_t dat)
{


	//debug
	//LedYellow(ON);

	I2cStart(I2C1, I2C_ADDRESS_BMP280, I2C_Direction_Transmitter);
	I2cWrite(I2C1, reg);
	I2cWrite(I2C1, dat);//MSB
	I2cStop(I2C1);


	//debug
	//LedYellow(OFF);

	return;

}

// ************************************************************* //
// ***       Function:I2cReadMultipleBytesBMP280             *** //
// ***       Purpose: reads bytes from register Register     *** //
// ***       Input: register - register of L3GD20H           *** //
// ***              buf - pointer to received data           *** //
// ***              num - number of bytes to receive         *** //
// ***       Return: none                                    *** //
// ************************************************************* //
void I2cReadMultipleBytesBMP280(uint8_t reg, uint8_t * buf, uint8_t num)
{


	uint8_t i;

	//debug
	//LedYellow(ON);

	I2cStart(I2C1, I2C_ADDRESS_BMP280, I2C_Direction_Transmitter);
	I2cWrite(I2C1, reg);//start register
	I2cRestart(I2C1, I2C_ADDRESS_BMP280, I2C_Direction_Receiver);
	for(i = 0; i < (num - 1); i++)
	{
		buf[i] = I2cReadAck(I2C1);
	}
	buf[i] = I2cReadNack(I2C1);

	//debug
	//LedYellow(OFF);

	return;

}

// ************************************************************* //
// ***       Function:I2cTriggerTemperatureMeasurementHTU21  *** //
// ***       Purpose: triggers measurement                   *** //
// ***       Input: none                                     *** //
// ***       Return: none                                    *** //
// ************************************************************* //
void I2cTriggerTemperatureMeasurementHTU21(void)
{


	//debug
	//LedYellow(ON);

	I2cStart(I2C1, I2C_ADDRESS_HTU21, I2C_Direction_Transmitter);
	I2cWrite(I2C1, HTU21_MEASURE_TEMPERATURE);
	I2cStop(I2C1);


	//debug
	//LedYellow(OFF);

	return;

}

// ************************************************************* //
// ***       Function:I2cTriggerHumidityMeasurementHTU21     *** //
// ***       Purpose: triggers measurement                   *** //
// ***       Input: none                                     *** //
// ***       Return: none                                    *** //
// ************************************************************* //
void I2cTriggerHumidityMeasurementHTU21(void)
{


	//debug
	//LedYellow(ON);

	I2cStart(I2C1, I2C_ADDRESS_HTU21, I2C_Direction_Transmitter);
	I2cWrite(I2C1, HTU21_MEASURE_HUMIDITY);
	I2cStop(I2C1);


	//debug
	//LedYellow(OFF);

	return;

}

// ******************************************************************* //
// ***       Function:I2cReadHTU21                                 *** //
// ***       Purpose: reads measurement value 2 bytes + checksum   *** //
// ***       Input: buf - pointer to received data                 *** //
// ***       Return: none                                          *** //
// ******************************************************************* //
void I2cReadHTU21(uint8_t * buf)
{


	uint8_t i;

	//debug
	//LedYellow(ON);


	I2cStart(I2C1, I2C_ADDRESS_HTU21, I2C_Direction_Receiver);
	for(i = 0; i < 2; i++)// read 3 bytes
	{
		buf[i] = I2cReadAck(I2C1);
	}
	buf[i] = I2cReadNack(I2C1);

	//debug
	//LedYellow(OFF);

	return;

}

// ************************************************************* //
// ***       Function:IsI2cLM35091Present                    *** //
// ***       Purpose: checks, if LM3509 is present           *** //
// ***       Input: none                                     *** //
// ***       Return: 0 - not present, 1 - present            *** //
// ************************************************************* //
uint8_t IsI2cLM35091Present(void)
{
	uint8_t u,v, adr;

	//LedRed(ON);

	adr = I2C_ADDRESS_LM3509;

	u = I2cStart(I2C2, adr, I2C_Direction_Receiver);
	v = I2cReadAck(I2C2);
	v = I2cReadAck(I2C2);
	v = I2cReadNack(I2C2);

	//debug
	//LedRed(OFF);

	if(u != I2C_NO_ERROR)
	{
		return 0;
	}

	return 1;

}

// ************************************************************* //
// ***       Function:I2cLM35091SwitchMain                   *** //
// ***       Purpose: checks, if LM3509 is present           *** //
// ***       Input: ON, OFF                                  *** //
// ***       Return: 0 - not present, 1 - present            *** //
// ************************************************************* //
void I2cLM35091SwitchMain(uint8_t state)
{
	uint8_t dat;

	//LedRed(ON);
	if(state == ON)
	{
		dat = 0x01;
	}
	else
	{
		dat = 0x00;
	}


	I2cStart(I2C1, I2C_ADDRESS_LM3509, I2C_Direction_Transmitter);
	I2cWrite(I2C1, LM3509_GENERAL_REGISTER);
	I2cWrite(I2C1, dat);//
	I2cStop(I2C1);

	//debug
	//LedRed(OFF);

	return;



}

// ************************************************************* //
// ***       Function:IsI2cLTC2481Present                    *** //
// ***       Purpose: checks, if LM3509 is present           *** //
// ***       Input: brightness (0 ... 31)                    *** //
// ***       Return: 0 - not present, 1 - present            *** //
// ************************************************************* //
void I2cLM35091SetMainBrightness(uint8_t brightness)
{
	uint8_t dat;

	//LedRed(ON);
	if(brightness > 31)
	{
		brightness = 31;
	}

	dat = 0xE0 | brightness;



	I2cStart(I2C1, I2C_ADDRESS_LM3509, I2C_Direction_Transmitter);
	I2cWrite(I2C1, LM3509_MAIN_BRIGHTNESS_REGISTER);
	I2cWrite(I2C1, dat);//
	I2cStop(I2C1);

	//debug
	//LedRed(OFF);

	return;



}

// ************************************************************* //
// ***       Function:I2cINA236SetConfig                     *** //
// ***       Purpose: configures INA 236                     *** //
// ***       Input: range (0 - 81mV; 1 - 20mV)               *** //
// ***              average (0 ... 7) = 1...1024x            *** //
// ***              bustime (0 ... 7) = 140 ... 8244us       *** //
// ***              shunttime (0 ... 7) = 140 ... 8244us     *** //
// ***              mode (0 ... 7)                           *** //
// ***       Return: none                                    *** //
// ************************************************************* //
void I2cINA236SetConfig(uint8_t range, uint8_t average, uint8_t bustime, uint8_t shunttime, uint8_t mode)
{
	uint8_t dath, datl;

	//LedRed(ON);

	//limit input values
	range = range & 0x01;
	average = average & 0x07;
	bustime = bustime & 0x07;
	shunttime = shunttime & 0x07;
	mode = mode & 0x07;


	dath = (range << 4) | (average << 1) | (bustime >> 2);

	datl = (bustime << 6) | (shunttime << 3) | (mode);



	I2cStart(I2C1, I2C_ADDRESS_INA236B, I2C_Direction_Transmitter);
	I2cWrite(I2C1, INA236_CONFIG_REGISTER);
	I2cWrite(I2C1, dath);//
	I2cWrite(I2C1, datl);//
	I2cStop(I2C1);

	//debug
	//LedRed(OFF);

	return;



}

// ************************************************************* //
// ***       Function:I2cINA236SetRegisterPointer            *** //
// ***       Purpose: set pointer to register                *** //
// ***       Input: reg  -   INA236_..._REGISTER             *** //
// ***       Return: none                                    *** //
// ************************************************************* //
void I2cINA236SetRegisterPointer(uint8_t reg)
{
	uint8_t dath, datl;

	//LedRed(ON);


	I2cStart(I2C1, I2C_ADDRESS_INA236B, I2C_Direction_Transmitter);
	I2cWrite(I2C1, reg);
	I2cStop(I2C1);

	//debug
	//LedRed(OFF);

	return;


}

// ************************************************************* //
// ***       Function:I2cReadINA236                          *** //
// ***       Purpose: returns ADC-Value                      *** //
// ***       Input: none                                     *** //
// ***       Return: ADC-Value                               *** //
// ************************************************************* //
int16_t I2cReadINA236(void)
{

	uint8_t temp8[3];
	uint16_t i;
	uint8_t adr;
	uint8_t ret;

	//debug
	//LedRed(ON);

	adr = I2C_ADDRESS_INA236B;


	ret = I2cStart(I2C1, adr, I2C_Direction_Receiver);
	temp8[0] = I2cReadAck(I2C1);
	temp8[1] = I2cReadNack(I2C1);


	i = temp8[0];
	i = i << 8;
	i |= temp8[1];


	//debug
	//LedRed(OFF);

	return i;


}


// ************************************************************* //
// ***       Function:I2cReadLTC2471                         *** //
// ***       Purpose: returns ADC-Value                      *** //
// ***       Input: none                                     *** //
// ***       Return: ADC-Value                               *** //
// ************************************************************* //
uint16_t I2cReadLTC2471(void)
{

	uint8_t temp8[3];
	uint16_t i;
	uint8_t adr, stat;

	//debug
	//LedRed(ON);

	adr = I2C_ADDRESS_LTC2471;


	stat = I2cStart(I2C1, adr, I2C_Direction_Receiver);
	temp8[0] = I2cReadAck(I2C1);
	temp8[1] = I2cReadNack(I2C1);


	i = temp8[0];
	i = i << 8;
	i |= temp8[1];


	//debug
	//LedRed(OFF);

	return i;


}





// ************************************************************* //
// ***       Function:I2cAddJob                              *** //
// ***       Purpose: add job to the list                    *** //
// ***       Input: job                                      *** //
// ***       Return: none                                    *** //
// ************************************************************* //
void I2cAddJob(uint8_t job, uint32_t par1, uint32_t par2, uint32_t par3)
{



	if(I2cJobIn >= I2C_MAX_JOBS)
	{
		I2cJobIn = 0;
	}

	I2CJobList[I2cJobIn].job = job;
	I2CJobList[I2cJobIn].par1 = par1;
	I2CJobList[I2cJobIn].par2 = par2;
	I2CJobList[I2cJobIn].par3 = par3;


	I2cJobIn++;

	return;

}

// ************************************************************* //
// ***       Function:WorkI2c                                *** //
// ***       Purpose:  cyclic from main called               *** //
// ***       Input: none                                     *** //
// ***       Return: none                                    *** //
// ************************************************************* //
void WorkI2c(void)
{

	if(I2cJobIn != I2cJobOut)
		{
			switch(I2CJobList[I2cJobOut].job)
			{
				case I2C_JOB_EEPROM_WRITE_BYTE:
					I2cWriteEepromByte(I2CJobList[I2cJobOut].par1,I2CJobList[I2cJobOut].par2);
				break;
				case I2C_JOB_EEPROM_WRITE_INTEGER:
					I2cWriteEepromInteger(I2CJobList[I2cJobOut].par1,I2CJobList[I2cJobOut].par2);
				break;
				case I2C_JOB_EEPROM_EXT_WRITE_BYTE:
					I2cWriteEepromExtByte(I2CJobList[I2cJobOut].par1,I2CJobList[I2cJobOut].par2);
				break;
				case I2C_JOB_EEPROM_EXT_WRITE_INTEGER:
					I2cWriteEepromExtInteger(I2CJobList[I2cJobOut].par1,I2CJobList[I2cJobOut].par2);
				break;
				case I2C_JOB_LM3509_SWITCH_MAIN:
					I2cLM35091SwitchMain(I2CJobList[I2cJobOut].par1);
				break;
				case I2C_JOB_LM3509_SET_BRIGHTNESS:
					I2cLM35091SetMainBrightness(I2CJobList[I2cJobOut].par1);
				break;
				case I2C_JOB_NONE:
					break;
				//default:

			}

			I2cJobOut++;

			if(I2cJobOut >= I2C_MAX_JOBS)
			{
				I2cJobOut = 0;
			}

			if(I2cJobIn == I2cJobOut)
			{
				I2cJobIn = 0;
				I2cJobOut = 0;
			}

		}




		return;

}

