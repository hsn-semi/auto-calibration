/******************************************************/
/***  Project:       FilterController               ***/
/***  File:          exposure.c                     ***/
/***  Created on:    17.02.2016                     ***/
/***  derived from AraController4: 25.11.2017       ***/
/***  Author:        Andreas St�rker                ***/
/***  Company:       Blink AG                       ***/
/***  Changes:                                      ***/
/******************************************************/

//#include "stm32f4_discovery.h"
#include "stm32f4xx_conf.h"
#include "stm32f4xx_tim.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx.h"

#include <stdio.h>
#include <string.h>

//#include "stm32f4xx_usart.h"
#include "main.h"
#include "can.h"
#include "i2c.h"

#include "exposure.h"


//variables:
unsigned char Exposure = 0;

unsigned char ExposureASyncEnabled = 0;
unsigned char ExposureBSyncEnabled = 0;



TypeExposureHW ExposureHW[EXPOSURE_NUMBER_OF_PORTS];

TypeExposureControl ExposureControl[EXPOSURE_NUMBER_OF_PORTS];

TypeExposureCalibration ExposureCalibration[EXPOSURE_NUMBER_OF_LINES];

TypeLedCurrentCalibration LedCurrentCalibration;



// ********************************************** //
// ***       Function: ExposureInit           *** //
// ***       Purpose: init Exposure           *** //
// ***       Input: none                      *** //
// ***       Return: none                     *** //
// ********************************************** //
void ExposureInit(void)
{

	  GPIO_InitTypeDef 		GPIO_InitStructure;
	  DAC_InitTypeDef  DAC_InitStructure;
	  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	  TIM_OCInitTypeDef  TIM_OCInitStructure;
	  //ADC_InitTypeDef       ADC_InitStruct;
	  //ADC_CommonInitTypeDef ADC_CommonInitStruct;
	  //DMA_InitTypeDef       DMA_InitStruct;
	  int8_t i, j;
	  uint16_t Prescalerval;
	  int32_t itemp;
	  int32_t intgr;
	  uint32_t utemp;

	  /* fill the structures */
	  ExposureHW[EXPOSURE_A].lineGPIO[0].port = EXPOSURE_A_LINE_1_PORT;
	  ExposureHW[EXPOSURE_A].lineGPIO[0].pin = EXPOSURE_A_LINE_1_PIN;
	  ExposureHW[EXPOSURE_A].lineGPIO[1].port = EXPOSURE_A_LINE_2_PORT;
	  ExposureHW[EXPOSURE_A].lineGPIO[1].pin = EXPOSURE_A_LINE_2_PIN;
	  ExposureHW[EXPOSURE_A].lineGPIO[2].port = EXPOSURE_A_LINE_3_PORT;
	  ExposureHW[EXPOSURE_A].lineGPIO[2].pin = EXPOSURE_A_LINE_3_PIN;
	  ExposureHW[EXPOSURE_A].lineGPIO[3].port = EXPOSURE_A_LINE_4_PORT;
	  ExposureHW[EXPOSURE_A].lineGPIO[3].pin = EXPOSURE_A_LINE_4_PIN;
	  ExposureHW[EXPOSURE_A].lineGPIO[4].port = EXPOSURE_A_LINE_5_PORT;
	  ExposureHW[EXPOSURE_A].lineGPIO[4].pin = EXPOSURE_A_LINE_5_PIN;
	  ExposureHW[EXPOSURE_A].dacGPIO.port = EXPOSURE_DAC_GPIO_PORT;
	  ExposureHW[EXPOSURE_A].dacGPIO.pin = EXPOSURE_DAC_A_PIN;
	  ExposureHW[EXPOSURE_A].dacChannel = EXPOSURE_DAC_A_CHANNEL;
	  ExposureHW[EXPOSURE_A].pwmGPIO.port = EXPOSURE_A_PWM_GPIO_PORT;
	  ExposureHW[EXPOSURE_A].pwmGPIO.pin = EXPOSURE_A_PWM_PIN;
	  ExposureHW[EXPOSURE_A].pwmPinSource = EXPOSURE_A_PWM_PIN_SOURCE;
	  ExposureHW[EXPOSURE_A].pwmAF = EXPOSURE_A_PWM_TIMER_AF;
	  ExposureHW[EXPOSURE_A].timer = EXPOSURE_A_PWM_TIMER;
	  ExposureHW[EXPOSURE_A].channel = EXPOSURE_A_PWM_CHANNEL;


	  /* reset the structure */
	  DAC_StructInit(&DAC_InitStructure);
	
	  /* GPIO clock enable */
	  RCC_AHB1PeriphClockCmd(EXPOSURE_GPIO_CLOCKS, ENABLE);

	  /* DAC-Clock Enable */
	  RCC_APB1PeriphClockCmd(EXPOSURE_DAC_CLK, ENABLE);

	  for( i = 0 ; i < EXPOSURE_NUMBER_OF_PORTS ; i++)
	  {
		  /* line output ports */
		  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
		  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
		  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;

		  for( j = 0 ; j < EXPOSURE_NUMBER_OF_LINES ; j++)
		  {
			  GPIO_InitStructure.GPIO_Pin = ExposureHW[i].lineGPIO[j].pin;
			  GPIO_Init(ExposureHW[i].lineGPIO[j].port, &GPIO_InitStructure);
		  }

		  /* Connect TIM pin to AF */
		  GPIO_PinAFConfig(ExposureHW[i].pwmGPIO.port, ExposureHW[i].pwmPinSource, ExposureHW[i].pwmAF);
		  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
		  GPIO_InitStructure.GPIO_Pin = ExposureHW[i].pwmGPIO.pin;
		  GPIO_Init(ExposureHW[i].pwmGPIO.port, &GPIO_InitStructure);

		  /* DAC GPIO */
		  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
		  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
		  GPIO_InitStructure.GPIO_Pin = ExposureHW[i].dacGPIO.pin;
		  GPIO_Init(ExposureHW[i].dacGPIO.port, &GPIO_InitStructure);

		  /* DAC-Config */
		  DAC_InitStructure.DAC_Trigger = DAC_Trigger_None;
		  DAC_InitStructure.DAC_WaveGeneration = DAC_WaveGeneration_None;
		  DAC_InitStructure.DAC_OutputBuffer = DAC_OutputBuffer_Enable;
		  DAC_Init(ExposureHW[i].dacChannel, &DAC_InitStructure);
		  /* DAC channel enable */
		  DAC_Cmd(ExposureHW[i].dacChannel, ENABLE);

		  /* TIM clock enable */
		  EnableTimerPeriphClock(ExposureHW[i].timer);

		  /* Prescaler */
		  Prescalerval = (uint16_t) ((SystemCoreClock /2) / 1000000) - 1;//1000000 -> 1MHz counter frequency

		  if(IsTimerOnAPB2(ExposureHW[i].timer))
		  {
			  Prescalerval = Prescalerval * 2;
		  }
		  /* Time base configuration */
		  TIM_TimeBaseStructure.TIM_Period = 2000;//in us ; 2000 = 2ms
		  TIM_TimeBaseStructure.TIM_Prescaler = Prescalerval;
		  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
		  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
		  TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
		  TIM_TimeBaseInit(ExposureHW[i].timer, &TIM_TimeBaseStructure);

		  /* PWM Mode configuration */
		  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
		  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
		  TIM_OCInitStructure.TIM_Pulse = 1000;
		  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
		  TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;

		  //Timer PWM channel
		  switch(ExposureHW[i].channel)
		  {
			  case 1:
				  TIM_OC1Init(ExposureHW[i].timer, &TIM_OCInitStructure);
				  TIM_OC1PreloadConfig(ExposureHW[i].timer, TIM_OCPreload_Enable);
			  break;
			  case 2:
				TIM_OC2Init(ExposureHW[i].timer, &TIM_OCInitStructure);
				TIM_OC2PreloadConfig(ExposureHW[i].timer, TIM_OCPreload_Enable);
			  break;
			  case 3:
				TIM_OC3Init(ExposureHW[i].timer, &TIM_OCInitStructure);
				TIM_OC3PreloadConfig(ExposureHW[i].timer, TIM_OCPreload_Enable);
			  break;
			  case 4:
				TIM_OC4Init(ExposureHW[i].timer, &TIM_OCInitStructure);
				TIM_OC4PreloadConfig(ExposureHW[i].timer, TIM_OCPreload_Enable);
			  break;
		  }

		  /* TIM enable */
		  TIM_Cmd(ExposureHW[i].timer, ENABLE);

		  /* set minimum current */
		  if(ExposureHW[i].dacChannel == DAC_Channel_1)
		  {
			  DAC_SetChannel1Data(DAC_Align_12b_R, 0);
		  }
		  if(ExposureHW[i].dacChannel == DAC_Channel_2)
		  {
			  DAC_SetChannel2Data(DAC_Align_12b_R, 0);
		  }


	  }

	  //ADC-config done in main.c
		memset(ExposureControl, 0, sizeof(ExposureControl));

		memset(ExposureCalibration, 0, sizeof(ExposureCalibration));

		//read the values from EEPROM
		/* Line 1 */
		ExposureCalibration[0].powcalibrationoffset = I2cReadEepromExtInteger(EEPROM_EXT_ADDRESS_CAL_EXP1_POW_OFFSET);
		intgr = I2cReadEepromExtInteger(EEPROM_EXT_ADDRESS_CAL_EXP1_POW_GRADIENT);
		ExposureCalibration[0].powcalibrationgradient = (float32_t)intgr / 1000000;
		ExposureCalibration[0].fotocalibrationoffset = I2cReadEepromExtInteger(EEPROM_EXT_ADDRESS_CAL_EXP1_FOTO_OFFSET);
		intgr = I2cReadEepromExtInteger(EEPROM_EXT_ADDRESS_CAL_EXP1_FOTO_GRADIENT);
		ExposureCalibration[0].fotocalibrationgradient = (float32_t)intgr / 1000000;
		/* Line 2 */
		ExposureCalibration[1].powcalibrationoffset = I2cReadEepromExtInteger(EEPROM_EXT_ADDRESS_CAL_EXP2_POW_OFFSET);
		intgr = I2cReadEepromExtInteger(EEPROM_EXT_ADDRESS_CAL_EXP2_POW_GRADIENT);
		ExposureCalibration[1].powcalibrationgradient = (float32_t)intgr / 1000000;
		ExposureCalibration[1].fotocalibrationoffset = I2cReadEepromExtInteger(EEPROM_EXT_ADDRESS_CAL_EXP2_FOTO_OFFSET);
		intgr = I2cReadEepromExtInteger(EEPROM_EXT_ADDRESS_CAL_EXP2_FOTO_GRADIENT);
		ExposureCalibration[1].fotocalibrationgradient = (float32_t)intgr / 1000000;
		/* Line 3 */
		ExposureCalibration[2].powcalibrationoffset = I2cReadEepromExtInteger(EEPROM_EXT_ADDRESS_CAL_EXP3_POW_OFFSET);
		intgr = I2cReadEepromExtInteger(EEPROM_EXT_ADDRESS_CAL_EXP3_POW_GRADIENT);
		ExposureCalibration[2].powcalibrationgradient = (float32_t)intgr / 1000000;
		ExposureCalibration[2].fotocalibrationoffset = I2cReadEepromExtInteger(EEPROM_EXT_ADDRESS_CAL_EXP3_FOTO_OFFSET);
		intgr = I2cReadEepromExtInteger(EEPROM_EXT_ADDRESS_CAL_EXP3_FOTO_GRADIENT);
		ExposureCalibration[2].fotocalibrationgradient = (float32_t)intgr / 1000000;
		/* Line 4 */
		ExposureCalibration[3].powcalibrationoffset = I2cReadEepromInteger(EEPROM_EXT_ADDRESS_CAL_EXP4_POW_OFFSET);
		intgr = I2cReadEepromExtInteger(EEPROM_EXT_ADDRESS_CAL_EXP4_POW_GRADIENT);
		ExposureCalibration[3].powcalibrationgradient = (float32_t)intgr / 1000000;
		ExposureCalibration[3].fotocalibrationoffset = I2cReadEepromExtInteger(EEPROM_EXT_ADDRESS_CAL_EXP4_FOTO_OFFSET);
		intgr = I2cReadEepromExtInteger(EEPROM_EXT_ADDRESS_CAL_EXP4_FOTO_GRADIENT);
		ExposureCalibration[3].fotocalibrationgradient = (float32_t)intgr / 1000000;
		/* Line 5 */
		ExposureCalibration[4].powcalibrationoffset = I2cReadEepromExtInteger(EEPROM_EXT_ADDRESS_CAL_EXP5_POW_OFFSET);
		intgr = I2cReadEepromExtInteger(EEPROM_EXT_ADDRESS_CAL_EXP5_POW_GRADIENT);
		ExposureCalibration[4].powcalibrationgradient = (float32_t)intgr / 1000000;
		ExposureCalibration[4].fotocalibrationoffset = I2cReadEepromExtInteger(EEPROM_EXT_ADDRESS_CAL_EXP5_FOTO_OFFSET);
		intgr = I2cReadEepromExtInteger(EEPROM_EXT_ADDRESS_CAL_EXP5_FOTO_GRADIENT);
		ExposureCalibration[4].fotocalibrationgradient = (float32_t)intgr / 1000000;

		for (i = 0 ; i < EXPOSURE_NUMBER_OF_LINES ; i++)
		{
			  if((ExposureCalibration[i].powcalibrationgradient > EXPOSURE_CAL_MIN_GRADIENT) && (ExposureCalibration[i].powcalibrationgradient < EXPOSURE_CAL_MAX_GRADIENT))
			  {
				  ExposureCalibration[i].calibrationok = 1;
			  }
			  else
			  {
				  ExposureCalibration[i].powcalibrationoffset = EXPOSURE_CAL_DEFAULT_POW_OFFSET;
				  ExposureCalibration[i].powcalibrationgradient = EXPOSURE_CAL_DEFAULT_POW_GRADIENT;
				  ExposureCalibration[i].fotocalibrationoffset = EXPOSURE_CAL_DEFAULT_FOTO_OFFSET;
				  ExposureCalibration[i].fotocalibrationgradient = EXPOSURE_CAL_DEFAULT_FOTO_GRADIENT;
				  ExposureCalibration[i].calibrationok = 0;
			  }
		}



		ExposureControl[0].safetydisabled = I2cReadEepromInteger(EEPROM_ADDRESS_EXPOSURE_SAFETY_DISABLE);
		ExposureControl[0].readsensors = 1;

		memset(&LedCurrentCalibration, 0, sizeof(LedCurrentCalibration));


		/* read current calibration value */
		LedCurrentCalibration.calibrationoffset = I2cReadEepromInteger(EEPROM_ADDRESS_CAL_CURRENT_OFFSET);
		intgr = I2cReadEepromInteger(EEPROM_ADDRESS_CAL_CURRENT_GRADIENT);
		LedCurrentCalibration.calibrationgradient = (float32_t)intgr / 1000000;
		//Plausibility check
		if((LedCurrentCalibration.calibrationgradient > CURRENT_CAL_MIN_GRADIENT) && (LedCurrentCalibration.calibrationgradient < CURRENT_CAL_MAX_GRADIENT))
		{
			LedCurrentCalibration.calibrationok = 1;
		}
		else
		{
			LedCurrentCalibration.calibrationoffset = 0;
			LedCurrentCalibration.calibrationgradient = 1.0;
			LedCurrentCalibration.calibrationok = 0;
		}

		//ON and OFF
		SetExposureModeSychron(0, OFF);
		SwitchExposureOn(0, 0, 50, 1000, 1000);
		Delayms(10);
		SetExposureModeSychron(0, OFF);
		SwitchExposureOff(0);


}



// ************************************************************* //
// ***       Function: SwitchExposureOn                      *** //
// ***       Purpose: switches exposure on                   *** //
// ***       Input: port - pointer to buffer                 *** //
// ***       Input: line - LED-Line (0 ... 4)                *** //
// ***       Input: curent - LED-current [mA]                *** //
// ***       Return: none                                    *** //
// ************************************************************* //
void SwitchExposureOn(unsigned char port, unsigned char line, uint16_t current, uint16_t period, uint16_t pulse)
{
	uint32_t dac, j;

	//calculate the dac value
	dac = ExposureCurrentToDac(current);

	if(port >= EXPOSURE_NUMBER_OF_PORTS)
	{
		return;
	}
	if(line > EXPOSURE_NUMBER_OF_LINES)
	{
		return;
	}

	//switch all lines off
	for( j = 0 ; j < EXPOSURE_NUMBER_OF_LINES ; j++)
	{
		GPIO_ResetBits(ExposureHW[port].lineGPIO[j].port, ExposureHW[port].lineGPIO[j].pin);
	}

		/* PWM */
	ExposureHW[port].timer->ARR = period;
	ExposureHW[port].timer->CCR1 = pulse;
		// DAC1-Wert setzen (12Bit, rechtsb�ndig)
	  if(ExposureHW[port].dacChannel == DAC_Channel_1)
	  {
		  DAC_SetChannel1Data(DAC_Align_12b_R, (uint16_t)dac);
	  }
	  if(ExposureHW[port].dacChannel == DAC_Channel_2)
	  {
		  DAC_SetChannel2Data(DAC_Align_12b_R, (uint16_t)dac);
	  }

	  /* activate the line */
	  GPIO_SetBits(ExposureHW[port].lineGPIO[line].port, ExposureHW[port].lineGPIO[line].pin);

	  /* remember the active line */
	  ExposureControl[port].active = 1;
	  ExposureControl[port].line = line;

	  /* remember the current */
	  ExposureControl[port].currentset = current;
}

// ************************************************************* //
// ***       Function: SwitchExposureOff                     *** //
// ***       Purpose: switches exposure off                  *** //
// ***       Input: port - Exposure Port (0..1)              *** //
// ***       Return: none                                    *** //
// ************************************************************* //
void SwitchExposureOff(unsigned char port)
{
	  int8_t j;

	  if(port >= EXPOSURE_NUMBER_OF_PORTS)
	  {
		  return;
	  }

	  //switch all lines off
	  for( j = 0 ; j < EXPOSURE_NUMBER_OF_LINES ; j++)
	  {
		  GPIO_ResetBits(ExposureHW[port].lineGPIO[j].port, ExposureHW[port].lineGPIO[j].pin);
	  }

	  /* set the minimum DAC value */
	  if(ExposureHW[port].dacChannel == DAC_Channel_1)
	  {
		  DAC_SetChannel1Data(DAC_Align_12b_R, 0);
	  }
	  if(ExposureHW[port].dacChannel == DAC_Channel_2)
	  {
		  DAC_SetChannel2Data(DAC_Align_12b_R, 0);
	  }

	  /* remember the port is deactivated */
	  ExposureControl[port].active = 0;
	  ExposureControl[port].currentset = 0;



}


// ************************************************************* //
// ***       Function: SetExposureModeSychron                *** //
// ***       Purpose: switches Synchron-Mode on/off          *** //
// ***       Input: port - Exposure Port (0..1)              *** //
// ***       Input: state - ON(1) or OFF (0)                 *** //
// ***       Return: none                                    *** //
// ************************************************************* //
void SetExposureModeSychron(unsigned char port, unsigned char state)
{

	uint16_t polarity, opmode;


	  if(port >= EXPOSURE_NUMBER_OF_PORTS)
	  {
		  return;
	  }

	  if(state == ON)
	  {
		  polarity = TIM_OCPolarity_Low;
		  opmode = TIM_OPMode_Single;
		  ExposureASyncEnabled = 1;/* used for ISR Encoder input */
	  }
	  if(state == OFF)
	  {
		  polarity = TIM_OCPolarity_High;
		  opmode = TIM_OPMode_Repetitive;
		  ExposureASyncEnabled = 0;/* used for ISR Encoder input */
	  }

	  TIM_SelectOnePulseMode(ExposureHW[port].timer, opmode);

	  //Timer PWM channel
	  switch(ExposureHW[port].channel)
	  {
		  case 1:
			  TIM_OC1PolarityConfig(ExposureHW[port].timer, polarity);
		  break;
		  case 2:
			  TIM_OC2PolarityConfig(ExposureHW[port].timer, polarity);
		  break;
		  case 3:
			  TIM_OC3PolarityConfig(ExposureHW[port].timer, polarity);
		  break;
		  case 4:
			  TIM_OC4PolarityConfig(ExposureHW[port].timer, polarity);
		  break;
	  }

	  if(state == OFF)
	 {
		  /* Enable the TIM Counter */
		  ExposureHW[port].timer->CR1 |= TIM_CR1_CEN;
	 }




}

// ************************************************************* //
// ***       Function: SetExposureSynchronDelay              *** //
// ***       Purpose: sets Delay in Synchron-Mode            *** //
// ***       Input: port - Exposure Port (0..1)              *** //
// ***       Input: delay in us                              *** //
// ***       Return: none                                    *** //
// ************************************************************* //
void SetExposureSynchronDelay(unsigned char port, uint16_t delay)
{

	uint32_t arr;
	uint32_t ccr;
	uint32_t impuls;

	if(port >= EXPOSURE_NUMBER_OF_PORTS)
	{
		return;
	}

	if(!ExposureASyncEnabled)
	{
		return;
	}

	//get the actual register values
	arr = ExposureHW[port].timer->ARR;// = delay + impuls;
	ccr = ExposureHW[port].timer->CCR1;// = delay;
	//calculate the actual pulse
	impuls = arr - ccr;
	//calculate the new register values
	ExposureHW[port].timer->CCR1 = delay;
	ExposureHW[port].timer->ARR = delay + impuls;


}

// ************************************************************* //
// ***       Function: SetExposureSynchronPulse              *** //
// ***       Purpose: sets Pulse length in Synchron-Mode     *** //
// ***       Input: port - Exposure Port (0..1)              *** //
// ***       Input: pulse in us                              *** //
// ***       Return: none                                    *** //
// ************************************************************* //
void SetExposureSynchronPulse(unsigned char port, uint16_t pulse)
{

	uint32_t delay;

	if(port >= EXPOSURE_NUMBER_OF_PORTS)
	{
		return;
	}

	if(!ExposureASyncEnabled)
	{
		return;
	}

	//get the actual register value for the actual delay
	delay = ExposureHW[port].timer->CCR1;// = delay;
	//calculate the new arr register value
	ExposureHW[port].timer->ARR = delay + pulse;



}

// ************************************************************* //
// ***       Function: ExposureWork                          *** //
// ***       Purpose: cyclic called for current measurement  *** //
// ***       Input: none                                     *** //
// ***       Return: none                                    *** //
// ************************************************************* //
void ExposureWork(void)
{
	//int i;
	uint32_t uitemp;

	if(ExposureControl[0].readsensors)
	{
		ExposureControl[EXPOSURE_A].timeslot++;

		ExposureControl[EXPOSURE_A].brightness = ExposureReadBrightness();//every 10ms


		if(ExposureControl[EXPOSURE_A].timeslot & 0x01)//every 20ms
		{
			ExposureControl[EXPOSURE_A].current = ExposureReadCurrent();//every 20ms
		}

		if((ExposureControl[EXPOSURE_A].timeslot % 9) == 0)//every 90ms
		{
			ExposureControl[EXPOSURE_A].voltage = ExposureReadVoltage();
		}



		if(ExposureControl[EXPOSURE_A].timeslot >= 50)//every 500ms
		{
			ExposureControl[EXPOSURE_A].timeslot = 0;
			ExposureControl[EXPOSURE_A].sensortemperature = ExposureReadSensorTemperature();
		}


	}

	if(LedCurrentCalibration.mode > CURRENT_CALIBRATION_MODE_OFF)
	{
		ExposureWorkCurrentCalibration();
	}












		ExposureControl[EXPOSURE_A].curadc = Current[CUR_EXP];
		/* average calculation */
		if(ExposureControl[EXPOSURE_A].curadcnum < EXPOSURE_CURENT_ADC_AVERAGE_NUMBER)
		{
			ExposureControl[EXPOSURE_A].curadcsum += ExposureControl[EXPOSURE_A].curadc;
			ExposureControl[EXPOSURE_A].curadcnum++;
		}
		else
		{
			uitemp = ExposureControl[EXPOSURE_A].curadcsum / ExposureControl[EXPOSURE_A].curadcnum;
			ExposureControl[EXPOSURE_A].curadcnum = 0;
			/* already in mA */
			ExposureControl[EXPOSURE_A].curaverage = uitemp;
		}
		/* save the atest active current */
		ExposureControl[EXPOSURE_A].curactual = Current[CUR_EXP];
		/* if exposure is on, store it as latest value */
		if(ExposureControl[EXPOSURE_A].active)
		{
			ExposureControl[EXPOSURE_A].curlatest = ExposureControl[EXPOSURE_A].curactual;
		}

		ExposureWatchChanges();


}

// ************************************************************* //
// ***       Function: ExposureSetLine                       *** //
// ***       Purpose: sets line of Exposure                  *** //
// ***       Input: port - 0 or 1                            *** //
// ***       Input: line - LED-Line (1 ... 3, 0 - off)       *** //
// ***       Return: none                                    *** //
// ************************************************************* //
void ExposureSetLine(unsigned char port, unsigned char line)
{
	//int8_t j;

	if(port >= EXPOSURE_NUMBER_OF_PORTS)
	{
		//invalid
		return;
	}
	if((line == 0) || (line > EXPOSURE_NUMBER_OF_LINES))
	{
		//exposure off
		SwitchExposureOff(port);
		return;
	}

	SetExposureModeSychron(port, OFF);
	SwitchExposureOn(port, line-1, ExposureControl[port].currentset, 1000, 1000);



}

// ************************************************************* //
// ***       Function: ExposureSetCurrent                    *** //
// ***       Purpose: sets line of Exposure                  *** //
// ***       Input: port - 0 or 1                            *** //
// ***       Input: current in mA                            *** //
// ***       Return: none                                    *** //
// ************************************************************* //
void ExposureSetCurrent(unsigned char port, uint16_t current)
{
	uint32_t dac;

	//calculate the dac value
	dac = ExposureCurrentToDac(current);

	if(port >= EXPOSURE_NUMBER_OF_PORTS)
	{
		//invalid
		return;
	}

	  /* set the  DAC value */
	  if(ExposureHW[port].dacChannel == DAC_Channel_1)
	  {
		  DAC_SetChannel1Data(DAC_Align_12b_R, (uint16_t)dac);
	  }
	  if(ExposureHW[port].dacChannel == DAC_Channel_2)
	  {
		  DAC_SetChannel2Data(DAC_Align_12b_R, (uint16_t)dac);
	  }

	  /* remember the current */
	  ExposureControl[port].currentset = current;


}

// ************************************************************* //
// ***       Function: ExposureSetPulse                      *** //
// ***       Purpose: sets line of Exposure                  *** //
// ***       Input: port - 0 or 1                            *** //
// ***       Input: period (0...65535) in us                 *** //
// ***       Input: pulse (0...period) in us                 *** //
// ***       Return: none                                    *** //
// ************************************************************* //
void ExposureSetPulse(unsigned char port, uint16_t period, uint16_t pulse)
{

	if(port >= EXPOSURE_NUMBER_OF_PORTS)
	{
		//invalid
		return;
	}

	/* set the PWM */
	ExposureHW[port].timer->ARR = period;
	ExposureHW[port].timer->CCR1 = pulse;


}

// ******************************************************************* //
// ***           ExposureWatchChanges()                            *** //
// ***       watches changes of current, voltage, brightness       *** //
// ***       sends CAN messages, if change detected                *** //
// ***     Input: none                                             *** //
// ***             inc: defines.h                                  *** //
// ******************************************************************* //
void ExposureWatchChanges(void)
{
	uint32_t state;

		/* check, if changes happened */
		if(    (CanExposureInfo.current != ExposureControl[EXPOSURE_A].current)
			|| (CanExposureInfo.voltage != ExposureControl[EXPOSURE_A].voltage) )
		{
			/* store the new values */
			CanExposureInfo.current = ExposureControl[EXPOSURE_A].current;
			CanExposureInfo.voltage = (uint16_t)ExposureControl[EXPOSURE_A].voltage;
			/* build the composite status */
			state = CanExposureInfo.voltage;
			state = state << 16;
			state |= (CanExposureInfo.current & 0xFFFF);
			/* send the message */
			CanAppendMessageToTxQueue(((MyCanId << 5) | CAN_ID_CONTROLLER), CAN_COMMAND_ANSWER_EXPOSURE_CURRENT, state, 0, 0);
		}

		if(    (CanExposureInfo.brightness != ExposureControl[EXPOSURE_A].brightness)
			|| (CanExposureInfo.sensortemperature != ExposureControl[EXPOSURE_A].sensortemperature) )
		{
			/* store the new values */
			CanExposureInfo.brightness = ExposureControl[EXPOSURE_A].brightness;
			CanExposureInfo.sensortemperature = ExposureControl[EXPOSURE_A].sensortemperature;
			/* build the composite status */
			state = CanExposureInfo.sensortemperature;
			state = state << 16;
			state |= CanExposureInfo.brightness;
			/* send the message */
			CanAppendMessageToTxQueue(((MyCanId << 5) | CAN_ID_CONTROLLER), CAN_COMMAND_ANSWER_EXPOSURE_BRIGHTNESS, state, 0, 0);
		}


}

// ******************************************************************** //
// ***      ExposureReadVoltage()                                   *** //
// ***  Reads ADC Value from via I2C and converts it to Voltage     *** //
// ***   return: Exposure Voltage in mV                             *** //
// ******************************************************************** //
int32_t ExposureReadVoltage(void)
{
	int32_t ret;
	uint32_t val;


	val = I2cReadLTC2481();

	ret = (int32_t)val;

	//0.5991 uV/dig (calculated)
	ret = ret * 1000;
	ret = ret / 3420;//? - (measured)

	return ret;

}

// ******************************************************************** //
// ***      ExposureReadCurrent()                                   *** //
// ***  Reads ADC Value from via I2C and converts it to Current     *** //
// ***   return: Exposure Current in mA                             *** //
// ******************************************************************** //
int32_t ExposureReadCurrent(void)
{
	int32_t dat;
	int16_t val;


	val = I2cReadINA236();

	dat = val * 25; //100uA per digit @ 81.92 (25uA @ 20.48)
	dat = dat / 1000;//mA



	return (int16_t)dat;

}

// ******************************************************************** //
// ***      ExposureReadBrightness()                                *** //
// ***  Reads ADC Value from via I2C and converts it to Current     *** //
// ***   return: Exposure Current in mA                             *** //
// ******************************************************************** //
uint32_t ExposureReadBrightness(void)
{
	uint32_t ret;
	uint16_t val;


	val = I2cReadLTC2471();

	// ToDo: calculation
	ret = val;

	return ret;

}

// ******************************************************************** //
// ***      ExposureReadSensorTemperature()                         *** //
// ***  Reads ADC Value from via I2C and converts it to Temperature *** //
// ***   return: Exposure Current in 1/1000�C                       *** //
// ******************************************************************** //
int32_t ExposureReadSensorTemperature(void)
{
	uint32_t ret;
		uint16_t val;

	val = I2cReadTMP275();

	// ToDo: calculation
	ret = val;

	return ret;

}

// ******************************************************************** //
// ***       Function: PwmCalibrationReset                          *** //
// ***       Purpose:  erases calibration values                    *** //
// ***       Input: none                                            *** //
// ***       Return: none                                           *** //
// ******************************************************************** //
void ExposureCalibrationReset(uint8_t line)
{

	if(line >= EXPOSURE_NUMBER_OF_LINES)
	{
		return;
	}

	switch(line)
	{
		case 0:
			I2cAddJob(I2C_JOB_EEPROM_EXT_WRITE_INTEGER, EEPROM_EXT_ADDRESS_CAL_EXP1_POW_OFFSET, 0xFFFFFFFF, 0);
			I2cAddJob(I2C_JOB_EEPROM_EXT_WRITE_INTEGER, EEPROM_EXT_ADDRESS_CAL_EXP1_POW_GRADIENT, 0xFFFFFFFF, 0);
			I2cAddJob(I2C_JOB_EEPROM_EXT_WRITE_INTEGER, EEPROM_EXT_ADDRESS_CAL_EXP1_FOTO_OFFSET, 0xFFFFFFFF, 0);
			I2cAddJob(I2C_JOB_EEPROM_EXT_WRITE_INTEGER, EEPROM_EXT_ADDRESS_CAL_EXP1_FOTO_GRADIENT, 0xFFFFFFFF, 0);
		break;
		case 1:
			I2cAddJob(I2C_JOB_EEPROM_EXT_WRITE_INTEGER, EEPROM_EXT_ADDRESS_CAL_EXP2_POW_OFFSET, 0xFFFFFFFF, 0);
			I2cAddJob(I2C_JOB_EEPROM_EXT_WRITE_INTEGER, EEPROM_EXT_ADDRESS_CAL_EXP2_POW_GRADIENT, 0xFFFFFFFF, 0);
			I2cAddJob(I2C_JOB_EEPROM_EXT_WRITE_INTEGER, EEPROM_EXT_ADDRESS_CAL_EXP2_FOTO_OFFSET, 0xFFFFFFFF, 0);
			I2cAddJob(I2C_JOB_EEPROM_EXT_WRITE_INTEGER, EEPROM_EXT_ADDRESS_CAL_EXP2_FOTO_GRADIENT, 0xFFFFFFFF, 0);
		break;
		case 2:
			I2cAddJob(I2C_JOB_EEPROM_EXT_WRITE_INTEGER, EEPROM_EXT_ADDRESS_CAL_EXP3_POW_OFFSET, 0xFFFFFFFF, 0);
			I2cAddJob(I2C_JOB_EEPROM_EXT_WRITE_INTEGER, EEPROM_EXT_ADDRESS_CAL_EXP3_POW_GRADIENT, 0xFFFFFFFF, 0);
			I2cAddJob(I2C_JOB_EEPROM_EXT_WRITE_INTEGER, EEPROM_EXT_ADDRESS_CAL_EXP3_FOTO_OFFSET, 0xFFFFFFFF, 0);
			I2cAddJob(I2C_JOB_EEPROM_EXT_WRITE_INTEGER, EEPROM_EXT_ADDRESS_CAL_EXP3_FOTO_GRADIENT, 0xFFFFFFFF, 0);
		break;
		case 3:
			I2cAddJob(I2C_JOB_EEPROM_EXT_WRITE_INTEGER, EEPROM_EXT_ADDRESS_CAL_EXP4_POW_OFFSET, 0xFFFFFFFF, 0);
			I2cAddJob(I2C_JOB_EEPROM_EXT_WRITE_INTEGER, EEPROM_EXT_ADDRESS_CAL_EXP4_POW_GRADIENT, 0xFFFFFFFF, 0);
			I2cAddJob(I2C_JOB_EEPROM_EXT_WRITE_INTEGER, EEPROM_EXT_ADDRESS_CAL_EXP4_FOTO_OFFSET, 0xFFFFFFFF, 0);
			I2cAddJob(I2C_JOB_EEPROM_EXT_WRITE_INTEGER, EEPROM_EXT_ADDRESS_CAL_EXP4_FOTO_GRADIENT, 0xFFFFFFFF, 0);
		break;
		case 4:
			I2cAddJob(I2C_JOB_EEPROM_EXT_WRITE_INTEGER, EEPROM_EXT_ADDRESS_CAL_EXP5_POW_OFFSET, 0xFFFFFFFF, 0);
			I2cAddJob(I2C_JOB_EEPROM_EXT_WRITE_INTEGER, EEPROM_EXT_ADDRESS_CAL_EXP5_POW_GRADIENT, 0xFFFFFFFF, 0);
			I2cAddJob(I2C_JOB_EEPROM_EXT_WRITE_INTEGER, EEPROM_EXT_ADDRESS_CAL_EXP5_FOTO_OFFSET, 0xFFFFFFFF, 0);
			I2cAddJob(I2C_JOB_EEPROM_EXT_WRITE_INTEGER, EEPROM_EXT_ADDRESS_CAL_EXP5_FOTO_GRADIENT, 0xFFFFFFFF, 0);
		break;
	}

	ExposureCalibration[line].powcalibrationoffset = EXPOSURE_CAL_DEFAULT_POW_OFFSET;
	ExposureCalibration[line].powcalibrationgradient = EXPOSURE_CAL_DEFAULT_POW_GRADIENT;
	ExposureCalibration[line].fotocalibrationoffset = EXPOSURE_CAL_DEFAULT_FOTO_OFFSET;
	ExposureCalibration[line].fotocalibrationgradient = EXPOSURE_CAL_DEFAULT_FOTO_GRADIENT;
	ExposureCalibration[line].calibrationok = 0;
	//LedGreen(OFF);


}

// ******************************************************************** //
// ***      ExposureCalibrationLow()                                *** //
// ***  sets exposure fotosensor calibration value                  *** //
// ***   Input: none                                                *** //
// ***   return: 1 - successful, 0 - error                          *** //
// ******************************************************************** //
void ExposureCalibrationLow(uint32_t pow)
{
	uint8_t line;

	if(ExposureControl[EXPOSURE_A].active)
	{
		line = ExposureControl[EXPOSURE_A].line;
		ExposureCalibration[line].powlow = pow;
		ExposureCalibration[line].fotolow = ExposureControl[EXPOSURE_A].brightness;
		ExposureCalibration[line].currentsetlow = ExposureControl[EXPOSURE_A].currentset;
	}
	//ToDo: else

}

// ******************************************************************** //
// ***      ExposureCalibrationHigh()                               *** //
// ***  sets exposure fotosensor calibration value                  *** //
// ***   Input: none                                                *** //
// ***   return: 1 - successful, 0 - error                          *** //
// ******************************************************************** //
uint8_t ExposureCalibrationHigh(uint32_t pow)
{
		int32_t powoffset, fotooffset;
		float32_t powgradient, fotogradient;
		int32_t powintgradient, fotointgradient;

		uint8_t line;

		if(!ExposureControl[EXPOSURE_A].active)
		{
			return 0;
		}

		line = ExposureControl[EXPOSURE_A].line;

		//check, if low value was set
		if(ExposureCalibration[line].fotolow == 0)
		{
			return 0;
		}

		ExposureCalibration[line].powhigh = pow;

		ExposureCalibration[line].fotohigh = ExposureControl[EXPOSURE_A].brightness;

		ExposureCalibration[line].currentsethigh = ExposureControl[EXPOSURE_A].currentset;


		/* calculation of offset and gradient for Power */
		powgradient = (ExposureCalibration[line].fotohigh - ExposureCalibration[line].fotolow);
		if(ExposureCalibration[line].powhigh == ExposureCalibration[line].powlow)
		{
			return 0;//prevent div 0
		}
		powgradient =	powgradient / (ExposureCalibration[line].powhigh - ExposureCalibration[line].powlow);
		powintgradient = 1000000 * powgradient;
		powoffset = ExposureCalibration[line].fotohigh - (powgradient * ExposureCalibration[line].powhigh);

		/* calculation of offset and gradient for Foto-Value */
		fotogradient = (ExposureCalibration[line].fotohigh - ExposureCalibration[line].fotolow);
		if(ExposureCalibration[line].powhigh == ExposureCalibration[line].powlow)
		{
			return 0;//prevent div 0
		}
		fotogradient =	fotogradient / (ExposureCalibration[line].powhigh - ExposureCalibration[line].powlow);
		fotointgradient = 1000000 * fotogradient;
		fotooffset = ExposureCalibration[line].fotohigh - (fotogradient * ExposureCalibration[line].powhigh);

		/* store the result */

		if((powoffset > EXPOSURE_CAL_MIN_OFFSET) && (powoffset < EXPOSURE_CAL_MAX_OFFSET) && (powgradient > EXPOSURE_CAL_MIN_GRADIENT) && (powgradient < EXPOSURE_CAL_MAX_GRADIENT))
		{
			switch(line)
			{
				case 0:
					I2cAddJob(I2C_JOB_EEPROM_EXT_WRITE_INTEGER, EEPROM_EXT_ADDRESS_CAL_EXP1_POW_OFFSET, powoffset, 0);
					I2cAddJob(I2C_JOB_EEPROM_EXT_WRITE_INTEGER, EEPROM_EXT_ADDRESS_CAL_EXP1_POW_GRADIENT, powintgradient, 0);
					I2cAddJob(I2C_JOB_EEPROM_EXT_WRITE_INTEGER, EEPROM_EXT_ADDRESS_CAL_EXP1_FOTO_OFFSET, fotooffset, 0);
					I2cAddJob(I2C_JOB_EEPROM_EXT_WRITE_INTEGER, EEPROM_EXT_ADDRESS_CAL_EXP1_FOTO_GRADIENT, fotointgradient, 0);
				break;
				case 1:
					I2cAddJob(I2C_JOB_EEPROM_EXT_WRITE_INTEGER, EEPROM_EXT_ADDRESS_CAL_EXP2_POW_OFFSET, powoffset, 0);
					I2cAddJob(I2C_JOB_EEPROM_EXT_WRITE_INTEGER, EEPROM_EXT_ADDRESS_CAL_EXP2_POW_GRADIENT, powintgradient, 0);
					I2cAddJob(I2C_JOB_EEPROM_EXT_WRITE_INTEGER, EEPROM_EXT_ADDRESS_CAL_EXP2_FOTO_OFFSET, fotooffset, 0);
					I2cAddJob(I2C_JOB_EEPROM_EXT_WRITE_INTEGER, EEPROM_EXT_ADDRESS_CAL_EXP2_FOTO_GRADIENT, fotointgradient, 0);
				break;
				case 2:
					I2cAddJob(I2C_JOB_EEPROM_EXT_WRITE_INTEGER, EEPROM_EXT_ADDRESS_CAL_EXP3_POW_OFFSET, powoffset, 0);
					I2cAddJob(I2C_JOB_EEPROM_EXT_WRITE_INTEGER, EEPROM_EXT_ADDRESS_CAL_EXP3_POW_GRADIENT, powintgradient, 0);
					I2cAddJob(I2C_JOB_EEPROM_EXT_WRITE_INTEGER, EEPROM_EXT_ADDRESS_CAL_EXP3_FOTO_OFFSET, fotooffset, 0);
					I2cAddJob(I2C_JOB_EEPROM_EXT_WRITE_INTEGER, EEPROM_EXT_ADDRESS_CAL_EXP3_FOTO_GRADIENT, fotointgradient, 0);
				break;
				case 3:
					I2cAddJob(I2C_JOB_EEPROM_EXT_WRITE_INTEGER, EEPROM_EXT_ADDRESS_CAL_EXP4_POW_OFFSET, powoffset, 0);
					I2cAddJob(I2C_JOB_EEPROM_EXT_WRITE_INTEGER, EEPROM_EXT_ADDRESS_CAL_EXP4_POW_GRADIENT, powintgradient, 0);
					I2cAddJob(I2C_JOB_EEPROM_EXT_WRITE_INTEGER, EEPROM_EXT_ADDRESS_CAL_EXP4_FOTO_OFFSET, fotooffset, 0);
					I2cAddJob(I2C_JOB_EEPROM_EXT_WRITE_INTEGER, EEPROM_EXT_ADDRESS_CAL_EXP4_FOTO_GRADIENT, fotointgradient, 0);
				break;
				case 4:
					I2cAddJob(I2C_JOB_EEPROM_EXT_WRITE_INTEGER, EEPROM_EXT_ADDRESS_CAL_EXP5_POW_OFFSET, powoffset, 0);
					I2cAddJob(I2C_JOB_EEPROM_EXT_WRITE_INTEGER, EEPROM_EXT_ADDRESS_CAL_EXP5_POW_GRADIENT, powintgradient, 0);
					I2cAddJob(I2C_JOB_EEPROM_EXT_WRITE_INTEGER, EEPROM_EXT_ADDRESS_CAL_EXP5_FOTO_OFFSET, fotooffset, 0);
					I2cAddJob(I2C_JOB_EEPROM_EXT_WRITE_INTEGER, EEPROM_EXT_ADDRESS_CAL_EXP5_FOTO_GRADIENT, fotointgradient, 0);
				break;
			}

			ExposureCalibration[line].powcalibrationoffset = powoffset;
			ExposureCalibration[line].powcalibrationgradient = powgradient;
			ExposureCalibration[line].fotocalibrationoffset = fotooffset;
			ExposureCalibration[line].fotocalibrationgradient = fotogradient;
			ExposureCalibration[line].calibrationok = 1;
				//LedGreen(ON);
		}
		else
		{
				//calculated values out of limit -> calibration failed
				ExposureCalibration[line].powcalibrationoffset = EXPOSURE_CAL_DEFAULT_POW_OFFSET;
				ExposureCalibration[line].powcalibrationgradient = EXPOSURE_CAL_DEFAULT_POW_GRADIENT;
				ExposureCalibration[line].fotocalibrationoffset = EXPOSURE_CAL_DEFAULT_FOTO_OFFSET;
				ExposureCalibration[line].fotocalibrationgradient = EXPOSURE_CAL_DEFAULT_FOTO_GRADIENT;
				ExposureCalibration[line].calibrationok = 0;
				return 0;
		}

		//erase the temporary stored values
		ExposureCalibration[line].fotolow = 0;
		ExposureCalibration[line].fotohigh = 0;
		ExposureCalibration[line].powlow = 0;
		ExposureCalibration[line].powhigh = 0;

		//success
		return 1;
}

// ************************************************************* //
// ***       Function: SwitchExposureRegulatedPowerOn        *** //
// ***       Purpose: switches exposure on                   *** //
// ***       Input: port - pointer to buffer                 *** //
// ***       Input: line - LED-Line (0 ... 4)                *** //
// ***       Input: power - Power [uW]                       *** //
// ***       Return: none                                    *** //
// ************************************************************* //
void SwitchExposureRegulatedPowerOn(unsigned char port, unsigned char line, uint32_t power)
{

	uint16_t dac;
	uint8_t j;
	float32_t f;


		if(port >= EXPOSURE_NUMBER_OF_PORTS)
		{
			return;
		}
		if(line > EXPOSURE_NUMBER_OF_LINES)
		{
			return;
		}

		//switch all lines off
		for( j = 0 ; j < EXPOSURE_NUMBER_OF_LINES ; j++)
		{
			GPIO_ResetBits(ExposureHW[port].lineGPIO[j].port, ExposureHW[port].lineGPIO[j].pin);
		}

		/* PWM - off */
		ExposureHW[port].timer->ARR = 1000;
		ExposureHW[port].timer->CCR1 = 1000;


		//calculate the foto value
		f = ExposureCalibration[line].powcalibrationgradient * power;
		f += ExposureCalibration[line].powcalibrationoffset;
		ExposureControl[port].fotosollwert = (uint16_t)f;

		//calculate the current value
		f = ExposureCalibration[line].fotocalibrationgradient * ExposureControl[port].fotosollwert;
		f += ExposureCalibration[line].fotocalibrationoffset;
		ExposureControl[port].startcurrent = (uint16_t)f;
		dac = ExposureCurrentToDac(ExposureControl[port].startcurrent);


		// DAC1-Wert setzen (12Bit, rechtsb�ndig)
		  if(ExposureHW[port].dacChannel == DAC_Channel_1)
		  {
			  DAC_SetChannel1Data(DAC_Align_12b_R, (uint16_t)dac);
		  }
		  if(ExposureHW[port].dacChannel == DAC_Channel_2)
		  {
			  DAC_SetChannel2Data(DAC_Align_12b_R, (uint16_t)dac);
		  }

		  /* activate the line */
		  GPIO_SetBits(ExposureHW[port].lineGPIO[line].port, ExposureHW[port].lineGPIO[line].pin);

		  /* remember the active line */
		  ExposureControl[port].active = 1;
		  ExposureControl[port].line = line;

		  /* remember the current */
		  ExposureControl[port].currentset = ExposureControl[port].startcurrent;
}

// ************************************************************* //
// ***       Function: ExposureCurrentToDac                  *** //
// ***       Purpose: switches exposure on                   *** //
// ***       Input: current - pointer to buffer              *** //
// ***       Input: line - LED-Line (0 ... 4)                *** //
// ***       Input: power - Power [uW]                       *** //
// ***       Return: DAC-Value                               *** //
// ************************************************************* //
uint16_t ExposureCurrentToDac(uint16_t current)
{
	uint32_t dac;
	float32_t f;

	//use EEPROM current Calibration value
	f = LedCurrentCalibration.calibrationgradient * current;
	f += LedCurrentCalibration.calibrationoffset;
	dac = (uint16_t)f;

	return (uint16_t)dac;
}

// ************************************************************* //
// ***       Function: ExposureStartCurrentCalibration       *** //
// ***       Purpose: work function for current calibration  *** //
// ***       Input: line - LED-Line (0...4)                  *** //
// ***       Return: none                                    *** //
// ************************************************************* //
void ExposureStartCurrentCalibration(uint8_t line)
{
	if(line >= EXPOSURE_NUMBER_OF_LINES)
	{
		return;
	}

	//reset the old values
	I2cAddJob(I2C_JOB_EEPROM_WRITE_INTEGER, EEPROM_ADDRESS_CAL_CURRENT_OFFSET, 0xFFFFFFFF, 0);
	I2cAddJob(I2C_JOB_EEPROM_WRITE_INTEGER, EEPROM_ADDRESS_CAL_CURRENT_GRADIENT, 0xFFFFFFFF, 0);

	//clear the variables
	LedCurrentCalibration.calibrationgradient = 1.0;
	LedCurrentCalibration.calibrationoffset = 0;
	LedCurrentCalibration.samplenum = 0;
	LedCurrentCalibration.samplesum = 0;
	LedCurrentCalibration.timeslot = 0;

	//switch on the LED-Line
	SwitchExposureOn(EXPOSURE_A, line, CURRENT_CALIBRATION_POINT_LOW, 1000, 1000);






	//start the calibration
	LedCurrentCalibration.mode = CURRENT_CALIBRATION_MODE_SET_LOW;


}

// ************************************************************* //
// ***       Function: ExposureWorkCurrentCalibration        *** //
// ***       Purpose: work function for current calibration  *** //
// ***       Input: none                                     *** //
// ***       Return: none                                    *** //
// ************************************************************* //
void ExposureWorkCurrentCalibration(void)
{


	LedCurrentCalibration.timeslot++;

	if(LedCurrentCalibration.timeslot < 10)
	{
		return;
	}
	LedCurrentCalibration.timeslot = 0;

	if(LedCurrentCalibration.delay > 0)
	{
		LedCurrentCalibration.delay--;
		return;
	}


	switch(LedCurrentCalibration.mode)
	{
		case CURRENT_CALIBRATION_MODE_SET_LOW:
			//LED is already on
			LedCurrentCalibration.mode = CURRENT_CALIBRATION_MODE_SAMPLE_LOW;
			LedCurrentCalibration.delay = 3;
			LedCurrentCalibration.samplenum = 0;
			LedCurrentCalibration.samplesum = 0;
		break;
		case CURRENT_CALIBRATION_MODE_SAMPLE_LOW:
			if(LedCurrentCalibration.samplenum < CURRENT_CALIBRATION_SAMPLE_NUMBER)
			{
				LedCurrentCalibration.samplesum += ExposureControl[EXPOSURE_A].current;
				LedCurrentCalibration.samplenum++;
			}
			else
			{
				LedCurrentCalibration.curlow = LedCurrentCalibration.samplesum / LedCurrentCalibration.samplenum;
				//set high current
				ExposureSetCurrent(EXPOSURE_A, CURRENT_CALIBRATION_POINT_HIGH);
				LedCurrentCalibration.mode = CURRENT_CALIBRATION_MODE_SET_HIGH;
			}
		break;
		case CURRENT_CALIBRATION_MODE_SET_HIGH:
			//high current is already set
			LedCurrentCalibration.mode = CURRENT_CALIBRATION_MODE_SAMPLE_HIGH;
			LedCurrentCalibration.delay = 3;
			LedCurrentCalibration.samplenum = 0;
			LedCurrentCalibration.samplesum = 0;
		break;
		case CURRENT_CALIBRATION_MODE_SAMPLE_HIGH:
			if(LedCurrentCalibration.samplenum < CURRENT_CALIBRATION_SAMPLE_NUMBER)
			{
				LedCurrentCalibration.samplesum += ExposureControl[EXPOSURE_A].current;
				LedCurrentCalibration.samplenum++;
			}
			else
			{
				LedCurrentCalibration.curhigh = LedCurrentCalibration.samplesum / LedCurrentCalibration.samplenum;
				//set high current
				SwitchExposureOff(EXPOSURE_A);
				//calculate the offset and gradient
				ExposureCurrentCalculation();
				if(LedCurrentCalibration.calibrationok)
				{
					LedCurrentCalibration.mode = CURRENT_CALIBRATION_MODE_FINISH;
				}
				else
				{
					LedCurrentCalibration.mode = CURRENT_CALIBRATION_MODE_FAILED;
				}
			}
		break;
		case CURRENT_CALIBRATION_MODE_FINISH:
			SendToCommunicationPort("!CurCalOk", 9);
			LedCurrentCalibration.mode = CURRENT_CALIBRATION_MODE_OFF;
		break;
		case CURRENT_CALIBRATION_MODE_FAILED:
			SendToCommunicationPort("!CurCalError", 12);
			LedCurrentCalibration.mode = CURRENT_CALIBRATION_MODE_OFF;
		break;


	}

}

// ************************************************************* //
// ***       Function: ExposureWorkCurrentCalculation        *** //
// ***       Purpose: calculates offset and gradient         *** //
// ***       Input: none                                     *** //
// ***       Return: none                                    *** //
// ************************************************************* //
void ExposureCurrentCalculation(void)
{

	int32_t offset, intgradient;
	float32_t gradient;

	/* calculation of offset and gradient for Current */
	gradient = (CURRENT_CALIBRATION_POINT_HIGH - CURRENT_CALIBRATION_POINT_LOW);
	if(LedCurrentCalibration.curhigh <= LedCurrentCalibration.curlow)
	{
		LedCurrentCalibration.calibrationok = 0;//triggers error message
		return;//prevent div 0
	}
	gradient =	gradient / (LedCurrentCalibration.curhigh - LedCurrentCalibration.curlow);
	intgradient = 1000000 * gradient;//for staring in EEPROM
	offset = CURRENT_CALIBRATION_POINT_HIGH - (gradient * LedCurrentCalibration.curhigh);

	//Plausibility check
	if((gradient > CURRENT_CAL_MIN_GRADIENT) && (gradient < CURRENT_CAL_MAX_GRADIENT))
	{
		LedCurrentCalibration.calibrationgradient = gradient;
		LedCurrentCalibration.calibrationoffset = offset;
		LedCurrentCalibration.calibrationok = 1;//triggers success message

		//Store the values
		I2cAddJob(I2C_JOB_EEPROM_WRITE_INTEGER, EEPROM_ADDRESS_CAL_CURRENT_OFFSET, offset, 0);
		I2cAddJob(I2C_JOB_EEPROM_WRITE_INTEGER, EEPROM_ADDRESS_CAL_CURRENT_GRADIENT, intgradient, 0);
		return;

	}
	else
	{
		LedCurrentCalibration.calibrationoffset = 0;
		LedCurrentCalibration.calibrationgradient = 1.0;
		LedCurrentCalibration.calibrationok = 0;//triggers error message
		return;
	}


}




