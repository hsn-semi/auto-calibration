/*********************************************************************/
/***  Project:       MultitubController                            ***/
/***  File:          stepmotor.c                                   ***/
/***  derived on:    14.07.2021  from OneOpticController2          ***/
/***  Author:        Andreas Stärker                               ***/
/***  Company:       Blink AG                                      ***/
/***  Changes:                                                     ***/
/***                                                               ***/
/*********************************************************************/

//#include "stm32f4_discovery.h"
#include "stm32f4xx_conf.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "defines.h"

#include "can.h"
#include "i2c.h"


#include "stepmotor.h"
#include "main.h"



//variables:
TypeStepmotorControl StepmotorControl[NUMBER_OF_STEPMOTORS];
TypeStepmotorHW StepmotorHW[NUMBER_OF_STEPMOTORS];




// ********************************************** //
// ***       Function: StepmotorInit          *** //
// ***       Purpose: init Stepmotor          *** //
// ***       Input: none                      *** //
// ***       Return: none                     *** //
// ********************************************** //
void StepmotorInit(void)
{

	  GPIO_InitTypeDef 		GPIO_InitStructure;
	  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	  NVIC_InitTypeDef NVIC_InitStructure;
	  EXTI_InitTypeDef	EXTI_InitStructure;
	  //DAC_InitTypeDef  DAC_InitStructure;
	  
	  uint16_t Prescalerval;
	  int32_t minpos, maxpos;

	  int i;

	  /* Enable SYSCFG clock */
	  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
	
	  /* GPIO clock enable */
	  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_GPIOD, ENABLE);
	  //RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD | RCC_AHB1Periph_GPIOE, ENABLE);

	  /* fill the Hardware structure with content */
	  /* Stepmotor X */
	  StepmotorHW[MOTOR_X].enableGPIO.port = STEPMOTOR_X_EN_GPIO_PORT;
	  StepmotorHW[MOTOR_X].enableGPIO.pin = STEPMOTOR_X_EN_PIN;
	  StepmotorHW[MOTOR_X].sleepGPIO.port = STEPMOTOR_X_SLEEP_GPIO_PORT;
	  StepmotorHW[MOTOR_X].sleepGPIO.pin = STEPMOTOR_X_SLEEP_PIN;
	  StepmotorHW[MOTOR_X].stepGPIO.port = STEPMOTOR_X_STEP_GPIO_PORT;
	  StepmotorHW[MOTOR_X].stepGPIO.pin = STEPMOTOR_X_STEP_PIN;
	  StepmotorHW[MOTOR_X].dirGPIO.port = STEPMOTOR_X_DIR_GPIO_PORT;
	  StepmotorHW[MOTOR_X].dirGPIO.pin = STEPMOTOR_X_DIR_PIN;
	  StepmotorHW[MOTOR_X].lbGPIO.port = STEPMOTOR_LB_GPIO_PORT;
	  StepmotorHW[MOTOR_X].lbGPIO.pin = STEPMOTOR_LB_PIN;
	  StepmotorHW[MOTOR_X].refGPIO.port = STEPMOTOR_X_REF_GPIO_PORT;
	  StepmotorHW[MOTOR_X].refGPIO.pin = STEPMOTOR_X_REF_PIN;
	  StepmotorHW[MOTOR_X].encaGPIO.port = STEPMOTOR_X_ENCA_GPIO_PORT;
	  StepmotorHW[MOTOR_X].encaGPIO.pin = STEPMOTOR_X_ENCA_PIN;
	  StepmotorHW[MOTOR_X].encbGPIO.port = STEPMOTOR_X_ENCB_GPIO_PORT;
	  StepmotorHW[MOTOR_X].encbGPIO.pin = STEPMOTOR_X_ENCB_PIN;
	  StepmotorHW[MOTOR_X].refdir = STEPMOTOR_X_REFERENCE_DRIVE_DIRECTION;
	  StepmotorHW[MOTOR_X].refspeed = STEPMOTOR_X_REFERENCE_DRIVE_SPEED;
	  StepmotorHW[MOTOR_X].refmaxsteps = STEPMOTOR_X_REFERENCE_DRIVE_MAX_STEPS;
	  StepmotorHW[MOTOR_X].refstepsin = STEPMOTOR_X_REFERENCE_DRIVE_STEPS_IN;
	  StepmotorHW[MOTOR_X].encresolution = STEPMOTOR_X_ENCODER_RESOLUTION;
	  StepmotorHW[MOTOR_X].direction = STEPMOTOR_X_DIRECTION;
	  StepmotorHW[MOTOR_X].reflogic = STEPMOTOR_X_REFERENCE_INPUT_LOGIC;
	  StepmotorHW[MOTOR_X].defaultspeed = STEPMOTOR_X_DEFAULT_POSITIONING_SPEED;
	  StepmotorHW[MOTOR_X].minsteppos = STEPMOTOR_X_MIN_POSITION_FULLSTEPS;
	  StepmotorHW[MOTOR_X].maxsteppos = STEPMOTOR_X_MAX_POSITION_FULLSTEPS;
	  StepmotorHW[MOTOR_X].steplosstolerance = STEPMOTOR_X_STEPLOSS_TOLERANCE;
	  /* Stepmotor Y */
	  StepmotorHW[MOTOR_Y].enableGPIO.port = STEPMOTOR_Y_EN_GPIO_PORT;
	  StepmotorHW[MOTOR_Y].enableGPIO.pin = STEPMOTOR_Y_EN_PIN;
	  StepmotorHW[MOTOR_Y].sleepGPIO.port = STEPMOTOR_Y_SLEEP_GPIO_PORT;
	  StepmotorHW[MOTOR_Y].sleepGPIO.pin = STEPMOTOR_Y_SLEEP_PIN;
	  StepmotorHW[MOTOR_Y].stepGPIO.port = STEPMOTOR_Y_STEP_GPIO_PORT;
	  StepmotorHW[MOTOR_Y].stepGPIO.pin = STEPMOTOR_Y_STEP_PIN;
	  StepmotorHW[MOTOR_Y].dirGPIO.port = STEPMOTOR_Y_DIR_GPIO_PORT;
	  StepmotorHW[MOTOR_Y].dirGPIO.pin = STEPMOTOR_Y_DIR_PIN;
	  StepmotorHW[MOTOR_Y].lbGPIO.port = STEPMOTOR_LB_GPIO_PORT;
	  StepmotorHW[MOTOR_Y].lbGPIO.pin = STEPMOTOR_LB_PIN;
	  StepmotorHW[MOTOR_Y].refGPIO.port = STEPMOTOR_Y_REF_GPIO_PORT;
	  StepmotorHW[MOTOR_Y].refGPIO.pin = STEPMOTOR_Y_REF_PIN;
	  StepmotorHW[MOTOR_Y].encaGPIO.port = STEPMOTOR_Y_ENCA_GPIO_PORT;
	  StepmotorHW[MOTOR_Y].encaGPIO.pin = STEPMOTOR_Y_ENCA_PIN;
	  StepmotorHW[MOTOR_Y].encbGPIO.port = STEPMOTOR_Y_ENCB_GPIO_PORT;
	  StepmotorHW[MOTOR_Y].encbGPIO.pin = STEPMOTOR_Y_ENCB_PIN;
	  StepmotorHW[MOTOR_Y].refdir = STEPMOTOR_Y_REFERENCE_DRIVE_DIRECTION;
	  StepmotorHW[MOTOR_Y].refspeed = STEPMOTOR_Y_REFERENCE_DRIVE_SPEED;
	  StepmotorHW[MOTOR_Y].refmaxsteps = STEPMOTOR_Y_REFERENCE_DRIVE_MAX_STEPS;
	  StepmotorHW[MOTOR_Y].refstepsin = STEPMOTOR_Y_REFERENCE_DRIVE_STEPS_IN;
	  StepmotorHW[MOTOR_Y].encresolution = STEPMOTOR_Y_ENCODER_RESOLUTION;
	  StepmotorHW[MOTOR_Y].direction = STEPMOTOR_Y_DIRECTION;
	  StepmotorHW[MOTOR_Y].reflogic = STEPMOTOR_Y_REFERENCE_INPUT_LOGIC;
	  StepmotorHW[MOTOR_Y].defaultspeed = STEPMOTOR_Y_DEFAULT_POSITIONING_SPEED;
	  StepmotorHW[MOTOR_Y].minsteppos = STEPMOTOR_Y_MIN_POSITION_FULLSTEPS;
	  StepmotorHW[MOTOR_Y].maxsteppos = STEPMOTOR_Y_MAX_POSITION_FULLSTEPS;
	  StepmotorHW[MOTOR_Y].steplosstolerance = STEPMOTOR_Y_STEPLOSS_TOLERANCE;





	  for( i = 0 ; i < NUMBER_OF_STEPMOTORS ; i++)/* Loop for all Stepmotors */
	  {
		  /* STEPMOTOR output Pins Configuration */
		  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
		  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
		  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
		  /* enable */
		  GPIO_InitStructure.GPIO_Pin = StepmotorHW[i].enableGPIO.pin;
		  GPIO_Init(StepmotorHW[i].enableGPIO.port, &GPIO_InitStructure);
		  /* sleep */
		  GPIO_InitStructure.GPIO_Pin = StepmotorHW[i].sleepGPIO.pin;
		  GPIO_Init(StepmotorHW[i].sleepGPIO.port, &GPIO_InitStructure);
		  /* step */
		  GPIO_InitStructure.GPIO_Pin = StepmotorHW[i].stepGPIO.pin;
		  GPIO_Init(StepmotorHW[i].stepGPIO.port, &GPIO_InitStructure);
		  /* direction */
		  GPIO_InitStructure.GPIO_Pin = StepmotorHW[i].dirGPIO.pin;
		  GPIO_Init(StepmotorHW[i].dirGPIO.port, &GPIO_InitStructure);
		  /* lightbarrier enable */
		  GPIO_InitStructure.GPIO_Pin = StepmotorHW[i].lbGPIO.pin;
		  GPIO_Init(StepmotorHW[i].lbGPIO.port, &GPIO_InitStructure);

		  /* STEPMOTOR input Pins Configuration */
		  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
		  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
		  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
		  /* Reference */
		  GPIO_InitStructure.GPIO_Pin = StepmotorHW[i].refGPIO.pin;
		  GPIO_Init(StepmotorHW[i].refGPIO.port, &GPIO_InitStructure);
		  /* Encoder ENC A */
		  GPIO_InitStructure.GPIO_Pin = StepmotorHW[i].encaGPIO.pin;
		  GPIO_Init(StepmotorHW[i].encaGPIO.port, &GPIO_InitStructure);
		  /* Encoder ENC B */
		  GPIO_InitStructure.GPIO_Pin = StepmotorHW[i].encbGPIO.pin;
		  GPIO_Init(StepmotorHW[i].encbGPIO.port, &GPIO_InitStructure);

		  memset(&StepmotorControl[i], 0, sizeof(StepmotorControl[i]));

	  }

	  /* STEPMOTOR Endpos input Pins  */
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
	  /* Focus */
	  GPIO_InitStructure.GPIO_Pin = STEPMOTOR_Y_END_PIN;
	  GPIO_Init(STEPMOTOR_Y_END_GPIO_PORT, &GPIO_InitStructure);



	  /* TIMER clocks enable */
	  EnableTimerPeriphClock(STEPMOTOR_X_TIMER);
	  EnableTimerPeriphClock(STEPMOTOR_Y_TIMER);

	  /* Timer config */
	  Prescalerval = (uint16_t) ((SystemCoreClock /2) / (1000000 * STEPMOTOR_PRESCALER_FACTOR)) - 1;//1000000 -> 1MHz counter frequency

	  TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);

	  /* Time base configuration */
	  TIM_TimeBaseStructure.TIM_Period = STEPMOTOR_PERIODE_IN_US_FOR_MIN_SPEED * STEPMOTOR_PRESCALER_FACTOR;//in us ; 20000 = 20ms
	  //TIM_TimeBaseStructure.TIM_Prescaler = Prescalerval;
	  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	  TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
	  /* Stepmotor X */
	  if(IsTimerOnAPB2(STEPMOTOR_X_TIMER))
	  {
		  TIM_TimeBaseStructure.TIM_Prescaler = Prescalerval * 2;
	  }
	  else
	  {
		  TIM_TimeBaseStructure.TIM_Prescaler = Prescalerval;
	  }
	  TIM_TimeBaseInit(STEPMOTOR_X_TIMER, &TIM_TimeBaseStructure);
	  TIM_ARRPreloadConfig(STEPMOTOR_X_TIMER, ENABLE);
	  /* Stepmotor Y */
	  if(IsTimerOnAPB2(STEPMOTOR_Y_TIMER))
	  {
		  TIM_TimeBaseStructure.TIM_Prescaler = Prescalerval * 2;
	  }
	  else
	  {
		  TIM_TimeBaseStructure.TIM_Prescaler = Prescalerval;
	  }
	  TIM_TimeBaseInit(STEPMOTOR_Y_TIMER, &TIM_TimeBaseStructure);
	  TIM_ARRPreloadConfig(STEPMOTOR_Y_TIMER, ENABLE);


	  /* write timer address in array for indexed runtime control */
	  StepmotorHW[MOTOR_X].timer = STEPMOTOR_X_TIMER;
	  StepmotorHW[MOTOR_Y].timer = STEPMOTOR_Y_TIMER;


	  /* Enable the TIMER X global Interrupt */
	  NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_TIM10_IRQn;
	  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 10;
	  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	  NVIC_Init(&NVIC_InitStructure);
	  /* Enable the TIMER Y global Interrupt */
	  NVIC_InitStructure.NVIC_IRQChannel = TIM1_TRG_COM_TIM11_IRQn;
	  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 11;
	  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	  NVIC_Init(&NVIC_InitStructure);



	  /* Enable the Update Interrupt Request */
	  TIM_ITConfig(STEPMOTOR_X_TIMER, TIM_IT_Update, ENABLE);
	  /* Enable the Update Interrupt Request */
	  TIM_ITConfig(STEPMOTOR_Y_TIMER, TIM_IT_Update, ENABLE);



	   /* Encoder External Interrupts */

	    /* Connect EXTI Lines for encoders to according Port pins */
	    SYSCFG_EXTILineConfig(STEPMOTOR_X_ENCB_PORT_SOURCE, STEPMOTOR_X_ENCB_PIN_SOURCE);/*X ENCB */

	    SYSCFG_EXTILineConfig(STEPMOTOR_Y_ENCB_PORT_SOURCE, STEPMOTOR_Y_ENCB_PIN_SOURCE);/*Disc Encoder ENCB */






	 	 /* Configure EXTI Lines */
	 	 EXTI_InitStructure.EXTI_Line =  (STEPMOTOR_X_ENCB_EXTI_LINE | STEPMOTOR_Y_ENCB_EXTI_LINE);
	 	 EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	 	 EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
	 	 EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	 	 EXTI_Init(&EXTI_InitStructure);

		 /* Enable and set EXTI Line Interrupt to low priority */
		 NVIC_InitStructure.NVIC_IRQChannel = STEPMOTOR_X_ENCB_IRQ | STEPMOTOR_Y_ENCB_IRQ;
		 NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 6;
		 NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
		 NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		 NVIC_Init(&NVIC_InitStructure);


		 StepmotorSetMicrostepConfig(MOTOR_X, MS_CONFIG_SIXTEENTH_STEP);
		 StepmotorSetMicrostepConfig(MOTOR_Y, MS_CONFIG_SIXTEENTH_STEP);


		 StepmotorSetAcceleration(MOTOR_X,STEPMOTOR_X_DEFAULT_ACCELERATION);
		 StepmotorSetAcceleration(MOTOR_Y,STEPMOTOR_Y_DEFAULT_ACCELERATION);

		 StepmotorSetDeceleration(MOTOR_X, STEPMOTOR_X_DEFAULT_DECELERATION);
		 StepmotorSetDeceleration(MOTOR_Y, STEPMOTOR_Y_DEFAULT_DECELERATION);

		 StepmotorSetSpeed(MOTOR_X, STEPMOTOR_X_REFERENCE_DRIVE_SPEED);
		 StepmotorSetSpeed(MOTOR_Y, STEPMOTOR_Y_REFERENCE_DRIVE_SPEED);


		 /* filter init */

		 memset(&Filter, 0, sizeof(Filter));







		 /* Check, if external EEPROM is present */
		 if(IsI2cAddressPresent(I2C1, I2C_ADDRESS_24LC256_EXT))
		 {
			 StepmotorControl[MOTOR_X].lockposition = (int32_t)I2cReadEepromExtInteger(EEPROM_EXT_ADDRESS_FILTER_LOCK_POSITION);//first read command invalid
			 StepmotorControl[MOTOR_X].lockposition = (int32_t)I2cReadEepromExtInteger(EEPROM_EXT_ADDRESS_FILTER_LOCK_POSITION);

			 // Filter Position Limits
			minpos = STEPMOTOR_X_MIN_POSITION_FULLSTEPS * STEPMOTOR_X_ENCODER_RESOLUTION;
			maxpos = STEPMOTOR_X_MAX_POSITION_FULLSTEPS * STEPMOTOR_X_ENCODER_RESOLUTION;
			// Minimum
			Filter.minposition = (int32_t)I2cReadEepromExtInteger(EEPROM_EXT_ADDRESS_FILTER_MIN_POSITION);
			if((Filter.minposition < minpos) || (Filter.minposition > maxpos) || (Filter.minposition == 0xFFFFFFFF))
			{
				Filter.minposition = minpos;
			}
			else
			{
				//Limit is valid
				minpos = Filter.minposition;
			}
			// Maximum
			Filter.maxposition = (int32_t)I2cReadEepromExtInteger(EEPROM_EXT_ADDRESS_FILTER_MAX_POSITION);
			if((Filter.maxposition < minpos) || (Filter.maxposition > maxpos) || (Filter.maxposition == 0xFFFFFFFF))
			{
				Filter.maxposition = maxpos;
			}
			else
			{
				//Limit is valid
				maxpos = Filter.maxposition;
			}
			// Set the values
			StepmotorHW[MOTOR_X].minsteppos = (minpos / STEPMOTOR_X_ENCODER_RESOLUTION);
			StepmotorHW[MOTOR_X].maxsteppos = (maxpos / STEPMOTOR_X_ENCODER_RESOLUTION);

			 /* read  filter positions from EEPROM */
			Filter.position[0] = (int32_t)I2cReadEepromExtInteger(EEPROM_EXT_ADDRESS_FILTER_POSITION_1);//first read command invalid
			Filter.position[0] = (int32_t)I2cReadEepromExtInteger(EEPROM_EXT_ADDRESS_FILTER_POSITION_1);
			if((Filter.position[0] < Filter.minposition) || (Filter.position[0] > Filter.maxposition) || (Filter.position[0] == 0xFFFFFFFF))
			{
				Filter.position[0] = FILTER_1_POSITION;
				Filter.error |= FILTER_ERROR_EEPROM_DATA;
			}
			Filter.position[1] = (int32_t)I2cReadEepromExtInteger(EEPROM_EXT_ADDRESS_FILTER_POSITION_2);
			if((Filter.position[1] < Filter.minposition) || (Filter.position[1] > Filter.maxposition) || (Filter.position[1] == 0xFFFFFFFF))
			{
				Filter.position[1] = FILTER_2_POSITION;
				Filter.error |= FILTER_ERROR_EEPROM_DATA;
			}
			Filter.position[2] = (int32_t)I2cReadEepromExtInteger(EEPROM_EXT_ADDRESS_FILTER_POSITION_3);
			if((Filter.position[2] < Filter.minposition) || (Filter.position[2] > Filter.maxposition) || (Filter.position[2] == 0xFFFFFFFF))
			{
				Filter.position[2] = FILTER_3_POSITION;
				Filter.error |= FILTER_ERROR_EEPROM_DATA;
			}
			Filter.position[3] = (int32_t)I2cReadEepromExtInteger(EEPROM_EXT_ADDRESS_FILTER_POSITION_4);
			if((Filter.position[3] < Filter.minposition) || (Filter.position[3] > Filter.maxposition) || (Filter.position[3] == 0xFFFFFFFF))
			{
				Filter.position[3] = FILTER_4_POSITION;
				Filter.error |= FILTER_ERROR_EEPROM_DATA;
			}
			Filter.position[4] = (int32_t)I2cReadEepromExtInteger(EEPROM_EXT_ADDRESS_FILTER_POSITION_5);
			if((Filter.position[4] < Filter.minposition) || (Filter.position[4] > Filter.maxposition) || (Filter.position[4] == 0xFFFFFFFF))
			{
				Filter.position[4] = FILTER_5_POSITION;
				Filter.error |= FILTER_ERROR_EEPROM_DATA;
			}


		 }
		 else
		 {
			 //set Error
			 Filter.error |= FILTER_ERROR_NO_EXT_EEPROM;

		 }



}



// *********************************************************************** //
// ***       Function: StepmotorStart                                  *** //
// ***       Purpose: starts stepmotor                                 *** //
// ***       Input: motor  MOTOR_X || MOTOR_Y || MOTOR_Z  || MOTOR_E   *** //
// ***       Input: speed -                                            *** //
// ***       Input: steps -  steps to go (in full steps)               *** //
// ***       Input: dir -  direction FW, RW                            *** //
// ***       Return: none                                              *** //
// *********************************************************************** //
void StepmotorStart(uint8_t motor, uint16_t speed, uint32_t steps, int8_t dir)
{

	if(motor >= NUMBER_OF_STEPMOTORS)
	{
		return;
	}


	/* calculate the microsteps from full steps */
    steps = steps * StepmotorControl[motor].mscount;

    StepmotorEnable(motor);
	GPIO_SetBits(StepmotorHW[motor].sleepGPIO.port, StepmotorHW[motor].sleepGPIO.pin);

	/* start ramp */
	StepmotorControl[motor].rampactualspeed = StepmotorControl[motor].rampstartspeed;
	StepmotorControl[motor].rampdelaycount = 0;

	 /* speed */
	StepmotorSetSpeed(motor, speed);

	/* steps */
	StepmotorControl[motor].stepstogo = steps;
	StepmotorControl[motor].counter = 0;


	/* calculate the position, from where the ramp down begins */
	CalculateRampDownTriggerPoint(motor);

	/* direction */
	if(dir == ST)
	{
		StepmotorStop(motor);
		return;
	}
	else
	{
		StepmotorSetDirection(motor, dir);
	}

	/* Start */
	TIM_Cmd(StepmotorHW[motor].timer, ENABLE);

	StepmotorControl[motor].finished = 0;


}

// ************************************************************************ //
// ***       Function: StepmotorSetSpeed                                *** //
// ***       Purpose: sets speed of stepmotor                           *** //
// ***       Input: motor  MOTOR_X || MOTOR_Y || MOTOR_Z  || MOTOR_E    *** //
// ***       Input: speed - for full step mode                          *** //
// ***       Return: none                                               *** //
// ************************************************************************ //
void StepmotorSetSpeed(uint8_t motor, uint16_t speed)
{
	uint16_t arr;

	if(motor >= NUMBER_OF_STEPMOTORS)
	{
		return;
	}


	speed = speed * StepmotorControl[motor].mscount;

	/* no change in start speed (minimal) */
	StepmotorControl[motor].rampstartspeed = STEPMOTOR_DEFAULT_RAMP_START_SPEED * StepmotorControl[motor].mscount;


	StepmotorControl[motor].positioningspeed = speed;
	StepmotorControl[motor].rampfinalspeed = speed;
	StepmotorControl[motor].rampactualspeed = StepmotorControl[motor].rampstartspeed;

	arr = (STEPMOTOR_PERIODE_IN_US_FOR_MIN_SPEED * STEPMOTOR_PRESCALER_FACTOR) / StepmotorControl[motor].rampstartspeed;
	StepmotorHW[motor].timer->ARR = arr;


}

// ************************************************************************ //
// ***       Function: StepmotorSetAcceleration                         *** //
// ***       Purpose: sets Acceleration of stepmotor                    *** //
// ***       Input: motor  MOTOR_X || MOTOR_Y || MOTOR_Z  || MOTOR_E    *** //
// ***       Input: acc - Acceleration; nominal:10                      *** //
// ***       Return: none                                               *** //
// ************************************************************************ //
void StepmotorSetAcceleration(uint8_t motor, uint16_t acc)
{

		if(motor >= NUMBER_OF_STEPMOTORS)
		{
			return;
		}

		if(acc < 0)
	    {
	    	//not possible
	    	return;
	    }

	    StepmotorControl[motor].acceleration = acc;

		/* no change in start speed (minimal) */
		StepmotorControl[motor].rampstartspeed = STEPMOTOR_DEFAULT_RAMP_START_SPEED * StepmotorControl[motor].mscount;

		StepmotorControl[motor].millispeedacc = acc * STEPMOTOR_MILLI_DELTA_PER_RAMP * StepmotorControl[motor].mscount;
		StepmotorControl[motor].millispeedsum = StepmotorControl[motor].rampstartspeed * 1000;

}

// ************************************************************************ //
// ***       Function: StepmotorSetDeceleration                         *** //
// ***       Purpose: sets Deceleration of stepmotor                    *** //
// ***       Input: motor  MOTOR_X || MOTOR_Y || MOTOR_Z  || MOTOR_E    *** //
// ***       Input: acc - Deceleration                                  *** //
// ***       Return: none                                               *** //
// ************************************************************************ //
void StepmotorSetDeceleration(uint8_t motor, uint16_t dec)
{

	if(motor >= NUMBER_OF_STEPMOTORS)
	{
		return;
	}

    if(dec < 0)
    {
    	//not possible
    	return;
    }

    StepmotorControl[motor].deceleration = dec;
    //Calculate the value which decreases the speed (in 1/1000 speed units) per Ramp step
	StepmotorControl[motor].millispeeddec = dec * STEPMOTOR_MILLI_DELTA_PER_RAMP * StepmotorControl[motor].mscount;


}

// ************************************************************************ //
// ***       Function: StepmotorSetDirection                            *** //
// ***       Purpose: sets direction pin of stepmotor                   *** //
// ***       Input: motor  MOTOR_X || MOTOR_Y || MOTOR_Z  || MOTOR_E    *** //
// ***       Input: dir - direction (FW or RW)                          *** //
// ***       Return: none                                               *** //
// ************************************************************************ //
void StepmotorSetDirection(uint8_t motor, int8_t dir)
{

	if(dir == StepmotorHW[motor].direction)
	{
		GPIO_SetBits(StepmotorHW[motor].dirGPIO.port, StepmotorHW[motor].dirGPIO.pin);
	}
	if(dir == (StepmotorHW[motor].direction * -1))
	{
		GPIO_ResetBits(StepmotorHW[motor].dirGPIO.port, StepmotorHW[motor].dirGPIO.pin);
	}
	/* set the variable for counting in Interrupt */
	StepmotorControl[motor].direction = dir;

}

// *********************************************************************** //
// ***       Function: StepmotorStop                                   *** //
// ***       Purpose: stops stepmotor                                  *** //
// ***       Input: motor  MOTOR_X || MOTOR_Y || MOTOR_Z  || MOTOR_E   *** //
// ***       Return: none                                              *** //
// *********************************************************************** //
void StepmotorStop(uint8_t motor)
{
		if(motor >= NUMBER_OF_STEPMOTORS)
		{
			return;
		}


		/* Stop */
		TIM_Cmd(StepmotorHW[motor].timer, DISABLE);
		/* reset the counter */
		StepmotorControl[motor].counter = 0;
		StepmotorControl[motor].stepstogo = 0;
		StepmotorControl[motor].finished = 1;
		/* stop encoder positioning */
		StepmotorControl[motor].encoderpositioning = 0;




}

// ********************************************************************************* //
// ***       Function: StepmotorReferenceDrive                                   *** //
// ***       Purpose: starts reference drive of step motor - for external usage  *** //
// ***       Input: motor  MOTOR_X || MOTOR_Y || MOTOR_Z  || MOTOR_E             *** //
// ***       Return: none                                                        *** //
// ********************************************************************************* //
void StepmotorReferenceDrive(uint8_t motor)
{
	if(motor >= NUMBER_OF_STEPMOTORS)
	{
		return;
	}

	/* switch on light barrier for reference */
	StepmotorSwitchLightBarrier(motor, ON);
	StepmotorStop(motor);
	StepmotorControl[motor].finished = 0;
	/* start reference drive in 200ms */
	StepmotorControl[motor].startref = STEPMOTOR_REFERENCE_START_DELAY;
}

// *************************************************************************** //
// ***       Function: StepmotorReferenceDrive                             *** //
// ***       Purpose: start reference drive  - for internal usage only!!!  *** //
// ***       Input: motor  MOTOR_X || MOTOR_Y || MOTOR_Z  || MOTOR_E       *** //
// ***       Return: none                                                  *** //
// *************************************************************************** //
void StepmotorStartReferenceDrive(uint8_t motor)
{

	if(motor >= NUMBER_OF_STEPMOTORS)
	{
		return;
	}

	if(IsStepmotorInReferenceLightbarrier(motor)) // Level High means no Reflection -> in Reference-Slot
	{
		/* drive out of Reference Point in opposite direction first */
		/* remember that reference drive is started */
		StepmotorControl[motor].reference = STEPMOTOR_REFERENCE_DRIVE_OUT;
		/* drive in opposite direction */
		StepmotorStart(motor, STEPMOTOR_DEFAULT_RAMP_START_SPEED, StepmotorHW[motor].refmaxsteps, (StepmotorHW[motor].refdir * -1));
	}
	else
	{
		/* remember that reference drive is started */
		StepmotorControl[motor].reference = STEPMOTOR_REFERENCE_DRIVE_IN;
		/* drive direction into reference */
		StepmotorStart(motor, StepmotorHW[motor].refspeed, StepmotorHW[motor].refmaxsteps, StepmotorHW[motor].refdir);
	}



}

// *********************************************************************** //
// ***       Function: StepmotorWork                                   *** //
// ***       Purpose: cyclic calling from main                         *** //
// ***       Return: none                                              *** //
// *********************************************************************** //
void StepmotorWork(void)
{

	int8_t i, j, ref;

	for(i = 0 ; i < NUMBER_OF_STEPMOTORS ; i++)
	{

			/* start reference drive after delay is finished */
		    if(StepmotorControl[i].startref == 1)
		    {
		    	StepmotorControl[i].startref = 0;//prevent multiple starts
		    	StepmotorStartReferenceDrive(i);
		    }


			/* watch reference drive of all motors */
			if(StepmotorControl[i].reference == STEPMOTOR_REFERENCE_DRIVE_IN)
			{
				if(StepmotorControl[i].finished)//timed out
				{
					StepmotorControl[i].reference = STEPMOTOR_REFERENCE_DRIVE_FAIL;
					StepmotorControl[i].error |= STEPMOTOR_ERROR_REFERENCE_TIMEOUT;
				}

				if(IsStepmotorInReferenceLightbarrier(i)) //Reference Lightbarrier detected
				{
					//StepmotorStop(MOTOR_X);
					StepmotorControl[i].reference = STEPMOTOR_REFERENCE_DRIVE_MORE_IN;
					/* drive further into lightbarrier */
					StepmotorStart(i, StepmotorHW[i].refspeed, StepmotorHW[i].refstepsin, (StepmotorHW[i].refdir));
				}


			}

			if(StepmotorControl[i].reference == STEPMOTOR_REFERENCE_DRIVE_MORE_IN)
			{
				if(StepmotorControl[i].finished)
				{
					/* execute the reference drive */
					StepmotorStartReferenceDrive(i);
				}
			}

			if(StepmotorControl[i].reference == STEPMOTOR_REFERENCE_DRIVE_OUT)
			{
				if(StepmotorControl[i].finished)//timed out
				{
					StepmotorControl[i].reference = STEPMOTOR_REFERENCE_DRIVE_FAIL;
					StepmotorControl[i].error |= STEPMOTOR_ERROR_REFERENCE_TIMEOUT;
				}

				if(!IsStepmotorInReferenceLightbarrier(i)) //out of Lightbarrier detected
				{
					StepmotorStop(i);
					StepmotorControl[i].reference = STEPMOTOR_REFERENCE_DRIVE_OK;
					/* check, if another reference drive is in process */
					ref = 0;
					for(j = 0 ; j < NUMBER_OF_STEPMOTORS ; j++)
					{
						if( (StepmotorControl[j].reference == STEPMOTOR_REFERENCE_DRIVE_IN)
								|| (StepmotorControl[j].reference == STEPMOTOR_REFERENCE_DRIVE_MORE_IN)
								|| (StepmotorControl[j].reference == STEPMOTOR_REFERENCE_DRIVE_OUT) )
						{
							ref++;
						}
					}
					if(ref == 0)
					{
						//no other reference drive in progress: switch lightbarrier off
						//keep it enabled StepmotorSwitchLightBarrier(i, OFF);
					}
					/* reset the counter */
					StepmotorControl[i].counter = 0;
					StepmotorControl[i].absolutecounter = 0;
					StepmotorControl[i].stepstogo = 0;
					StepmotorControl[i].finished = 1;
					StepmotorControl[i].encoderposition = 0;
					//reset the errors
					StepmotorControl[i].error = 0;
					//set the default speed
					StepmotorSetSpeed(i, StepmotorHW[i].defaultspeed);
				}

			}

			if(StepmotorControl[i].finished)
			{

				if(StepmotorControl[i].encoderpositioning)
				{
					if(abs(StepmotorControl[i].encoderposition - StepmotorControl[i].targetencoderposition) > StepmotorHW[i].encresolution)
					{
						/* position not yet reached -> repeat positioning */
						if((StepmotorControl[i].error & 0x000F) < 0xF)
						{
							StepmotorControl[i].error++;
						}
						if(StepmotorControl[i].positioningtrycount >= ENCODER_POSITIONING_MAX_TRY)
						{
							/* trigger error */
							StepmotorControl[i].error |= STEPMOTOR_ERROR_POSITIONING_FAIL;
							/* exit positioning -> erase positioning flag */
							StepmotorControl[i].encoderpositioning = 0;
						}
						else
						{
							/* increase the counter for error detection and try positioning again */
							StepmotorControl[i].positioningtrycount++;
							StepmotorGoToEncoderPosition(i, StepmotorControl[i].targetencoderposition, 0);

						}
					}
					else
					{
						/* position reached -> erase positioning flag */
						StepmotorControl[i].encoderpositioning = 0;
					}
				}

			}
			else
			{

			}



	}//for i


}

// *********************************************************************** //
// ***       Function: StepmotorRampWork                               *** //
// ***       Purpose: cyclic calling from main every 10ms              *** //
// ***       Return: none                                              *** //
// *********************************************************************** //
void StepmotorRampWork(void)
{
	int8_t i;

		for(i = 0 ; i < NUMBER_OF_STEPMOTORS ; i++)
		{
			if(!StepmotorControl[i].finished)
			{
				if((StepmotorControl[i].stepstogo - StepmotorControl[i].counter) < StepmotorControl[i].rampdowntriggerpoint)
				{
					//Ramp Down
					//set the end speed of the ramp
					//StepmotorControl[i].rampfinalspeed = StepmotorControl[i].rampstartspeed;
					//decrease the speed
					StepmotorControl[i].millispeedsum -= StepmotorControl[i].millispeeddec;
					//set the register
					StepmotorUpdateSpeed(i);
				}
				else
				{
					if(StepmotorControl[i].rampactualspeed < StepmotorControl[i].rampfinalspeed)
					{
						//Ramp Up
						//increase the speed
						StepmotorControl[i].millispeedsum += StepmotorControl[i].millispeedacc;
						//set the register
						StepmotorUpdateSpeed(i);
					}
					else
					{
						//debug
						//LedGreen(ON);
					}
				}
			}

				/* watch changes in position and status and sent it to CAN */
				StepmotorWatchChanges(i);

				/* delay to start reference drive */
				if(StepmotorControl[i].startref > 1)
				{
					StepmotorControl[i].startref--;
				}



		}//for i

		/* delay to deactivate enable pins  */
		if(StepmotorControl[MOTOR_X].finished && StepmotorControl[MOTOR_Y].finished)
		{
			if(StepmotorControl[MOTOR_X].disablecountdown > 0)
			{
				StepmotorControl[MOTOR_X].disablecountdown--;
			}
			if(StepmotorControl[MOTOR_Y].disablecountdown > 0)
			{
				StepmotorControl[MOTOR_Y].disablecountdown--;
			}
			if((StepmotorControl[MOTOR_X].disablecountdown <= 0) && (StepmotorControl[MOTOR_Y].disablecountdown <= 0))
			{
				StepmotorDisable(MOTOR_X);//both steppers on one enable pin
			}
		}



		//LedGreen(OFF);

}

// *********************************************************************** //
// ***       Function: StepmotorUpdateSpeed                            *** //
// ***       Purpose: called from StepmotorRampWork                    *** //
// ***       Return: none                                              *** //
// *********************************************************************** //
void StepmotorUpdateSpeed(uint8_t motor)
{
	uint16_t arr;
	int32_t speed;

	if(motor >= NUMBER_OF_STEPMOTORS)
	{
		return;
	}

	speed = StepmotorControl[motor].millispeedsum / 1000;
	if(speed < STEPMOTOR_DEFAULT_RAMP_START_SPEED * StepmotorControl[motor].mscount)
	{
		speed = STEPMOTOR_DEFAULT_RAMP_START_SPEED * StepmotorControl[motor].mscount;
	}
	if(speed > StepmotorControl[motor].positioningspeed)
	{
		speed = StepmotorControl[motor].positioningspeed;
	}
	StepmotorControl[motor].rampactualspeed = speed;
	arr = (STEPMOTOR_PERIODE_IN_US_FOR_MIN_SPEED * STEPMOTOR_PRESCALER_FACTOR) / StepmotorControl[motor].rampactualspeed;
	StepmotorHW[motor].timer->ARR = arr;

}

// *********************************************************************** //
// ***       Function: StepmotorGoToPosition                           *** //
// ***       Purpose: goes to stepmotor position                       *** //
// ***       Input: motor  MOTOR_X || MOTOR_Y || MOTOR_Z               *** //
// ***       Input: pos -  target position                             *** //
// ***       Return: none                                              *** //
// *********************************************************************** //
void StepmotorGoToPosition(uint8_t motor, int32_t pos)
{

	if(motor >= NUMBER_OF_STEPMOTORS)
	{
		return;
	}


	/* check, if reference drive was successfully done */
	if(StepmotorControl[motor].reference != STEPMOTOR_REFERENCE_DRIVE_OK)
	{
		StepmotorControl[motor].error |= STEPMOTOR_ERROR_NO_REFERENCE;
		return;
	}

	/* check, if Position Min exceed */
	if(pos < StepmotorHW[motor].minsteppos)
	{
		/* set the error flag */
		StepmotorControl[motor].error |= STEPMOTOR_ERROR_BEYOND_MIN_POS;
		return;
	}
	else
	{
		/* erase the error flag */
		StepmotorControl[motor].error &= ~STEPMOTOR_ERROR_BEYOND_MIN_POS;
	}

	/* check, if Position Max exceed */
	if(pos > StepmotorHW[motor].maxsteppos)
	{
		/* set the error flag */
		StepmotorControl[motor].error |= STEPMOTOR_ERROR_BEYOND_MAX_POS;
		return;
	}
	else
	{
		/* erase the error flag */
		StepmotorControl[motor].error &= ~STEPMOTOR_ERROR_BEYOND_MAX_POS;
	}

	/* calculate the microsteps from full steps */
	pos = pos * StepmotorControl[motor].mscount;


	/* set the speed */
	StepmotorControl[motor].rampactualspeed = StepmotorControl[motor].rampstartspeed;
	StepmotorControl[motor].rampfinalspeed = StepmotorControl[motor].positioningspeed;
	StepmotorControl[motor].rampdelaycount = 0;

	/* enable Driver-IC */
	StepmotorEnable(motor);
	GPIO_SetBits(StepmotorHW[motor].sleepGPIO.port, StepmotorHW[motor].sleepGPIO.pin);


	/* steps */
	StepmotorControl[motor].counter = 0;
	if(pos > StepmotorControl[motor].absolutecounter)
	{
		StepmotorSetDirection(motor, FW);
		StepmotorControl[motor].stepstogo = pos - StepmotorControl[motor].absolutecounter;
	}
	else
	{
		StepmotorSetDirection(motor, RW);
		StepmotorControl[motor].stepstogo = StepmotorControl[motor].absolutecounter - pos;
	}

	/* calculate the position, from where the ramp down begins */
	CalculateRampDownTriggerPoint(motor);

	/* Start */
	TIM_Cmd(StepmotorHW[motor].timer, ENABLE);

	StepmotorControl[motor].finished = 0;


}

// ************************************************************************** //
// ***       Function: StepmotorGoToEncoderPosition                       *** //
// ***       Purpose: goes to stepmotor position                          *** //
// ***       Input: motor  MOTOR_X || MOTOR_Y || MOTOR_Z                  *** //
// ***       Input: pos -  target encoder position                        *** //
// ***       Input: useoffset -  1 - use the table-offset 0 - no offset   *** //
// ***       Return: none                                                 *** //
// ************************************************************************** //
void StepmotorGoToEncoderPosition(uint8_t motor, int32_t pos, uint8_t useoffset)
{

	int32_t i;
	//uint8_t permit;

	if(motor >= NUMBER_OF_STEPMOTORS)
	{
		return;
	}

	//add the offset
	if(useoffset)
	{
		pos += StepmotorControl[motor].encoderoffset;
	}



	//Clear the error
	StepmotorControl[motor].error &= ~STEPMOTOR_ERROR_NOT_PERMITTED;


	/* check, if reference drive was successfully done */
	if(StepmotorControl[motor].reference != STEPMOTOR_REFERENCE_DRIVE_OK)
	{
		StepmotorControl[motor].error |= STEPMOTOR_ERROR_NO_REFERENCE;
		return;
	}


	/* check, if Position Min exceed */
	if(pos < (StepmotorHW[motor].minsteppos * StepmotorHW[motor].encresolution))
	{
		/* set the error flag */
		StepmotorControl[motor].error |= STEPMOTOR_ERROR_BEYOND_MIN_POS;
		return;
	}
	else
	{
		/* erase the error flag */
		StepmotorControl[motor].error &= ~STEPMOTOR_ERROR_BEYOND_MIN_POS;
	}

	/* check, if Position Max exceed */
	if(pos > (StepmotorHW[motor].maxsteppos * StepmotorHW[motor].encresolution))
	{
		/* set the error flag */
		StepmotorControl[motor].error |= STEPMOTOR_ERROR_BEYOND_MAX_POS;
		return;
	}
	else
	{
		/* erase the error flag */
		StepmotorControl[motor].error &= ~STEPMOTOR_ERROR_BEYOND_MAX_POS;
	}

	/* erase the liquid error flag */
	StepmotorControl[motor].error &= ~STEPMOTOR_ERROR_MAX_FORCE_DETECTED;

	/* erase the positioning error flag */
	StepmotorControl[motor].error &= ~STEPMOTOR_ERROR_POSITIONING_FAIL;

	if(StepmotorControl[motor].encoderpositioning == 0)
	{
	    /* reset the counter for positioning error detection */
	    StepmotorControl[motor].positioningtrycount = 0;
	}

	/* set the speed */
	StepmotorControl[motor].rampactualspeed = StepmotorControl[motor].rampstartspeed;
	StepmotorControl[motor].rampfinalspeed = StepmotorControl[motor].positioningspeed;
	StepmotorControl[motor].rampdelaycount = 0;

	/* enable Driver-IC */
	StepmotorEnable(motor);
	GPIO_SetBits(StepmotorHW[motor].sleepGPIO.port, StepmotorHW[motor].sleepGPIO.pin);

	/* increments to go */
    i = pos - StepmotorControl[motor].encoderposition;
    /* full steps to go */
    i = i / StepmotorHW[motor].encresolution;
    /* micro steps to go according actual ms setting*/
    i = i * StepmotorControl[motor].mscount;



	/* steps */
	StepmotorControl[motor].counter = 0;
	if(i > 0)
	{
		StepmotorSetDirection(motor, FW);
		StepmotorControl[motor].stepstogo = i;
	}
	else
	{
		StepmotorSetDirection(motor, RW);
		StepmotorControl[motor].stepstogo = i * -1;
	}

	/* calculate the position, from where the ramp down begins */
	CalculateRampDownTriggerPoint(motor);

	/* Start */
	TIM_Cmd(StepmotorHW[motor].timer, ENABLE);

	StepmotorControl[motor].finished = 0;

	/* set the variables for position correction */
	StepmotorControl[motor].encoderpositioning = 1;
	StepmotorControl[motor].targetencoderposition = pos;



}

// *********************************************************************** //
// ***       Function: StepmotorSetMicrostepConfig                     *** //
// ***       Purpose: Dummy: fixed ms at 16th                          *** //
// ***       Input: motor  MOTOR_X                                     *** //
// ***       Input: ms -  microstep-configuration                      *** //
// ***       Return: none                                              *** //
// *********************************************************************** //
void StepmotorSetMicrostepConfig(uint8_t motor, uint8_t ms)
{

	uint8_t oldmscount = 1;
	//int32_t fullstepcount = 0;

	/* remember the old microstep settings for calculating the actual position*/
	oldmscount = StepmotorControl[motor].mscount;


	StepmotorControl[motor].mscount = 16;


	/* scale the speed-values to the microstep setting */
	//StepmotorSetSpeed(motor, StepmotorControl[motor].positioningspeed  / oldmscount);

	/* scale the acceleration-values to the microstep setting */
	StepmotorSetAcceleration(motor, StepmotorControl[motor].acceleration);
	StepmotorSetDeceleration(motor, StepmotorControl[motor].deceleration);

}



// ***************************************************************************** //
// ***       Function:    IsStepmotorBeg                                     *** //
// ***       Purpose:  reads End switch Beg pin                              *** //
// ***       Input:   motor  MOTOR_X || MOTOR_Y || MOTOR_Z                   *** //
// ***       Return: 1 - active, 0 - passive                                 *** //
// ***************************************************************************** //
uint8_t IsStepmotorBeg(uint8_t motor)
{

		if(GPIO_ReadInputDataBit(StepmotorHW[motor].begGPIO.port, StepmotorHW[motor].begGPIO.pin))
		{
			//High = passive
			return 0;
		}

		//Low = active
		return 1;

}

// ***************************************************************************** //
// ***       Function:    IsStepmotorEnd                                     *** //
// ***       Purpose:  reads End  pin                                        *** //
// ***       Input:   motor  MOTOR_X || MOTOR_Y || MOTOR_Z                   *** //
// ***       Return: 1 - active, 0 - passive                                 *** //
// ***************************************************************************** //
uint8_t IsStepmotorEnd(uint8_t motor)
{

		if(GPIO_ReadInputDataBit(StepmotorHW[motor].endGPIO.port, StepmotorHW[motor].endGPIO.pin))
		{
				//High = passive
				return 0;
		}

		//Low = active
		return 1;

}

// ******************************************************************* //
// ***           SwitchLightBarier()                               *** //
// ***       switches 3P3 of Light Barrier                         *** //
// ***  Input: motor  MOTOR_X || MOTOR_Y || MOTOR_Z  || MOTOR_E    *** //
// ***       ON - switches Voltage on                              *** //
// ***       OFF - switches Voltage  off                           *** //
// ***             inc: defines.h                                  *** //
// ******************************************************************* //
void StepmotorSwitchLightBarrier(uint8_t motor, uint8_t state)
{
	if(state == ON)
	{
		GPIO_ResetBits(StepmotorHW[motor].lbGPIO.port, StepmotorHW[motor].lbGPIO.pin);
	}
	else
	{
		GPIO_SetBits(StepmotorHW[motor].lbGPIO.port, StepmotorHW[motor].lbGPIO.pin);
	}
}

// ***************************************************************************** //
// ***       Function:    IsStepmotorInReferenceLightbarrier                 *** //
// ***       Purpose:  reads Level of Ref-Pin                                *** //
// ***          Lightbarrier output through inverting Schmitt-Trigger        *** //
// ***  Input: motor  MOTOR_X || MOTOR_Y || MOTOR_Z  || MOTOR_E              *** //
// ***       Return: 1 - in Lightbarrier, 0 - out of Lightbarrier            *** //
// ***************************************************************************** //
uint8_t IsStepmotorInReferenceLightbarrier(uint8_t motor)
{

		if(GPIO_ReadInputDataBit(StepmotorHW[motor].refGPIO.port, StepmotorHW[motor].refGPIO.pin))
		{
			if(StepmotorHW[motor].reflogic == ACTIVE_LOW)
			{
				return 0;// Level Low means no Light -> Position out of Lightbarrier-Slot
			}
			return 1;//if input is active high
		}

		if(StepmotorHW[motor].reflogic == ACTIVE_LOW)
		{
			return 1;// Level High means Light -> Position in Lightbarrier-Slot
		}
		return 0;//if input is active high

}

// ***************************************************************************** //
// ***       Function:    CalculateRampDownTriggerPoint                      *** //
// ***       Purpose:  calculation of step position of start of decelaration *** //
// ***                                                                       *** //
// ***  Input: motor  MOTOR_X || MOTOR_Y || MOTOR_Z  || MOTOR_E              *** //
// ***       Return: none                                                    *** //
// ***************************************************************************** //
void CalculateRampDownTriggerPoint(uint8_t motor)
{

	int32_t millistepsup, millistepsdown, rampstepsup, rampstepsdown;
	int32_t millispeeddown, millispeedup, intervalus, millistepstogo, millifinalspeed, temp;

	//LedRed(ON);

	//set the deceleration-step in 1/1000 speed units
	StepmotorControl[motor].millispeeddec = StepmotorControl[motor].deceleration * STEPMOTOR_MILLI_DELTA_PER_RAMP * StepmotorControl[motor].mscount;
	StepmotorControl[motor].millispeedacc = StepmotorControl[motor].acceleration * STEPMOTOR_MILLI_DELTA_PER_RAMP * StepmotorControl[motor].mscount;
	StepmotorControl[motor].millispeedsum = StepmotorControl[motor].rampstartspeed * 1000;

	millistepstogo = StepmotorControl[motor].stepstogo * 1000;
	millifinalspeed = StepmotorControl[motor].rampfinalspeed * 1000;
	temp = 0;
	rampstepsup = 0;
	rampstepsdown = 0;
	millistepsdown = 0;
	millistepsup = 0;
	millispeeddown = StepmotorControl[motor].rampstartspeed * 1000;
	millispeedup = StepmotorControl[motor].rampstartspeed * 1000;
	while(1)
	{
		if(millispeeddown >= millifinalspeed)
		{
			break;
		}
		while(millispeeddown <= millispeedup)
		{
			intervalus = 1000000 * STEP_INTERVAL / millispeeddown;
			temp = RAMP_WORK_INTERVAL * 1000000 / intervalus;
			millistepsdown += temp;
			rampstepsdown++;
			millispeeddown += StepmotorControl[motor].millispeeddec;
		}
		while(millispeedup < millispeeddown)
		{
			intervalus = 1000000 * STEP_INTERVAL / millispeedup;
			millistepsup += RAMP_WORK_INTERVAL * 1000000 / intervalus;
			rampstepsup++;
			millispeedup += StepmotorControl[motor].millispeedacc;
		}
		if((millistepsup + millistepsdown) > millistepstogo)
		{
			break;
		}
	}

	StepmotorControl[motor].rampdowntriggerpoint = (millistepsdown - temp) / 1000;//subtract the steps of the last ramp interval

	//LedRed(OFF);


}

// ******************************************************************* //
// ***           StepmotorWatchChanges()                           *** //
// ***       watches changes of status, position and encoder       *** //
// ***       sends CAN messages, if change detected                *** //
// ***  Input: motor  MOTOR_X || MOTOR_Y || MOTOR_Z  || MOTOR_E    *** //
// ***             inc: defines.h                                  *** //
// ******************************************************************* //
void StepmotorWatchChanges(uint8_t motor)
{
	uint32_t state;

		/* check, if changes happend */
		if(    (CanStepperInfo[MyCanId][motor].finished != StepmotorControl[motor].finished)
			|| (CanStepperInfo[MyCanId][motor].reference != StepmotorControl[motor].reference)
			|| (CanStepperInfo[MyCanId][motor].error != StepmotorControl[motor].error)  )
		{
			/* store the new values */
			CanStepperInfo[MyCanId][motor].finished = StepmotorControl[motor].finished;
			CanStepperInfo[MyCanId][motor].reference = StepmotorControl[motor].reference;
			CanStepperInfo[MyCanId][motor].error = StepmotorControl[motor].error;
			/* build the composite status */
			state = StepmotorControl[motor].finished;
			state = state << 23;
			state |= (StepmotorControl[motor].error & 0xFFFF);
			state = state << 8;
			state |= (StepmotorControl[motor].reference& 0xFF);
			/* send the message */
			CanAppendMessageToTxQueue(((MyCanId << 5) | CAN_ID_CONTROLLER),StepmotorCanCommand[motor].answerstate, state, 0, 0);
		}

		/* check, if changes happend */
		if(CanStepperInfo[MyCanId][motor].position != StepmotorControl[motor].absolutecounter/StepmotorControl[motor].mscount)
		{
			/* store the new value */
			CanStepperInfo[MyCanId][motor].position = StepmotorControl[motor].absolutecounter/StepmotorControl[motor].mscount;
			/* send the message */
			CanAppendMessageToTxQueue(((MyCanId << 5) | CAN_ID_CONTROLLER),StepmotorCanCommand[motor].answerposition, CanStepperInfo[MyCanId][motor].position, 0, 0);
		}

		/* check, if changes happend */
		if(CanStepperInfo[MyCanId][motor].encoderposition != StepmotorControl[motor].encoderposition)
		{
			/* store the new value */
			CanStepperInfo[MyCanId][motor].encoderposition = StepmotorControl[motor].encoderposition;
			/* send the message */
			CanAppendMessageToTxQueue(((MyCanId << 5) | CAN_ID_CONTROLLER),StepmotorCanCommand[motor].answerencoder, CanStepperInfo[MyCanId][motor].encoderposition, 0, 0);
		}

		/* check, if changes happend */

		if(CanFilterInfo.position != StepmotorControl[MOTOR_X].encoderposition)
		{
			// store the new value
			CanFilterInfo.position = StepmotorControl[MOTOR_X].encoderposition;
			// send the message
			//Don't send this redundant data:
			//CanAppendMessageToTxQueue(((MyCanId << 5) | CAN_ID_CONTROLLER), CAN_COMMAND_FILTER_ANSWER_POSITION, CanFilterInfo.position, 0, 0);
		}


		Filter.finished = StepmotorControl[MOTOR_X].finished;
		Filter.error = (Filter.error & 0x000F) | (StepmotorControl[MOTOR_X].error & 0xFFF0);
		/* check, if changes happend */
		if(   (CanFilterInfo.finished != Filter.finished)
				|| (CanFilterInfo.error != Filter.error)
				|| (CanFilterInfo.filterpos != Filter.actualfilter) )//error of filter servo (SERVO_B)
		{
			/* store the new value */
			CanFilterInfo.finished = Filter.finished;
			CanFilterInfo.error = Filter.error;
			CanFilterInfo.filterpos = Filter.actualfilter;
			/* create the actual state */
			state = (CanFilterInfo.error << 16) | (CanFilterInfo.filterpos << 8) | CanFilterInfo.finished;
			/* send the message */
			CanAppendMessageToTxQueue(((MyCanId << 5) | CAN_ID_CONTROLLER), CAN_COMMAND_FILTER_ANSWER_STATE, state, 0, 0);
		}
}

// ******************************************************************* //
// ***           StepmotorSetCurrent()                             *** //
// ***       Dummy: fixed current setting                          *** //
// ***  Input:  -                                                  *** //
// ***         cur -                                               *** //
// ******************************************************************* //
void StepmotorSetCurrent(uint8_t motor, uint16_t cur)
{

	return;


}

// ******************************************************************* //
// ***           StepmotorLiquidDetect()                           *** //
// ***       called from extern to stop motors which expect touch  *** //
// ***            Input: none                                      *** //
// ******************************************************************* //
void StepmotorLiquidDetect(void)
{
	int8_t i;

	for(i = 0 ; i < NUMBER_OF_STEPMOTORS ; i++)
	{
		if( !StepmotorControl[i].finished  && StepmotorControl[i].stopontouch)
		{
			StepmotorStop(i);
		}
	}
}

// ******************************************************************* //
// ***           StepmotorSteplossDetect()                         *** //
// ***       cyclic called every 10ms to detect steploss           *** //
// ***            Input: none                                      *** //
// ******************************************************************* //
void StepmotorSteplossDetect(void)
{
	uint8_t i;
	int32_t deltastep, deltaencoder;

	for(i = 0 ; i < NUMBER_OF_STEPMOTORS ; i++)
	{
		if( !StepmotorControl[i].finished)
		{
			deltastep = StepmotorControl[i].absolutecounter - StepmotorControl[i].lastabsolutecounter;
			deltastep = deltastep / StepmotorControl[i].mscount;//calculate the fullsteps
			deltaencoder = StepmotorControl[i].encoderposition - StepmotorControl[i].lastencoderposition;
			deltaencoder = deltaencoder / StepmotorHW[i].encresolution;//calculate the fullsteps actually made
			// check, if steploss happend
			StepmotorControl[i].steploss = abs(deltastep) - abs(deltaencoder);
			if(StepmotorControl[i].steploss > StepmotorHW[i].steplosstolerance)
			{
				//positioning error
				StepmotorControl[i].error |= STEPMOTOR_ERROR_POSITIONING_FAIL;
				StepmotorStop(i);
			}
		}
		//store the actual values
		StepmotorControl[i].lastabsolutecounter = StepmotorControl[i].absolutecounter;
		StepmotorControl[i].lastencoderposition = StepmotorControl[i].encoderposition;
	}
}

// ******************************************************************* //
// ***           StepmotorEnable()                                 *** //
// ***       enables stepmotor driver                              *** //
// ***            Input: motor                                      *** //
// ******************************************************************* //
void StepmotorEnable(uint8_t motor)
{
	GPIO_ResetBits(StepmotorHW[motor].enableGPIO.port, StepmotorHW[motor].enableGPIO.pin);
	StepmotorControl[motor].disablecountdown = STEPMOTOR_DISABLE_DELAY;
}

// ******************************************************************* //
// ***           StepmotorDisable()                                *** //
// ***       disables stepmotor driver                             *** //
// ***            Input: motor                                     *** //
// ******************************************************************* //
void StepmotorDisable(uint8_t motor)
{
	GPIO_SetBits(StepmotorHW[motor].enableGPIO.port, StepmotorHW[motor].enableGPIO.pin);
}



// *********************************************************************** //
// ***       Function: StepmotorSetFilterPosition                      *** //
// ***       Purpose: sets Filter                                      *** //
// ***       Input: motor                                              *** //
// ***       Input: filter  0 - 4                                      *** //
// ***       Return: none                                              *** //
// *********************************************************************** //
void StepmotorSetFilterPosition(uint8_t motor, uint8_t filter)
{
	int32_t position = 0;


	if(motor >= NUMBER_OF_STEPMOTORS)
	{
		return;
	}

	//reset error
	StepmotorControl[motor].error = 0;


	switch(filter)
	{
	case 1:
		position = Filter.position[0];
	break;
	case 2:
		position = Filter.position[1];
	break;
	case 3:
		position = Filter.position[2];
	break;
	case 4:
		position = Filter.position[3];
	break;
	case 5:
		position = Filter.position[4];
	break;
	case 0:
	default:
		//off
		position = StepmotorControl[motor].encoderposition;// no position change
	}

	StepmotorGoToEncoderPosition(motor, position, 0);


	//ToDo: check the filter position after movement finished
	Filter.actualfilter = filter;






}


// ******************************************************************* //
// ***           StepmotorSetEncoderOffset()                       *** //
// ***       stores Encoder Offset into EEPROM                     *** //
// ***            Input: motor                                     *** //
// ******************************************************************* //
void StepmotorSetEncoderOffset(uint8_t motor, int32_t val)
{

}

// ******************************************************************* //
// ***           StepmotorSetLockPosition()                        *** //
// ***      stores lock position into EEPROM                       *** //
// ***            Input: motor                                     *** //
// ******************************************************************* //
void StepmotorSetLockPosition(uint8_t motor, int32_t val)
{

	if(motor == MOTOR_X)
	{
		I2cAddJob(I2C_JOB_EEPROM_EXT_WRITE_INTEGER, EEPROM_EXT_ADDRESS_FILTER_LOCK_POSITION, val, 0);
		StepmotorControl[motor].lockposition = val;
	}

	if(motor == MOTOR_Y)
	{
		I2cAddJob(I2C_JOB_EEPROM_WRITE_INTEGER, EEPROM_ADDRESS_FOCUS_LOCK_POSITION, val, 0);
		StepmotorControl[motor].lockposition = val;
	}


}

// ******************************************************************* //
// ***           StepmotorGoToLockPosition()                       *** //
// ***         moves to stored Lock Position                       *** //
// ***            Input: motor                                     *** //
// ******************************************************************* //
void StepmotorGoToLockPosition(uint8_t motor)
{
	if(motor >= NUMBER_OF_STEPMOTORS)
	{
		return;
	}

	// Lockposition is independent to offset
	StepmotorGoToEncoderPosition(motor, StepmotorControl[motor].lockposition, 0);


}








