/******************************************************/
/***  Project:       AraController                  ***/
/***  File:          stepmotor.h                    ***/
/***  Created on:    22.07.2016                     ***/
/***  Author:        Andreas Stärker                ***/
/***  Company:       Blink AG                       ***/
/***  Changes:       for 5 Motors                   ***/
/******************************************************/

#ifndef STEPMOTOR_H
#define STEPMOTOR_H

//#include "defines.h"

/* Motor Direction */
#define FW		 1
#define ST	     0
#define RW		-1

#define BEG_FORBIDDEN_DIRECTION     RW
#define END_FORBIDDEN_DIRECTION     FW

/* Motor Definition */
#define MOTOR_X		0
#define MOTOR_Y		1
//#define MOTOR_Z		2
//#define MOTOR_E		3

#define NUMBER_OF_STEPMOTORS 2

//default values
#define FILTER_1_POSITION    50
#define FILTER_2_POSITION    700
#define FILTER_3_POSITION    1350
#define FILTER_4_POSITION    2000
#define FILTER_5_POSITION    2650


/* Speed definition */
#define STEPMOTOR_PRESCALER_FACTOR             3 //1 for 1MHz timer clock, 2 for 2MHz timer clock or 3 for 3MHz timer clock
#define STEPMOTOR_PERIODE_IN_US_FOR_MIN_SPEED  20000 //20ms = 50Hz
/* (STEPMOTOR_PRESCALER_FACTOR * STEPMOTOR_PERIODE_IN_US_FOR_MIN_SPEED) must not be more than 65535 for 16-bit-timers ! */

#define RAMP_WORK_INTERVAL          10 /* in ms */
#define STEP_INTERVAL               (STEPMOTOR_PERIODE_IN_US_FOR_MIN_SPEED / 500) /* in ms at speed 1*/ /* two intervals for one step, form us in ms: *2 /1000 */
#define RAMP_CALCULATION_FACTOR     10 /* in ms at speed 1*/
#define STEPMOTOR_MILLI_DELTA_PER_RAMP        40 //20//increase (or decrease) per 10ms per 1 acc (or dec) for fullstep in 1/1000 speed units

#define ENCODER_POSITIONING_MAX_TRY     		2 //1 /* max tries for encoder positioning */
#define STEPMOTOR_DISABLE_DELAY       		  10//100 /* 100 = 1s */
#define STEPMOTOR_REFERENCE_START_DELAY       20 /* 20 = 200ms */


/* Microstep Definition according Driver-IC A4988*/
#define MS_CONFIG_FULL_STEP		        0
#define MS_CONFIG_HALF_STEP		        1
#define MS_CONFIG_QUARTER_STEP		    2
#define MS_CONFIG_EIGHTH_STEP		    3
#define MS_CONFIG_SIXTEENTH_STEP		7

/* Motor Error according */
//bis 0...3: counter for repeated positioning, reset after successful reference drive
//flags
#define STEPMOTOR_ERROR_REFERENCE_TIMEOUT		        0x0010;
#define STEPMOTOR_ERROR_POSITIONING_FAIL		        0x0020;
#define STEPMOTOR_ERROR_LIMIT_BEG		                0x0040;
#define STEPMOTOR_ERROR_LIMIT_END		                0x0080;
#define STEPMOTOR_ERROR_MAX_FORCE_DETECTED		        0x0100;
#define STEPMOTOR_ERROR_NO_REFERENCE  		            0x0200;
#define STEPMOTOR_ERROR_BEYOND_MIN_POS  		        0x0400;
#define STEPMOTOR_ERROR_BEYOND_MAX_POS  		        0x0800;
#define STEPMOTOR_ERROR_NOT_PERMITTED  		            0x1000;



#define STEPMOTOR_DEFAULT_RAMP_START_SPEED    1//in 1...100


/* Reference Drive Data */
#define STEPMOTOR_X_DIRECTION   				RW
#define STEPMOTOR_X_REFERENCE_INPUT_LOGIC       ACTIVE_LOW//ACTIVE_LOW (e.g.Slot-Lightbarrier with inverter) or ACTIVE_HIGH (e.g.switch to GND and pull-up with inverter)
#define STEPMOTOR_X_REFERENCE_DRIVE_SPEED       5
#define STEPMOTOR_X_REFERENCE_DRIVE_DIRECTION   RW
#define STEPMOTOR_X_REFERENCE_DRIVE_STEPS_IN    20//200 = 1mm
#define STEPMOTOR_X_REFERENCE_DRIVE_MAX_STEPS   26600//13000 fullsteps
#define STEPMOTOR_X_DEFAULT_ACCELERATION        50//25//1 to 100
#define STEPMOTOR_X_DEFAULT_POSITIONING_SPEED   80//15//50 //1 to 1000
#define STEPMOTOR_X_ENCODER_DIRECTION           0//0 or 1
#define STEPMOTOR_X_ENCODER_RESOLUTION          5// 10//Increments per Full-Step (5: Encoder NOE1-05-B14)
#define STEPMOTOR_X_MIN_POSITION_FULLSTEPS      -2000//full steps // -10000inc
#define STEPMOTOR_X_MAX_POSITION_FULLSTEPS      3000//full steps // 15000inc
#define STEPMOTOR_X_DEFAULT_DECELERATION        50// to prevent position overshooting and retries // 200 //0 to 1000
#define STEPMOTOR_X_STEPLOSS_TOLERANCE          20 /* difference between step executed and encoder value */

/* Reference Drive Data */
#define STEPMOTOR_Y_DIRECTION   				RW
#define STEPMOTOR_Y_REFERENCE_INPUT_LOGIC       ACTIVE_LOW//ACTIVE_LOW (e.g.Slot-Lightbarrier with inverter) or ACTIVE_HIGH (e.g.switch to GND and pull-up with inverter)
#define STEPMOTOR_Y_REFERENCE_DRIVE_SPEED       40// 2
#define STEPMOTOR_Y_REFERENCE_DRIVE_DIRECTION   RW
#define STEPMOTOR_Y_REFERENCE_DRIVE_STEPS_IN    20//200 = 1mm
#define STEPMOTOR_Y_REFERENCE_DRIVE_MAX_STEPS   50500//3156 fullsteps
#define STEPMOTOR_Y_DEFAULT_ACCELERATION        25//1 to 100
#define STEPMOTOR_Y_DEFAULT_POSITIONING_SPEED   50//50 //1 to 1000
#define STEPMOTOR_Y_ENCODER_DIRECTION           1//0 or 1
#define STEPMOTOR_Y_ENCODER_RESOLUTION          5// 10//Increments per Full-Step (5: Encoder NOE1-05-B14)
#define STEPMOTOR_Y_MIN_POSITION_FULLSTEPS    -1000//full steps
#define STEPMOTOR_Y_MAX_POSITION_FULLSTEPS    50000// full steps
#define STEPMOTOR_Y_DEFAULT_DECELERATION        50// to prevent position overshooting and retries // 200 //0 to 1000
#define STEPMOTOR_Y_STEPLOSS_TOLERANCE          20 /* difference between step executed and encoder value */



/* Port and Pin definitions */

/* Stepmotor X */
/* general enable pins */
#define STEPMOTOR_X_EN_GPIO_PORT          GPIOC
#define STEPMOTOR_X_EN_PIN			      GPIO_Pin_1
#define STEPMOTOR_X_SLEEP_GPIO_PORT       GPIOC
#define STEPMOTOR_X_SLEEP_PIN			  GPIO_Pin_2
/* Driver Pins */
#define STEPMOTOR_X_STEP_GPIO_PORT        GPIOB
#define STEPMOTOR_X_STEP_PIN			  GPIO_Pin_14
#define STEPMOTOR_X_DIR_GPIO_PORT         GPIOB
#define STEPMOTOR_X_DIR_PIN			      GPIO_Pin_15
/* Reference Input */
#define STEPMOTOR_X_REF_GPIO_PORT         GPIOC
#define STEPMOTOR_X_REF_PIN			      GPIO_Pin_3
/* Encoder ENC A */
#define STEPMOTOR_X_ENCA_GPIO_PORT        GPIOB
#define STEPMOTOR_X_ENCA_PIN			  GPIO_Pin_4
/* Encoder ENC B */
#define STEPMOTOR_X_ENCB_GPIO_PORT        GPIOB
#define STEPMOTOR_X_ENCB_PIN			  GPIO_Pin_5
#define STEPMOTOR_X_ENCB_PORT_SOURCE	  EXTI_PortSourceGPIOB
#define STEPMOTOR_X_ENCB_PIN_SOURCE	      EXTI_PinSource5
#define STEPMOTOR_X_ENCB_EXTI_LINE		  EXTI_Line5
#define STEPMOTOR_X_ENCB_IRQ			  EXTI9_5_IRQn
/* Timer */
#define STEPMOTOR_X_TIMER          		  TIM10

/* Stepmotor Y */
/* general enable pins */
#define STEPMOTOR_Y_EN_GPIO_PORT          GPIOC
#define STEPMOTOR_Y_EN_PIN			      GPIO_Pin_1
#define STEPMOTOR_Y_SLEEP_GPIO_PORT       GPIOC
#define STEPMOTOR_Y_SLEEP_PIN			  GPIO_Pin_2
/* Driver Pins */
#define STEPMOTOR_Y_STEP_GPIO_PORT        GPIOC
#define STEPMOTOR_Y_STEP_PIN			  GPIO_Pin_4
#define STEPMOTOR_Y_DIR_GPIO_PORT         GPIOC
#define STEPMOTOR_Y_DIR_PIN			      GPIO_Pin_5
/* Reference Input */
#define STEPMOTOR_Y_REF_GPIO_PORT         GPIOC
#define STEPMOTOR_Y_REF_PIN			      GPIO_Pin_14
/* End Input */
#define STEPMOTOR_Y_END_GPIO_PORT         GPIOC
#define STEPMOTOR_Y_END_PIN			      GPIO_Pin_13
/* Encoder ENC A*/
#define STEPMOTOR_Y_ENCA_GPIO_PORT        GPIOB
#define STEPMOTOR_Y_ENCA_PIN			  GPIO_Pin_6 //Disc Encoder
/* Encoder ENC B*/
#define STEPMOTOR_Y_ENCB_GPIO_PORT        GPIOB
#define STEPMOTOR_Y_ENCB_PIN			  GPIO_Pin_7 //Disc Encoder
#define STEPMOTOR_Y_ENCB_PORT_SOURCE	  EXTI_PortSourceGPIOB
#define STEPMOTOR_Y_ENCB_PIN_SOURCE	      EXTI_PinSource7
#define STEPMOTOR_Y_ENCB_EXTI_LINE		  EXTI_Line7
#define STEPMOTOR_Y_ENCB_IRQ			  EXTI9_5_IRQn
/* Timer */
#define STEPMOTOR_Y_TIMER          		  TIM11



/* Lightbarrier enable for all Stepmotors */
#define STEPMOTOR_LB_GPIO_PORT          GPIOC
#define STEPMOTOR_LB_PIN			    GPIO_Pin_0



//variables for extern usage:
extern TypeStepmotorControl StepmotorControl[NUMBER_OF_STEPMOTORS];
extern TypeStepmotorHW StepmotorHW[NUMBER_OF_STEPMOTORS];



//functions:
void StepmotorInit(void);
void StepmotorStart(uint8_t motor, uint16_t speed, uint32_t steps, int8_t dir);
void StepmotorSetSpeed(uint8_t motor, uint16_t speed);
void StepmotorSetAcceleration(uint8_t motor, uint16_t acc);
void StepmotorSetDeceleration(uint8_t motor, uint16_t dec);
void StepmotorSetDirection(uint8_t motor, int8_t dir);
void StepmotorStop(uint8_t motor);
void StepmotorReferenceDrive(uint8_t motor);
void StepmotorStartReferenceDrive(uint8_t motor);
void StepmotorWork(void);
void StepmotorRampWork(void);
void StepmotorUpdateSpeed(uint8_t motor);
void StepmotorGoToPosition(uint8_t motor, int32_t pos);
void StepmotorGoToEncoderPosition(uint8_t motor, int32_t pos, uint8_t useoffset);
void StepmotorSetMicrostepConfig(uint8_t motor, uint8_t ms);
uint8_t IsStepmotorBeg(uint8_t motor);
uint8_t IsStepmotorEnd(uint8_t motor);
void StepmotorSwitchLightBarrier(uint8_t motor, uint8_t state);
uint8_t IsStepmotorInReferenceLightbarrier(uint8_t motor);
void CalculateRampDownTriggerPoint(uint8_t motor);
void StepmotorWatchChanges(uint8_t motor);
void StepmotorSetCurrent(uint8_t motor, uint16_t cur);
void StepmotorLiquidDetect(void);
void StepmotorSteplossDetect(void);
void StepmotorEnable(uint8_t motor);
void StepmotorDisable(uint8_t motor);
void StepmotorSetFilterPosition(uint8_t motor, uint8_t filter);
void StepmotorSetEncoderOffset(uint8_t motor, int32_t val);
void StepmotorSetLockPosition(uint8_t motor, int32_t val);
void StepmotorGoToLockPosition(uint8_t motor);

#endif

