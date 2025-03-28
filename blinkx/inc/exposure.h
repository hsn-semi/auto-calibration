/******************************************************/
/***  Project:       OneOpticController             ***/
/***  File:          exposure.h                     ***/
/***  Created on:    17.02.2016                     ***/
/***  derived from AraController4: 25.11.2017       ***/
/***  Author:        Andreas Stärker                ***/
/***  Company:       Blink AG                       ***/
/***  Changes:                                      ***/
/******************************************************/

#ifndef EXPOSURE_H
#define EXPOSURE_H

#include "defines.h"

#define EXPOSURE_A						0
#define EXPOSURE_NUMBER_OF_PORTS        1 /* = Number of Exposure Ports (used for current measure via ADC-DMA) */
#define EXPOSURE_NUMBER_OF_LINES        5

/* for Exposure */
typedef struct
{
	TypeGPIOPortPin lineGPIO[EXPOSURE_NUMBER_OF_LINES]; /* line outputs */
	TypeGPIOPortPin dacGPIO;  /* fault input */
	uint32_t dacChannel;
	TypeGPIOPortPin pwmGPIO; /* PWM a output */
	uint8_t pwmAF;
	uint16_t pwmPinSource;
	TIM_TypeDef* timer; /* Timer number */
	uint8_t channel; /* Timer channel */
}TypeExposureHW;



#define EXPOSURE_GPIO_CLOCKS		   (RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOD | RCC_AHB1Periph_GPIOE)

#define EXPOSURE_DAC_CLK               RCC_APB1Periph_DAC
#define EXPOSURE_DAC_GPIO_PORT         GPIOA /* always GPIOA */


#define EXPOSURE_DAC_A_PIN             GPIO_Pin_4					/* PA.4 */
#define EXPOSURE_DAC_A_CHANNEL         DAC_Channel_1


#define EXPOSURE_A_LINE_1_PORT			GPIOC
#define EXPOSURE_A_LINE_1_PIN			GPIO_Pin_11
#define EXPOSURE_A_LINE_2_PORT			GPIOC
#define EXPOSURE_A_LINE_2_PIN			GPIO_Pin_12
#define EXPOSURE_A_LINE_3_PORT			GPIOC
#define EXPOSURE_A_LINE_3_PIN			GPIO_Pin_6
#define EXPOSURE_A_LINE_4_PORT			GPIOC
#define EXPOSURE_A_LINE_4_PIN			GPIO_Pin_7
#define EXPOSURE_A_LINE_5_PORT			GPIOC
#define EXPOSURE_A_LINE_5_PIN			GPIO_Pin_8

#define EXPOSURE_A_PWM_GPIO_PORT        GPIOA
#define EXPOSURE_A_PWM_PIN			    GPIO_Pin_7
#define EXPOSURE_A_PWM_PIN_SOURCE	    GPIO_PinSource7
#define EXPOSURE_A_PWM_TIMER_AF		    GPIO_AF_TIM14
#define EXPOSURE_A_PWM_TIMER			TIM14
#define EXPOSURE_A_PWM_CHANNEL			1//Timer Channel (1...4)




/* for Exposure Control*/
typedef struct
{
	int8_t active;//1 - on, 0 - off
	uint8_t line;//0 ... EXPOSURE_NUMBER_OF_LINES -1  off
	uint16_t curactual;//during exposure was on
	uint16_t curlatest;//during exposure was on
	uint16_t curaverage;//actual average
	uint16_t curadc;//actual ADC value
	uint32_t curadcsum;//for average calculation
	uint16_t curadcnum;//for average calculation
	uint16_t currentset;//set current
	int32_t safetydisabled;//if 1 safety feature is disabled (otherwise exposure only allowed if drawer i inserted)
	int16_t current;
	uint32_t voltage;
	uint16_t brightness;
	int16_t sensortemperature;
	uint8_t timeslot;
	uint8_t readsensors;
	uint8_t regulationactive;//power regulation active
	uint16_t fotosollwert;//sollwert for regulation
	uint16_t startcurrent;//start stellwert for regulation
	int32_t P;//P constant for regulation
	int32_t I;//I constant for regulation
	int32_t D;//D constant for regulation
	int32_t p;//actual p value for regulation
	int32_t i;//actual i value for regulation
	int32_t d;//actual d value for regulation
	int32_t k;//actual k value for regulation
	int32_t error; //actual regulation error
	int32_t elast; //last regulation error
	int32_t esum; //integral for i regulation
}TypeExposureControl;

/* for Exposure Fotosensor calibration*/
typedef struct
{
	int32_t fotolow;//low foto value
	int32_t fotohigh;//high foto value
	int32_t powlow;//power at low calibration point
	int32_t powhigh;//power at high calibration point
	int32_t currentsetlow;//low current
	int32_t currentsethigh;//high current
	int32_t powcalibrationoffset;
	float32_t powcalibrationgradient;
	int32_t fotocalibrationoffset;//offset for set current
	float32_t fotocalibrationgradient;//gradient for set current
	uint8_t calibrationok;

}TypeExposureCalibration;

typedef struct
{
	uint8_t mode;
	uint8_t timeslot;
	uint8_t delay;
	uint8_t samplenum;//number of measured sample for averaging
	int32_t samplesum;//sum of measured values for averaging
	int32_t curlow;//measured low current value
	int32_t curhigh;//measured high current value
	int32_t calibrationoffset;
	float32_t calibrationgradient;
	uint8_t calibrationok;

}TypeLedCurrentCalibration;

#define CURRENT_CALIBRATION_MODE_OFF			0
#define CURRENT_CALIBRATION_MODE_SET_LOW		1
#define CURRENT_CALIBRATION_MODE_SAMPLE_LOW		2
#define CURRENT_CALIBRATION_MODE_SET_HIGH		3
#define CURRENT_CALIBRATION_MODE_SAMPLE_HIGH	4
#define CURRENT_CALIBRATION_MODE_FINISH			5
#define CURRENT_CALIBRATION_MODE_FAILED			6

#define CURRENT_CALIBRATION_POINT_LOW			100 // mA
#define CURRENT_CALIBRATION_POINT_HIGH			500 // mA
#define CURRENT_CALIBRATION_SAMPLE_NUMBER		10 // mA

/* - - - PWM Calibration - - - */
#define CURRENT_CAL_MAX_GRADIENT                10.0 //
#define CURRENT_CAL_MIN_GRADIENT                0.01 //

/* - - - PWM Calibration - - - */
#define EXPOSURE_CAL_MAX_OFFSET                 10000 // uW
#define EXPOSURE_CAL_MIN_OFFSET                -10000 // uW
#define EXPOSURE_CAL_MAX_GRADIENT                10 //
#define EXPOSURE_CAL_MIN_GRADIENT                0.01 //

#define EXPOSURE_CAL_DEFAULT_POW_OFFSET              0 // no offset
#define EXPOSURE_CAL_DEFAULT_POW_GRADIENT            1.0 //?
#define EXPOSURE_CAL_DEFAULT_FOTO_OFFSET              0 // no offset
#define EXPOSURE_CAL_DEFAULT_FOTO_GRADIENT            1.0 //?


#define EXPOSURE_CURENT_ADC_AVERAGE_NUMBER   10 //calculate average of 10 values

extern TypeExposureHW ExposureHW[EXPOSURE_NUMBER_OF_PORTS];

extern TypeExposureControl ExposureControl[EXPOSURE_NUMBER_OF_PORTS];

extern TypeExposureCalibration ExposureCalibration[EXPOSURE_NUMBER_OF_LINES];

extern TypeLedCurrentCalibration LedCurrentCalibration;

//functions:
void ExposureInit(void);
void SwitchExposureOn(unsigned char port, unsigned char line, uint16_t current, uint16_t period, uint16_t pulse);
void SwitchExposureOff(unsigned char port);
void SetExposureModeSychron(unsigned char port, unsigned char state);
void SetExposureSynchronDelay(unsigned char port, uint16_t delay);
void SetExposureSynchronPulse(unsigned char port, uint16_t pulse);
void ExposureWork(void);
void ExposureSetLine(unsigned char port, unsigned char line);
void ExposureSetCurrent(unsigned char port, uint16_t current);
void ExposureSetPulse(unsigned char port, uint16_t period, uint16_t pulse);
void ExposureWatchChanges(void);
int32_t ExposureReadVoltage(void);
int32_t ExposureReadCurrent(void);
uint32_t ExposureReadBrightness(void);
int32_t ExposureReadSensorTemperature(void);
void ExposureCalibrationReset(uint8_t line);
void ExposureCalibrationLow(uint32_t pow);
uint8_t ExposureCalibrationHigh(uint32_t pow);
void SwitchExposureRegulatedPowerOn(unsigned char port, unsigned char line, uint32_t power);
uint16_t ExposureCurrentToDac(uint16_t current);
void ExposureStartCurrentCalibration(uint8_t line);
void ExposureWorkCurrentCalibration(void);
void ExposureCurrentCalculation(void);


#endif

