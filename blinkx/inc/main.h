/***********************************************************************/
/*                 header file for main.c                              */
/*                                                                     */
/*	@author 	A.Stärker                                              */
/*	@company	Blink GmbH                                             */
/*	@ide		System Workbench STM32 AC6                             */
/*	@date	    2017-02-23                                             */
/***********************************************************************/

#ifndef MAIN_H
#define MAIN_H

//for float type
#include <arm_math.h>



typedef struct
{
	uint32_t sum;
	uint8_t count;
	uint32_t result;
}TypeAverage;



#define PC_CURRENT_AVERAGE_COUNT   20

/* - - - - - - Public defines - - - - - -  */

#define NUMBER_OF_FILTERS     5

typedef struct
{
	int32_t position[5];
	uint8_t actualfilter; //0...5
	uint8_t finished;
	uint16_t error;//1 - in snapped position, 0 - not in snapped position
	int8_t edgesnapped;//not used anymore
	int8_t oldsnapped;//not used anymore
	uint8_t delaysnapped;//not used anymore
	uint8_t snapped;//not used anymore
	uint8_t swingover;//not used anymore
	int32_t minposition;
	int32_t maxposition;
}TypeFilter;

/* members of Filter.error */
#define FILTER_ERROR_NO_EXT_EEPROM  		        0x0001
#define FILTER_ERROR_EEPROM_DATA  		        	0x0002


/* Drawer */
typedef struct
{
	uint8_t mode;
	uint8_t state;
	int16_t delay;
	uint8_t detect;
}TypeDrawer;

/* Darkfield */
typedef struct
{
	uint8_t i2cpresent;
	uint8_t state;
	uint8_t brightness;
}TypeDarkfield;

/* Filter Date and Time */
typedef struct
{
	uint16_t year;
	uint8_t month;
	uint8_t day;
	uint8_t hour;
	uint8_t minute;
}TypeDateTime;



/* Local EEPROM */
#define EEPROM_ADDRESS_DEVICE_ID                 			4 //4 byte
#define EEPROM_ADDRESS_DEVICE_SERIAL_NUM         			8 //4 byte
#define EEPROM_ADDRESS_EXPOSURE_SAFETY_DISABLE   			12 //4 byte
#define EEPROM_ADDRESS_DEVICE_NUM_DIGITS_SERIAL   			16 //4 byte
#define EEPROM_ADDRESS_QC_DATETIME			   				20 //4 byte // last QC Date and Time
#define EEPROM_ADDRESS_FAN_TEMP_LOW							24
#define EEPROM_ADDRESS_FAN_TEMP_HIGH						28
#define EEPROM_ADDRESS_FOCUS_LOCK_POSITION					32
#define EEPROM_ADDRESS_CAL_CURRENT_OFFSET					36
#define EEPROM_ADDRESS_CAL_CURRENT_GRADIENT					40


/* External EEPROM */
#define EEPROM_EXT_ADDRESS_OPTIC_TYP                   4 //4 byte
#define EEPROM_EXT_ADDRESS_OPTIC_SN     			   8 //4 byte
#define EEPROM_EXT_ADDRESS_FILTER_POSITION_1          12
#define EEPROM_EXT_ADDRESS_FILTER_POSITION_2          16
#define EEPROM_EXT_ADDRESS_FILTER_POSITION_3          20
#define EEPROM_EXT_ADDRESS_FILTER_POSITION_4          24
#define EEPROM_EXT_ADDRESS_FILTER_POSITION_5          28
#define EEPROM_EXT_ADDRESS_SERVO_MIN_POSITION         32
#define EEPROM_EXT_ADDRESS_SERVO_MAX_POSITION         36
#define EEPROM_EXT_ADDRESS_FILTER_DATETIME		      40
#define EEPROM_EXT_ADDRESS_FILTER_LOCK_POSITION	      44
#define EEPROM_EXT_ADDRESS_FILTER_MIN_POSITION	      48
#define EEPROM_EXT_ADDRESS_FILTER_MAX_POSITION	      52
#define EEPROM_EXT_ADDRESS_PIXEL_RESOLUTION		      56
#define EEPROM_EXT_ADDRESS_CAL_EXP1_POW_OFFSET        60
#define EEPROM_EXT_ADDRESS_CAL_EXP1_POW_GRADIENT 	  64
#define EEPROM_EXT_ADDRESS_CAL_EXP1_FOTO_OFFSET       68
#define EEPROM_EXT_ADDRESS_CAL_EXP1_FOTO_GRADIENT 	  72
#define EEPROM_EXT_ADDRESS_CAL_EXP2_POW_OFFSET        76
#define EEPROM_EXT_ADDRESS_CAL_EXP2_POW_GRADIENT 	  80
#define EEPROM_EXT_ADDRESS_CAL_EXP2_FOTO_OFFSET       84
#define EEPROM_EXT_ADDRESS_CAL_EXP2_FOTO_GRADIENT 	  88
#define EEPROM_EXT_ADDRESS_CAL_EXP3_POW_OFFSET        92
#define EEPROM_EXT_ADDRESS_CAL_EXP3_POW_GRADIENT 	  96
#define EEPROM_EXT_ADDRESS_CAL_EXP3_FOTO_OFFSET      100
#define EEPROM_EXT_ADDRESS_CAL_EXP3_FOTO_GRADIENT 	 104
#define EEPROM_EXT_ADDRESS_CAL_EXP4_POW_OFFSET       108
#define EEPROM_EXT_ADDRESS_CAL_EXP4_POW_GRADIENT 	 112
#define EEPROM_EXT_ADDRESS_CAL_EXP4_FOTO_OFFSET      116
#define EEPROM_EXT_ADDRESS_CAL_EXP4_FOTO_GRADIENT 	 120
#define EEPROM_EXT_ADDRESS_CAL_EXP5_POW_OFFSET       124
#define EEPROM_EXT_ADDRESS_CAL_EXP5_POW_GRADIENT 	 128
#define EEPROM_EXT_ADDRESS_CAL_EXP5_FOTO_OFFSET      132
#define EEPROM_EXT_ADDRESS_CAL_EXP5_FOTO_GRADIENT 	 136





#define SYSTEM_TIMER            			TIM7 // TIM6
#define SYSTEM_TIMER_IRQ_CHANNEL            TIM7_IRQn // TIM6_DAC_IRQn

/* Motor Error according */
//bis 0...3: counter for repeated positioning, reset after successful reference drive
//flags
#define MOTOR_ERROR_REFERENCE_TIMEOUT		        0x0010
#define MOTOR_ERROR_POSITIONING_FAIL		        0x0020
#define MOTOR_ERROR_LIMIT_BEG		                0x0040
#define MOTOR_ERROR_LIMIT_END		                0x0080
#define MOTOR_ERROR_OVERCURRENT				        0x0100
#define MOTOR_ERROR_NO_REFERENCE  		            0x0200
#define MOTOR_ERROR_BEYOND_MIN_POS  		        0x0400
#define MOTOR_ERROR_BEYOND_MAX_POS  		        0x0800





/* index of AdcConversionResult ADC1 */
#define ADC_INDEX_NTC_BOARD          	0
#define ADC_INDEX_UB             		1
#define NUMBER_OF_CONVERSION_RESULTS    2


/* index of Voltage */
#define VOLT_UB               0
#define VOLT_EXPOSURE         1 // via I2C
#define NUMBER_OF_VOLTAGE     2

/* index of Current */
#define CUR_EXP              0 // via I2C
#define NUMBER_OF_CURRENT    1

#define DEFAULT_CURRENT_CALIBRATION_VALUE		1000 // 1000 -> 1mA = 1 DAC-digit



/* constants for NTC calculation */
#define NTC_T1                        298.15 // NTC  T1 in K (298.15K = 25°C)
#define NTC_RT1                     10000.0  //resistane of NTC @ T1 in Ohm
#define NTC_BETAS_FAN                 3478.0  //beta value 25/85 of NTC Fan: B57540G1103F000 from Epcos TDK
#define NTC_BETAS_LYSIS               3478.0  //beta value 25/85 of NTC Lysis: B57540G1103F000 from Epcos TDK //3976.0  //beta value 25/85 of NTC Lysis: 10K3AA1B from MeasurementSpecialities
#define NTC_BETAS_BOARD               3434.0  //beta value 25/85 of NTC Board 0603: NCP18XH103F03RB from Murata
#define NTC_TNULL                     273.15 //absolute zero in -°C
//specific defines
#define NTC_RPULLUP                  1500.0  //pullup resistane of NTC voltage divider in Ohm;
#define NTC_UREF                        3.3  //reference voltage of the ADC
#define NTC_UB                          3.3  //voltage to which the Rpullup is connected

#define UNUSED_PINS_PORTA            (GPIO_Pin_3 | GPIO_Pin_5 | GPIO_Pin_15)
#define UNUSED_PINS_PORTB            (GPIO_Pin_12 | GPIO_Pin_13)
#define UNUSED_PINS_PORTC            (GPIO_Pin_10 | GPIO_Pin_15)
//#define UNUSED_PINS_PORTD            (GPIO_Pin_12)
//#define UNUSED_PINS_PORTE            (GPIO_Pin_2)
//#define UNUSED_PINS_PORTF            (GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_11)
//#define UNUSED_PINS_PORTG            (GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14)

#define LED_GREEN_GPIO_CLOCK           	RCC_AHB1Periph_GPIOA
#define LED_GREEN_GPIO_PORT          	GPIOA
#define LED_GREEN_PIN			       	GPIO_Pin_8

#define LED_RED_GPIO_CLOCK           	RCC_AHB1Periph_GPIOC
#define LED_RED_GPIO_PORT          		GPIOC
#define LED_RED_PIN			       		GPIO_Pin_9


#define HW1_GPIO_CLOCK           		RCC_AHB1Periph_GPIOB
#define HW1_GPIO_PORT          			GPIOB
#define HW1_PIN			       			GPIO_Pin_0

#define HW2_GPIO_CLOCK           		RCC_AHB1Periph_GPIOB
#define HW2_GPIO_PORT          			GPIOB
#define HW2_PIN			       			GPIO_Pin_1

#define HW3_GPIO_CLOCK           		RCC_AHB1Periph_GPIOB
#define HW3_GPIO_PORT          			GPIOB
#define HW3_PIN			       			GPIO_Pin_2

/* Fan */
#define FAN_GPIO_RCC				  RCC_AHB1Periph_GPIOA
#define FAN_GPIO_PORT                 GPIOA
#define FAN_GPIO_PIN			      GPIO_Pin_2
#define FAN_PIN_SOURCE	              GPIO_PinSource2
#define FAN_TIMER_AF			      GPIO_AF_TIM9
#define FAN_TIMER				      TIM9
#define FAN_TIMER_CHANNEL			  1//Timer Channel (1...4)

#define FAN_PWM_PERIODE                   100
#define FAN_DEFAULT_TEMPLOW				35000
#define FAN_DEFAULT_TEMPHIGH			40000


#define	DEFAULT_DATE	0x76C10800 //= 01.01.1900 0:00


extern uint16_t AdcConversionResult[NUMBER_OF_CONVERSION_RESULTS];

extern uint32_t Current[NUMBER_OF_CURRENT];

extern unsigned char AverageReadyServo;

extern uint8_t ValidCommandReceived;
extern int32_t TimeWithoutValidCommand;

extern uint32_t Voltage[NUMBER_OF_VOLTAGE];
extern uint32_t Current[NUMBER_OF_CURRENT];
extern int32_t BoardTemperature;
extern TypeFilter Filter;


extern TypeDateTime FilterDateTime;

extern TypeDateTime QcDateTime;

extern uint32_t SystemTimeIn100ms;

extern uint8_t LastAnimationButton;
extern uint8_t LastAnimationRing;
extern uint8_t ReferenceAnimationActive;


void EnableTimerPeriphClock(TIM_TypeDef* TIMX);
uint8_t IsTimerOnAPB2(TIM_TypeDef* TIMX);
void SetTimerCompareValue(TIM_TypeDef* TIMX, uint8_t ch, uint16_t val);
void SetDacValue(uint32_t channel, uint16_t val);
void GPIOConfigUnusedPins(void);
void GPIOConfig(void);
void ADCConfig(void);
void FanInit(void);
void FanSetPwm(uint16_t pwm);
void SystemTimerConfig(void);
void LedRed(uint8_t state);
void LedGreen(uint8_t state);
void Switch12P(uint8_t state);
uint8_t IsWakeupHigh(void);
uint8_t ReadHardwareVersion(void);
void Decoder(char * buf);
void WorkSendLogData(void);
void WorkScript(void);
void NextScriptIndex(void);
void ExitScript(void);
int32_t GetParameter(char * out, char * in, uint8_t num, uint8_t strsize);
void LoadDemoScript(uint8_t ui);
void SendToCommunicationPort(char * buf, unsigned char len);
void WorkDemo(void);
void WorkAdc(void);
int32_t CalculateNtcTemperature(int32_t adc, float betas);
void StopScript(void);
void StopScriptAndPeltierOff(void);
void StartScript(void);
void ScriptWatchChanges(void);
void ClearScript(void);
void AddScriptEntry(uint8_t typ, uint32_t par1, uint32_t par2, uint32_t par3, uint32_t par4, uint32_t par5, uint32_t par6);
void EditScriptEntry(uint8_t num, uint32_t par);
void SetLogOutput(uint8_t state);
void DebugInitTimer1(void);
void DarkfieldInit(void);
void SetFilterDate(uint16_t year, uint8_t month, uint8_t day, uint8_t hour, uint8_t minute);
void ReadFilterDate(void);
void DecodeFilterDate(uint32_t data);
void SetQcDate(uint16_t year, uint8_t month, uint8_t day, uint8_t hour, uint8_t minute);
void ReadQcDate(void);
void WorkFan(void);







#endif
