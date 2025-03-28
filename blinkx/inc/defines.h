/***********************************************************************/
/*  Defines for the entire project at one place                        */
/*                                                                     */
/*	@author 	A.Stärker                                              */
/*	@company	Blink GmbH                                             */
/*	@ide		System Workbench STM32 AC6                             */
/*	@date	    2015-09-24                                             */
/*  @change	    2017-02-23 DC-Motor control                            */
/***********************************************************************/

#ifndef BLINK_DEFINES_H
#define BLINK_DEFINES_H

//for float type
#include <arm_math.h>

typedef struct
{


}TypeServoCurrent;


typedef struct
{
	uint8_t typ;
    uint32_t par1;
    uint32_t par2;
    uint32_t par3;
    uint32_t par4;
    uint32_t par5;
    uint32_t par6;
}TypeScriptEntry;

#define MAX_SCRIPT_ENTRIES  100

/* Script for PCR in Demo-Mode */
typedef struct
{
	uint32_t temperature1;
	uint32_t temperature2;
	uint32_t temperature3;
	uint32_t cycles;
	uint32_t holdtime1;
	uint32_t holdtime2;
	uint32_t holdtime3;

}TypeDemoPcr;

typedef struct
{
	uint8_t running;//1 - running, 0 - not running
    int16_t entries;//number of filled entries
    int16_t index;//actual index
    uint8_t loopactive;//1 if in loop
    int32_t loopcount;//counter of loops
    int32_t  remainingtime;//remaining time of step in 100ms
    uint32_t running_time;//time since starting the script in 100ms
    int32_t  timeout;//count down timeout for some types
    uint16_t error;//error counter
    uint16_t canid;//for can commands
    uint16_t mot;//for indexed stepper motor access

}TypeScriptControl;

/* for QTouch-Processing */
typedef struct
{
	uint8_t sensorstate;
    uint8_t previoussensorstate;
    uint8_t ledstate;
    uint8_t action; /* cyclic counter to MAX_ACTION_x */
}TypeQtouch;


/* Members of TypeDisplay.state */
#define DISPLAY_STATE_MOTOR1    0
#define DISPLAY_STATE_MOTOR2    1
#define DISPLAY_STATE_MOTOR3    2
#define DISPLAY_STATE_SPEED     3
#define NUMBER_OF_STATES        4

/* for Display */
typedef struct
{
	uint8_t init;
    uint16_t timeslot;
    uint8_t refresh;
    uint8_t state;
    uint8_t overwrite;
    uint32_t value[NUMBER_OF_STATES]; /* cyclic counter to MAX_ACTION_x */
}TypeDisplay;


/* for Stepmotor */
typedef struct
{
	uint32_t counter;
	uint32_t stepstogo;
	uint8_t finished;
	int8_t direction;
	int32_t absolutecounter;
	int8_t reference;
	uint16_t positioningspeed;
	uint16_t rampstartspeed;
	uint16_t rampfinalspeed;
	uint16_t rampactualspeed;
	uint16_t rampstep;//in % (1...100)
	uint16_t rampdelay;//in multiples of 10ms
	uint16_t rampdelaycount;//in multiples of 10ms
	uint32_t rampdowntriggerpoint;
	volatile int8_t olda;
	volatile int8_t oldb;
	int32_t encoderposition;
	int8_t mscount;//1 - Full-Step, 2, 4, 8, 16
	int8_t encoderpositioning;//0 or 1
	int32_t targetencoderposition;//encoder position to go to
	uint16_t error;//flags according MOTOR_ERROR
	uint16_t positioningtrycount;//
	uint16_t acceleration;//
	uint8_t stopontouch;//
	int32_t lastabsolutecounter;
	int32_t lastencoderposition;
	int16_t disablecountdown;
	int32_t steploss;
	uint16_t deceleration;//
	int32_t millispeedacc;//acceleration per ramp step in 1/1000 speed units
	int32_t millispeeddec;//acceleration per ramp step in 1/1000 speed units
	int32_t millispeedsum;//speed after ramp caculation in 1/1000 speed units
	int32_t encodercounter;//for detection of absolute steploss
	int8_t startref;//1 - start Reference Drive (if > 1: delay in 1/10s)
	int32_t encoderoffset;//for EEPROM-Adjustable Reference point
	int32_t lockposition;//for transport lock position
}TypeStepmotorControl;

/* pidmode */
#define PID_MODE_SPEED       0
#define PID_MODE_POSITION    1

/* for DC-Motor */
typedef struct
{
	uint8_t finished;
	int8_t direction;
	int8_t reference;
	uint16_t positioningspeed;
	volatile int8_t olda;
	volatile int8_t oldb;
	int32_t encoderposition;
	int8_t encoderpositioning;//0 or 1
	int32_t targetencoderposition;//encoder position to go to
	uint16_t error;//flags according MOTOR_ERROR
	uint16_t positioningtrycount;//
	int32_t speedlastencval;//
	int16_t measuredspeed;//
	int32_t oldstellwert;
	int8_t countfinish;
	int32_t Pspeed;
	int32_t Pposition;
	int32_t Ispeed;
	int32_t Iposition;
	int32_t D;
	int32_t p;
	int32_t i;
	int32_t d;
	int32_t esum;
	int32_t elast;
	int32_t pwm;
	uint16_t burstcount;
	uint8_t cnt;
	int32_t stallencoderposition;
	uint8_t stallcount;
	uint8_t timeoutcan;
	int8_t olddirection;
	int8_t changedircount;
	uint8_t drivemode;
	int8_t endposition;
	int8_t lightbarrierstate;
	int32_t timecountdown;
	int8_t lightbarrieredge; //remembers edges (-1: falling, +1: rising)
	int16_t caliblbontime; //calibration lightbarrier end position time in 100ms
	int16_t calibfullcircletime; //calibration time for full circle in 100ms
	uint8_t calibstate; // calibration state
}TypeDcMotorControl;

/* for indexed runtime control */
typedef struct
{
	GPIO_TypeDef* port; /* GPIOx */
	uint16_t pin;       /* GPIO_Pin_y */
}TypeGPIOPortPin;

/* for DC-Motor */
typedef struct
{
	TypeGPIOPortPin sleepGPIO; /* sleep driver output */
	TypeGPIOPortPin faultGPIO;  /* fault input */
	TypeGPIOPortPin aGPIO; /* PWM a output */
	TypeGPIOPortPin bGPIO;  /* PWM b output */
	TypeGPIOPortPin lbGPIO;   /* lightbarrier enable output */
	TypeGPIOPortPin refGPIO;  /* reference input */
	TypeGPIOPortPin begGPIO;  /* switch begin input */
	TypeGPIOPortPin endGPIO;  /* switch end input */
	TypeGPIOPortPin encaGPIO; /* encoder A input */
	TypeGPIOPortPin encbGPIO; /* encoder B input */
	TIM_TypeDef* timerA; /* Timer number */
	uint8_t channelA; /* Timer channel */
	TIM_TypeDef* timerB; /* Timer number */
	uint8_t channelB; /* Timer channel */
	int8_t refdir;/* reference drive speed */
	int32_t reftimeout;/* reference timeout */
	int32_t refouttimeout;/* reference timeout for drive out of light barrier*/
	int8_t direction;
	int8_t reflogic;
	int16_t defaultspeed;/* default positioning speed */
	int16_t refspeed;/* reference drive speed */
	int32_t eposmax; /* encoder position max */
	int32_t eposmin; /* encoder position min */
	uint16_t drawerdelayin; /* delay drawer in */
	uint16_t drawerdelayeject; /* delay drawer eject */
}TypeDcMotorHW;



/* for Stepmotor */
typedef struct
{
	TypeGPIOPortPin enableGPIO; /* enable driver output */
	TypeGPIOPortPin sleepGPIO; /* sleep driver output */
	TypeGPIOPortPin ms1GPIO;  /* microstep 1 configuration output */
	TypeGPIOPortPin ms2GPIO;  /* microstep 2 configuration output */
	TypeGPIOPortPin ms3GPIO;  /* microstep 3 configuration output */
	TypeGPIOPortPin stepGPIO; /* step output */
	TypeGPIOPortPin dirGPIO;  /* direction output */
	TypeGPIOPortPin lbGPIO;   /* lightbarrier enable output */
	TypeGPIOPortPin refGPIO;  /* reference input */
	TypeGPIOPortPin begGPIO;  /* switch begin input */
	TypeGPIOPortPin endGPIO;  /* switch end input */
	TypeGPIOPortPin encaGPIO; /* encoder A input */
	TypeGPIOPortPin encbGPIO; /* encoder B input */
	TIM_TypeDef* timer; /* Timer number */
	int32_t refspeed;/* reference drive speed */
	int8_t refdir;/* reference drive speed */
	int32_t refstepsin;/* reference steps out */
	int32_t refmaxsteps;/* reference max steps */
	int8_t encresolution;/* encoder resolution */
	int8_t direction;
	int8_t reflogic;
	int16_t defaultspeed;/* default positioning speed */
	uint8_t dacchannel;
	uint16_t maxcurrent; /* stepmotor currentcurrent, if DAC set to max */
	TIM_TypeDef* curtimer; /* Timer number for pwm current setting */
	uint8_t curchannel; /* Timer channel for pwm current setting  */
	int32_t minsteppos; /* minmum step position in fullsteps */
	int32_t maxsteppos; /* maximum step position in fullsteps */
	uint16_t steplosstolerance; /* steploss tolerance */
}TypeStepmotorHW;

/* for Stepmotor CAN commands*/
typedef struct
{
	uint16_t setspeed;
	uint16_t setacceleration;
	uint16_t drivesteps;
	uint16_t referencedrive;
	uint16_t gotoposition;
	uint16_t gotoencoder;
	uint16_t stop;
	uint16_t requestposition;
	uint16_t requestencoder;
	uint16_t requeststate;
	uint16_t answerposition;
	uint16_t answerencoder;
	uint16_t answerstate;
	uint16_t setmsconfig;
	uint16_t gotoencoderliquid;
	uint16_t setdeceleration;
	uint16_t gotoencoderwatchforce;
}TypeStepmotorCanCommand;

/* for DC-Motor CAN commands*/
typedef struct
{
	uint16_t setspeed;
	uint16_t referencedrive;
	uint16_t gotoencoder;
	uint16_t stop;
	uint16_t requestencoder;
	uint16_t requeststate;
	uint16_t requestspeed;
	uint16_t answerencoder;
	uint16_t answerstate;
	uint16_t answerspeed;
	uint16_t gotocartpos;
	uint16_t gotolbhigh;
	uint16_t gotolblow;
	uint16_t starttime;
	uint16_t open;
	uint16_t close;
	uint16_t requestendpos;
	uint16_t answerendpos;
}TypeDcMotorCanCommand;

#define MAX_EXPOSURE_CHANNELS			2
#define MAX_LYSIS_PORTS		            2

/* for Exposure CAN commands*/
typedef struct
{
	uint16_t setline;
	uint16_t setcurrent;
	uint16_t setpulse;
}TypeExposureCanCommand;

/* for Lysis CAN commands*/
typedef struct
{
	uint16_t setTime;
	uint16_t setTemperature;
	uint16_t start;
	uint16_t stop;
}TypeLysisCanCommand;

/* for Sensor commands*/
typedef struct
{
	uint16_t request;
	uint16_t answer;
}TypeSensorCanCommand;

/* for Servomotor CAN commands*/
typedef struct
{
	uint16_t setpos;
	uint16_t requeststate;
	uint16_t requestcurrent;
	uint16_t answerstate;//fuse state and pos
	uint16_t answercurrent;
	uint16_t resetfuse;
	uint16_t requestendpos;
	uint16_t answerendpos;
	uint16_t movein;
	uint16_t moveout;
	uint16_t calibratepositionin;
	uint16_t calibratepositionout;
	uint16_t setspeed;
}TypeServomotorCanCommand;


/* for Servomotor */
typedef struct
{
	uint16_t position;
	int8_t fuse;
	uint16_t meascur;//actual measured current
	uint16_t sumcur;//sum of current
	uint16_t sampnum;//number samples
	uint16_t avrcur;//actual measured current
	uint8_t finished;
	uint16_t movetime;//move time count down
	int16_t fusedelay;//move time count down
	uint8_t error;//error
	int32_t minpos;//minimum Position
	int32_t maxpos;//maximum Position
}TypeServomotorControl;

#define SERVO_FUSE_OK                 0
#define SERVO_FUSE_ACTIVATED          1
#define SERVO_FUSE_REQUEST_RESET      2
#define SERVO_FUSE_IN_RESET           3
#define SERVO_FUSE_IN_SET             4
#define SERVO_FUSE_RELEASE_SET        5


/* Members of TypeStepmotorControl.reference */
#define STEPMOTOR_REFERENCE_NONE              0
#define STEPMOTOR_REFERENCE_DRIVE_IN          1 //drive out of reference point
#define STEPMOTOR_REFERENCE_DRIVE_MORE_IN     2 //drive a bit further out of reference point
#define STEPMOTOR_REFERENCE_DRIVE_OUT         3 //drive into reference point
#define STEPMOTOR_REFERENCE_DRIVE_OK          4 //reference drive successfully done
#define STEPMOTOR_REFERENCE_DRIVE_FAIL       -1 //reference drive failed

/* Members of TypeStepmotorControl.reference */
#define DCMOTOR_REFERENCE_NONE              0
#define DCMOTOR_REFERENCE_DRIVE_IN          1 //drive out of reference point
#define DCMOTOR_REFERENCE_DRIVE_MORE_IN     2 //drive a bit further out of reference point
#define DCMOTOR_REFERENCE_DRIVE_OUT         3 //drive into reference point
#define DCMOTOR_REFERENCE_DRIVE_OK          4 //reference drive successfully done
#define DCMOTOR_REFERENCE_DRIVE_FAIL       -1 //reference drive failed


#define SPI_RX_BUFFER_SIZE     128
#define SPI_TX_BUFFER_SIZE     128

/* for Interrupt-Controlled SPI-Interface */
typedef struct
{
	uint8_t rxbuffer[SPI_RX_BUFFER_SIZE];
	uint8_t rxpos;
	uint8_t txbuffer[SPI_TX_BUFFER_SIZE];
	uint8_t txpos;
	uint8_t txlen;
	uint8_t busy;
	uint8_t enablerx;
	uint32_t adcdata;
	uint32_t adcnanovolt;
	int32_t adcmillikelvin;
	int32_t powcalibrationoffset;
	float32_t powcalibrationgradient;
}TypeSpiControl;

/* for NTC-Values vial I2C-Interface */

typedef struct
{
	uint32_t adc;
	int32_t adcmilivolt;
	int32_t milikelvin;
	int32_t powcalibrationoffset;
	int32_t powcalibrationgradient;
}TypeThermNtcControl;

typedef struct
{
	uint8_t id;//actual id
	int32_t templow;//low temperature value of sensor
	int32_t temphigh;//high temperature value of sensor
	int32_t reallow;//low temperature value of master
	int32_t realhigh;//high temperature value of sensor
}TypeCalibrationControl;

typedef struct
{
	uint8_t regulation;//1 - active; 0 - not active
	uint8_t state;//actual state of fan 0 - off 1 - on
	int32_t templow;//low temperature - off switch point
	int32_t temphigh;//high temperature - on switch point
	int16_t onpwm;//pwm value for off state
	int16_t offpwm;//pwm value for on state
	int32_t temperature;//actual temperature in mK
	uint16_t pwm;// actual PWM
}TypeFanControl;

#define OSZILLOSKOP_SIZE     3600

#define OSZILLOSKOP_RECORD_OFF   0
#define OSZILLOSKOP_RECORD_WAIT  1//wait for phase zero crossing
#define OSZILLOSKOP_RECORD_ON    2

#define OSZILLOSKOP_PLAY_OFF   0
#define OSZILLOSKOP_PLAY_ON    1

/* for Oszilloskope */
typedef struct
{
	uint16_t recordindex;
	uint16_t playindex;
	uint16_t phase[OSZILLOSKOP_SIZE];
	int16_t magU[OSZILLOSKOP_SIZE];
	int16_t magV[OSZILLOSKOP_SIZE];
	int16_t magW[OSZILLOSKOP_SIZE];
	int16_t curU[OSZILLOSKOP_SIZE];
	int16_t curV[OSZILLOSKOP_SIZE];
	int16_t curW[OSZILLOSKOP_SIZE];
	uint8_t record;
	uint8_t play;
}TypeOszilloskop;


//types definitions for script commands
#define SCRIPT_INVALID_TYPE            		 		0 //
#define SCRIPT_PAUSE          				 		1 //wait certain time
#define SCRIPT_LOOP           				 		2 //Loop execution of previous commands
#define SCRIPT_CENTRIFUGE_RAMP          	 		3 //keep the speed for a certain time
#define SCRIPT_CENTRIFUGE_STOP          	 		4 //stop Motor
#define SCRIPT_EXPOSURE    					 		5 //Exposure current
#define SCRIPT_THERM_SET_TEMPERATURE    	 		6 //set Solltemperature
#define SCRIPT_THERM_REGULATION    			 		7 //Regulation ON/OFF
#define SCRIPT_STEPPER_REFERENCE    		 		8 //Reference drive Stepper motor
#define SCRIPT_STEPPER_SET_SPEED        	 		9 //Set Speed of Stepper motor
#define SCRIPT_STEPPER_POSITIONING     				10 //Positioning Stepper motor
#define SCRIPT_STEPPER_ENCODER_POSITIONING 			11 //Positioning Stepper motor
#define SCRIPT_CAN_STEPPER_REFERENCE    			12 //Reference drive Stepper motor
#define SCRIPT_CAN_STEPPER_SET_SPEED        		13 //Set Speed of Stepper motor
#define SCRIPT_CAN_STEPPER_POSITIONING     			14 //Positioning Stepper motor
#define SCRIPT_CAN_STEPPER_ENCODER_POSITIONING     	15 //Positioning Stepper motor
#define SCRIPT_DCMOTOR_REFERENCE    		 		16 //Reference drive DC-motor
#define SCRIPT_DCMOTOR_SET_SPEED        	 		17 //Set Speed of DC-motor
#define SCRIPT_DCMOTOR_ENCODER_POSITIONING 			18 //Positioning DC-motor
#define SCRIPT_CAN_DCMOTOR_REFERENCE    		 	19 //Reference drive CAN-DC-motor
#define SCRIPT_CAN_DCMOTOR_SET_SPEED        	 	20 //Set Speed of CAN-DC-motor
#define SCRIPT_CAN_DCMOTOR_ENCODER_POSITIONING 		21 //Positioning CAN-DC-motor
#define SCRIPT_LIGHT_SET_PANEL				 		22 //set Panel color and brightness
#define SCRIPT_LIGHT_SET_LINE				 		23 //set Line color and intensity
#define SCRIPT_LIGHT_SET_COLUMN				 		24 //set Column color and intensity
#define SCRIPT_LIGHT_SET_PIXEL				 		25 //set Pixel color and intensity
#define SCRIPT_STEPPER_GOTO_ENCODER_LIQUID 		    26 //stepper motor positioning until touch detection
#define SCRIPT_CAN_STEPPER_GOTO_ENCODER_LIQUID 		27 // CAN stepper motor positioning until touch detection
#define SCRIPT_CAN_EXPOSURE    				 		28 // switches Exposure via CAN
#define SCRIPT_BRUSHLESS_POSITIONING           		29 // Positioning of brushless motor
#define SCRIPT_CAN_BRUSHLESS_POSITIONING           	30 // Positioning of brushless motor via CAN
#define SCRIPT_FILTER_POSITIONING           		31 // Positioning of Filter
#define SCRIPT_CAN_FILTER_POSITIONING           	32 // Positioning of Filter via CAN
#define SCRIPT_LIGHT_START_ANIMATION           		33 // starts Animation of LED Panel
#define SCRIPT_LIGHT_STOP_ANIMATION           		34 // stops Animation of LED Panel
#define SCRIPT_CAN_LIGHT_START_ANIMATION           	35 // starts Animation of LED Panel via CAN
#define SCRIPT_CAN_LIGHT_STOP_ANIMATION           	36 // stops Animation of LED Panel via CAN
#define SCRIPT_SERVO_POSITIONING           			37 // sets position of servo motor
#define SCRIPT_CAN_SERVO_POSITIONING           		38 // sets position of servo motor via CAN
#define SCRIPT_CARTRIDGE_POSITIONING           		39 // moves Cartridge to Rotary Valve Position
#define SCRIPT_CAN_CARTRIDGE_POSITIONING           	40 // moves Cartridge to Rotary Valve Position via CAN
#define SCRIPT_LIFT_POSITIONING           			41 // moves Lift up/down
#define SCRIPT_CAN_LIFT_POSITIONING           		42 // moves Lift up/down via CAN
#define SCRIPT_CHAMBER_POSITIONING           		43 // opens/closes PCR-Chamber
#define SCRIPT_CAN_CHAMBER_POSITIONING           	44 // opens/closes PCR-Chamber via CAN
#define SCRIPT_MIXER           				 		45 // starts Mixer
#define SCRIPT_CAN_MIXER           				 	46 // starts Mixer via CAN
#define SCRIPT_THERM_SELF_CALIBRATE 				47 // Run Thermal self calibration
#define SCRIPT_THERM_SET_PWM		 				48 // Set PWM
#define SCRIPT_THERM_TEMPERATURE_RAMP		 		49 // Temperature Ramp
#define SCRIPT_SERVO_SOFT_RELEASE					50 // Regulates Servo position until a current of 0 is reached and activates fuse
#define SCRIPT_SERVO_IN								51 // Move Lysis to mechanical Stop and soft release
#define SCRIPT_SERVO_OUT							52 // Move Lysis to mechanical Stop and soft release
#define SCRIPT_LYSIS_B_SET_TEMPERATURE				53 // Sets Temperature of Lysis heater
#define SCRIPT_LYSIS_B_RAMP							54 // Runs Ramp between two Temperatures



#define USE_USB_OTG_FS /*   */

#define RX_BUFFER_SIZE  200
#define TX_BUFFER_SIZE  200

#define COMMUNICATION_PORT_UART  0
#define COMMUNICATION_PORT_USB   1
#define COMMUNICATION_PORT_CAN   2



//definition of flags for current calculation AverageReady
#define AVERAGE_READY_U   0x01
#define AVERAGE_READY_V   0x02
#define AVERAGE_READY_W   0x04

#define TOGGLE 3
#define HIGH_Z 2
#define ON     1
#define OFF    0

#define ACTIVE_HIGH     1
#define ACTIVE_LOW      0

#define CURRENT_OFFSET_U  2040
#define CURRENT_OFFSET_V  2040
#define CURRENT_OFFSET_W  2080

#define THERM_CONTROL_MODE_NTC      0
#define THERM_CONTROL_MODE_PT100    1

/* for AraTherm */
typedef struct
{
	uint8_t mode;//THERM_CONTROL_MODE_NTC or THERM_CONTROL_MODE_PT100
	uint16_t pwm;
	uint8_t hk;
	uint8_t enable;
	uint8_t regulationactive;
	int32_t solltemperature;//in 1/1000 °C
	int32_t isttemperature;//in 1/1000 °C
	int8_t reached;//solltemperature erreicht
	int32_t P;
	int32_t I;
	int32_t D;
	int32_t p;
	int32_t i;
	int32_t d;
	int32_t k;
	int32_t error;
	int32_t elast;
	int32_t esum;
	int32_t stellwert;
	uint16_t abtastzeit;
	int32_t cocontroller_isttemperature[2];//in 1/1000 °C [0]: PT100 or NTC1; [1]: NTC2;
	int32_t virtualtemperature;//from pacemaker, in 1/1000 °C
	int32_t estimatedtemperature;//from pacemaker, in 1/1000 °C
}TypeThermControl;

#define I2C_JOB_NONE                      0
#define I2C_JOB_EEPROM_WRITE_BYTE         1
#define I2C_JOB_EEPROM_WRITE_INTEGER      2
#define I2C_JOB_EEPROM_EXT_WRITE_BYTE     3
#define I2C_JOB_EEPROM_EXT_WRITE_INTEGER  4
#define I2C_JOB_LM3509_SWITCH_MAIN        5
#define I2C_JOB_LM3509_SET_BRIGHTNESS     6


/* for I2C */
typedef struct
{
	uint8_t job;
	uint32_t par1;
	uint32_t par2;
	uint32_t par3;

}TypeI2cJob;

/* for Batterie */
typedef struct
{
	uint8_t status;
	uint32_t charge;
	int32_t chargereal;//offset subtracted
	uint16_t voltage;
	int16_t current;
	uint16_t temperature;
	uint16_t adc;
	uint16_t ubat;
	uint8_t i2cpresent;
	uint8_t i2cfailure;
	uint32_t i2cretry;
	uint8_t cnt;
	uint8_t chstate;
	uint8_t dcinok;
	uint8_t perr;
	uint8_t cntoff;
	int32_t percent;
	uint32_t ubatcalibration;
}TypeBatteryInfo;

typedef struct
{
  unsigned char oldstate;
  unsigned int presstime;
  unsigned int releasetime;
  unsigned char release;
  unsigned char doublepress;
  unsigned char state;
}TypeButton;

/* for CAN Stepper motor answers */
typedef struct
{
	int8_t reference;
	uint16_t error;
	uint8_t finished;
	int32_t position;
	int32_t encoderposition;

}TypeCanStepperInfo;

/* maximum number of stepper motors on one board */
#define MAX_STEPPER_AXIS      4

/* for CAN DC-Motor answers */
typedef struct
{
	int8_t reference;
	uint16_t error;
	uint8_t finished;
	int32_t encoderposition;
	uint16_t speed;
	int8_t endpos;
	int8_t overshoot;

}TypeCanDcMotorInfo;

/* maximum number of DC-Motors on one board */
#define MAX_DC_MOTOR_AXIS      3

/* for Lysis status */
typedef struct
{
	uint8_t channel;
	uint8_t active;
	uint8_t heater;
	uint8_t error;
	int32_t tempint;
	int32_t tempext;

}TypeCanLysisInfo;

/* for Dosage status */
typedef struct
{
	uint8_t touch;
	int32_t pressure;

}TypeCanDosageInfo;

/* for CAN Servomotor answers */
typedef struct
{
	uint16_t pos;
	int16_t cur;
	int8_t fuse;
	uint8_t finished;
	int8_t endpos;
	uint16_t error;

}TypeCanServomotorInfo;

/* maximum number of DC-Motors on one board */
#define SERVO_1                  0
#define SERVO_2                  1
#define SERVO_3                  2
#define MAX_SERVOMOTOR_AXIS      3

/* for CAN DC-Motor answers */
typedef struct
{
	int8_t state;
	int32_t value;

}TypeCanSensorInfo;

/* for CAN Brushless answers (optic Filter changer) */
typedef struct
{
	uint8_t status;
	uint8_t filterpos;
	int32_t position;
	uint8_t finished;//position reached
	uint8_t error;
	uint8_t reference;

}TypeCanBrushlessInfo;

/* for CAN Therm answers (PCR-Module) */
typedef struct
{
	int32_t actualtemperature;
	int32_t solltemperature;
	int32_t heatsinktemperature;
	int32_t ntc2temperature;
	int32_t ntc3temperature;
	int32_t slidetemperature;
	uint8_t scriptrunning;
	int16_t scriptindex;
	uint8_t scripterror;
	uint8_t peltierenabled;
	uint8_t regulationenabled;
	uint8_t hk;
	uint16_t pwm;
	uint8_t proximity;
	uint8_t reached;
	uint32_t errorcode;
	uint32_t warningcode;
	int32_t virtualtemperature;//from pacemaker, in 1/1000 °C
	int32_t estimatedtemperature;//from pacemaker, in 1/1000 °C
}TypeCanThermInfo;

/* for CAN XY-Table answers */
typedef struct
{
	int16_t xpos;
	int32_t ypos;
	int8_t reference;
	uint16_t error;
	uint8_t finished;

}TypeCanXYTableInfo;

/* for CAN Bloodsensor answers */
typedef struct
{
	uint16_t red;
	uint16_t blue;
	uint16_t off;
	uint8_t detect;
	int8_t edge;
	int32_t encvolume;
	uint16_t error;

}TypeCanBloodsensorInfo;

/* for CAN Weightcell answers */
typedef struct
{
	int32_t value;
	uint8_t detect;
	uint32_t force;

}TypeCanWeighcellInfo;

/* for CAN Weightcell answers */
typedef struct
{
	int32_t boardtemperature;

}TypeCanEnvironmentInfo;

#define MAX_TAG_DATA_FIELDS       16
#define MAX_TAG_DATA_FIELD_LEN    32

#define FIELD_TYPE_DECIMAL        0x00 //only 2 most significant bits matter
#define FIELD_TYPE_HEX        	  0x40
#define FIELD_TYPE_ASCII          0x80

#define CHECKSUM_NOT_PRESENT      0x00
#define CHECKSUM_OK      		  0x01
#define CHECKSUM_FAIL             0x02



typedef struct
{
	uint8_t sign;//Kennung
	uint8_t typ;//field type
	uint8_t len;//length of valid bytes
	uint8_t data[MAX_TAG_DATA_FIELD_LEN];//length of valid bytes
	uint8_t chksum;

}TypeTagDataField;

typedef struct
{
	uint8_t data[MAX_TAG_DATA_FIELD_LEN + 8];
	uint8_t idxexpected;
	uint8_t adrexpected;
	uint8_t lenexpected;
	uint8_t lastadr;
	TypeTagDataField receivefield;
	TypeTagDataField sendfield;
	uint8_t adrsend;
	uint8_t sendactive;
	uint8_t senddata[MAX_TAG_DATA_FIELD_LEN + 8];
	uint8_t sendidx;

}TypeRfidCanBuffer;



/* for CAN RFID answers  */
typedef struct
{
	uint32_t error;
	uint8_t handler;
	uint8_t mode;

}TypeCanRfidInfo;

/* for CAN Filter info (optic Filter changer) */
typedef struct
{
	uint8_t status;
	uint8_t filterpos;
	int32_t position;
	uint8_t finished;//position reached
	uint16_t error;
	uint8_t reference;

}TypeCanFilterInfo;

/* for CAN Exposure info (optic) */
typedef struct
{
	int16_t current;
	uint16_t voltage;
	uint16_t brightness;
	int16_t sensortemperature;
}TypeCanExposureInfo;



/* maximum number of Sensors on one board */
#define SENSOR_1         0
#define SENSOR_2         1
#define SENSOR_3         2
#define MAX_SENSORS      3


/* for Servomotor */
typedef struct
{
	TypeGPIOPortPin pwmGPIO; /* sPWM output */
	TypeGPIOPortPin fuseGPIO;  /* fuse in/out */
	TIM_TypeDef* timer; /* Timer number */
	uint16_t pwmPinSource;
	uint8_t timerAF;
	uint8_t channel; /* Timer channel */
}TypeServomotorHW;




/* RGB-Color definitions (8 bit): */
/*                          0xRRGGBB */
#define COLOR_RED          	0xFF0000;
#define COLOR_GREEN         0x00FF00;
#define COLOR_BLUE          0x0000FF;
#define COLOR_YELLOW       	0xFFFF00;
#define COLOR_LIME       	0xCCFF00;
#define COLOR_ORANGE       	0xFFCC00;
#define COLOR_CYAN       	0x00FFFF;
#define COLOR_TURQUOISE    	0x00CCFF;
#define COLOR_MINT      	0x00FFCC;
#define COLOR_MAGENTA      	0xFF00FF;
#define COLOR_MAROON       	0xFF00CC;
#define COLOR_PURPLE       	0xCC00FF;
#define COLOR_WHITE       	0xFFFFFF;
#define COLOR_BLACK       	0x000000;

#endif
