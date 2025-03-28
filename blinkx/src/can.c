/*******************************************************************/
/***  Project:       Blink ARA                                   ***/
/***  File:          can.c                                       ***/
/***  Created on:    13.07.2016                                  ***/
/***  Author:        Andreas Stärker                             ***/
/***  Company:       Blink AG                                    ***/
/***  Changes:       02.01.2017 stepper can commands table       ***/
/***                 26.04.2017 DC-Motor can commands table      ***/
/***                 28.06.2017 decode of DC-Motor can commands  ***/
/***                 23.08.2017 LightController implemented      ***/
/***                 13.10.2017 CAN-ACK management implemented   ***/
/***                 07.02.2018 Dosage commands added            ***/
/***                 08.02.2018 stepmotorstatus: finished bit31  ***/
/*******************************************************************/



#include <stdio.h>
#include <string.h>
#include "stdlib.h"


//#include "stm32f4_discovery.h"
//#include "stm32f4xx_conf.h"
//#include "stm32f4xx_tim.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_can.h"
#include "stm32f4xx.h"

#include "stm32f4xx_usart.h"

#include "defines.h"

#include "main.h"
#include "uartpc.h"
#include "stepmotor.h"
#include "uartpc.h"
#include "exposure.h"
#include "i2c.h"

#include "can.h"



//variables:
TypeCanError CanError;
TypeCanTxQueue CanTxQueue;
TypeCanAcknowledge CanAcknowledge;

TypeCanStepperInfo CanStepperInfo[NUMBER_OF_CAN_IDS][MAX_STEPPER_AXIS];
TypeStepmotorCanCommand StepmotorCanCommand[MAX_STEPPER_AXIS];

TypeCanDcMotorInfo CanDcMotorInfo[NUMBER_OF_CAN_IDS][MAX_DC_MOTOR_AXIS];
TypeDcMotorCanCommand DcMotorCanCommand[MAX_DC_MOTOR_AXIS];
TypeExposureCanCommand ExposureCanCommand[MAX_EXPOSURE_CHANNELS];
TypeLysisCanCommand LysisCanCommand[MAX_LYSIS_PORTS];
TypeCanLysisInfo CanLysisInfo[MAX_LYSIS_PORTS];
TypeCanDosageInfo CanDosageInfo;

TypeCanServomotorInfo CanServomotorInfo[NUMBER_OF_CAN_IDS][MAX_SERVOMOTOR_AXIS];
TypeServomotorCanCommand ServomotorCanCommand[MAX_SERVOMOTOR_AXIS];

TypeCanSensorInfo CanSensorInfo[NUMBER_OF_CAN_IDS][MAX_SENSORS];
TypeSensorCanCommand SensorCanCommand[MAX_SENSORS];

TypeCanBrushlessInfo CanBrushlessInfo;
TypeCanThermInfo CanThermInfo;
TypeCanXYTableInfo CanXYTableInfo;
TypeCanBloodsensorInfo CanBloodsensorInfo;
TypeCanWeighcellInfo CanWeighcellInfo;
TypeCanEnvironmentInfo CanEnvironmentInfo[NUMBER_OF_CAN_IDS];
TypeCanRfidInfo CanRfidInfo;
TypeCanFilterInfo CanFilterInfo;
TypeCanExposureInfo CanExposureInfo;

uint8_t MyCanId = 0;

char CanFirmwareVersion[30];
char CanFirmwareDate[30];

uint8_t CanAckReceived[NUMBER_OF_CAN_IDS];


const char CanAscii[NUMBER_OF_CAN_IDS] =
{ 'C','D','O','P','Z','a','b','c','d','A','L','F','M','E','X','S','R','H' };



// ********************************************** //
// ***       Function: CanInit                *** //
// ***       Purpose: init Can Interface      *** //
// ***       Input: none                      *** //
// ***       Return: none                     *** //
// ********************************************** //
void CanInit(uint8_t addr)
{

	  GPIO_InitTypeDef 		     GPIO_InitStructure;
	  CAN_InitTypeDef 	         CAN_InitStructure;
	  CAN_FilterInitTypeDef      CAN_FilterInitStructure;
	  NVIC_InitTypeDef 		     NVIC_InitStructure;

	  //int i,j;

	  CAN_StructInit(&CAN_InitStructure);

	  /* Enable SYSCFG clock */
	  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

	  /* CAN clock enable */
	  RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);

	  CAN_DeInit(CAN1);

	  CAN_InitStructure.CAN_TTCM = DISABLE;
	  CAN_InitStructure.CAN_ABOM = ENABLE;//ENABLE for automatic recovery from Bus-Off state
	  CAN_InitStructure.CAN_AWUM = DISABLE;
	  CAN_InitStructure.CAN_NART = ENABLE;//DISABLE;
	  CAN_InitStructure.CAN_RFLM = DISABLE;
	  CAN_InitStructure.CAN_RFLM = DISABLE;
	  CAN_InitStructure.CAN_TXFP = DISABLE;
	  CAN_InitStructure.CAN_Mode = CAN_Mode_Normal;
	  CAN_InitStructure.CAN_SJW = CAN_SJW_1tq;
	  /* Baudrate */
	  CAN_InitStructure.CAN_BS1 = CAN_BS1_6tq;
	  CAN_InitStructure.CAN_BS2 = CAN_BS2_8tq;
	  CAN_InitStructure.CAN_Prescaler = 10;//20;//2;
	  CAN_Init(CAN1, &CAN_InitStructure);

	  /* GPIO clock enable */
	  RCC_AHB1PeriphClockCmd(CAN_GPIO_RCC, ENABLE);

	  /* GPIO Configuration: CAN1 RX*/
	  GPIO_InitStructure.GPIO_Pin = CAN_RX_PIN;
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
	  GPIO_Init(CAN_GPIO_PORT, &GPIO_InitStructure);

	  /* GPIO Configuration: CAN1 TX*/
	  GPIO_InitStructure.GPIO_Pin = CAN_TX_PIN;
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
	  GPIO_Init(CAN_GPIO_PORT, &GPIO_InitStructure);

	  /* connect pins to alternate function CAN1 */
	  GPIO_PinAFConfig(CAN_GPIO_PORT, CAN_RX_PIN_SOURCE, GPIO_AF_CAN1);
	  GPIO_PinAFConfig(CAN_GPIO_PORT, CAN_TX_PIN_SOURCE, GPIO_AF_CAN1);

	  /*CAN1 filter init - only messages as MyCanAddress as receiver will pass*/
	  CAN_FilterInitStructure.CAN_FilterNumber = 0;
	  CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask;
	  CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_16bit;
	  CAN_FilterInitStructure.CAN_FilterIdHigh = addr << 5;//MyCanAddress as receiver
	  CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0x001F << 5;//care for the last 5 bits = receiver address
	  CAN_FilterInitStructure.CAN_FilterIdLow = 0x001F << 5;//universal address
	  CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0x001F << 5;//care for the last 5 bits = receiver address
	  CAN_FilterInitStructure.CAN_FilterFIFOAssignment = 0;
	  CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;
	  CAN_FilterInit(&CAN_FilterInitStructure);




	  /* Interrupt settings */
	  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);

	  NVIC_InitStructure.NVIC_IRQChannel = CAN1_RX0_IRQn;
	  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0;
	  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0;
	  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	  NVIC_Init(&NVIC_InitStructure);

	  NVIC_InitStructure.NVIC_IRQChannel = CAN1_RX1_IRQn;
	  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0;
	  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0;
	  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	  NVIC_Init(&NVIC_InitStructure);

	  NVIC_InitStructure.NVIC_IRQChannel = CAN1_TX_IRQn;
	  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0;
	  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0;
	  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	  NVIC_Init(&NVIC_InitStructure);


	  /* Enable the CAN global Interrupts */
	  //CAN_ITConfig(CAN1, CAN_IT_TME | CAN_IT_FMP0 | CAN_IT_FMP1, ENABLE);
	  
	  CanResetInfoStructures();





	  /* fill the stepmotor command table for indexed access */
	  	  /* motor X */
	  	  StepmotorCanCommand[0].setspeed = CAN_COMMAND_MOTOR_X_SET_SPEED;
	  	  StepmotorCanCommand[0].setacceleration = CAN_COMMAND_MOTOR_X_SET_ACCELERATION;
	  	  StepmotorCanCommand[0].referencedrive = CAN_COMMAND_MOTOR_X_DO_REFERENCE_DRIVE;
	  	  StepmotorCanCommand[0].drivesteps = CAN_COMMAND_MOTOR_X_DRIVE_STEPS;
	  	  StepmotorCanCommand[0].gotoposition = CAN_COMMAND_MOTOR_X_GOTO_POSITION;
	  	  StepmotorCanCommand[0].gotoencoder = CAN_COMMAND_MOTOR_X_GOTO_ENCODER_POSITION;
	  	  StepmotorCanCommand[0].stop = CAN_COMMAND_MOTOR_X_STOP;
	  	  StepmotorCanCommand[0].requestposition = CAN_COMMAND_REQUEST_MOTOR_X_POSITION;
	  	  StepmotorCanCommand[0].requestencoder = CAN_COMMAND_REQUEST_ENCODER_X_POSITION;
	  	  StepmotorCanCommand[0].requeststate = CAN_COMMAND_REQUEST_MOTOR_X_STATE;
	  	  StepmotorCanCommand[0].answerposition = CAN_COMMAND_ANSWER_MOTOR_X_POSITION;
	  	  StepmotorCanCommand[0].answerencoder = CAN_COMMAND_ANSWER_ENCODER_X_POSITION;
	  	  StepmotorCanCommand[0].answerstate = CAN_COMMAND_ANSWER_MOTOR_X_STATE;
	  	  StepmotorCanCommand[0].setmsconfig = CAN_COMMAND_MOTOR_X_SET_MS_CONFIG;
	  	  StepmotorCanCommand[0].gotoencoderliquid = CAN_COMMAND_MOTOR_X_GOTO_ENCODER_LIQUID;
	  	  StepmotorCanCommand[0].setdeceleration = CAN_COMMAND_MOTOR_X_SET_DECELERATION;
	  	  StepmotorCanCommand[0].gotoencoderwatchforce = CAN_COMMAND_MOTOR_X_GOTO_ENCODER_WATCH_FORCE;
	  	  /* motor Y */
	  	  StepmotorCanCommand[1].setspeed = CAN_COMMAND_MOTOR_Y_SET_SPEED;
	  	  StepmotorCanCommand[1].setacceleration = CAN_COMMAND_MOTOR_Y_SET_ACCELERATION;
	  	  StepmotorCanCommand[1].referencedrive = CAN_COMMAND_MOTOR_Y_DO_REFERENCE_DRIVE;
	  	  StepmotorCanCommand[1].drivesteps = CAN_COMMAND_MOTOR_Y_DRIVE_STEPS;
	  	  StepmotorCanCommand[1].gotoposition = CAN_COMMAND_MOTOR_Y_GOTO_POSITION;
	  	  StepmotorCanCommand[1].gotoencoder = CAN_COMMAND_MOTOR_Y_GOTO_ENCODER_POSITION;
	  	  StepmotorCanCommand[1].stop = CAN_COMMAND_MOTOR_Y_STOP;
	  	  StepmotorCanCommand[1].requestposition = CAN_COMMAND_REQUEST_MOTOR_Y_POSITION;
	  	  StepmotorCanCommand[1].requestencoder = CAN_COMMAND_REQUEST_ENCODER_Y_POSITION;
	  	  StepmotorCanCommand[1].requeststate = CAN_COMMAND_REQUEST_MOTOR_Y_STATE;
	  	  StepmotorCanCommand[1].answerposition = CAN_COMMAND_ANSWER_MOTOR_Y_POSITION;
	  	  StepmotorCanCommand[1].answerencoder = CAN_COMMAND_ANSWER_ENCODER_Y_POSITION;
	  	  StepmotorCanCommand[1].answerstate = CAN_COMMAND_ANSWER_MOTOR_Y_STATE;
	  	  StepmotorCanCommand[1].setmsconfig = CAN_COMMAND_MOTOR_Y_SET_MS_CONFIG;
	  	  StepmotorCanCommand[1].gotoencoderliquid = CAN_COMMAND_MOTOR_Y_GOTO_ENCODER_LIQUID;
	  	  StepmotorCanCommand[1].setdeceleration = CAN_COMMAND_MOTOR_Y_SET_DECELERATION;
	  	  StepmotorCanCommand[1].gotoencoderwatchforce = CAN_COMMAND_MOTOR_Y_GOTO_ENCODER_WATCH_FORCE;
	  	  /* motor Z */
	  	  StepmotorCanCommand[2].setspeed = CAN_COMMAND_MOTOR_Z_SET_SPEED;
	  	  StepmotorCanCommand[2].setacceleration = CAN_COMMAND_MOTOR_Z_SET_ACCELERATION;
	  	  StepmotorCanCommand[2].referencedrive = CAN_COMMAND_MOTOR_Z_DO_REFERENCE_DRIVE;
	  	  StepmotorCanCommand[2].drivesteps = CAN_COMMAND_MOTOR_Z_DRIVE_STEPS;
	  	  StepmotorCanCommand[2].gotoposition = CAN_COMMAND_MOTOR_Z_GOTO_POSITION;
	  	  StepmotorCanCommand[2].gotoencoder = CAN_COMMAND_MOTOR_Z_GOTO_ENCODER_POSITION;
	  	  StepmotorCanCommand[2].stop = CAN_COMMAND_MOTOR_Z_STOP;
	  	  StepmotorCanCommand[2].requestposition = CAN_COMMAND_REQUEST_MOTOR_Z_POSITION;
	  	  StepmotorCanCommand[2].requestencoder = CAN_COMMAND_REQUEST_ENCODER_Z_POSITION;
	  	  StepmotorCanCommand[2].requeststate = CAN_COMMAND_REQUEST_MOTOR_Z_STATE;
	  	  StepmotorCanCommand[2].answerposition = CAN_COMMAND_ANSWER_MOTOR_Z_POSITION;
	  	  StepmotorCanCommand[2].answerencoder = CAN_COMMAND_ANSWER_ENCODER_Z_POSITION;
	  	  StepmotorCanCommand[2].answerstate = CAN_COMMAND_ANSWER_MOTOR_Z_STATE;
	  	  StepmotorCanCommand[2].setmsconfig = CAN_COMMAND_MOTOR_Z_SET_MS_CONFIG;
	  	  StepmotorCanCommand[2].gotoencoderliquid = CAN_COMMAND_MOTOR_Z_GOTO_ENCODER_LIQUID;
	  	  StepmotorCanCommand[2].setdeceleration = CAN_COMMAND_MOTOR_Z_SET_DECELERATION;
	  	  StepmotorCanCommand[2].gotoencoderwatchforce = CAN_COMMAND_MOTOR_Z_GOTO_ENCODER_WATCH_FORCE;
	  	  /* motor E */
	  	  StepmotorCanCommand[3].setspeed = CAN_COMMAND_MOTOR_E_SET_SPEED;
	  	  StepmotorCanCommand[3].setacceleration = CAN_COMMAND_MOTOR_E_SET_ACCELERATION;
	  	  StepmotorCanCommand[3].referencedrive = CAN_COMMAND_MOTOR_E_DO_REFERENCE_DRIVE;
	  	  StepmotorCanCommand[3].drivesteps = CAN_COMMAND_MOTOR_E_DRIVE_STEPS;
	  	  StepmotorCanCommand[3].gotoposition = CAN_COMMAND_MOTOR_E_GOTO_POSITION;
	  	  StepmotorCanCommand[3].gotoencoder = CAN_COMMAND_MOTOR_E_GOTO_ENCODER_POSITION;
	  	  StepmotorCanCommand[3].stop = CAN_COMMAND_MOTOR_E_STOP;
	  	  StepmotorCanCommand[3].requestposition = CAN_COMMAND_REQUEST_MOTOR_E_POSITION;
	  	  StepmotorCanCommand[3].requestencoder = CAN_COMMAND_REQUEST_ENCODER_E_POSITION;
	  	  StepmotorCanCommand[3].requeststate = CAN_COMMAND_REQUEST_MOTOR_E_STATE;
	  	  StepmotorCanCommand[3].answerposition = CAN_COMMAND_ANSWER_MOTOR_E_POSITION;
	  	  StepmotorCanCommand[3].answerencoder = CAN_COMMAND_ANSWER_ENCODER_E_POSITION;
	  	  StepmotorCanCommand[3].answerstate = CAN_COMMAND_ANSWER_MOTOR_E_STATE;
	  	  StepmotorCanCommand[3].setmsconfig = CAN_COMMAND_MOTOR_E_SET_MS_CONFIG;
	  	  StepmotorCanCommand[3].setdeceleration = CAN_COMMAND_MOTOR_E_SET_DECELERATION;


	  	  /* fill the dc-motor command table for indexed access */
		  /* motor 1 */
		  DcMotorCanCommand[0].setspeed = CAN_COMMAND_DC_MOTOR_1_SET_SPEED;
		  DcMotorCanCommand[0].referencedrive = CAN_COMMAND_DC_MOTOR_1_DO_REFERENCE_DRIVE;
		  DcMotorCanCommand[0].gotoencoder = CAN_COMMAND_DC_MOTOR_1_GOTO_ENCODER_POSITION;
		  DcMotorCanCommand[0].stop = CAN_COMMAND_DC_MOTOR_1_STOP;
		  DcMotorCanCommand[0].requestencoder = CAN_COMMAND_REQUEST_ENCODER_1_POSITION;
		  DcMotorCanCommand[0].requeststate = CAN_COMMAND_REQUEST_DC_MOTOR_1_STATE;
		  DcMotorCanCommand[0].requestspeed = CAN_COMMAND_REQUEST_DC_MOTOR_1_SPEED;
		  DcMotorCanCommand[0].answerencoder = CAN_COMMAND_ANSWER_ENCODER_1_POSITION;
		  DcMotorCanCommand[0].answerstate = CAN_COMMAND_ANSWER_DC_MOTOR_1_STATE;
		  DcMotorCanCommand[0].answerspeed = CAN_COMMAND_ANSWER_DC_MOTOR_1_SPEED;
		  DcMotorCanCommand[0].gotocartpos = CAN_COMMAND_DC_MOTOR_1_GOTO_CART_POS;
		  DcMotorCanCommand[0].gotolbhigh = CAN_COMMAND_DC_MOTOR_1_GOTO_LB_HIGH;
		  DcMotorCanCommand[0].gotolblow = CAN_COMMAND_DC_MOTOR_1_GOTO_LB_LOW;
		  DcMotorCanCommand[0].starttime = CAN_COMMAND_DC_MOTOR_1_START_TIME;
		  DcMotorCanCommand[0].open = CAN_COMMAND_DC_MOTOR_1_OPEN;
		  DcMotorCanCommand[0].close = CAN_COMMAND_DC_MOTOR_1_CLOSE;
		  DcMotorCanCommand[0].requestendpos = CAN_COMMAND_REQUEST_DC_MOTOR_1_ENDPOS;
		  DcMotorCanCommand[0].answerendpos = CAN_COMMAND_ANSWER_DC_MOTOR_1_ENDPOS;
		  /* motor 2 */
		  DcMotorCanCommand[1].setspeed = CAN_COMMAND_DC_MOTOR_2_SET_SPEED;
		  DcMotorCanCommand[1].referencedrive = CAN_COMMAND_DC_MOTOR_2_DO_REFERENCE_DRIVE;
		  DcMotorCanCommand[1].gotoencoder = CAN_COMMAND_DC_MOTOR_2_GOTO_ENCODER_POSITION;
		  DcMotorCanCommand[1].stop = CAN_COMMAND_DC_MOTOR_2_STOP;
		  DcMotorCanCommand[1].requestencoder = CAN_COMMAND_REQUEST_ENCODER_2_POSITION;
		  DcMotorCanCommand[1].requeststate = CAN_COMMAND_REQUEST_DC_MOTOR_2_STATE;
		  DcMotorCanCommand[1].requestspeed = CAN_COMMAND_REQUEST_DC_MOTOR_2_SPEED;
		  DcMotorCanCommand[1].answerencoder = CAN_COMMAND_ANSWER_ENCODER_2_POSITION;
		  DcMotorCanCommand[1].answerstate = CAN_COMMAND_ANSWER_DC_MOTOR_2_STATE;
		  DcMotorCanCommand[1].answerspeed = CAN_COMMAND_ANSWER_DC_MOTOR_2_SPEED;
		  DcMotorCanCommand[1].gotocartpos = CAN_COMMAND_DC_MOTOR_2_GOTO_CART_POS;
		  DcMotorCanCommand[1].gotolbhigh = CAN_COMMAND_DC_MOTOR_2_GOTO_LB_HIGH;
		  DcMotorCanCommand[1].gotolblow = CAN_COMMAND_DC_MOTOR_2_GOTO_LB_LOW;
		  DcMotorCanCommand[1].starttime = CAN_COMMAND_DC_MOTOR_2_START_TIME;
		  DcMotorCanCommand[1].open = CAN_COMMAND_DC_MOTOR_2_OPEN;
		  DcMotorCanCommand[1].close = CAN_COMMAND_DC_MOTOR_2_CLOSE;
		  DcMotorCanCommand[1].requestendpos = CAN_COMMAND_REQUEST_DC_MOTOR_2_ENDPOS;
		  DcMotorCanCommand[1].answerendpos = CAN_COMMAND_ANSWER_DC_MOTOR_2_ENDPOS;
		  /* motor 3 */
		  DcMotorCanCommand[2].setspeed = CAN_COMMAND_DC_MOTOR_3_SET_SPEED;
		  DcMotorCanCommand[2].referencedrive = CAN_COMMAND_DC_MOTOR_3_DO_REFERENCE_DRIVE;
		  DcMotorCanCommand[2].gotoencoder = CAN_COMMAND_DC_MOTOR_3_GOTO_ENCODER_POSITION;
		  DcMotorCanCommand[2].stop = CAN_COMMAND_DC_MOTOR_3_STOP;
		  DcMotorCanCommand[2].requestencoder = CAN_COMMAND_REQUEST_ENCODER_3_POSITION;
		  DcMotorCanCommand[2].requeststate = CAN_COMMAND_REQUEST_DC_MOTOR_3_STATE;
		  DcMotorCanCommand[2].requestspeed = CAN_COMMAND_REQUEST_DC_MOTOR_3_SPEED;
		  DcMotorCanCommand[2].answerencoder = CAN_COMMAND_ANSWER_ENCODER_3_POSITION;
		  DcMotorCanCommand[2].answerstate = CAN_COMMAND_ANSWER_DC_MOTOR_3_STATE;
		  DcMotorCanCommand[2].answerspeed = CAN_COMMAND_ANSWER_DC_MOTOR_3_SPEED;
		  DcMotorCanCommand[2].gotocartpos = CAN_COMMAND_DC_MOTOR_3_GOTO_CART_POS;
		  DcMotorCanCommand[2].gotolbhigh = CAN_COMMAND_DC_MOTOR_3_GOTO_LB_HIGH;
		  DcMotorCanCommand[2].gotolblow = CAN_COMMAND_DC_MOTOR_3_GOTO_LB_LOW;
		  DcMotorCanCommand[2].starttime = CAN_COMMAND_DC_MOTOR_3_START_TIME;
		  DcMotorCanCommand[2].open = CAN_COMMAND_DC_MOTOR_3_OPEN;
		  DcMotorCanCommand[2].close = CAN_COMMAND_DC_MOTOR_3_CLOSE;
		  DcMotorCanCommand[2].requestendpos = CAN_COMMAND_REQUEST_DC_MOTOR_3_ENDPOS;
		  DcMotorCanCommand[2].answerendpos = CAN_COMMAND_ANSWER_DC_MOTOR_3_ENDPOS;

		  /* Exposure Channel A */
		  ExposureCanCommand[0].setline = CAN_COMMAND_EXPOSURE_A_LINE;
		  ExposureCanCommand[0].setcurrent = CAN_COMMAND_EXPOSURE_A_CURRENT;
		  ExposureCanCommand[0].setpulse = CAN_COMMAND_EXPOSURE_A_PULSE;
		  /* Exposure Channel B */
		  ExposureCanCommand[1].setline = CAN_COMMAND_EXPOSURE_B_LINE;
		  ExposureCanCommand[1].setcurrent = CAN_COMMAND_EXPOSURE_B_CURRENT;
		  ExposureCanCommand[1].setpulse = CAN_COMMAND_EXPOSURE_B_PULSE;

		  /* Lysis Port A */
		  LysisCanCommand[0].setTemperature = CAN_COMMAND_LYSIS_PORT_A_SET_TEMPERATURE;
		  LysisCanCommand[0].setTime = CAN_COMMAND_LYSIS_PORT_A_SET_TIME;
		  LysisCanCommand[0].start = CAN_COMMAND_LYSIS_PORT_A_START;
		  LysisCanCommand[0].stop = CAN_COMMAND_LYSIS_PORT_A_STOP;
		  /* Lysis Port B */
		  LysisCanCommand[1].setTemperature = CAN_COMMAND_LYSIS_PORT_B_SET_TEMPERATURE;
		  LysisCanCommand[1].setTime = CAN_COMMAND_LYSIS_PORT_B_SET_TIME;
		  LysisCanCommand[1].start = CAN_COMMAND_LYSIS_PORT_B_START;
		  LysisCanCommand[1].stop = CAN_COMMAND_LYSIS_PORT_B_STOP;

		  /* Sensors */
		  /* 1 */
		  SensorCanCommand[0].request = CAN_COMMAND_REQUEST_SENSOR_1;
		  SensorCanCommand[0].answer = CAN_COMMAND_ANSWER_SENSOR_1;
		  /* 2 */
		  SensorCanCommand[1].request = CAN_COMMAND_REQUEST_SENSOR_2;
		  SensorCanCommand[1].answer = CAN_COMMAND_ANSWER_SENSOR_2;
		  /* 3 */
		  SensorCanCommand[2].request = CAN_COMMAND_REQUEST_SENSOR_3;
		  SensorCanCommand[2].answer = CAN_COMMAND_ANSWER_SENSOR_3;

		  /* Servomotor */
		  /* SERVO_1 */
		  ServomotorCanCommand[0].setpos = CAN_COMMAND_SERVO_1_SET_POS;
		  ServomotorCanCommand[0].resetfuse = CAN_COMMAND_SERVO_1_RESET_FUSE;
		  ServomotorCanCommand[0].requeststate = CAN_COMMAND_REQUEST_SERVO_1_STATE;
		  ServomotorCanCommand[0].answerstate = CAN_COMMAND_ANSWER_SERVO_1_STATE;
		  ServomotorCanCommand[0].requestcurrent = CAN_COMMAND_REQUEST_SERVO_1_CURRENT;
		  ServomotorCanCommand[0].answercurrent = CAN_COMMAND_ANSWER_SERVO_1_CURRENT;
		  ServomotorCanCommand[0].requestendpos = CAN_COMMAND_REQUEST_SERVO_1_ENDPOS;
		  ServomotorCanCommand[0].answerendpos = CAN_COMMAND_ANSWER_SERVO_1_ENDPOS;
		  ServomotorCanCommand[0].movein = CAN_COMMAND_SERVO_1_MOVE_IN;
		  ServomotorCanCommand[0].moveout = CAN_COMMAND_SERVO_1_MOVE_OUT;
		  ServomotorCanCommand[0].calibratepositionin = CAN_COMMAND_SERVO_1_SET_CALIBRATION_IN;
		  ServomotorCanCommand[0].calibratepositionout = CAN_COMMAND_SERVO_1_SET_CALIBRATION_OUT;
		  ServomotorCanCommand[0].setspeed = CAN_COMMAND_SERVO_1_SET_SPEED;
		  /* SERVO_2 */
		  ServomotorCanCommand[1].setpos = CAN_COMMAND_SERVO_2_SET_POS;
		  ServomotorCanCommand[1].resetfuse = CAN_COMMAND_SERVO_2_RESET_FUSE;
		  ServomotorCanCommand[1].requeststate = CAN_COMMAND_REQUEST_SERVO_2_STATE;
		  ServomotorCanCommand[1].answerstate = CAN_COMMAND_ANSWER_SERVO_2_STATE;
		  ServomotorCanCommand[1].requestcurrent = CAN_COMMAND_REQUEST_SERVO_2_CURRENT;
		  ServomotorCanCommand[1].answercurrent = CAN_COMMAND_ANSWER_SERVO_2_CURRENT;
		  ServomotorCanCommand[1].requestendpos = CAN_COMMAND_REQUEST_SERVO_2_ENDPOS;
		  ServomotorCanCommand[1].answerendpos = CAN_COMMAND_ANSWER_SERVO_2_ENDPOS;
		  ServomotorCanCommand[1].movein = CAN_COMMAND_SERVO_2_MOVE_IN;
		  ServomotorCanCommand[1].moveout = CAN_COMMAND_SERVO_2_MOVE_OUT;
		  ServomotorCanCommand[1].calibratepositionin = CAN_COMMAND_SERVO_2_SET_CALIBRATION_IN;
		  ServomotorCanCommand[1].calibratepositionout = CAN_COMMAND_SERVO_2_SET_CALIBRATION_OUT;
		  ServomotorCanCommand[1].setspeed = CAN_COMMAND_SERVO_2_SET_SPEED;
		  /* SERVO_3 */
		  ServomotorCanCommand[2].setpos = CAN_COMMAND_SERVO_3_SET_POS;
		  ServomotorCanCommand[2].resetfuse = CAN_COMMAND_SERVO_3_RESET_FUSE;
		  ServomotorCanCommand[2].requeststate = CAN_COMMAND_REQUEST_SERVO_3_STATE;
		  ServomotorCanCommand[2].answerstate = CAN_COMMAND_ANSWER_SERVO_3_STATE;
		  ServomotorCanCommand[2].requestcurrent = CAN_COMMAND_REQUEST_SERVO_3_CURRENT;
		  ServomotorCanCommand[2].answercurrent = CAN_COMMAND_ANSWER_SERVO_3_CURRENT;
		  ServomotorCanCommand[2].requestendpos = CAN_COMMAND_REQUEST_SERVO_3_ENDPOS;
		  ServomotorCanCommand[2].answerendpos = CAN_COMMAND_ANSWER_SERVO_3_ENDPOS;
		  ServomotorCanCommand[2].movein = CAN_COMMAND_SERVO_3_MOVE_IN;
		  ServomotorCanCommand[2].moveout = CAN_COMMAND_SERVO_3_MOVE_OUT;
		  ServomotorCanCommand[2].calibratepositionin = CAN_COMMAND_SERVO_3_SET_CALIBRATION_IN;
		  ServomotorCanCommand[2].calibratepositionout = CAN_COMMAND_SERVO_3_SET_CALIBRATION_OUT;
		  ServomotorCanCommand[2].setspeed = CAN_COMMAND_SERVO_3_SET_SPEED;



}

// ******************************************************************* //
// ***       Function: CanSend                                     *** //
// ***       Purpose: sends Telegram to CAN-Bus                    *** //
// ***       Input: id - ID of Message                             *** //
// ***              cmd - command (part of CAN-data)               *** //
// ***              data - command parameter (part of CAN-data)    *** //
// ***              dataext - extented command parameter           *** //
// ***       Return: none                                          *** //
// ******************************************************************* //
void CanSend(uint16_t id, uint16_t cmd, uint32_t data, uint16_t dataext)
{
	CanTxMsg msg;


	msg.IDE = CAN_Id_Standard;
	msg.StdId = id;
	msg.DLC = 8;//length
	msg.RTR = CAN_RTR_Data;//Data Frame (or: CAN_RTR_Remote - remote frame)
	//copy the command into the data field
	msg.Data[0] = cmd >> 8;//msb
	msg.Data[1] = cmd & 0xFF;//lsb
	//copy the data into the data field
	msg.Data[2] = data >> 24;//msb
	msg.Data[3] = (data >> 16) & 0xFF;//msb
	msg.Data[4] = (data >> 8) & 0xFF;//msb
	msg.Data[5] = data  & 0xFF;//msb
	msg.Data[6] = dataext >> 8;//extented data
	msg.Data[7] = dataext & 0xFF;//extented data


	CAN_Transmit(CAN1, &msg);

}

// **************************************************************************** //
// ***       Function: CanReceive                                           *** //
// ***       Purpose: receives Telegram from CAN RX FIFO                    *** //
// ***       Input: CANx - CAN Interface   (CAN1 or CAN2)                   *** //
// ***              FIFONumber - Fifo (CAN_FIFO0 or CAN_FIFO1)              *** //
// ***              data - command parameter (part of CAN-data)             *** //
// ***       Return: none                                                   *** //
// **************************************************************************** //
void CanReceive(CAN_TypeDef* CANx, uint8_t FIFONumber, CanRxMsg* pmsg)
{


	CAN_Receive(CANx, FIFONumber, pmsg);


}

// **************************************************************************** //
// ***       Function: CanDecode                                            *** //
// ***       Purpose: decodes received Telegram from CAN-Bus                *** //
// ***       Input: CANx - CAN Interface   (CAN1 or CAN2)                   *** //
// ***              FIFONumber - Fifo (CAN_FIFO0 or CAN_FIFO1)              *** //
// ***              data - command parameter (part of CAN-data)             *** //
// ***       Return: none                                                   *** //
// **************************************************************************** //
void CanDecode(CanRxMsg *msg)
{

	uint16_t id;
	uint16_t cmd;
	uint32_t par;
	uint16_t sender;
	int32_t ipar;
	uint8_t mot, sen, i, exposure;
	uint32_t state;
	uint8_t ack = 0;
	char str[40];
	char strtemp[40];
	uint16_t parext;
	uint32_t val;

	id = msg->StdId;
	//extract the command
	cmd = msg->Data[0];
	cmd = cmd << 8;
	cmd |= msg->Data[1];
	//extract the parameter
	par = msg->Data[2];
	par = par << 8;
	par |=  msg->Data[3];
	par = par << 8;
	par |=  msg->Data[4];
	par = par << 8;
	par |=  msg->Data[5];
	//extented parameter
	parext = msg->Data[6];
	parext = parext << 8;
	parext |=  msg->Data[7];

	sender = (id & 0x3FF) >> 5;

	switch (cmd)
	{

		case CAN_COMMAND_LED_DEBUG_GREEN:
			LedGreen(par);
		break;
		case CAN_COMMAND_LED_DEBUG_RED:
			LedRed(par);
		break;
		case CAN_COMMAND_FILTER_SET_POSITION:
					StepmotorSetFilterPosition(MOTOR_X, par);
					ack = 1;
				break;
				case CAN_COMMAND_FILTER_REQUEST_POSITION:
					//Filter Position
					CanAppendMessageToTxQueue(((MyCanId << 5) | CAN_ID_CONTROLLER), CAN_COMMAND_FILTER_ANSWER_POSITION, StepmotorControl[MOTOR_X].encoderposition, 0, 0);
					ack = 2;//send nothing
				break;
				case CAN_COMMAND_FILTER_REQUEST_STATE:
					//Filter state
					state = (CanFilterInfo.error << 16) | (CanFilterInfo.filterpos << 8) | CanFilterInfo.finished;
					CanAppendMessageToTxQueue(((MyCanId << 5) | CAN_ID_CONTROLLER), CAN_COMMAND_FILTER_ANSWER_STATE, state, 0, 0);
					ack = 2;//send nothing
				break;
				case CAN_COMMAND_FILTER_1_SET_CALIBRATION:
					I2cAddJob(I2C_JOB_EEPROM_EXT_WRITE_INTEGER, EEPROM_EXT_ADDRESS_FILTER_POSITION_1, par, 0);
					Filter.position[0] = par;
					ack = 1;
				break;
				case CAN_COMMAND_FILTER_2_SET_CALIBRATION:
					I2cAddJob(I2C_JOB_EEPROM_EXT_WRITE_INTEGER, EEPROM_EXT_ADDRESS_FILTER_POSITION_2, par, 0);
					Filter.position[1] = par;
					ack = 1;
				break;
				case CAN_COMMAND_FILTER_3_SET_CALIBRATION:
					I2cAddJob(I2C_JOB_EEPROM_EXT_WRITE_INTEGER, EEPROM_EXT_ADDRESS_FILTER_POSITION_3, par, 0);
					Filter.position[2] = par;
					ack = 1;
				break;
				case CAN_COMMAND_FILTER_4_SET_CALIBRATION:
					I2cAddJob(I2C_JOB_EEPROM_EXT_WRITE_INTEGER, EEPROM_EXT_ADDRESS_FILTER_POSITION_4, par, 0);
					Filter.position[3] = par;
					ack = 1;
				break;
				case CAN_COMMAND_FILTER_5_SET_CALIBRATION:
					I2cAddJob(I2C_JOB_EEPROM_EXT_WRITE_INTEGER, EEPROM_EXT_ADDRESS_FILTER_POSITION_5, par, 0);
					Filter.position[4] = par;
					ack = 1;
				break;
				case CAN_COMMAND_FILTER_SET_MIN_POSITION:
					I2cAddJob(I2C_JOB_EEPROM_EXT_WRITE_INTEGER, EEPROM_EXT_ADDRESS_FILTER_MIN_POSITION, par, 0);
					Filter.minposition = par;
					ack = 1;
				break;
				case CAN_COMMAND_FILTER_SET_MAX_POSITION:
					I2cAddJob(I2C_JOB_EEPROM_EXT_WRITE_INTEGER, EEPROM_EXT_ADDRESS_FILTER_MAX_POSITION, par, 0);
					Filter.maxposition = par;
					ack = 1;
				break;
				case CAN_COMMAND_FILTER_SET_DATE:
					I2cAddJob(I2C_JOB_EEPROM_EXT_WRITE_INTEGER, EEPROM_EXT_ADDRESS_FILTER_DATETIME, par, 0);
					DecodeFilterDate(par);//update filter date time structure
					ack = 1;
				break;
				case CAN_COMMAND_SET_PIXEL_RESOLUTION:
					I2cAddJob(I2C_JOB_EEPROM_EXT_WRITE_INTEGER, EEPROM_EXT_ADDRESS_PIXEL_RESOLUTION, par, 0);
					ack = 1;
				break;
				case CAN_COMMAND_STEPPER_SET_LOCK_POSITION:
					StepmotorSetLockPosition(parext, par);
					ack = 1;
				break;
				case CAN_COMMAND_SCRIPT_STOP:
					StopScript();
					ack = 1;
				break;
				case CAN_COMMAND_SCRIPT_CLEAR:
					ClearScript();
					ack = 1;
				break;
				case CAN_COMMAND_SCRIPT_ADD_ENTRY:
					AddScriptEntry(par & 0xFF, 0, 0, 0, 0, 0, 0);
					ack = 1;
				break;
				case CAN_COMMAND_SCRIPT_EDIT_PAR_1:
					EditScriptEntry(1, par);
					ack = 1;
				break;
				case CAN_COMMAND_SCRIPT_EDIT_PAR_2:
					EditScriptEntry(2, par);
					ack = 1;
				break;
				case CAN_COMMAND_SCRIPT_EDIT_PAR_3:
					EditScriptEntry(3, par);
					ack = 1;
				break;
				case CAN_COMMAND_SCRIPT_EDIT_PAR_4:
					EditScriptEntry(4, par);
					ack = 1;
				break;
				case CAN_COMMAND_SCRIPT_EDIT_PAR_5:
					EditScriptEntry(5, par);
					ack = 1;
				break;
				case CAN_COMMAND_SCRIPT_EDIT_PAR_6:
					EditScriptEntry(6, par);
					ack = 1;
				break;
				case CAN_COMMAND_SCRIPT_START:
					StartScript();
					ack = 1;
				break;
				case CAN_COMMAND_REQUEST_DEVICE_ID:
					val = I2cReadEepromInteger(EEPROM_ADDRESS_DEVICE_ID);
					CanAppendMessageToTxQueue(((MyCanId << 5) | CAN_ID_CONTROLLER), CAN_COMMAND_ANSWER_DEVICE_ID, val, 0, 0);
					ack = 2;//send nothing
				break;
				case CAN_COMMAND_REQUEST_SERIAL_NUMBER:
					val = I2cReadEepromInteger(EEPROM_ADDRESS_DEVICE_SERIAL_NUM);
					CanAppendMessageToTxQueue(((MyCanId << 5) | CAN_ID_CONTROLLER), CAN_COMMAND_ANSWER_SERIAL_NUMBER, val, 0, 0);
					ack = 2;//send nothing
				break;
				case CAN_COMMAND_REQUEST_OPTIC_TYPE:
					val = I2cReadEepromExtInteger(EEPROM_EXT_ADDRESS_OPTIC_TYP);
					CanAppendMessageToTxQueue(((MyCanId << 5) | CAN_ID_CONTROLLER), CAN_COMMAND_ANSWER_OPTIC_TYPE, val, 0, 0);
					ack = 2;//send nothing
				break;
				case CAN_COMMAND_REQUEST_OPTIC_SERIAL:
					val = I2cReadEepromExtInteger(EEPROM_EXT_ADDRESS_OPTIC_SN);
					CanAppendMessageToTxQueue(((MyCanId << 5) | CAN_ID_CONTROLLER), CAN_COMMAND_ANSWER_OPTIC_SERIAL, val, 0, 0);
					ack = 2;//send nothing
				break;
				case CAN_COMMAND_REQUEST_PIXEL_RESOLUTION:
					val = I2cReadEepromExtInteger(EEPROM_EXT_ADDRESS_PIXEL_RESOLUTION);
					CanAppendMessageToTxQueue(((MyCanId << 5) | CAN_ID_CONTROLLER), CAN_COMMAND_ANSWER_PIXEL_RESOLUTION, val, 0, 0);
					ack = 2;//send nothing
				break;
				case CAN_COMMAND_REQUEST_BOARD_TEMPERATURE:
					CanAppendMessageToTxQueue(((MyCanId << 5) | CAN_ID_CONTROLLER), CAN_COMMAND_ANSWER_BOARD_TEMPERATURE, BoardTemperature, 0, 0);
					ack = 2;//send nothing
				break;
				case CAN_COMMAND_REQUEST_EXPOSURE_CURRENT:
					state = ExposureControl[EXPOSURE_A].voltage;
					state = state << 16;
					state |= ExposureControl[EXPOSURE_A].current;
					CanAppendMessageToTxQueue(((MyCanId << 5) | CAN_ID_CONTROLLER), CAN_COMMAND_ANSWER_EXPOSURE_CURRENT, state, 0, 0);
					ack = 2;//send nothing
				break;
				case CAN_COMMAND_REQUEST_EXPOSURE_BRIGHTNESS:
					state = ExposureControl[EXPOSURE_A].sensortemperature;
					state = state << 16;
					state |= ExposureControl[EXPOSURE_A].brightness;
					CanAppendMessageToTxQueue(((MyCanId << 5) | CAN_ID_CONTROLLER), CAN_COMMAND_ANSWER_EXPOSURE_BRIGHTNESS, state, 0, 0);
					ack = 2;//send nothing
				break;
				case CAN_COMMAND_FILTER_REQUEST_MAX_POSITION:
					CanAppendMessageToTxQueue(((MyCanId << 5) | CAN_ID_CONTROLLER), CAN_COMMAND_FILTER_ANSWER_MAX_POSITION, Filter.maxposition, 0, 0);
					ack = 2;//send nothing
				break;
				case CAN_COMMAND_FILTER_REQUEST_MIN_POSITION:
					CanAppendMessageToTxQueue(((MyCanId << 5) | CAN_ID_CONTROLLER), CAN_COMMAND_FILTER_ANSWER_MIN_POSITION, Filter.minposition, 0, 0);
					ack = 2;//send nothing
				break;
				case CAN_COMMAND_FILTER_REQUEST_CALIBRATION:
					if(parext < NUMBER_OF_FILTERS)
					{
						CanAppendMessageToTxQueue(((MyCanId << 5) | CAN_ID_CONTROLLER), CAN_COMMAND_FILTER_ANSWER_CALIBRATION, Filter.position[parext], parext, 0);
						ack = 2;//send nothing
					}
				break;
				case CAN_COMMAND_FILTER_REQUEST_DATE:
					val = I2cReadEepromExtInteger(EEPROM_EXT_ADDRESS_FILTER_DATETIME);
					CanAppendMessageToTxQueue(((MyCanId << 5) | CAN_ID_CONTROLLER), CAN_COMMAND_FILTER_ANSWER_DATE, val, 0, 0);
					ack = 2;//send nothing
				break;
				case CAN_COMMAND_STEPPER_REQUEST_LOCK_POSITION:
					if(parext == 0)
					{
						val = I2cReadEepromExtInteger(EEPROM_EXT_ADDRESS_FILTER_LOCK_POSITION);
						CanAppendMessageToTxQueue(((MyCanId << 5) | CAN_ID_CONTROLLER), CAN_COMMAND_STEPPER_ANSWER_LOCK_POSITION, val, parext, 0);
					}
					if(parext == 1)
					{
						val = I2cReadEepromInteger(EEPROM_ADDRESS_FOCUS_LOCK_POSITION);
						CanAppendMessageToTxQueue(((MyCanId << 5) | CAN_ID_CONTROLLER), CAN_COMMAND_STEPPER_ANSWER_LOCK_POSITION, val, parext, 0);
					}
					ack = 2;//send nothing
				break;
				case CAN_COMMAND_STEPPER_GOTO_LOCK_POSITION:
					StepmotorGoToLockPosition(parext);
					ack = 1;//send ack
				break;

			}

			/* Stepper Motor Commands */
				for(mot = 0 ; mot < NUMBER_OF_STEPMOTORS ; mot++)
				{

						if(cmd == StepmotorCanCommand[mot].setspeed)
						{
							StepmotorSetSpeed(mot, par);
							ack = 1;//send ack
						}
						if(cmd == StepmotorCanCommand[mot].setacceleration)
						{
							StepmotorSetAcceleration(mot, par);
							ack = 1;//send ack
						}
						if(cmd == StepmotorCanCommand[mot].setdeceleration)
						{
							StepmotorSetDeceleration(mot, par);
							ack = 1;//send ack
						}
						if(cmd == StepmotorCanCommand[mot].drivesteps)
						{
							i = 1;
							ipar = (signed)par;
							if(ipar < 0)
							{
								// reverse direction and invert steps
								i = -1;
								ipar = ipar * -1;
							}
							StepmotorStart(mot, StepmotorControl[mot].positioningspeed, ipar, i);
							ack = 1;//send ack
						}
						if(cmd == StepmotorCanCommand[mot].referencedrive)
						{
							StepmotorReferenceDrive(mot);
							ack = 1;//send ack
						}
						if(cmd == StepmotorCanCommand[mot].gotoposition)
						{
							StepmotorGoToPosition(mot, par);
							ack = 1;//send ack
						}
						if(cmd == StepmotorCanCommand[mot].gotoencoder)
						{
							StepmotorGoToEncoderPosition(mot, par, 0);
							ack = 1;//send ack
						}
						if(cmd == StepmotorCanCommand[mot].stop)
						{
							StepmotorStop(mot);
							ack = 1;//send ack
						}
						if(cmd == StepmotorCanCommand[mot].requestposition)
						{
							CanAppendMessageToTxQueue(((MyCanId << 5) | CAN_ID_CONTROLLER),StepmotorCanCommand[mot].answerposition, StepmotorControl[mot].absolutecounter/StepmotorControl[mot].mscount, 0, 0);
							ack = 2;//send nothing
						}
						if(cmd == StepmotorCanCommand[mot].requestencoder)
						{
							CanAppendMessageToTxQueue(((MyCanId << 5) | CAN_ID_CONTROLLER),StepmotorCanCommand[mot].answerencoder, StepmotorControl[mot].encoderposition, 0, 0);
							ack = 2;//send nothing
						}
						if(cmd == StepmotorCanCommand[mot].requeststate)
						{
							state = (StepmotorControl[mot].finished << 31) | (StepmotorControl[mot].error << 8) | StepmotorControl[mot].reference;
							CanAppendMessageToTxQueue(((MyCanId << 5) | CAN_ID_CONTROLLER),StepmotorCanCommand[mot].answerstate, state, 0, 0);
							ack = 2;//send nothing
						}
						if(cmd == StepmotorCanCommand[mot].setmsconfig)
						{
							StepmotorSetMicrostepConfig(mot, par);
							ack = 2;//send nothing
						}
				}



				/* Exposure */
				for(exposure = 0 ; exposure < EXPOSURE_NUMBER_OF_PORTS ; exposure++)
				{
					if(cmd ==  ExposureCanCommand[exposure].setcurrent)
					{
						ExposureSetCurrent(exposure, par);
						ack = 1;//send ack
					}
					if(cmd == ExposureCanCommand[exposure].setline)
					{
						ExposureSetLine(exposure, par);
						ack = 1;//send ack
					}
					if(cmd == ExposureCanCommand[exposure].setpulse)
					{
						ExposureSetPulse(exposure, (par >> 16), (par & 0xFFFF));
						ack = 1;//send ack
					}
				}


			/* don't send ACK/NACK to other controllers except Main Controller */
			if(sender != CAN_ID_CONTROLLER)
			{
				return;
			}


			if(ack == 0)//send NACK
			{
				CanAppendMessageToTxQueue(((MyCanId << 5) | sender), CAN_COMMAND_ACKNOWLEDGE, 0, 0, 0);
			}
			if(ack == 1)//send ACK
			{
				CanAppendMessageToTxQueue(((MyCanId << 5) | sender), CAN_COMMAND_ACKNOWLEDGE, 1, 0, 0);
			}






}

// **************************************************************************** //
// ***       Function: CanRxWork                                              *** //
// ***       Purpose: triggers receive function                             *** //
// ***       Input: none                                                    *** //
// ***       Return: none                                                   *** //
// **************************************************************************** //
void CanRxWork(void)
{

	CanRxMsg msg;
	//uint8_t err;

	if(CAN_MessagePending(CAN1, CAN_FIFO0))
	{
		CanReceive(CAN1, CAN_FIFO0, &msg);
		CanDecode(&msg);

	}


	/*check CAN errors
	err = CanReadError();

	if((err | 0x01) || (err | 0x02))
	{
		//reset can interface
		CanInit(MyCanId);
	}
	*/



}

// **************************************************************************** //
// ***       Function: CanTxWork                                            *** //
// ***       Purpose: triggers transmit function                            *** //
// ***       Input: none                                                    *** //
// ***       Return: none                                                   *** //
// **************************************************************************** //
void CanTxWork(void)
{


	/* send pending messages */
	CanSendMessageFromTxQueue();

}


// **************************************************************************** //
// ***       Function: CanReadError                                         *** //
// ***       Purpose: readout can error status register                     *** //
// ***       Input: none                                                    *** //
// ***       Return: last error                                             *** //
// **************************************************************************** //
uint8_t CanReadError(void)
{

	CanError.errorcode = CAN_GetLastErrorCode(CAN_INTERFACE);
	CanError.rxerrcount = CAN_GetReceiveErrorCounter(CAN_INTERFACE);
	CanError.txerrcount = CAN_GetLSBTransmitErrorCounter(CAN_INTERFACE);
	CanError.warning = ((uint8_t)CAN_INTERFACE->ESR) & 0x01;
	CanError.buspassive = (((uint8_t)CAN_INTERFACE->ESR) & 0x02) >> 1;
	CanError.busoff = (((uint8_t)CAN_INTERFACE->ESR) & 0x04) >> 2;

	return CanError.errorcode;


}

// ******************************************************************* //
// ***       Function: CanAppendMessageToTxQueue                   *** //
// ***       Purpose: appends Telegram to CAN-Bus Send Queue       *** //
// ***       Input: id - ID of Message                             *** //
// ***              cmd - command (part of CAN-data)               *** //
// ***              data - command parameter (part of CAN-data)    *** //
// ***       Return: none                                          *** //
// ******************************************************************* //
void CanAppendMessageToTxQueue(uint16_t id, uint16_t cmd, uint32_t data, uint16_t dataext, uint8_t getack)
{
	uint8_t adr;

	/* prevent overrun */
	if(CanTxQueue.in >= CAN_TX_QUEUE_SIZE)
	{
		CanTxQueue.in = 0;
	}

	CanTxQueue.id[CanTxQueue.in] = id;
	CanTxQueue.cmd[CanTxQueue.in] = cmd;
	CanTxQueue.data[CanTxQueue.in] = data;
	CanTxQueue.dataext[CanTxQueue.in] = dataext;


	CanTxQueue.in++;

	adr = id & 0x1F;

	if(getack)
	{
		CanAcknowledge.owecnt[adr]++;
		CanAcknowledge.timeout[adr] = CAN_ACKNOWLEDGE_TIMEOUT;
	}

	return;

}

// ******************************************************************* //
// ***       Function: CanSendMessageFromTxQueue                   *** //
// ***       Purpose: appends Telegram to CAN-Bus Send Queue       *** //
// ***       Input: none                                           *** //
// ***       Return: none                                          *** //
// ******************************************************************* //
void CanSendMessageFromTxQueue(void)
{

	/* check, if message to send in Queue available */
	if(CanTxQueue.in != CanTxQueue.out)
	{
		/* send the message */
		CanSend(CanTxQueue.id[CanTxQueue.out], CanTxQueue.cmd[CanTxQueue.out], CanTxQueue.data[CanTxQueue.out], CanTxQueue.dataext[CanTxQueue.out]);

		/* increase output pointer */
		CanTxQueue.out++;

		/* prevent overrun of output pointer */
		if(CanTxQueue.out >= CAN_TX_QUEUE_SIZE)
		{
			CanTxQueue.out = 0;
		}

		/* reset pointers, if last message had been sent */
		if(CanTxQueue.in == CanTxQueue.out)
		{
			CanTxQueue.in = 0;
			CanTxQueue.out = 0;
		}

	}


	return;

}

// ******************************************************************* //
// ***       Function: CanResetInfoStructures                      *** //
// ***       Purpose: resets CanInfo structures                    *** //
// ***       Input: none                                           *** //
// ***       Return: none                                          *** //
// ******************************************************************* //
void CanResetInfoStructures(void)
{

	  int8_t i,j;

	  memset(&CanError, 0, sizeof(CanError));
	  memset(&CanAcknowledge, 0, sizeof(CanAcknowledge));

	  memset(CanSensorInfo, 0, sizeof(CanSensorInfo));
	  memset(&CanXYTableInfo, 0, sizeof(CanXYTableInfo));
	  memset(&CanBloodsensorInfo, 0, sizeof(CanBloodsensorInfo));
	  memset(&CanWeighcellInfo, 0, sizeof(CanWeighcellInfo));

	  memset(CanServomotorInfo, 0, sizeof(CanServomotorInfo));
	  /* set all finished flags */
	  	  for(i = 0 ; i < NUMBER_OF_CAN_IDS; i++)
	  	  {
	  		  for(j = 0 ; j < MAX_SERVOMOTOR_AXIS; j++)
	  		  {
	  			  CanServomotorInfo[i][j].finished = 1;
	  		  }
	  	  }

	  memset(CanStepperInfo, 0, sizeof(CanStepperInfo));
	  /* set all finished flags */
	  for(i = 0 ; i < NUMBER_OF_CAN_IDS; i++)
	  {
		  for(j = 0 ; j < MAX_STEPPER_AXIS; j++)
		  {
			  CanStepperInfo[i][j].finished = 1;
		  }
	  }
	  memset(CanDcMotorInfo, 0, sizeof(CanDcMotorInfo));
	  /* set all finished flags */
	  for(i = 0 ; i < NUMBER_OF_CAN_IDS; i++)
	  {
		  for(j = 0 ; j < MAX_DC_MOTOR_AXIS; j++)
		  {
			  CanDcMotorInfo[i][j].finished = 1;
		  }
	  }

	  memset(CanLysisInfo, 0, sizeof(CanLysisInfo));
	  memset(&CanDosageInfo, 0, sizeof(CanDosageInfo));

	  memset(&CanBrushlessInfo, 0, sizeof(CanBrushlessInfo));
	  memset(CanEnvironmentInfo, 0, sizeof(CanEnvironmentInfo));

	  memset(&CanRfidInfo, 0, sizeof(CanRfidInfo));

	  memset(&CanFilterInfo, 0, sizeof(CanFilterInfo));
	  memset(&CanExposureInfo, 0, sizeof(CanExposureInfo));

	  memset(CanAckReceived, 0, sizeof(CanAckReceived));

}




