/*********************************************************************/
/***  Project:       Blink ONE                                     ***/
/***  File:          can.h                                         ***/
/***  Created on:    13.07.2016                                    ***/
/***  Author:        Andreas Stärker                               ***/
/***  Company:       Blink AG                                      ***/
/***  Changes:       20.12.2016: Commands updated                  ***/
/***                 19.04.2017: AxisController implemented        ***/
/***                             Exposure implemented              ***/
/***                 26.04.2017: DC-Motor Speed request commands   ***/
/***                 23.08.2017 Light commands added               ***/
/***                 21.11.2017 CAN Addresses for BlinkBox added   ***/
/***                 15.12.2017 Lysis commands added               ***/
/***                 17.12.2017 Lysis request commands added       ***/
/***                 07.02.2018 Dosage commands, DosageInfo added  ***/
/*********************************************************************/

#ifndef CAN_H
#define CAN_H

#include "defines.h"

/* structures */

typedef struct
{
	uint8_t errorcode;
    uint8_t rxerrcount;
    uint16_t txerrcount;
    uint8_t busoff;// Bus Off Flag
    uint8_t buspassive;// Bus Passive Flag
    uint8_t warning;// Error warning
}TypeCanError;

#define CAN_TX_QUEUE_SIZE		32

typedef struct
{
	uint8_t in;// position of incoming message
    uint8_t out;//position of outgoing message
    uint16_t id[CAN_TX_QUEUE_SIZE];
    uint16_t cmd[CAN_TX_QUEUE_SIZE];
    uint32_t data[CAN_TX_QUEUE_SIZE];
    uint16_t dataext[CAN_TX_QUEUE_SIZE];
}TypeCanTxQueue;


/* defines */
//possible Values of MyCanId:
#define CAN_ID_CONTROLLER                     0x00 /* C */
#define CAN_ID_CO_CONTROLLER                  0x01 /* D */
#define CAN_ID_OPTIC                          0x02 /* O */
#define CAN_ID_PIPETTE                        0x03 /* P */
#define CAN_ID_CENTRIFUGE                     0x04 /* Z */
#define CAN_ID_STEPPER                        0x05 /* a */
#define CAN_ID_STEPPER_2ND                    0x06 /* b */
#define CAN_ID_STEPPER_3RD                    0x07 /* c */
#define CAN_ID_RFID                    		  0x08 /* d */
#define CAN_ID_AXIS                           0x09 /* A */
#define CAN_ID_LIGHT                          0x0A /* L */
#define CAN_ID_FILTER                         0x0B /* F */
#define CAN_ID_MAGAZINE                       0x0C /* M */
#define CAN_ID_CENTERLINE                     0x0D /* E */
#define CAN_ID_AXLE                           0x0E /* X */
#define CAN_ID_DOSAGE                         0x0F /* S */
#define CAN_ID_PCR                            0x10 /* R */
#define CAN_ID_MECH                           0x11 /* C */
#define NUMBER_OF_CAN_IDS                       18

extern const char CanAscii[NUMBER_OF_CAN_IDS];

typedef struct
{
	uint8_t owecnt[NUMBER_OF_CAN_IDS];// number of sent messages which are waiting for ack
	uint16_t timeout[NUMBER_OF_CAN_IDS];// count down in ms until timeout command to UART
}TypeCanAcknowledge;

#define CAN_ACKNOWLEDGE_TIMEOUT                    500 //in ms

extern TypeCanError CanError;
extern TypeCanTxQueue CanTxQueue;
extern TypeCanAcknowledge CanAcknowledge;


extern uint8_t MyCanId;


#define CAN_MOTOR_1ST                    1
#define CAN_MOTOR_2ND                    2
#define CAN_MOTOR_3RD                    3
#define CAN_MOTOR_4TH                    4

/* bits 0...4: Target, bits 5...9 Source, bit 10 always 0 */
#define CAN_ID_CONTROLLER_TO_ALL              (CAN_ID_CONTROLLER << 5)
#define CAN_ID_CONTROLLER_TO_CO_CONTROLLER    ((CAN_ID_CONTROLLER << 5) | CAN_ID_CO_CONTROLLER)
#define CAN_ID_CONTROLLER_TO_OPTIC            ((CAN_ID_CONTROLLER << 5) | CAN_ID_CO_OPTIC)
#define CAN_ID_CONTROLLER_TO_PIPETTE          ((CAN_ID_CONTROLLER << 5) | CAN_ID_CO_PIPETTE)
#define CAN_ID_CONTROLLER_TO_CENTRIFUGE       ((CAN_ID_CONTROLLER << 5) | CAN_ID_CO_CENTRIFUGE)
#define CAN_ID_CONTROLLER_TO_STEPPER          ((CAN_ID_CONTROLLER << 5) | CAN_ID_STEPPER)
#define CAN_ID_CONTROLLER_TO_STEPPER_2ND      ((CAN_ID_CONTROLLER << 5) | CAN_ID_STEPPER_2ND)
#define CAN_ID_CONTROLLER_TO_STEPPER_3RD      ((CAN_ID_CONTROLLER << 5) | CAN_ID_STEPPER_3RD)
#define CAN_ID_CONTROLLER_TO_STEPPER_4TH      ((CAN_ID_CONTROLLER << 5) | CAN_ID_STEPPER_4TH)
#define CAN_ID_CONTROLLER_TO_AXIS             ((CAN_ID_CONTROLLER << 5) | CAN_ID_AXIS)
#define CAN_ID_CONTROLLER_TO_LIGHT            ((CAN_ID_CONTROLLER << 5) | CAN_ID_LIGHT)

#define CAN_ID_CO_CONTROLLER_TO_ALL           (CAN_ID_CO_CONTROLLER << 5)
#define CAN_ID_CO_CONTROLLER_TO_CONTROLLER    ((CAN_ID_CO_CONTROLLER << 5) | CAN_ID_CONTROLLER)
#define CAN_ID_CO_CONTROLLER_TO_OPTIC         ((CAN_ID_CO_CONTROLLER << 5) | CAN_ID_CO_OPTIC)
#define CAN_ID_CO_CONTROLLER_TO_PIPETTE       ((CAN_ID_CO_CONTROLLER << 5) | CAN_ID_CO_PIPETTE)
#define CAN_ID_CO_CONTROLLER_TO_CENTRIFUGE    ((CAN_ID_CO_CONTROLLER << 5) | CAN_ID_CO_CENTRIFUGE)
#define CAN_ID_CO_CONTROLLER_TO_STEPPER_2ND   ((CAN_ID_CO_CONTROLLER << 5) | CAN_ID_STEPPER_2ND)
#define CAN_ID_CO_CONTROLLER_TO_STEPPER_3RD   ((CAN_ID_CO_CONTROLLER << 5) | CAN_ID_STEPPER_3RD)
#define CAN_ID_CO_CONTROLLER_TO_STEPPER_4TH   ((CAN_ID_CO_CONTROLLER << 5) | CAN_ID_STEPPER_4TH)
#define CAN_ID_CO_CONTROLLER_TO_AXIS          ((CAN_ID_CO_CONTROLLER << 5) | CAN_ID_AXIS)
#define CAN_ID_CO_CONTROLLER_TO_LIGHT         ((CAN_ID_CO_CONTROLLER << 5) | CAN_ID_LIGHT)

#define CAN_ID_STEPPER_TO_CONTROLLER          ((CAN_ID_STEPPER << 5) | CAN_ID_CONTROLLER)
#define CAN_ID_STEPPER_2ND_TO_CONTROLLER      ((CAN_ID_STEPPER_2ND << 5) | CAN_ID_CONTROLLER)
#define CAN_ID_STEPPER_3RD_TO_CONTROLLER      ((CAN_ID_STEPPER_3RD << 5) | CAN_ID_CONTROLLER)
#define CAN_ID_STEPPER_4TH_TO_CONTROLLER      ((CAN_ID_STEPPER_4TH << 5) | CAN_ID_CONTROLLER)
#define CAN_ID_PIPETTE_TO_CONTROLLER          ((CAN_ID_PIPETTE << 5) | CAN_ID_CONTROLLER)
#define CAN_ID_AXIS_TO_CONTROLLER             ((CAN_ID_AXIS << 5) | CAN_ID_CONTROLLER)
#define CAN_ID_LIGHT_TO_CONTROLLER            ((CAN_ID_LIGHT << 5) | CAN_ID_CONTROLLER)

//#define CAN_ID_OPTIC_TO_CONTROLLER            0x101
//#define CAN_ID_CENTRIFUGE_TO_CONTROLLER       0x002

//Commands:
#define CAN_COMMAND_LED_DEBUG_GREEN         		     1 //Parameter: 0 - OFF; 1 - ON
#define CAN_COMMAND_LED_DEBUG_YELLOW         		     2 //Parameter: 0 - OFF; 1 - ON
#define CAN_COMMAND_LED_DEBUG_RED            		     3 //Parameter: 0 - OFF; 1 - ON
#define CAN_COMMAND_LED_DEBUG_BLUE           		     4 //Parameter: 0 - OFF; 1 - ON
#define CAN_COMMAND_REQUEST_PCR_TEMPERATURE  		     5 //Parameter: none
#define CAN_COMMAND_ANSWER_PCR_TEMPERATURE   		     6 //Parameter: actual Temperature in 1/1000°C
#define CAN_COMMAND_MOTOR_X_SET_SPEED          		     7 //Parameter: Speed from 1...1000
#define CAN_COMMAND_MOTOR_X_SET_ACCELERATION    	     8 //Parameter: Acceleration from 1...100
#define CAN_COMMAND_MOTOR_X_SET_MS_CONFIG       	     9 //Parameter: microstep config from 0...3, 7
#define CAN_COMMAND_MOTOR_X_GOTO_POSITION      	        10 //Parameter: Absolute Position (signed)
#define CAN_COMMAND_MOTOR_X_GOTO_ENCODER_POSITION       11 //Parameter: Absolute Encoder Position (signed)
#define CAN_COMMAND_MOTOR_X_DRIVE_STEPS        	        12 //Parameter: relative Steps (signed)
#define CAN_COMMAND_MOTOR_X_DO_REFERENCE_DRIVE          13 //Parameter: none
#define CAN_COMMAND_MOTOR_X_STOP                        14 //Parameter: none
#define CAN_COMMAND_REQUEST_MOTOR_X_POSITION  	        15 //Parameter: none
#define CAN_COMMAND_ANSWER_MOTOR_X_POSITION   	        16 //Parameter: actual absolute Position in steps
#define CAN_COMMAND_REQUEST_MOTOR_X_STATE  	            17 //Parameter: none
#define CAN_COMMAND_ANSWER_MOTOR_X_STATE   	            18 //Parameter: Error State, MoveState, Reference State
#define CAN_COMMAND_REQUEST_ENCODER_X_POSITION          19 //Parameter: none
#define CAN_COMMAND_ANSWER_ENCODER_X_POSITION           20 //Parameter: actual encoder position
#define CAN_COMMAND_MOTOR_Y_SET_SPEED          	        21 //Parameter: Speed from 1...1000
#define CAN_COMMAND_MOTOR_Y_SET_ACCELERATION            22 //Parameter: Acceleration from 1...100
#define CAN_COMMAND_MOTOR_Y_SET_MS_CONFIG               23 //Parameter: microstep config from 0...3, 7
#define CAN_COMMAND_MOTOR_Y_GOTO_POSITION      	        24 //Parameter: Absolute Position (signed)
#define CAN_COMMAND_MOTOR_Y_GOTO_ENCODER_POSITION       25 //Parameter: Absolute Encoder Position (signed)
#define CAN_COMMAND_MOTOR_Y_DRIVE_STEPS        	        26 //Parameter: relative Steps (signed)
#define CAN_COMMAND_MOTOR_Y_DO_REFERENCE_DRIVE          27 //Parameter: none
#define CAN_COMMAND_MOTOR_Y_STOP                        28 //Parameter: none
#define CAN_COMMAND_REQUEST_MOTOR_Y_POSITION  	        29 //Parameter: none
#define CAN_COMMAND_ANSWER_MOTOR_Y_POSITION   	        30 //Parameter: actual absolute Position in steps
#define CAN_COMMAND_REQUEST_MOTOR_Y_STATE  	            31 //Parameter: none
#define CAN_COMMAND_ANSWER_MOTOR_Y_STATE   	            32 //Parameter: Error State, MoveState, Reference State
#define CAN_COMMAND_REQUEST_ENCODER_Y_POSITION          33 //Parameter: none
#define CAN_COMMAND_ANSWER_ENCODER_Y_POSITION           34 //Parameter: actual encoder position
#define CAN_COMMAND_MOTOR_Z_SET_SPEED          	        35 //Parameter: Speed from 1...1000
#define CAN_COMMAND_MOTOR_Z_SET_ACCELERATION            36 //Parameter: Acceleration from 1...100
#define CAN_COMMAND_MOTOR_Z_SET_MS_CONFIG               37 //Parameter: microstep config from 0...3, 7
#define CAN_COMMAND_MOTOR_Z_GOTO_POSITION      	        38 //Parameter: Absolute Position (signed)
#define CAN_COMMAND_MOTOR_Z_GOTO_ENCODER_POSITION       39 //Parameter: Absolute Encoder Position (signed)
#define CAN_COMMAND_MOTOR_Z_DRIVE_STEPS        	        40 //Parameter: relative Steps (signed)
#define CAN_COMMAND_MOTOR_Z_DO_REFERENCE_DRIVE          41 //Parameter: none
#define CAN_COMMAND_MOTOR_Z_STOP                        42 //Parameter: none
#define CAN_COMMAND_REQUEST_MOTOR_Z_POSITION  	        43 //Parameter: none
#define CAN_COMMAND_ANSWER_MOTOR_Z_POSITION   	        44 //Parameter: actual absolute Position in steps
#define CAN_COMMAND_REQUEST_MOTOR_Z_STATE  	            45 //Parameter: none
#define CAN_COMMAND_ANSWER_MOTOR_Z_STATE   	            46 //Parameter: Error State, MoveState, Reference State
#define CAN_COMMAND_REQUEST_ENCODER_Z_POSITION          47 //Parameter: none
#define CAN_COMMAND_ANSWER_ENCODER_Z_POSITION           48 //Parameter: actual encoder position
#define CAN_COMMAND_MOTOR_E_SET_SPEED          	        49 //Parameter: Speed from 1...1000
#define CAN_COMMAND_MOTOR_E_SET_ACCELERATION            50 //Parameter: Acceleration from 1...100
#define CAN_COMMAND_MOTOR_E_SET_MS_CONFIG               51 //Parameter: microstep config from 0...3, 7
#define CAN_COMMAND_MOTOR_E_GOTO_POSITION      	        52 //Parameter: Absolute Position (signed)
#define CAN_COMMAND_MOTOR_E_GOTO_ENCODER_POSITION       53 //Parameter: Absolute Encoder Position (signed)
#define CAN_COMMAND_MOTOR_E_DRIVE_STEPS        	        54 //Parameter: relative Steps (signed)
#define CAN_COMMAND_MOTOR_E_DO_REFERENCE_DRIVE          55 //Parameter: none
#define CAN_COMMAND_MOTOR_E_STOP                        56 //Parameter: none
#define CAN_COMMAND_REQUEST_MOTOR_E_POSITION  	        57 //Parameter: none
#define CAN_COMMAND_ANSWER_MOTOR_E_POSITION   	        58 //Parameter: actual absolute Position in steps
#define CAN_COMMAND_REQUEST_MOTOR_E_STATE  	            59 //Parameter: none
#define CAN_COMMAND_ANSWER_MOTOR_E_STATE   	            60 //Parameter: Error State, MoveState, Reference State
#define CAN_COMMAND_REQUEST_ENCODER_E_POSITION          61 //Parameter: none
#define CAN_COMMAND_ANSWER_ENCODER_E_POSITION           62 //Parameter: actual encoder position
#define CAN_COMMAND_REQUEST_PCR_TEMPERATURE_2  	        63 //Parameter: none (Temperature of 2nd NTC)
#define CAN_COMMAND_ANSWER_PCR_TEMPERATURE_2   	        64 //Parameter: actual Temperature of 2nd NTC in 1/1000°C
#define CAN_COMMAND_DC_MOTOR_1_SET_SPEED                65 //Parameter: Speed from 1...100
#define CAN_COMMAND_DC_MOTOR_1_DO_REFERENCE_DRIVE       66 //Parameter: none
#define CAN_COMMAND_DC_MOTOR_1_GOTO_ENCODER_POSITION    67 //Parameter: Absolute Encoder Position (signed)
#define CAN_COMMAND_DC_MOTOR_1_STOP                     68 //Parameter: none
#define CAN_COMMAND_REQUEST_DC_MOTOR_1_STATE  	        69 //Parameter: none
#define CAN_COMMAND_ANSWER_DC_MOTOR_1_STATE   	        70 //Parameter: Error State, MoveState, Reference State
#define CAN_COMMAND_REQUEST_DC_MOTOR_1_SPEED            71 //Parameter: none
#define CAN_COMMAND_ANSWER_DC_MOTOR_1_SPEED             72 //Parameter: actual encoder position
#define CAN_COMMAND_REQUEST_ENCODER_1_POSITION          73 //Parameter: none
#define CAN_COMMAND_ANSWER_ENCODER_1_POSITION           74 //Parameter: actual encoder position
#define CAN_COMMAND_DC_MOTOR_2_SET_SPEED                75 //Parameter: Speed from 1...100
#define CAN_COMMAND_DC_MOTOR_2_DO_REFERENCE_DRIVE       76 //Parameter: none
#define CAN_COMMAND_DC_MOTOR_2_GOTO_ENCODER_POSITION    77 //Parameter: Absolute Encoder Position (signed)
#define CAN_COMMAND_DC_MOTOR_2_STOP                     78 //Parameter: none
#define CAN_COMMAND_REQUEST_DC_MOTOR_2_STATE  	        79 //Parameter: none
#define CAN_COMMAND_ANSWER_DC_MOTOR_2_STATE   	        80 //Parameter: Error State, MoveState, Reference State
#define CAN_COMMAND_REQUEST_DC_MOTOR_2_SPEED            81 //Parameter: none
#define CAN_COMMAND_ANSWER_DC_MOTOR_2_SPEED             82 //Parameter: actual encoder position
#define CAN_COMMAND_REQUEST_ENCODER_2_POSITION          83 //Parameter: none
#define CAN_COMMAND_ANSWER_ENCODER_2_POSITION           84 //Parameter: actual encoder position
#define CAN_COMMAND_DC_MOTOR_3_SET_SPEED                85 //Parameter: Speed from 1...100
#define CAN_COMMAND_DC_MOTOR_3_DO_REFERENCE_DRIVE       86 //Parameter: none
#define CAN_COMMAND_DC_MOTOR_3_GOTO_ENCODER_POSITION    87 //Parameter: Absolute Encoder Position (signed)
#define CAN_COMMAND_DC_MOTOR_3_STOP                     88 //Parameter: none
#define CAN_COMMAND_REQUEST_DC_MOTOR_3_STATE  	        89 //Parameter: none
#define CAN_COMMAND_ANSWER_DC_MOTOR_3_STATE   	        90 //Parameter: Error State, MoveState, Reference State
#define CAN_COMMAND_REQUEST_DC_MOTOR_3_SPEED            91 //Parameter: none
#define CAN_COMMAND_ANSWER_DC_MOTOR_3_SPEED             92 //Parameter: actual encoder position
#define CAN_COMMAND_REQUEST_ENCODER_3_POSITION          93 //Parameter: none
#define CAN_COMMAND_ANSWER_ENCODER_3_POSITION           94 //Parameter: actual encoder position
#define CAN_COMMAND_EXPOSURE_A_LINE                     95 //Parameter: line
#define CAN_COMMAND_EXPOSURE_A_CURRENT                  96 //Parameter: current
#define CAN_COMMAND_EXPOSURE_A_PULSE                    97 //Parameter: period, width
#define CAN_COMMAND_EXPOSURE_B_LINE                     98 //Parameter: line
#define CAN_COMMAND_EXPOSURE_B_CURRENT                  99 //Parameter: current
#define CAN_COMMAND_EXPOSURE_B_PULSE                   100 //Parameter: period, width
#define CAN_COMMAND_LIGHT_START_ANIMATION              101 //Parameter: index of Animation to start
#define CAN_COMMAND_LIGHT_STOP_ANIMATION               102 //Parameter: none
#define CAN_COMMAND_LIGHT_SET_FADE                     103 //Parameter: fade steps, 0 = fade off
#define CAN_COMMAND_LIGHT_START_RANDOM                 104 //Parameter: time per image in 100ms
#define CAN_COMMAND_LIGHT_PANEL_SET_COLOR              105 //Parameter: color 0x00RRGGBB
#define CAN_COMMAND_LIGHT_PANEL_SET_BRIGHTNESS         106 //Parameter: brightness 0...127
#define CAN_COMMAND_ACKNOWLEDGE                        107 //Parameter: 1 - ACK, 0 - NACK
#define CAN_COMMAND_LYSIS_PORT_A_SET_TEMPERATURE       108 //Parameter: Temperature  in 1/1000°C
#define CAN_COMMAND_LYSIS_PORT_A_SET_TIME              109 //Parameter: Time  in 1/100s
#define CAN_COMMAND_LYSIS_PORT_A_START                 110 //Parameter: Channel 1...16 0 - OFF
#define CAN_COMMAND_LYSIS_PORT_A_STOP                  111 //Parameter: none
#define CAN_COMMAND_LYSIS_PORT_B_SET_TEMPERATURE       112 //Parameter: Temperature  in 1/1000°C
#define CAN_COMMAND_LYSIS_PORT_B_SET_TIME              113 //Parameter: Time  in 1/100s
#define CAN_COMMAND_LYSIS_PORT_B_START                 114 //Parameter: Channel 1...16 0 - OFF
#define CAN_COMMAND_LYSIS_PORT_B_STOP                  115 //Parameter: none
#define CAN_COMMAND_LYSIS_PORT_A_REQUEST_STATUS        116 //Parameter: none
#define CAN_COMMAND_LYSIS_PORT_B_REQUEST_STATUS        117 //Parameter: none
#define CAN_COMMAND_LYSIS_PORT_A_ANSWER_STATUS         118 //Parameter: actual status
#define CAN_COMMAND_LYSIS_PORT_B_ANSWER_STATUS         119 //Parameter: actual status
#define CAN_COMMAND_LYSIS_PORT_A_ANSWER_TEMPINT        120 //Parameter: actual internal temperature
#define CAN_COMMAND_LYSIS_PORT_B_ANSWER_TEMPINT        121 //Parameter: actual internal temperature
#define CAN_COMMAND_LYSIS_PORT_A_ANSWER_TEMPEXT        122 //Parameter: actual external temperature
#define CAN_COMMAND_LYSIS_PORT_B_ANSWER_TEMPEXT        123 //Parameter: actual external temperature
#define CAN_COMMAND_LIGHT_START_SCRIPT                 124 //Parameter: index of Script to start
#define CAN_COMMAND_LIGHT_STOP_SCRIPT                  125 //Parameter: none
#define CAN_COMMAND_LIGHT_SET_LINE_COLOR               126 //Parameter: Line (MSByte), RRGGBB
#define CAN_COMMAND_LIGHT_SET_LINE_INTENSITY           127 //Parameter: Line (MSByte), Intensity
#define CAN_COMMAND_LIGHT_SET_COLUMN_COLOR             128 //Parameter: Column (MSByte), RRGGBB
#define CAN_COMMAND_LIGHT_SET_COLUMN_INTENSITY         129 //Parameter: Column (MSByte), Intensity
#define CAN_COMMAND_LIGHT_SAVE_PANEL_TO_EEPROM         130 //Parameter: none
#define CAN_COMMAND_LIGHT_OPEN_PANEL_FROM_EEPROM       131 //Parameter: none
#define CAN_COMMAND_LIGHT_RESET_MEMORY                 132 //Parameter: none
#define CAN_COMMAND_LIGHT_SET_PIXEL_COORDINATES        133 //Parameter: line, column (LSByte)
#define CAN_COMMAND_LIGHT_SET_PIXEL_COLOR              134 //Parameter: color 0x00RRGGBB
#define CAN_COMMAND_LIGHT_SET_PIXEL_INTENSITY          135 //Parameter: Intensity in %
#define CAN_COMMAND_DOSAGE_REQUEST_TOUCH               136 //Parameter: none
#define CAN_COMMAND_DOSAGE_ANSWER_TOUCH                137 //Parameter: new state
#define CAN_COMMAND_DOSAGE_REQUEST_PRESSURE            138 //Parameter: none
#define CAN_COMMAND_DOSAGE_ANSWER_PRESSURE             139 //Parameter: pressure in mBar
#define CAN_COMMAND_MOTOR_X_GOTO_ENCODER_LIQUID        140 //Parameter: maximum absolute Encoder Position (signed)
#define CAN_COMMAND_MOTOR_Y_GOTO_ENCODER_LIQUID	       141 //Parameter: maximum absolute Encoder Position (signed)
#define CAN_COMMAND_MOTOR_Z_GOTO_ENCODER_LIQUID        142 //Parameter: maximum absolute Encoder Position (signed)
#define CAN_COMMAND_MOTOR_X_SET_DECELERATION           143 //Parameter: Deceleration from 0...1000
#define CAN_COMMAND_MOTOR_Y_SET_DECELERATION           144 //Parameter: Deceleration from 0...1000
#define CAN_COMMAND_MOTOR_Z_SET_DECELERATION           145 //Parameter: Deceleration from 0...1000
#define CAN_COMMAND_MOTOR_E_SET_DECELERATION           146 //Parameter: Deceleration from 0...1000
#define CAN_COMMAND_DC_MOTOR_1_GOTO_CART_POS           147 //Parameter: Mode (MSByte), Position (2ndByte), PWM (3.,4.Byte)
#define CAN_COMMAND_DC_MOTOR_2_GOTO_CART_POS           148 //Parameter: Mode (MSByte), Position (2ndByte), PWM (3.,4.Byte)
#define CAN_COMMAND_DC_MOTOR_3_GOTO_CART_POS           149 //Parameter: Mode (MSByte), Position (2ndByte), PWM (3.,4.Byte)
#define CAN_COMMAND_DC_MOTOR_1_GOTO_LB_HIGH            150 //Parameter: PWM
#define CAN_COMMAND_DC_MOTOR_2_GOTO_LB_HIGH            151 //Parameter: PWM
#define CAN_COMMAND_DC_MOTOR_3_GOTO_LB_HIGH            152 //Parameter: PWM
#define CAN_COMMAND_DC_MOTOR_1_GOTO_LB_LOW             153 //Parameter: PWM
#define CAN_COMMAND_DC_MOTOR_2_GOTO_LB_LOW             154 //Parameter: PWM
#define CAN_COMMAND_DC_MOTOR_3_GOTO_LB_LOW             155 //Parameter: PWM
#define CAN_COMMAND_DC_MOTOR_1_START_TIME              156 //Parameter: Direction (MSByte), time (2.-4.Byte) in 1/10 s
#define CAN_COMMAND_DC_MOTOR_2_START_TIME              157 //Parameter: Direction (MSByte), time (2.-4.Byte) in 1/10 s
#define CAN_COMMAND_DC_MOTOR_3_START_TIME              158 //Parameter: Direction (MSByte), time (2.-4.Byte) in 1/10 s
#define CAN_COMMAND_REQUEST_SENSOR_1  	        	   159 //Parameter: none
#define CAN_COMMAND_ANSWER_SENSOR_1   	               160 //Parameter: State (MSByte), value (2.-4.Byte)
#define CAN_COMMAND_REQUEST_SENSOR_2  	        	   161 //Parameter: none
#define CAN_COMMAND_ANSWER_SENSOR_2   	               162 //Parameter: State (MSByte), value (2.-4.Byte)
#define CAN_COMMAND_REQUEST_SENSOR_3  	        	   163 //Parameter: none
#define CAN_COMMAND_ANSWER_SENSOR_3   	               164 //Parameter: State (MSByte), value (2.-4.Byte)
#define CAN_COMMAND_SERVO_1_SET_POS  	        	   165 //Parameter: position
#define CAN_COMMAND_SERVO_2_SET_POS  	        	   166 //Parameter: position
#define CAN_COMMAND_SERVO_3_SET_POS  	        	   167 //Parameter: position
#define CAN_COMMAND_SERVO_1_RESET_FUSE  	           168 //Parameter: none
#define CAN_COMMAND_SERVO_2_RESET_FUSE  	           169 //Parameter: none
#define CAN_COMMAND_SERVO_3_RESET_FUSE  	           170 //Parameter: none
#define CAN_COMMAND_REQUEST_SERVO_1_STATE 	           171 //Parameter: none
#define CAN_COMMAND_ANSWER_SERVO_1_STATE 	           172 //Parameter: Fuse-State (MSByte), finished (2.Byte), position (3.-4.Byte)
#define CAN_COMMAND_REQUEST_SERVO_2_STATE 	           173 //Parameter: none
#define CAN_COMMAND_ANSWER_SERVO_2_STATE 	           174 //Parameter: Fuse-State (MSByte), finished (2.Byte), position (3.-4.Byte)
#define CAN_COMMAND_REQUEST_SERVO_3_STATE 	           175 //Parameter: none
#define CAN_COMMAND_ANSWER_SERVO_3_STATE 	           176 //Parameter: Fuse-State (MSByte), finished (2.Byte), position (3.-4.Byte)
#define CAN_COMMAND_REQUEST_SERVO_1_CURRENT 	       177 //Parameter: none
#define CAN_COMMAND_ANSWER_SERVO_1_CURRENT 	           178 //Parameter: current [mA]
#define CAN_COMMAND_REQUEST_SERVO_2_CURRENT 	       179 //Parameter: none
#define CAN_COMMAND_ANSWER_SERVO_2_CURRENT 	           180 //Parameter: current [mA]
#define CAN_COMMAND_REQUEST_SERVO_3_CURRENT 	       181 //Parameter: none
#define CAN_COMMAND_ANSWER_SERVO_3_CURRENT 	           182 //Parameter: current [mA]
#define CAN_COMMAND_BRUSHLESS_SET_POSITION 	           183 //Parameter: position (int32_t)
#define CAN_COMMAND_BRUSHLESS_REQUEST_POSITION 	       184 //Parameter: position (int32_t)
#define CAN_COMMAND_BRUSHLESS_ANSWER_POSITION 	       185 //Parameter: position (int32_t)
#define CAN_COMMAND_BRUSHLESS_REQUEST_STATE 	       186 //Parameter: position (int32_t)
#define CAN_COMMAND_BRUSHLESS_ANSWER_STATE  	       187 //Parameter: MSB: error, 2.Byte: Filterposition, 3.Byte: CANOpen-Status, LSB: finished (position ok)
#define CAN_COMMAND_FILTER_SET_POSITION 	           188 //Parameter: LSB (uint8_t): 1...3 - go to position , 0-> deactivate
#define CAN_COMMAND_FILTER_REQUEST_POSITION 	       189 //Parameter:  none
#define CAN_COMMAND_FILTER_ANSWER_POSITION 	           190 //Parameter: LSB (uint8_t): 0...3
#define CAN_COMMAND_THERM_SET_TEMPERATURE 	           191 //Parameter: temperature (int32_t)[1/1000°C], activates regulation
#define CAN_COMMAND_THERM_PELTIER_OFF   	           192 //Parameter: none, disables peltier and regulation
#define CAN_COMMAND_THERM_SCRIPT_SET_TEMP_1   	       193 //Parameter: temperature (int32_t) [1/1000°C]
#define CAN_COMMAND_THERM_SCRIPT_SET_TEMP_2   	       194 //Parameter: temperature (int32_t) [1/1000°C]
#define CAN_COMMAND_THERM_SCRIPT_SET_TEMP_3   	       195 //Parameter: temperature (int32_t) [1/1000°C]
#define CAN_COMMAND_THERM_SCRIPT_SET_TIME_1   	       196 //Parameter: time (uint32_t)[1/10s]
#define CAN_COMMAND_THERM_SCRIPT_SET_TIME_2   	       197 //Parameter: time (uint32_t)[1/10s]
#define CAN_COMMAND_THERM_SCRIPT_SET_TIME_3   	       198 //Parameter: time (uint32_t)[1/10s]
#define CAN_COMMAND_THERM_SCRIPT_SET_LOOP   	       199 //Parameter: number of loops (uint32_t)
#define CAN_COMMAND_THERM_SCRIPT_START	     	       200 //Parameter: index of demo script (0: PCR-Script)
#define CAN_COMMAND_THERM_SCRIPT_STOP	     	       201 //Parameter: none, also disables peltier and regulation
#define CAN_COMMAND_THERM_SCRIPT_REQUEST_STATE 	       202 //Parameter: none
#define CAN_COMMAND_THERM_SCRIPT_ANSWER_STATE  	       203 //Parameter: MSB: error 2.Byte...3.Byte: Index,  LSB: running
#define CAN_COMMAND_THERM_REQUEST_PELTIER 	           204 //Parameter: none
#define CAN_COMMAND_THERM_ANSWER_PELTIER  	           205 //Parameter: MSB...2.Byte: PWM, 3.Byte: h/k, regulation,  LSB: reached, enabled
#define CAN_COMMAND_THERM_REQUEST_SOLL_TEMPERATURE 	   206 //Parameter: none
#define CAN_COMMAND_THERM_ANSWER_SOLL_TEMPERATURE  	   207 //Parameter: Soll-Temperature
#define CAN_COMMAND_THERM_REQUEST_SINK_TEMPERATURE 	   208 //Parameter: none
#define CAN_COMMAND_THERM_ANSWER_SINK_TEMPERATURE  	   209 //Parameter: Heat-Sink-Temperature
#define CAN_COMMAND_THERM_REQUEST_SLIDE_TEMPERATURE    210 //Parameter: none
#define CAN_COMMAND_THERM_ANSWER_SLIDE_TEMPERATURE     211 //Parameter: Slide-Temperature
#define CAN_COMMAND_DC_MOTOR_1_CLOSE				   212 //Parameter: PWM
#define CAN_COMMAND_DC_MOTOR_1_OPEN					   213 //Parameter: PWM
#define CAN_COMMAND_DC_MOTOR_2_CLOSE				   214 //Parameter: PWM
#define CAN_COMMAND_DC_MOTOR_2_OPEN					   215 //Parameter: PWM
#define CAN_COMMAND_DC_MOTOR_3_CLOSE				   216 //Parameter: PWM
#define CAN_COMMAND_DC_MOTOR_3_OPEN					   217 //Parameter: PWM
#define CAN_COMMAND_SCRIPT_ADD_ENTRY         	       218 //Parameter: script type
#define CAN_COMMAND_SCRIPT_EDIT_PAR_1         	       219 //Parameter: par1
#define CAN_COMMAND_SCRIPT_EDIT_PAR_2         	       220 //Parameter: par2
#define CAN_COMMAND_SCRIPT_EDIT_PAR_3         	       221 //Parameter: par3
#define CAN_COMMAND_SCRIPT_EDIT_PAR_4         	       222 //Parameter: par4
#define CAN_COMMAND_SCRIPT_EDIT_PAR_5         	       223 //Parameter: par5
#define CAN_COMMAND_SCRIPT_START         	           224 //Parameter: none
#define CAN_COMMAND_SCRIPT_STOP         	           225 //Parameter: none
#define CAN_COMMAND_SCRIPT_CLEAR         	           226 //Parameter: none
#define CAN_COMMAND_LIGHT_START_BATTERY_ANIMATION      227 //Parameter: percentage of Battery Charge
#define CAN_COMMAND_LIGHT_START_CHARGE_ANIMATION       228 //Parameter: percentage of Battery Charge
#define CAN_COMMAND_MOTOR_X_SET_INJECTOR_REFERENCE     229 //Parameter: injector Reference (stores into EEPROM)
#define CAN_COMMAND_MOTOR_X_REQUEST_INJECTOR_REFERENCE 230 //Parameter: injector Reference (reads from EEPROM)
#define CAN_COMMAND_MOTOR_X_ANSWER_INJECTOR_REFERENCE  231 //Parameter: injector Reference
#define CAN_COMMAND_MOTOR_X_GOTO_INJECTOR_REFERENCE    232 //Parameter: none
#define CAN_COMMAND_REQUEST_DEVICE_ID                  233 //Parameter: none
#define CAN_COMMAND_ANSWER_DEVICE_ID                   234 //Parameter: Device-ID
#define CAN_COMMAND_REQUEST_SERIAL_NUMBER              235 //Parameter: none
#define CAN_COMMAND_ANSWER_SERIAL_NUMBER               236 //Parameter: SerialNumber
#define CAN_COMMAND_MOTOR_X_SET_INJECTOR_DOCKPOS       237 //Parameter: injector Reference (stores into EEPROM)
#define CAN_COMMAND_MOTOR_X_REQUEST_INJECTOR_DOCKPOS   238 //Parameter: injector Reference (reads from EEPROM)
#define CAN_COMMAND_MOTOR_X_ANSWER_INJECTOR_DOCKPOS    239 //Parameter: injector Reference
#define CAN_COMMAND_MOTOR_X_GOTO_INJECTOR_DOCKPOS      240 //Parameter: none
#define CAN_COMMAND_BRUSHLESS_DO_REFERENCE_DRIVE       241 //Parameter: position (int32_t)
#define CAN_COMMAND_THERM_SET_MODE		 	           242 //Parameter: Mode
#define CAN_COMMAND_THERM_SET_MAX_FLUID_TEMP		   243 //Parameter: max fluid temperature (int32_t)[1/1000°C]
#define CAN_COMMAND_XY_TABLE_SET_POSITION		       244 //Parameter: MSB: Position X (int16_t) LSB: Position Y (int16_t)
#define CAN_COMMAND_XY_TABLE_REQUEST_POSITION		   245 //Parameter: none
#define CAN_COMMAND_XY_TABLE_ANSWER_POSITION		   246 //Parameter: MSB: Position X (int16_t) LSB: Position Y (int16_t)
#define CAN_COMMAND_XY_TABLE_REQUEST_STATE		       247 //Parameter: none
#define CAN_COMMAND_XY_TABLE_ANSWER_STATE		       248 //Parameter: Error State, MoveState, Reference State
#define CAN_COMMAND_XY_TABLE_DO_REFERENCE_DRIVE	       249 //Parameter: none
#define CAN_COMMAND_XY_TABLE_STOP		       		   250 //Parameter: none
#define CAN_COMMAND_BLOODSENSOR_START		       	   251 //Parameter: none
#define CAN_COMMAND_BLOODSENSOR_STOP		       	   252 //Parameter: none
#define CAN_COMMAND_BLOODSENSOR_REQUEST_DATA           253 //Parameter: 0 - off-data, 2 - red data, 4 - blue data, 8 - detect state
#define CAN_COMMAND_BLOODSENSOR_ANSWER_OFF             254 //Parameter: off-value (16 bit)
#define CAN_COMMAND_BLOODSENSOR_ANSWER_RED             255 //Parameter: red-value (16 bit)
#define CAN_COMMAND_BLOODSENSOR_ANSWER_BLUE            256 //Parameter: blue-value (16 bit)
#define CAN_COMMAND_BLOODSENSOR_ANSWER_DETECT          257 //Parameter: off-value (16 bit)
#define CAN_COMMAND_WEIGHCELL_START		       	       258 //Parameter: none
#define CAN_COMMAND_WEIGHCELL_STOP		       	       259 //Parameter: none
#define CAN_COMMAND_WEIGHCELL_REQUEST_DATA             260 //Parameter: 0
#define CAN_COMMAND_WEIGHCELL_ANSWER_DATA              261 //Parameter:  measurement result (adc value)
#define CAN_COMMAND_REQUEST_DC_MOTOR_1_ENDPOS  	       262 //Parameter: none
#define CAN_COMMAND_ANSWER_DC_MOTOR_1_ENDPOS   	       263 //Parameter:  Bit 15..8: Overshoot, Bit 7...0: Endpos
#define CAN_COMMAND_REQUEST_DC_MOTOR_2_ENDPOS  	       264 //Parameter: none
#define CAN_COMMAND_ANSWER_DC_MOTOR_2_ENDPOS   	       265 //Parameter:  Bit 15..8: Overshoot, Bit 7...0: Endpos
#define CAN_COMMAND_REQUEST_DC_MOTOR_3_ENDPOS  	       266 //Parameter: none
#define CAN_COMMAND_ANSWER_DC_MOTOR_3_ENDPOS   	       267 //Parameter:  Bit 15..8: Overshoot, Bit 7...0: Endpos
#define CAN_COMMAND_REQUEST_SERVO_1_ENDPOS 	           268 //Parameter: none
#define CAN_COMMAND_ANSWER_SERVO_1_ENDPOS 	           269 //Parameter: error bits 23...8, endpos bits 7...0
#define CAN_COMMAND_REQUEST_SERVO_2_ENDPOS 	           270 //Parameter: none
#define CAN_COMMAND_ANSWER_SERVO_2_ENDPOS 	           271 //Parameter: error bits 23...8, endpos bits 7...0
#define CAN_COMMAND_REQUEST_SERVO_3_ENDPOS 	           272 //Parameter: none
#define CAN_COMMAND_ANSWER_SERVO_3_ENDPOS 	           273 //Parameter: error bits 23...8, endpos bits 7...0
#define CAN_COMMAND_SERVO_1_MOVE_IN 	           	   274 //Parameter: none
#define CAN_COMMAND_SERVO_1_MOVE_OUT 	           	   275 //Parameter: none
#define CAN_COMMAND_SERVO_2_MOVE_IN 	           	   276 //Parameter: none
#define CAN_COMMAND_SERVO_2_MOVE_OUT 	           	   277 //Parameter: none
#define CAN_COMMAND_SERVO_3_MOVE_IN 	           	   278 //Parameter: none
#define CAN_COMMAND_SERVO_3_MOVE_OUT 	           	   279 //Parameter: none
#define CAN_COMMAND_ADD_MOVEMENT_PERMIT_JOKER      	   280 //Parameter: none
#define CAN_COMMAND_MOTOR_X_FORCE_SET_ENCODER          281 //Parameter: absolute Encoder Position for force drive (signed)
#define CAN_COMMAND_MOTOR_Y_FORCE_SET_ENCODER	       282 //Parameter: absolute Encoder Position for force drive (signed)
#define CAN_COMMAND_MOTOR_Z_FORCE_SET_ENCODER          283 //Parameter: absolute Encoder Position for force drive (signed)
#define CAN_COMMAND_MOTOR_X_GOTO_ENCODER_WATCH_FORCE   284 //Parameter: none
#define CAN_COMMAND_MOTOR_Y_GOTO_ENCODER_WATCH_FORCE   285 //Parameter: none
#define CAN_COMMAND_MOTOR_Z_GOTO_ENCODER_WATCH_FORCE   286 //Parameter: none
#define CAN_COMMAND_REQUEST_BOARD_TEMPERATURE 	       287 //Parameter: none
#define CAN_COMMAND_ANSWER_BOARD_TEMPERATURE 	       288 //Parameter: temperature in 1/1000 °C (signed)
#define CAN_COMMAND_THERM_REQUEST_ERRORCODE			   289 //Parameter: none
#define CAN_COMMAND_THERM_ANSWER_ERRORCODE			   290 //Parameter: none
#define CAN_COMMAND_THERM_REQUEST_WARNINGCODE		   291 //Parameter: none
#define CAN_COMMAND_THERM_ANSWER_WARNINGCODE		   292 //Parameter: none
#define CAN_COMMAND_BLOODSENSOR_ANSWER_VOLUME          293 //Parameter: encoder-volume (int 32 bit)
#define CAN_COMMAND_BLOODSENSOR_ANSWER_ERROR           294 //Parameter: bit 23...16: edge, bit 15..0: error
#define CAN_COMMAND_FILTER_1_SET_CALIBRATION 	       295 //Parameter: Position Value
#define CAN_COMMAND_FILTER_2_SET_CALIBRATION 	       296 //Parameter: Position Value
#define CAN_COMMAND_FILTER_3_SET_CALIBRATION 	       297 //Parameter: Position Value
#define CAN_COMMAND_FILTER_4_SET_CALIBRATION 	       298 //Parameter: Position Value
#define CAN_COMMAND_SERVO_1_SET_CALIBRATION_IN 	       299 //Parameter: Position Value
#define CAN_COMMAND_SERVO_1_SET_CALIBRATION_OUT 	   300 //Parameter: Position Value
#define CAN_COMMAND_SERVO_2_SET_CALIBRATION_IN 	       301 //Parameter: Position Value
#define CAN_COMMAND_SERVO_2_SET_CALIBRATION_OUT 	   302 //Parameter: Position Value
#define CAN_COMMAND_SERVO_3_SET_CALIBRATION_IN 	       303 //Parameter: Position Value
#define CAN_COMMAND_SERVO_3_SET_CALIBRATION_OUT 	   304 //Parameter: Position Value
#define CAN_COMMAND_SERVO_1_SET_SPEED 	           	   305 //Parameter: Speed Value
#define CAN_COMMAND_SERVO_2_SET_SPEED 	           	   306 //Parameter: Speed Value
#define CAN_COMMAND_SERVO_3_SET_SPEED 	           	   307 //Parameter: Speed Value
#define CAN_COMMAND_LIFT_SET_DOWN_POSITION             308 //Parameter: Lift Down Position (writes to EEPROM)
#define CAN_COMMAND_LIFT_REQUEST_DOWN_POSITION         309 //Parameter: Lift Down Position (reads from EEPROM)
#define CAN_COMMAND_LIFT_ANSWER_DOWN_POSITION  		   310 //Parameter: Lift Down Position
#define CAN_COMMAND_LIFT_GOTO_DOWN_POSITION    		   311 //Parameter: none
#define CAN_COMMAND_LIFT_SET_BARCODE_POSITION          312 //Parameter: Lift Barcode Position (writes to EEPROM)
#define CAN_COMMAND_LIFT_REQUEST_BARCODE_POSITION      313 //Parameter: Lift Barcode Position (reads from EEPROM)
#define CAN_COMMAND_LIFT_ANSWER_BARCODE_POSITION  	   314 //Parameter: Lift Barcode Position
#define CAN_COMMAND_LIFT_GOTO_BARCODE_POSITION    	   315 //Parameter: none
#define CAN_COMMAND_WRITE_EEPROM_INTEGER      	       316 //Parameter: none
#define CAN_COMMAND_READ_EEPROM_INTEGER      	       317 //Parameter: none
#define CAN_COMMAND_ANSWER_READ_EEPROM_INTEGER         318 //Parameter: none
#define CAN_COMMAND_WRITE_EEPROM_BYTE      	           319 //Parameter: none
#define CAN_COMMAND_READ_EEPROM_BYTE      	           320 //Parameter: none
#define CAN_COMMAND_ANSWER_READ_EEPROM_BYTE            321 //Parameter: none
#define CAN_COMMAND_WEIGHCELL_SET_FORCE_LIMIT          322 //Parameter: limit pressure in mbar
#define CAN_COMMAND_WEIGHCELL_SET_FORCE_CALIB_W        323 //Parameter: ADC value equal to 1000 N
#define CAN_COMMAND_WEIGHCELL_SET_FORCE_CALIB_I        324 //Parameter: ADC value equal to 1000 N
#define CAN_COMMAND_WEIGHCELL_RESET			           325 //Parameter:none
#define CAN_COMMAND_WEIGHCELL_REQUEST_FORCE            326 //Parameter:none
#define CAN_COMMAND_WEIGHCELL_ANSWER_FORCE             327 //Parameter: actual Force in 1/10 mN
#define CAN_COMMAND_INJECTOR_SET_BACKLASH              328 //Parameter: Injector Backlash (writes to EEPROM)
#define CAN_COMMAND_INJECTOR_REQUEST_BACKLASH          329 //Parameter: none (reads from EEPROM)
#define CAN_COMMAND_INJECTOR_ANSWER_BACKLASH  		   330 //Parameter: Injector Backlash (from EEPROM)
#define CAN_COMMAND_RFID_READ_TAG  		               331 //Parameter: none
#define CAN_COMMAND_RFID_REQUEST_FIELD_DATA  		   332 //Parameter: LSB: sign (0 ... )
#define CAN_COMMAND_RFID_ANSWER_FIELD_DATA  		   333 //Parameter: 4 byte data, 1byte Sign(0 ... ), 1 byte adr
#define CAN_COMMAND_RFID_CLEAR_FIELD_DATA  		       334 //Parameter: none
#define CAN_COMMAND_RFID_SET_FIELD_DATA  		   	   335 //Parameter: 4 byte data, 1byte Sign(0 ... ), 1 byte adr
#define CAN_COMMAND_RFID_UPDATE_TAG  		           336 //Parameter: none
#define CAN_COMMAND_RFID_REQUEST_STATE  		       337 //Parameter: none
#define CAN_COMMAND_RFID_ANSWER_STATE  		           338 //Parameter: Error (16 bit)(MSB), Handler (8 bit), Mode (8 bit)(LSB)
#define CAN_COMMAND_FILTER_REQUEST_STATE 	       	   339 //Parameter: position (int32_t)
#define CAN_COMMAND_FILTER_ANSWER_STATE  	       	   340 //Parameter: 16bit MSB: error, 3.Byte: Filterposition, 4.Byte: LSB: finished (position ok)
#define CAN_COMMAND_THERM_WRITE_RAM_MODEL_DATA		   341 //Parameter: data (32bit), parext: index (RAM Address)
#define CAN_COMMAND_THERM_SET_TEMP_TOLERANCE		   342 //Parameter: temperature Tolerance in 1/1000°C
#define CAN_COMMAND_THERM_SET_OVERSTOOT		   		   343 //Parameter: overshoot
#define CAN_COMMAND_THERM_SET_UNDERSHOOT		   	   344 //Parameter: undershoot
#define CAN_COMMAND_THERM_SET_BALANCE_TIME		   	   345 //Parameter: balance time
#define CAN_COMMAND_THERM_SET_COMP_RELEASE		   	   346 //Parameter: comp release
#define CAN_COMMAND_THERM_SET_REGULATION_MODEL		   347 //Parameter: regulation model
#define CAN_COMMAND_SOLENOID_RESET_SEQUENCES	       348 //Parameter: none
#define CAN_COMMAND_SOLENOID_SET_SEQUENCE_A	       	   349 //Parameter: 1.byte MSB: output, 24 bit LSB: time in 100us, ext.Param: index (0...n)
#define CAN_COMMAND_SOLENOID_SET_SEQUENCE_B	       	   350 //Parameter: 1.byte MSB: output, 24 bit LSB: time in 100us, ext.Param: index (0...n)
#define CAN_COMMAND_SOLENOID_START	       	           351 //Parameter: 16 Byte MSB: Repeats A, 16 Byte LSB: Repeats B ext.Param: Repeats of total Sequences
#define CAN_COMMAND_SOLENOID_STOP					   352 //Parameter: none
#define CAN_COMMAND_SOLENOID_REQUEST_STATE 	       	   353 //Parameter: none
#define CAN_COMMAND_SOLENOID_ANSWER_STATE 	       	   354 //Parameter: MSByte: error; 2nd Byte: active; 3rd Byte: power good; LSByte: power on
#define CAN_COMMAND_LYSIS_SET_TEMPERATURE	       	   355 //Parameter: Temperature in 1/1000°C
#define CAN_COMMAND_LYSIS_SET_VOLUME	       	   	   356 //Parameter: Volume in ul
#define CAN_COMMAND_LYSIS_SET_ENVIRONMENT      	   	   357 //Parameter: Environmental temperature in 1/1000°C
#define CAN_COMMAND_LYSIS_START      	   	   		   358 //Parameter: Time in 1/10s extPar: Type
#define CAN_COMMAND_LYSIS_STOP      	   	   		   359 //Parameter: none
#define CAN_COMMAND_LYSIS_REQUEST_STATE 	       	   360 //Parameter: none
#define CAN_COMMAND_LYSIS_ANSWER_STATE 	       	       361 //Parameter: MSByte: error; 2nd Byte: active; 3rd Byte: heater; LSByte: phase
#define CAN_COMMAND_LYSIS_REQUEST_TEMPERATURE 	       362 //Parameter: none
#define CAN_COMMAND_LYSIS_ANSWER_TEMPERATURE 	       363 //Parameter: actual Temperature in 1/1000°C
#define CAN_COMMAND_LYSIS_REQUEST_TIME 	       	       364 //Parameter: none
#define CAN_COMMAND_LYSIS_ANSWER_TIME 	       	       365 //Parameter: phase time in 1/10s
#define CAN_COMMAND_LYSIS_REQUEST_REACHED_TIME 	       366 //Parameter: none
#define CAN_COMMAND_LYSIS_ANSWER_REACHED_TIME 	       367 //Parameter: reached time in 1/10s
#define CAN_COMMAND_REQUEST_DRAWER_SN                  368 //Parameter: none (BlinkX)
#define CAN_COMMAND_ANSWER_DRAWER_SN                   369 //Parameter: Drawer-ID (BlinkX)
#define CAN_COMMAND_REQUEST_DRAWER_TYP                 370 //Parameter: none (BlinkX)
#define CAN_COMMAND_ANSWER_DRAWER_TYP                  371 //Parameter: Drawer-ID (BlinkX)
#define CAN_COMMAND_RESET_SYSTEM_TIME                  372 //Parameter: none
#define CAN_COMMAND_THERM_WRITE_CALIBRATION_DATA	   373 //Parameter: data (32bit), parext: index (EEPROM ADDRESS count [0...n])
#define CAN_COMMAND_THERM_REQUEST_NTC3_TEMPERATURE 	   374 //Parameter: none
#define CAN_COMMAND_THERM_ANSWER_NTC3_TEMPERATURE  	   375 //Parameter: NTC3-Temperature
#define CAN_COMMAND_REQUEST_VIRTUAL_TEMPERATURE  	   376 //Parameter: none
#define CAN_COMMAND_ANSWER_VIRTUAL_TEMPERATURE   	   377	//Parameter: pacemaker virtual Temperature in 1/1000°C
#define CAN_COMMAND_REQUEST_ESTIMATED_TEMPERATURE  	   378 //Parameter: none
#define CAN_COMMAND_ANSWER_ESTIMATED_TEMPERATURE   	   379	//Parameter: pacemaker virtual Temperature in 1/1000°C
#define CAN_COMMAND_DC_MOTOR_START                    	380 //Parameter: Direction (1, -1) // parext: Axis Index
#define CAN_COMMAND_DC_MOTOR_SET_SPEED                 	381 //Parameter: Speed from 1...100 // parext: Axis Index
#define CAN_COMMAND_DC_MOTOR_SET_PWM                	382 //Parameter: bit 31: store in EEPROM, Bit 7...0: PWM from 1...100 // parext: Axis Index
#define CAN_COMMAND_DC_MOTOR_DO_REFERENCE_DRIVE       	383 //Parameter: none // parext: Axis Index
#define CAN_COMMAND_DC_MOTOR_GOTO_ENCODER_POSITION    	384 //Parameter: Absolute Encoder Position (signed) // parext: Axis Index
#define CAN_COMMAND_DC_MOTOR_GOTO_BEGIN			    	385 //Parameter: PMW (LSB) // parext: Axis Index
#define CAN_COMMAND_DC_MOTOR_GOTO_MID			    	386 //Parameter: PMW (LSB) // parext: Axis Index
#define CAN_COMMAND_DC_MOTOR_GOTO_END			    	387 //Parameter: PMW (LSB) // parext: Axis Index
#define CAN_COMMAND_DC_MOTOR_OPEN			    	    388 //Parameter: PMW (LSB) // parext: Axis Index
#define CAN_COMMAND_DC_MOTOR_CLOSE			    	    389 //Parameter: PMW (LSB) // parext: Axis Index
#define CAN_COMMAND_DC_MOTOR_STOP                     	390 //Parameter: none // parext: Axis Index
#define CAN_COMMAND_REQUEST_DC_MOTOR_STATE  	        391 //Parameter: none // parext: Axis Index
#define CAN_COMMAND_ANSWER_DC_MOTOR_STATE   	        392 //Parameter: Bit 31: finished, Bits 27..24: endpos, Bits 23...8: error, Bits 7...0: reference // parext: Axis Index
#define CAN_COMMAND_REQUEST_DC_MOTOR_SPEED            	393 //Parameter: none // parext: Axis Index
#define CAN_COMMAND_ANSWER_DC_MOTOR_SPEED             	394 //Parameter: actual encoder position // parext: Axis Index
#define CAN_COMMAND_REQUEST_DC_MOTOR_ENCODER_POSITION   395 //Parameter: none // parext: Axis Index
#define CAN_COMMAND_ANSWER_DC_MOTOR_ENCODER_POSITION    396 //Parameter: actual encoder position // parext: Axis Index
#define CAN_COMMAND_REQUEST_DC_MOTOR_CURRENT            397 //Parameter: none // parext: Axis Index
#define CAN_COMMAND_ANSWER_DC_MOTOR_CURRENT             398 //Parameter: actual current in mA // parext: Axis Index
#define CAN_COMMAND_SOLENOID_REQUEST_CURRENT 	       	399 //Parameter: none
#define CAN_COMMAND_SOLENOID_ANSWER_CURRENT 	       	400 //Parameter: 1st byte reserve, 2nd byte reserve, bit 15...0: current in mA
#define CAN_COMMAND_CARTRIDGE_REQUEST_STATE 	       	401 //Parameter: none
#define CAN_COMMAND_CARTRIDGE_ANSWER_STATE 	       		402 //Parameter: Bit 31: finished, Bits 27..26:LB Reference, Bits 25..24: LB Position,  finished Bits 23..16: overshoot, Bits 15..8: maxpos, Bits 7..0: position,
#define CAN_COMMAND_CARTRIDGE_REQUEST_ERROR 	       	403 //Parameter: none
#define CAN_COMMAND_CARTRIDGE_ANSWER_ERROR 	       	    404 //Parameter: 1st-3rd byte reserve, bit 8...0: error
#define CAN_COMMAND_ANSWER_SERVO_CYCLE_STATE 	        405 //Parameter: Move-State (Bits 31..24), cyclecount (Bits 15..0) // parext: Axis Index
#define CAN_COMMAND_REQUEST_SERVO_CYCLE_STATE 	        406 //Parameter: none
#define CAN_COMMAND_SOLENOID_SWITCH_POWER		        407 //Parameter: 1 - 32P on, 0 - 32P off
#define CAN_COMMAND_SERVO_SET_CYCLE_POS		        	408 //Parameter: Bit 23...16: startpos, Bit15 ... 0: endpos
#define CAN_COMMAND_SERVO_START_CYCLE		        	409 //Parameter: Bit 23...16: speed, Bit15 ... 0: Cycles
#define CAN_COMMAND_SERVO_STOP		        			410 //Parameter: 0 - normal Stop, 1 - soft stop
#define CAN_COMMAND_CARTRIDGE_REQUEST_CLAMPSTATE 	    411 //Parameter: none
#define CAN_COMMAND_CARTRIDGE_ANSWER_CLAMPSTATE 	    412 //Parameter: Bits 31: Cartridge Detect Bits 30...16: Clamp-Error, Bits 15..0: Clampstate,
#define CAN_COMMAND_CARTRIDGE_CLAMP 	    			413 //Parameter: none
#define CAN_COMMAND_CARTRIDGE_RELEASE 	    			414 //Parameter: none
#define CAN_COMMAND_MECH_INIT 	    					415 //Parameter: none
#define CAN_COMMAND_MECH_REQUEST_INITSTATE 	    		416 //Parameter: none
#define CAN_COMMAND_MECH_ANSWER_INITSTATE 	        	417 //Parameter: Bits 31..24: reserve,  Bits 23...16: Init-Error, Bits 15..0: Initstate,
#define CAN_COMMAND_MECH_REQUEST_LBSTATE 	    		418 //Parameter: none
#define CAN_COMMAND_MECH_ANSWER_LBSTATE 	        	419 //Parameter: Bits 31..0: Lightbarrier state
#define CAN_COMMAND_DC_MOTOR_GOTO_CARTRIDGE_POSITION   	420 //Parameter: Bits 31..16: reserve, Bits 15..8: mode (FW,RW,shortest) Bits 7..0: position// parext: Axis Index
#define CAN_COMMAND_RFID_REQUEST_UID					421 //Parameter: none
#define CAN_COMMAND_RFID_ANSWER_UID					    422 //Parameter: Bits 31..0 UID-Part // parext: UID-Part Index (0 - LSB; 1 - MSB;)
#define CAN_COMMAND_THERM_REQUEST_REGULATION_MODEL		423 //Parameter: none
#define CAN_COMMAND_THERM_ANSWER_REGULATION_MODEL		424 //Parameter: Checksum; parext: regulation model
#define CAN_COMMAND_REQUEST_HARDWARE_OPTIONS			425 //Parameter: none
#define CAN_COMMAND_ANSWER_HARDWARE_OPTIONS				426 //Parameter: Bits 31..0: hardware options
#define CAN_COMMAND_REQUEST_FIRMWARE_VERSION			427 //Parameter: none
#define CAN_COMMAND_ANSWER_FIRMWARE_VERSION				428 //Parameter: 1st-4th Byte: ASCII-Bytes; parext: index of telegram (0..n)
#define CAN_COMMAND_REQUEST_FIRMWARE_DATE				429 //Parameter: none
#define CAN_COMMAND_ANSWER_FIRMWARE_DATE				430 //Parameter: 1st-4th Byte: ASCII-Bytes; parext: index of telegram (0..n)
#define CAN_COMMAND_SCRIPT_EDIT_PAR_6         	        431 //Parameter: par6
#define CAN_COMMAND_FILTER_5_SET_CALIBRATION 	        432 //Parameter: Position Value
#define CAN_COMMAND_DRAWER_EJECT					    433 //Parameter: speed
#define CAN_COMMAND_DRAWER_INSERT					    434 //Parameter: speed
#define CAN_COMMAND_REQUEST_EXPOSURE_CURRENT  	        435 //Parameter: none //
#define CAN_COMMAND_ANSWER_EXPOSURE_CURRENT   	        436 //Parameter: 16bit MSB: LED voltage [mV], 16bit LSB: LED current [mA]
#define CAN_COMMAND_REQUEST_EXPOSURE_BRIGHTNESS  	    437 //Parameter: none //
#define CAN_COMMAND_ANSWER_EXPOSURE_BRIGHTNESS   	    438 //Parameter: 16bit MSB: Sensor Temperature [1/10°C], 16bit LSB: Brightness Value
#define CAN_COMMAND_FILTER_REQUEST_CALIBRATION 	        439 //Parameter: none; parext: index of Filter (0...4)
#define CAN_COMMAND_FILTER_ANSWER_CALIBRATION 	        440 //Parameter: Position Value; parext: index of Filter (0...4)
#define CAN_COMMAND_FILTER_SET_MIN_POSITION 	        441 //Parameter: Position Value
#define CAN_COMMAND_FILTER_REQUEST_MIN_POSITION 	    442 //Parameter: none
#define CAN_COMMAND_FILTER_ANSWER_MIN_POSITION 	        443 //Parameter: Position Value
#define CAN_COMMAND_FILTER_SET_MAX_POSITION 	        444 //Parameter: Position Value
#define CAN_COMMAND_FILTER_REQUEST_MAX_POSITION 	    445 //Parameter: none
#define CAN_COMMAND_FILTER_ANSWER_MAX_POSITION 	        446 //Parameter: Position Value
#define CAN_COMMAND_FILTER_SET_DATE 	        		447 //Parameter: Date
#define CAN_COMMAND_FILTER_REQUEST_DATE 	    		448 //Parameter: none
#define CAN_COMMAND_FILTER_ANSWER_DATE 	        		449 //Parameter: Date
#define CAN_COMMAND_SET_PIXEL_RESOLUTION 	        	450 //Parameter: Pixel Resolution
#define CAN_COMMAND_REQUEST_PIXEL_RESOLUTION 	    	451 //Parameter: none
#define CAN_COMMAND_ANSWER_PIXEL_RESOLUTION 	        452 //Parameter: Pixel Resolution
#define CAN_COMMAND_STEPPER_SET_LOCK_POSITION 	        453 //Parameter: position; parext: Axis (0...3)
#define CAN_COMMAND_STEPPER_REQUEST_LOCK_POSITION 	    454 //Parameter: none parext: Axis (0...3)
#define CAN_COMMAND_STEPPER_ANSWER_LOCK_POSITION 	    455 //Parameter: position; parext: Axis (0...3)
#define CAN_COMMAND_STEPPER_GOTO_LOCK_POSITION 	        456 //Parameter: none parext: Axis (0...3)
#define CAN_COMMAND_REQUEST_OPTIC_TYPE 	    			457 //Parameter: none
#define CAN_COMMAND_ANSWER_OPTIC_TYPE 	        		458 //Parameter: Optic Type
#define CAN_COMMAND_REQUEST_OPTIC_SERIAL 	    		459 //Parameter: none
#define CAN_COMMAND_ANSWER_OPTIC_SERIAL 	        	460 //Parameter: Optic Serial


/* for initialization */
#define CAN_GPIO_PORT				GPIOA /* GPIOB */
#define CAN_GPIO_RCC				RCC_AHB1Periph_GPIOA /* RCC_AHB1Periph_GPIOB */
#define CAN_RX_PIN					GPIO_Pin_11 /* GPIO_Pin_8 */
#define CAN_RX_PIN_SOURCE			GPIO_PinSource11 /* GPIO_PinSource8 */
#define CAN_TX_PIN					GPIO_Pin_12 /* GPIO_Pin_9 */
#define CAN_TX_PIN_SOURCE			GPIO_PinSource12 /* GPIO_PinSource9 */

#define CAN_INTERFACE				CAN1 /* CAN2 */


extern TypeCanStepperInfo CanStepperInfo[NUMBER_OF_CAN_IDS][MAX_STEPPER_AXIS];
extern TypeStepmotorCanCommand StepmotorCanCommand[MAX_STEPPER_AXIS];

extern TypeCanDcMotorInfo CanDcMotorInfo[NUMBER_OF_CAN_IDS][MAX_DC_MOTOR_AXIS];
extern TypeDcMotorCanCommand DcMotorCanCommand[MAX_DC_MOTOR_AXIS];
extern TypeExposureCanCommand ExposureCanCommand[MAX_EXPOSURE_CHANNELS];
extern TypeLysisCanCommand LysisCanCommand[MAX_LYSIS_PORTS];
extern TypeCanLysisInfo CanLysisInfo[MAX_LYSIS_PORTS];
extern TypeCanDosageInfo CanDosageInfo;
extern TypeCanServomotorInfo CanServomotorInfo[NUMBER_OF_CAN_IDS][MAX_SERVOMOTOR_AXIS];
extern TypeServomotorCanCommand ServomotorCanCommand[MAX_SERVOMOTOR_AXIS];
extern TypeCanSensorInfo CanSensorInfo[NUMBER_OF_CAN_IDS][MAX_SENSORS];
extern TypeSensorCanCommand SensorCanCommand[MAX_SENSORS];

extern TypeCanBrushlessInfo CanBrushlessInfo;
extern TypeCanThermInfo CanThermInfo;
extern TypeCanXYTableInfo CanXYTableInfo;
extern TypeCanBloodsensorInfo CanBloodsensorInfo;
extern TypeCanWeighcellInfo CanWeighcellInfo;
extern TypeCanEnvironmentInfo CanEnvironmentInfo[NUMBER_OF_CAN_IDS];
extern TypeCanRfidInfo CanRfidInfo;
extern TypeCanFilterInfo CanFilterInfo;
extern TypeCanExposureInfo CanExposureInfo;

extern uint8_t CanAckReceived[NUMBER_OF_CAN_IDS];


//functions:
void CanInit(uint8_t addr);
void CanSend(uint16_t id, uint16_t cmd, uint32_t data, uint16_t dataext);
/* Private usage: */
void CanReceive(CAN_TypeDef* CANx, uint8_t FIFONumber, CanRxMsg* pmsg);
void CanDecode(CanRxMsg *msg);
/* - - - */
void CanRxWork(void);
void CanTxWork(void);
uint8_t CanReadError(void);
void CanAppendMessageToTxQueue(uint16_t id, uint16_t cmd, uint32_t data, uint16_t dataext, uint8_t getack);
void CanSendMessageFromTxQueue(void);
void CanResetInfoStructures(void);




#endif

