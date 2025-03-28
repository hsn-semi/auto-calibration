/******************************************************/
/***  Project:       AraController                  ***/
/***  File:          i2c.h                          ***/
/***  Created on:    22.07.2016                     ***/
/***  Author:        Andreas Stärker                ***/
/***  Company:       Blink AG                       ***/
/***  Changes:                                      ***/
/******************************************************/

#ifndef I2C_H
#define I2C_H

#include "defines.h"


//extern unsigned char I2CA;

#define LTC2943_STATUS_REGISTER                  0x00
#define LTC2943_CONTROL_REGISTER                 0x01
#define LTC2943_ACCUMULATED_CHARGE_REGISTER      0x02
#define LTC2943_VOLTAGE_REGISTER                 0x08
#define LTC2943_CURRENT_REGISTER                 0x0E
#define LTC2943_TEMPERATURE_REGISTER             0x14

#define INA236_CONFIG_REGISTER					0x00
#define INA236_SHUNT_VOLTAGE_REGISTER			0x01
#define INA236_BUS_VOLTAGE_REGISTER				0x02
#define INA236_POWER_REGISTER					0x03
#define INA236_CURRENT_REGISTER					0x04
#define INA236_CALIBRATION_REGISTER				0x05
#define INA236_MASK_REGISTER					0x06
#define INA236_ALERT_REGISTER					0x07
#define INA236_MANUFACTURER_ID_REGISTER			0x3E
#define INA236_DEVICE_ID_REGISTER				0x3F

/* INA 236 Config defines */
/* Range */
#define INA236_RANGE_81 						0x00 //+-81.92mV - default
#define INA236_RANGE_20 						0x01 //+-20.48mV
/* Average */
#define INA236_AVERAGE_1 						0x00 //default
#define INA236_AVERAGE_4 						0x01
#define INA236_AVERAGE_16 						0x02
#define INA236_AVERAGE_64 						0x03
#define INA236_AVERAGE_128 						0x04
#define INA236_AVERAGE_256 						0x05
#define INA236_AVERAGE_512 						0x06
#define INA236_AVERAGE_1024						0x07
/* Conversion Times (Bus and Shunt) */
#define INA236_TIME_140 						0x00 //140 us
#define INA236_TIME_204 						0x01 //204us
#define INA236_TIME_332 						0x02 //332us
#define INA236_TIME_588 						0x03 //588us
#define INA236_TIME_1100 						0x04 // 1100 us - default
#define INA236_TIME_2116 						0x05 //2116us
#define INA236_TIME_4156 						0x06 //4156us
#define INA236_TIME_8244						0x07 //8244us
/* Modes */
#define INA236_MODE_SHUTDOWN 							0x00
#define INA236_MODE_SHUNT_VOLTAGE_SINGLE 				0x01
#define INA236_MODE_BUS_VOLTAGE_SINGLE 					0x02
#define INA236_MODE_BUS_AND_SHUNT_VOLTAGE_SINGLE 		0x03
#define INA236_MODE_SHUTDOWN_ 							0x04
#define INA236_MODE_SHUNT_VOLTAGE_CONTINOUS 			0x05
#define INA236_MODE_BUS_VOLTAGE_CONTINOUS 				0x06
#define INA236_MODE_BUS_AND_SHUNT_VOLTAGE_CONTINOUS 	0x07 //default


#define L3GD20H_CTRL1_REGISTER             		0x20
#define L3GD20H_CTRL2_REGISTER             		0x21
#define L3GD20H_CTRL3_REGISTER             		0x22
#define L3GD20H_CTRL4_REGISTER             		0x23
#define L3GD20H_CTRL5_REGISTER             		0x24
#define L3GD20H_X_REGISTER             			0x28
#define L3GD20H_Y_REGISTER             			0x2A
#define L3GD20H_Z_REGISTER             			0x2C
#define L3GD20H_FIFO_CTRL_REGISTER             	0x2E
#define L3GD20H_FIFO_SCR_REGISTER             	0x2F
#define L3GD20H_IG_CFG_REGISTER             	0x30
#define L3GD20H_IG_SRC_REGISTER             	0x31

#define BMP280_1ST_CAL_REGISTER             	0x88 //length: 26 Bytes
#define BMP280_1ST_DATA_REGISTER           		0xF7 //length: 6 Bytes
#define BMP280_CONFIG_REGISTER_F5             	0xF5
#define BMP280_CONFIG_REGISTER_F4				0xF4

#define HTU21_MEASURE_TEMPERATURE_HOLD       0xE3 //master will be blocked during measurement
#define HTU21_MEASURE_HUMIDITY_HOLD          0xE5 //master will be blocked during measurement
#define HTU21_MEASURE_TEMPERATURE            0xF3 //normal mode
#define HTU21_MEASURE_HUMIDITY               0xF5 //normal mode

#define LM3509_GENERAL_REGISTER             	 0x10
#define LM3509_GPIO_REGISTER             		 0x80
#define LM3509_MAIN_BRIGHTNESS_REGISTER          0xA0
#define LM3509_SUB_BRIGHTNESS_REGISTER           0xB0




#define I2C_NO_ERROR                             0x00
#define I2C_ERROR_BUSY                           0x01
#define I2C_ERROR_MASTER_MODE_SELECT             0x02
#define I2C_ERROR_MASTER_RX_MODE_SELECT          0x03
#define I2C_ERROR_MASTER_TX_MODE_SELECT          0x04
#define I2C_ERROR_MASTER_BYTE_TRANSMIT           0x05
#define I2C_ERROR_MASTER_BYTE_RECEIVE            0x06


//defines:
#define I2C_ADDRESS_TMP275    		0x92 //= 146 (b10010010 A2..A1 = 0; A0 = 1)
#define I2C_ADDRESS_24LC256   		0xA0 //= 160 (b10100000 A0..A2 = 0)
#define I2C_ADDRESS_24LC256_EXT		0xA8 //= 168 (b10101000 A2 = 1, A1..A0 = 0)
#define I2C_ADDRESS_LTC2943   		0xC8 //= 200 (b11001000)
#define I2C_ADDRESS_LTC2451         0x28 //= 40
#define I2C_ADDRESS_LTC2471         0x28 //= 40
#define I2C_ADDRESS_LTC2481         0x2E //= 46
#define I2C_ADDRESS_L3GD20H         0xD4 //= 212
#define I2C_ADDRESS_BMP280          0xEC //= 236
#define I2C_ADDRESS_HTU21           0x80 //= 128
#define I2C_ADDRESS_LM3509          0x6C //= 108
#define I2C_ADDRESS_INA236B         0x90 //= 144
#define I2C_ADDRESS_GLOBAL          0xEE //= 238

#define I2C_TIMEOUT          1000 // 100000 //

//functions:
void I2cInit(I2C_TypeDef* I2Cx);
/* Private usage: */
uint8_t I2cStart(I2C_TypeDef* I2Cx, uint8_t address, uint8_t direction);
uint8_t I2cRestart(I2C_TypeDef * I2Cx, uint8_t address, uint8_t direction);
uint8_t I2cWrite(I2C_TypeDef* I2Cx, uint8_t data);
uint8_t I2cReadAck(I2C_TypeDef* I2Cx);
uint8_t I2cReadNack(I2C_TypeDef* I2Cx);
uint8_t I2cStop(I2C_TypeDef* I2Cx);
/* Public Usage */
uint8_t IsI2cAddressPresent(I2C_TypeDef* I2Cx, uint8_t adr);
void I2cTMP275Config(uint8_t resolution);
void I2cTMP275SetTemperatureRegister(void);
int32_t I2cReadTMP275(void);
uint8_t I2cReadEepromByte(uint16_t address);
uint8_t I2cReadEepromExtByte(uint16_t address);
int32_t I2cReadEepromInteger(uint16_t address);
int32_t I2cReadEepromExtInteger(uint16_t address);
void I2cReadEepromAscii(uint16_t address, uint8_t len, uint8_t * buf);
void I2cReadEepromExtAscii(uint16_t address, uint8_t len, uint8_t * buf);
void I2cWriteEepromByte(uint16_t address, uint8_t data);
void I2cWriteEepromExtByte(uint16_t address, uint8_t data);
void I2cWriteEepromInteger(uint16_t address, int32_t data);
void I2cWriteEepromExtInteger(uint16_t address, int32_t data);
void I2cWriteEepromAscii(uint16_t address, uint8_t len, uint8_t * buf);
void I2cWriteEepromExtAscii(uint16_t address, uint8_t len, uint8_t * buf);
uint8_t I2cWriteLTC2943(uint8_t reg, int8_t data);
void I2cWriteChargeRegisterLTC2943(uint16_t dat);
uint16_t I2cReadWordLTC2943(uint8_t reg);
uint8_t I2cReadStatusLTC2943(void);
uint16_t I2cReadLTC2451(void);
void I2cSetModeLTC2481(uint8_t gain, uint8_t tempr, uint8_t rej, uint8_t speed);
uint32_t I2cReadLTC2481(void);
uint8_t IsI2cLTC2481Present(void);
uint8_t IsI2cLTC2943Present(void);
void I2cWriteRegisterL3GD20H(uint8_t reg, uint8_t dat);
uint16_t I2cReadWordL3GD20H(uint8_t reg);
void I2cReadMultipleBytesL3GD20H(uint8_t reg, uint8_t * buf, uint8_t num);
void I2cWriteRegisterBMP280(uint8_t reg, uint8_t dat);
void I2cReadMultipleBytesBMP280(uint8_t reg, uint8_t * buf, uint8_t num);
void I2cTriggerTemperatureMeasurementHTU21(void);
void I2cTriggerHumidityMeasurementHTU21(void);
void I2cReadHTU21(uint8_t * buf);
uint8_t IsI2cLM35091Present(void);
void I2cLM35091SwitchMain(uint8_t state);
void I2cLM35091SetMainBrightness(uint8_t brightness);
void I2cINA236SetConfig(uint8_t range, uint8_t average, uint8_t bustime, uint8_t shunttime, uint8_t mode);
void I2cINA236SetRegisterPointer(uint8_t reg);
int16_t I2cReadINA236(void);
uint16_t I2cReadLTC2471(void);
void I2cAddJob(uint8_t job, uint32_t par1, uint32_t par2, uint32_t par3);
void WorkI2c(void);






#endif

