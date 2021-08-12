/*
 * BME280.h
 *
 *  Created on: Jun 2, 2021
 *      Author: CarlosPach
 */

#ifndef INC_BME280_H_
#define INC_BME280_H_

#include "main.h"

// register addresses
#define BME280_ADDR				0x76

#define BME_ID_ADDR				0xD0
#define BME_RESET_ADDR			0xE0
#define BME_HUMID_CTRL_ADDR		0xF2
#define BME_STATUS_ADDR			0xF3
#define BME_MEAS_CTRL_ADDR		0xF4
#define BME_CONFIG_ADDR			0xF5
#define PRESSURE_ADDR			0xF7
#define TEMP_ADDR				0xFA
#define HUMID_ADDR				0xFD

// compensation addresses
#define DIG_T1_ADDR				0x88
#define DIG_P1_ADDR				0x8E
#define DIG_H1_ADDR				0xA1
#define DIG_H2_ADDR				0xE1

// write register address
#define ID_MASK					0x60
#define RESET_MASK				0xB6
#define STATUS_MASK				0x09

// misc. defines
#define MAX_TEMP_ARR_SIZE		6			// size of temperature comp array
#define MAX_PRESSURE_ARR_SIZE	18			// size of pressure comp array
#define MAX_HUMID_ARR_SIZE		8			// size of humidity comp array
#define MAX_COMP_ARR			18			// max size of compensation data array
#define MAX_ARR_SIZE			3			// used to gather sensor data in array
#define MIN_PRESSURE			30000UL		// minimum compensated pressure value
#define MAX_PRESSURE			110000UL	// maximum compensated pressure value
#define MIN_TEMP				-40			// minimum temperature value (in Celcius)
#define MAX_TEMP				85			// maximum temperature value (in Celcius)
#define MIN_HUMID				0UL			// minimum humidity value
#define MAX_HUMID				100UL		// maximum humidity value

// macro function
#define BME280_CONCAT(MSB, LSB)			((uint16_t)MSB << 8) | ((uint16_t)LSB)



struct digits{	// digit register addrs for BME280
	uint16_t dig_T1, dig_P1 ;	// T1 and P1 head
	int16_t dig_T2, dig_T3, dig_P2, dig_P3,
			dig_P4, dig_P5, dig_P6, dig_P7,
			dig_P8, dig_P9, dig_H2, dig_H4, dig_H5 ;
	uint8_t dig_H1, dig_H3 ;	// H1 and H3 head
	int8_t dig_H6 ;
} ;

struct weatherSensor {
	struct Peripheral periph ;			// contains generic peripheral members
	struct digits digit ;				// contains data from BME280 registers
	volatile uint8_t pressureAddr, temperatureAddr, humidityAddr ;	// addresses for BME280
	volatile uint32_t pressureData, temperatureData, humidityData ;	// raw data from sensors
	volatile double compPressure, compTemp, compHumid ;	// finalized data values
} ;



typedef enum{ timeStandby_5, timeStandby_62, timeStandby_125, timeStandby_250,
			  timeStandby_500, timeStandby_1000, timeStandby_10, timeStandby_20
			} tStandbyTime ;	// measured in [ms]

typedef enum{ filterOff, filterOn_2, filterOn_4,
			  filterOn_8, filterOn_16
			} tIIR ;

// over sampling rate works for both pressure and temperature
typedef enum{ overSample_0, overSample_1, overSample_2, overSample_4,
			  overSample_8, overSample_16
			} tSampling ;



typedef enum{ BME_SLEEP = 0x00,
			  BME_FAST_1 = 0x01,	// forced mode
			  BME_FAST_2 = 0x02,	// forced mode
			  BME_NORMAL = 0x03
} BME280_MODE ;


volatile int32_t t_fine ;	// fine temperature reading (MUST CALCULATE PRIOR TO HUMID AND PRESS)


Bool BME280_initParams(struct weatherSensor* ptr) ;
Bool BME280_reset(struct weatherSensor* ptr, I2C_HandleTypeDef* pI2C) ;
Bool BME280_init(struct weatherSensor* ptr, I2C_HandleTypeDef* pI2C) ;
Bool BME280_readID(struct weatherSensor* ptr, I2C_HandleTypeDef* pI2C) ;
Bool BME280_ctrlMeas(struct weatherSensor* ptr, I2C_HandleTypeDef* pI2C) ;
Bool BME280_ctrlHumid(struct weatherSensor* ptr, I2C_HandleTypeDef* pI2C) ;
Bool BME280_checkStatus(struct weatherSensor* ptr, I2C_HandleTypeDef* pI2C) ;
Bool BME280_writeReg(struct weatherSensor* ptr, I2C_HandleTypeDef* pI2C, uint8_t regAddr, uint8_t data) ;
Bool BME280_readReg(struct weatherSensor* ptr, I2C_HandleTypeDef* pI2C, uint8_t regAddr, uint8_t n) ;
Bool BME280_readMultipleRegs(struct weatherSensor* ptr, I2C_HandleTypeDef* pI2C, uint8_t regAddr, uint8_t* arr,
							uint8_t n) ;
// function prototypes with Bool moved to BME280.c
void BME280_readSensorData(struct weatherSensor* ptr, I2C_HandleTypeDef *pI2C, uint8_t sensorAddr) ;
void BME280_parseData(struct weatherSensor* ptr, uint8_t* arr, uint8_t sensorAddr) ;
double BME280_compensatePressure(struct weatherSensor* ptr) ;
double BME280_compensateTemperature(struct weatherSensor* ptr) ;
double BME280_compensateHumidity(struct weatherSensor* ptr) ;
void BME280_getCompensationVals(struct weatherSensor* ptr, I2C_HandleTypeDef* pI2C, uint8_t compAddr) ;
void BME280_sleep(void) ;

#endif /* INC_BME280_H_ */
