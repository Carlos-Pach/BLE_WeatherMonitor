/*
 * BME280.c
 *
 *  Created on: Jun 2, 2021
 *      Author: CarlosPach
 */


#include "main.h"			// includes typedef enum Bool
#include "BME280.h"

// function prototypes	(cannot be declared in BME280.h from lack of 'Bool')
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


/*
 *	===============================
 *	Function name: BME280_initParams
 *	Purpose: Initializes the configuration register with predetermined values
 *	Details: Initializes the configuration register with a standby time of
 *			 500 [ms], 4x filter coefficient, and I2C mode.
 *
 *	Params:
 *		Name			Details
 *		[In] *ptr		- pointer to weatherSensor struct
 *	Return value:
 *		N/A
 * 	===============================
*/
Bool BME280_initParams(struct weatherSensor* ptr){
	// settings: 500 [ms] --> 1000 [ms] --> 500 [ms], filterOn_4 --> filterOff,
	// tStandbyTime initStandby = timeStandby_1000 ;	// on standby for 1000 [ms]
	tStandbyTime initStandby = timeStandby_1000 ;
	tIIR initIIR = filterOff ;	// filter constant set to 4
	uint8_t configData = ((initStandby << 5) | (initIIR << 2)) | 0x00 ;

	//ptr->periph->registerData = configData ;	// assign registerData with configuration

	ptr->periph.registerData = configData ;

	return True ;
}



/*
 *	===============================
 *	Function name: BME280_reset
 *
 *	Purpose: Performs a soft reset on the BME280
 *	Details: Writes a reset code into the BME280's reset
 *			 MEM address. Equivalent to powering on the module.
 *
 *	Params:
 *		Name				Details
 *		[In] *ptr			- pointer to weatherSensors struct
 *		[In] *pI2C			- pointer to HAL I2C struct
 *	Return value:
 *		None
 *
 *	Notes:
 *		Complete function to allow direct writing via I2C here
 * 	===============================
*/
Bool BME280_reset(struct weatherSensor *ptr, I2C_HandleTypeDef *pI2C){
	ptr->periph.devAddr = (uint8_t)BME280_ADDR << 1 ;	// device address for BME280
	ptr->periph.registerData = RESET_MASK ;
	ptr->periph.devID = BME_ID_ADDR ;

	if(BME280_writeReg(ptr, pI2C, BME_RESET_ADDR, ptr->periph.registerData) != True){
		return False ;
	}

	// fill sensor register addresses with corresponding vals
	ptr->pressureAddr = PRESSURE_ADDR ;		// address for pressure sensor
	ptr->humidityAddr = HUMID_ADDR ;		// address for humidity sensor
	ptr->temperatureAddr = TEMP_ADDR ;		// address for temperature sensor

	// reset interpreted data from sensors
	ptr->temperatureData = 0 ;
	ptr->humidityData = 0 ;
	ptr->pressureData = 0 ;

	return True ;
}



/*
 *	===============================
 * 	Function name: BME280_init
 *
 * 	Purpose: Initializes the BME280 struct
 * 	Details: Checks if the weatherSensor struct has expected
 * 			 data. If it does not, function returns False
 *
 *	Params:
 *		Name			Details
 *		[In] *ptr		- pointer to weatherSensors struct
 *		[In] *pI2C		- pointer to HAL I2C struct
 *	Return value:
 *		True on success
 *	===============================
*/
Bool BME280_init(struct weatherSensor* ptr, I2C_HandleTypeDef* pI2C){
	// create array to fill initialization data
	uint8_t initData[2] = {BME_CONFIG_ADDR, ptr->periph.registerData} ;

	// check if struct has valid i2c address
	if(ptr->periph.devAddr != (BME280_ADDR << 1)){	// check for addr left shift
		return False ;
	}

	// check if struct has correct ID
	if(ptr->periph.devID != BME_ID_ADDR){
		return False ;
	}

	// write configurations to BME280
	if(BME280_writeReg(ptr, pI2C, initData[0], initData[1]) != True){
		return False ;
	}

	// TODO: add more checks
	return True ;
}



/*
 *	===============================
 *	Function name: BME280_readID
 *	Purpose: Reads ID of BME280
 *	Details: Accesses ID address register in order to verify BME280 ID
 *
 *	Params:
 *		NAME				Details
 *		[in] *ptr			- points to weatherSensor struct
 *		[in] *pI2C			- points to HAL I2C struct
 *	Return value:
 *		True on success
 *	===============================
*/
Bool BME280_readID(struct weatherSensor* ptr, I2C_HandleTypeDef* pI2C){
	// create var that will hold ID read from BME280 ID
	//static uint8_t readID ;	// get rid of var after finishing driver

	// read ID from BME280 and assign it to ID in struct
	if(BME280_readReg(ptr, pI2C, ptr->periph.devID, 1) != True){
		return False ;
	}

	// sanity check: is device ID 0x60? no ... pointing to wrong area in memory?
	if(ptr->periph.registerData != 0x60){
		return False ;
	}

	return True ;
}



/*
 *	===============================
 *	Function name: BME280_ctrlMeas
 *	Purpose: Initializes and writes control measurement commands
 *	Details: Sets values for sampling rate and mode selection for
 *			 temperature and pressure sensors. Is successfully written
 *			 after ctrlHumid is called first.
 *
 *	Params:
 *		NAME					Details
 *		[In] *ptr				- points to weatherSensor struct
 *		[in] *pI2C				- points to HAL I2C struct
 *
 *	Return Value:
 *		True on success
 *	===============================
*/
Bool BME280_ctrlMeas(struct weatherSensor* ptr, I2C_HandleTypeDef* pI2C){
	// create arr for control measurements
	static uint8_t ctrlData[2] ;

	// set to default control measurements
	tSampling pressureSampling, temperatureSampling ;
	// pressure oversampling
	pressureSampling = overSample_1 ;
	// temperature oversampling
	temperatureSampling = overSample_1 ;
	// select mode
	BME280_MODE mode = BME_NORMAL ;	// BME_NORMAL --> BME_FAST_1 (forced mode) --> BME_NORMAL

	// fill arr with data to send
	ctrlData[0] = BME_MEAS_CTRL_ADDR ;
	ctrlData[1] = (pressureSampling << 5) | (temperatureSampling << 2) | (mode) ;

	// set measurements to address
	if(BME280_writeReg(ptr, pI2C, BME_MEAS_CTRL_ADDR, ctrlData[1]) != True){
		return False ;
	}

	return True ;
}



/*
 *	===============================
 *	Function name: BME280_ctrlHumid
 *	Purpose: Configures and updates humidity sensor of BME280
 *	Details: Configures over-sampling rate for humidity sensor
 *			 in BME280.
 *
 *	Params:
 *		NAME				Details
 *		[In] *ptr			- pointer to weatherSensor struct
 *		[In] *pI2C			- pointer to HAL I2C struct
 *
 *	Return value:
 *		True on success
 *	===============================
*/
Bool BME280_ctrlHumid(struct weatherSensor* ptr, I2C_HandleTypeDef* pI2C){
	static uint8_t ctrlData[2] ;

	// set up sampling rate for humidity sensor
	tSampling humiditySampling = overSample_1 ;

	// fill array with address and data
	ctrlData[0] = BME_HUMID_CTRL_ADDR ;
	ctrlData[1] = humiditySampling | 0x00 ;

	// write to humidity control address
	if(BME280_writeReg(ptr, pI2C, BME_HUMID_CTRL_ADDR, ctrlData[1]) != True){
		return False ;
	}

	return True ;
}



/*
 *	===============================
 *	Function name: BME280_checkStatus
 *	Purpose: Checks current status of BME280
 *	Details: Checks status from BME280 if it is currently measuring data
 *			 or if it is updating data inside its registers.
 *
 *	Params:
 *		NAME				Details
 *		[In] *ptr			- pointer to weatherSensor struct
 *		[In] *pI2C			- pointer to HAL I2C struct
 *
 *	Return value:
 *		False for error when reading register or BME280 is still updating vals
 *		True on success
 *	===============================
*/
Bool BME280_checkStatus(struct weatherSensor* ptr, I2C_HandleTypeDef* pI2C){
	static uint8_t readStatus ;

	if(HAL_I2C_Mem_Read(pI2C, ptr->periph.devAddr, BME_STATUS_ADDR, 1, &(ptr->periph.registerData), 1, HAL_MAX_DELAY) != HAL_OK){
		return False ;
	}

	readStatus = ptr->periph.registerData ;

	// check if b3 and b0 are both 0
	if((readStatus & STATUS_MASK) != 0) {
		return False ;
	}

	return True ;	// not updating inside BME280
}



/*
 *	===============================
 *	Function name: BME280_writeReg
 *	Purpose: Writes to register addresses of BME280
 *	Details: Writes directly to addresses in BME280 via I2C.
 *			 Uses pointers to access struct members more easily.
 *
 *	Params:
 *		NAME				Details
 *		[In] *ptr			- pointer to weatherSensor struct
 *		[In] *pI2C			- pointer to HAL I2C struct
 *		[In] regAddr		- register address for BME280
 *		[In] data			- data that will be written to register address for BME280
 *
 *	Return value:
 *		True on success
 *	===============================
*/
Bool BME280_writeReg(struct weatherSensor* ptr, I2C_HandleTypeDef* pI2C, uint8_t regAddr, uint8_t data){
	// create array that writes data to BME280
	static uint8_t regData[2] ;

	regData[0] = regAddr ;
	regData[1] = data ;

	// write to BME280 register address
	if(HAL_I2C_Master_Transmit(pI2C, ptr->periph.devAddr, regData, 2, HAL_MAX_DELAY) != HAL_OK){
		return False ;
	}

	return True ;
}



/*
 *	===============================
 *	Function name: BME280_readReg
 *	Purpose: Reads from register addresses of BME280
 *	Details: Reads directly from memory addresses in BME280 via I2C.
 *			 Uses pointers to access struct members more easily
 *
 *	Params:
 *		NAME				Details
 *		[In] *ptr			- pointer to weatherSensor struct
 *		[In] *pI2C			- pointer to HAL I2C struct
 *		[In] regAddr		- register address for BME280
 *		[In] n				- number of bytes to read
 *
 *	Return value:
 *		True on success
 *	===============================
*/
Bool BME280_readReg(struct weatherSensor* ptr, I2C_HandleTypeDef* pI2C, uint8_t regAddr, uint8_t n){
	// read from BME280 register address
	if(HAL_I2C_Mem_Read(pI2C, ptr->periph.devAddr, regAddr, 1, &(ptr->periph.registerData), 1, HAL_MAX_DELAY) != HAL_OK){
		return False ;
	}

	return True ;
}



/*
 *	===============================
 *	Function name: BME280_readMultipleRegs
 *	Purpose: Reads from register addresses of BME280
 *	Details: Reads directly from memory addresses in BME280 via I2C.
 *			 Uses pointers to access struct members more easily
 *
 *	Params:
 *		NAME				Details
 *		[In] *ptr			- pointer to weatherSensor struct
 *		[In] *pI2C			- pointer to HAL I2C struct
 *		[In] regAddr		- register address for BME280
 *		[In] *arr			- address of array
 *		[In] n				- number of bytes to read
 *
 *	Return value:
 *		True on success
 *	===============================
*/
Bool BME280_readMultipleRegs(struct weatherSensor* ptr, I2C_HandleTypeDef* pI2C, uint8_t regAddr, uint8_t* arr,
		uint8_t n){
	// read from multiple BME280 register address sequentially
	if(HAL_I2C_Mem_Read(pI2C, ptr->periph.devAddr, regAddr, 1, arr, n, HAL_MAX_DELAY) != HAL_OK){
		return False ;
	}
	return True ;
}



/*
 *	===============================
 *	Function name: BME280_readSensorData
 *	Purpose: Reads from sensor register addresses of BME280
 *	Details: Reads sensor data and places read values into an array
 *			 that will be passed into an algorithm. The data array
 *			 will be overwritten everything this function is called.
 *
 *	Params:
 *		NAME				Details
 *		[In] *ptr			- pointer to weatherSensor struct
 *		[In] *pI2C			- pointer to HAL I2C struct
 *		[In] sensorAddr		- register address for BME280 sensors
 *
 *	Return value:
 *		None
 *	===============================
*/
void BME280_readSensorData(struct weatherSensor* ptr, I2C_HandleTypeDef* pI2C, uint8_t sensorAddr){
	static uint8_t regPtr ;	// iterates through register addresses for sensor reads

	static uint8_t sensorData[MAX_ARR_SIZE] = {} ;	// create empty arr of size 3

	switch(sensorAddr){
		case 0xF7:	// start of pressure read
			// read pressure sensor
			while(regPtr < 3){
				BME280_readReg(ptr, pI2C, (sensorAddr + regPtr), 1) ;
				sensorData[regPtr] = ptr->periph.registerData ;	// fill array with data vals
				regPtr++ ;	// increment regPtr
			}
			break ;

		case 0xFA:	// start of temperature read
			// read temperature sensor
			while(regPtr < 3){
				BME280_readReg(ptr, pI2C, (sensorAddr + regPtr), 1) ;
				sensorData[regPtr] = ptr->periph.registerData ;	// fill array with data vals
				regPtr++ ;	// increment regPtr
			}
			break ;

		case 0xFD:	// start of humidity read
			// read humidity sensor
			while(regPtr < 2){
				BME280_readReg(ptr, pI2C, (sensorAddr + regPtr), 1) ;
				sensorData[regPtr] = ptr->periph.registerData ;	// fill array with data vals
				regPtr++ ;	// increment regPtr
			}
			break ;

		default:	// not found
			// light LED here or something
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET) ;
			break ;
	}

	regPtr = 0 ;	// reset regPtr for next use

	// call function that interprets data from sensor
	BME280_parseData(ptr, sensorData, sensorAddr) ;
}



/*
 *	===============================
 *	Function name: BME280_parseData
 *	Purpose: Reads data from array and parses it into a single variable
 *	Details: Reads sensor data and adjusts the values using bitwise operators.
 *			 Adjusted values are pre-compensated.
 *
 *	Params:
 *		NAME				Details
 *		[In] *ptr			- pointer to weatherSensor struct
 *		[In] *arr			- pointer to array address
 *		[In] sensorAddr		- register address for BME280 sensors
 *
 *	Return value:
 *		None
 *	===============================
*/
void BME280_parseData(struct weatherSensor* ptr, uint8_t* arr, uint8_t sensorAddr){
	// create vars to hold sensor data from arr
	static uint32_t data_xlsb, data_lsb, data_msb ;

	switch(sensorAddr){
		case 0xF7:	// pressure data
			data_msb = (uint32_t)arr[0] << 12 ;
			data_lsb = ((uint32_t)(arr[1])) << 4 ;
			data_xlsb = ((uint32_t)(arr[2])) >> 4 ;
			ptr->pressureData = data_msb | data_lsb | data_xlsb ;
			break ;

		case 0xFA:	// temperature data
			data_msb = ((uint32_t)arr[0]) << 12 ;
			data_lsb = ((uint32_t)arr[1]) << 4 ;
			data_xlsb = ((uint32_t)arr[2]) >> 4 ;
			ptr->temperatureData = data_msb | data_lsb | data_xlsb ;
			break ;

		case 0xFD:	// humidity data
			data_msb = ((uint32_t)arr[0]) << 8 ;
			data_lsb = (uint32_t)arr[1] ;
			ptr->humidityData = data_msb | data_lsb ;
			break ;

		default:	// not found
			// light LED here or something
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET) ;
			break ;
	}

	// reset vars to 0
	data_xlsb = 0, data_lsb = 0, data_msb = 0 ;

	return ;
}



/*
 *	===============================
 *	Function name: BME280_compensatePressure
 *	Purpose: Calculates pressure from compensation values from BME280
 *	Details: Calculates pressure from retrieved register data and measured
 *			 pressure data.
 *
 *	Params:
 *		NAME				Details
 *		[In] *ptr			- pointer to weatherSensor struct
 *
 *	Return value:
 *		P
 *	===============================
*/
double BME280_compensatePressure(struct weatherSensor* ptr){
	// create vars for calculating pressure
	double var1, var2, var3, P ;

	var1 = ((double)t_fine / 2.0) - 64000.0 ;
	var2 = (var1 * var1) * (((double)ptr->digit.dig_P6) / 32768.0) ;
	var2 = var2 + var1 * ((double)ptr->digit.dig_P5) * 2.0 ;
	var2 = (var2 / 4.0) + (((double)ptr->digit.dig_P4) * 65536.0) ;
	var3 = ((double)ptr->digit.dig_P3) * var1 * var1 / 524288.0 ;
	var1 = (var3 + ((double)ptr->digit.dig_P2) * var1) / 524288.0 ;
	var1 = (1.0 + var1 / 32768.0) * ((double)ptr->digit.dig_P1) ;

	// check if denominator (var1) is zero
	if(var1 > 0.0){
		P = 1048576.0 - (double)ptr->pressureData ;
		P = (P - (var2 / 4096.0)) * 6250.0 / var1 ;
		var1 = ((double)ptr->digit.dig_P9) * P * P / 2147483648.0 ;
		var2 = P * ((double)ptr->digit.dig_P8) / 32768.0;
		P = P + (var1 + var2 + ((double)ptr->digit.dig_P7)) / 16.0;

		// check if pressure is less than min
		if(P < MIN_PRESSURE){
			P = MIN_PRESSURE ;
			return P ;
		}
		// check if pressure is greater than max
		if(P > MAX_PRESSURE){
			P = MAX_PRESSURE ;
			return P ;
		}
	} else {
		P = MIN_PRESSURE ;
	}

	return P ;	// in Pa
}



/*
 *	===============================
 *	Function name: BME280_compensateTemperature
 *	Purpose: Calculates temperature from compensation values from BME280
 *	Details: Calculates temperature from retrieved register data and measured
 *			 temperature data.
 *
 *	Params:
 *		NAME				Details
 *		[In] *ptr			- pointer to weatherSensor struct
 *
 *	Return value:
 *		T
 *	===============================
*/
double BME280_compensateTemperature(struct weatherSensor* ptr){
	// create vars for calculating pressure
	double var1, var2, T ;

	var1 = (((double)ptr->temperatureData) / 16384.0) - (((double)ptr->digit.dig_T1) / 1024.0) ;
	var1 = var1 * ((double)ptr->digit.dig_T2) ;
	var2 = (((double)ptr->temperatureData) / 131072.0) - (((double)ptr->digit.dig_T1) / 8192.0) ;
	var2 = (var2 * var2) * ((double)ptr->digit.dig_T3) ;

	t_fine = (int32_t)(var1 + var2) ;

	T = (var1 + var2) / 5120.0 ;

	// check if temperature exceeds max
	if(T > MAX_TEMP){
		T = MAX_TEMP ;
		return T ;
	}

	// check if temperature goes below minimum
	if(T < MIN_TEMP){
		T = MIN_TEMP ;
		return T ;
	}

	return T ;
}



/*
 *	===============================
 *	Function name: BME280_compensateHumidity
 *	Purpose: Calculates humidity from compensation values from BME280
 *	Details: Calculates humidity from retrieved register data and measured
 *			 humidity data.
 *
 *	Params:
 *		NAME				Details
 *		[In] *ptr			- pointer to weatherSensor struct
 *
 *	Return value:
 *		H
 *	===============================
*/
double BME280_compensateHumidity(struct weatherSensor* ptr){
	double var1, var2, var3, var4, var5, var6, H ;

	var1 = ((double)t_fine) - 76800.0 ;
	var2 = (((double)ptr->digit.dig_H4) * 64.0 + (((double)ptr->digit.dig_H5) / 16384.0) * var1) ;
	var3 = ptr->humidityData - var2 ;
	var4 = ((double)ptr->digit.dig_H2) / 65536.0 ;
	var5 = (1.0 + (((double)ptr->digit.dig_H3) / 67108864.0) * var1) ;
	var6 = 1.0 + (((double)ptr->digit.dig_H6) / 67108864.0) * var1 * var5 ;
	var6 = var3 * var4 * (var5 * var6) ;
	H = var6 * (1.0 - ((double)ptr->digit.dig_H1) * var6 / 524288.0) ;

	// check if H is within 0 and 100 %
	if(H < MIN_HUMID){
		return MIN_HUMID ;
	}
	if(H > MAX_HUMID){
		return MAX_HUMID ;
	}

	return H ;
}



/*
 *	===============================
 *	Function name: BME280_getCompensationVals
 *	Purpose: Retrieves compensation values from BME280 registers
 *	Details: Reads pre-determined compensation values from BME280
 *			 in order to be used for calculating temperature,
 *			 humidity, and pressure
 *
 *	Params:
 *		NAME				Details
 *		[In] *ptr			- pointer to weatherSensor struct
 *		[In] *pI2C			- pointer to I2C struct
 *		[In] compAddr		- register address for BME280
 *
 *	Return value:
 *		None
 *	===============================
*/
void BME280_getCompensationVals(struct weatherSensor* ptr, I2C_HandleTypeDef* pI2C, uint8_t compAddr){
	// maxAddr goes until next address 'head' is reached per comp. sensor
	static uint8_t arr[MAX_COMP_ARR] ;
	static int16_t dig_h4_lsb, dig_h4_msb, dig_h5_lsb, dig_h5_msb ;

	switch(compAddr){
		case 0x88:	// temperature compensation addr
			// read multiple compensation data vals for temperature
			BME280_readMultipleRegs(ptr, pI2C, compAddr, arr, MAX_TEMP_ARR_SIZE) ;

			// concat bytes
			ptr->digit.dig_T1 = BME280_CONCAT(arr[1], arr[0]) ;
			ptr->digit.dig_T2 = (int16_t)BME280_CONCAT(arr[3], arr[2]) ;
			ptr->digit.dig_T3 = (int16_t)BME280_CONCAT(arr[5], arr[4]) ;

			break ;

		case 0x8E:
			// read multiple compensation data vals for pressure
			BME280_readMultipleRegs(ptr, pI2C, compAddr, arr, MAX_PRESSURE_ARR_SIZE) ;

			// concat bytes
			ptr->digit.dig_P1 = BME280_CONCAT(arr[1], arr[0]) ;
			ptr->digit.dig_P2 = (int16_t)BME280_CONCAT(arr[3], arr[2]) ;
			ptr->digit.dig_P3 = (int16_t)BME280_CONCAT(arr[5], arr[4]) ;
			ptr->digit.dig_P4 = (int16_t)BME280_CONCAT(arr[7], arr[6]) ;
			ptr->digit.dig_P5 = (int16_t)BME280_CONCAT(arr[9], arr[8]) ;
			ptr->digit.dig_P6 = (int16_t)BME280_CONCAT(arr[11], arr[10]) ;
			ptr->digit.dig_P7 = (int16_t)BME280_CONCAT(arr[13], arr[12]) ;
			ptr->digit.dig_P8 = (int16_t)BME280_CONCAT(arr[15], arr[14]) ;
			ptr->digit.dig_P9 = (int16_t)BME280_CONCAT(arr[17], arr[16]) ;

			break ;

		case 0xA1:
			// read from 0xA1 first, then from 0xE1 thru 0xE7
			BME280_readReg(ptr, pI2C, compAddr, 1) ;

			ptr->digit.dig_H1 = ptr->periph.registerData ;

			// begin reading from 0xE1 thru 0xE7
			BME280_readMultipleRegs(ptr, pI2C, DIG_H2_ADDR, arr, MAX_HUMID_ARR_SIZE) ;
			ptr->digit.dig_H2 = (int16_t)BME280_CONCAT(arr[1], arr[0]) ;
			ptr->digit.dig_H3 = arr[2] ;
			dig_h4_msb = (int16_t)(int8_t)arr[3] * 16 ;
			dig_h4_lsb = (int16_t)(arr[4] & 0x0F) ;
			ptr->digit.dig_H4 = dig_h4_msb | dig_h4_lsb ;
			dig_h5_msb = (int16_t)(int8_t)arr[5] * 16 ;
			dig_h5_lsb = (int16_t)(arr[4] >> 4) ;
			ptr->digit.dig_H5 = dig_h5_msb | dig_h5_lsb ;
			ptr->digit.dig_H6 = (int8_t)arr[6] ;

			break ;

		default:
			__asm("nop") ;	// do nothing here
			break ;
	}

}



/*
 *	===============================
 *	Function name: BME280_fastTempConversion
 *	Purpose: Quickly approximates temperature in terms of degrees F
 *	Details: Uses bit operations in order to save time when converting data.
 *			 Works best when measured temperature is less than ~30 C
 *
 *	Params:
 *		NAME				Details
 *		[In] temperature	- temperature value in C
 *
 *	Return value:
 *		Degrees (F)
 *	===============================
*/
int16_t BME280_fastTempConversion(double temperature){
	return ((int16_t)temperature << 1) + 28 ;	// F ~ (2*C + 28)
}
