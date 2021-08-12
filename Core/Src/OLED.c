/*
 * OLED.c
 *
 *  Created on: Apr 14, 2021
 *      Author: CarlosPach
 */

#include "main.h"
#include "OLED.h"

// initialization buffer
uint8_t initBuffer_g[]= {
		MUX_MODE,
		MUX_HEIGHT,
		OLED_OFFSET,
		0, 0,
		START_LINE,
		SEGMENT_MAP_INV,	// SEGMENT_MAP --> SEGMENT_MAP_INV
		COM_SCAN_DEC,		// COM_SCAN --> COM_SCAN_DEC
		COM_PINS,
		0x12,
		CONTRAST,
		CONTRAST_RES,
		//DISPLAY_ON_RES,	// output contents from RAM ... comment out when not testing device
		DISPLAY_NORM,
		OLED_CLK,
		OLED_FREQ,
		PRE_CHARGE,
		0x22,
		CHARGE_PUMP,
		ENABLE_PUMP,
		TURN_ON_OLED
} ;

// horizontal mode buffer
uint8_t horizMode_g[HORIZ_MODE_LEN] = {
	MEM_ADDR_MODE,		// address mode
	0,					// select horizontal
	DISPLAY_ON_RES,
	TURN_ON_OLED,
	0x22,				// page address (2 byte command)
	0,					// starting page
	7,					// ending page
	0x21,				// column address (2 byte command)
	0 ,					// starting column
	127					// ending column
} ;


// symbols to convert BME280 vals for OLED display
static uint8_t symbolsBME280[NUM_SYMBOLS][8] = {					// num of symbols x 8 columns
			{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x60, 0x60, 0x00 },		// '.'
			{ 0x00, 0x60, 0x70, 0x38, 0x1C, 0x0E, 0x06, 0x00 }, 	// '/'
			{ 0x00, 0x3C, 0x42, 0x42, 0x42, 0x42, 0x3C, 0x00 },		// '0'
			{ 0x00, 0x00, 0x42, 0x41, 0x7F, 0x40, 0x40, 0x00 },		// '1'
			{ 0x00, 0x84, 0xC2, 0xA1, 0x91, 0x8A, 0x84, 0x00 },		// '2'
			{ 0x00, 0x42, 0x81, 0x81, 0x99, 0xA5, 0x42, 0x00 },		// '3'
			{ 0x00, 0x10, 0x18, 0x14, 0x12, 0xFF, 0x10, 0x00 },		// '4'
			{ 0x00, 0x87, 0x89, 0x89, 0x89, 0x89, 0x71, 0x00 },		// '5'
			{ 0x00, 0x70, 0x98, 0x94, 0x92, 0xE1, 0x01, 0x00 }, 	// '6'
			{ 0x00, 0x01, 0x09, 0x09, 0x09, 0xFF, 0x08, 0x00 },		// '7'
			{ 0x00, 0x66, 0x99, 0x99, 0x99, 0x99, 0x66, 0x00 }, 	// '8'
			{ 0x00, 0x06, 0x09, 0x09, 0x09, 0x09, 0xFE, 0x00 },		// '9'
			{ 0x00, 0x44, 0x2E, 0x14, 0x28, 0x74, 0x22, 0x00 },		// percent sign '%'
			{ 0x0C, 0x12, 0x12, 0x0C, 0x00, 0x3C, 0x42, 0x42 }, 	// degrees celcius sign '*C'
			{ 0x00, 0x20, 0x7F, 0x81, 0x81, 0x7F, 0x20, 0x00 }		// pressure sign (downwards arrow)
	} ;

/*
 *	===========================================
 *	SPI Interface (4 Wire)
 *	===========================================
 *		Function		CS		DC		D0
 *
 *		Write command	0		0		1
 *		Write data		0		1		1
 *	===========================================
 */



/*
 *	=====================================================
 *	Function name: initOLEDParams
 *	Purpose: Initializes on/off commands in OLED struct
 *	Details: Assigns data and reset data command that
 *			 will be used to turn on/off the OLED panel
 *
 *	Params:
 *		Name					Details
 *		[In] *pDisplay		 	- pointer to OLED struct
 *
 *	Return value:
 *		N/A
 *	=====================================================
*/
void initOLEDParams(struct display* pDisplay){
	pDisplay->periphOLED.registerData = TURN_ON_OLED ;
	pDisplay->resetData = TURN_OFF_OLED ;
	pDisplay->pDataBuf = NULL ;					// unused for now
	pDisplay->periphOLED.devID = NULL ;			// unused for now (SPI OLED)
}



/*
 *	=====================================================
 *	Function name: resetOLED
 *	Purpose: Resets OLED panel
 *	Details: Sends reset command to OLED panel
 *
 *	Params:
 *		Name					Details
 *		[In] *pDisplay		 	- pointer to OLED struct
 *		[In] *pSPI				- pointer to SPI struct
 *
 *	Return value:
 *		N/A
 *	=====================================================
*/
void resetOLED(struct display* pDisplay, SPI_HandleTypeDef* pSPI){
	// select OLED for SPI
	GPIOD->ODR &= ~(1 << 8) ;	// CS low

	// reset OLED
	GPIOB->ODR &= ~(1 << 12) ;	// RES low
	HAL_Delay(10) ;
	GPIOB->ODR |= (1 << 12) ;	// RES high
	HAL_Delay(150) ;

	// select OLED
	GPIOB->ODR &= ~(1 << 14) ;	// DC low
	HAL_SPI_Transmit(pSPI, &(pDisplay->resetData), 1, HAL_MAX_DELAY) ;
	// unselect OLED
	GPIOD->ODR |= (1 << 8) ;
}



/*
 *	=====================================================
 *	Function name: writeCommandOLED
 *	Purpose: Sends a single command to OLED
 *	Details: Turns on command mode to send a single byte
 *			 command to OLED panel
 *
 *	Params:
 *		Name					Details
 *		[In] *pDisplay		 	- pointer to OLED struct
 *		[In] *pSPI				- pointer to SPI struct
 *		[In] n					- number of bytes to send (1)
 *
 *	Return value:
 *		N/A
 *	=====================================================
*/
OLED_ERR_MSG writeCommandOLED(struct display* pDisplay, SPI_HandleTypeDef *pSPI, uint8_t n){
	// check if registerData is NULL
	if(pDisplay->periphOLED.registerData == NULL){
		return OLED_NULL ;
	}

	// turn off DC
	GPIOB->ODR &= ~(1 << 14) ;

	HAL_SPI_Transmit(pSPI, &(pDisplay->periphOLED.registerData), n, HAL_MAX_DELAY) ;

	// turn on DC
	GPIOB->ODR |= (1 << 14) ;
}



OLED_ERR_MSG writeDataOLED(struct display* pDisplay, SPI_HandleTypeDef* pSPI, uint8_t n){
	// check if registerData is NULL
	if(pDisplay->periphOLED.registerData == NULL){
		return OLED_NULL ;
	}

	// turn on DC
	GPIOB->ODR |= (1 << 14) ;
	HAL_SPI_Transmit(pSPI, &(pDisplay->periphOLED.registerData), n, HAL_MAX_DELAY) ;
	// turn off DC
	GPIOB->ODR &= ~(1 << 14) ;
}



void initOLED_buffer(struct display* pDisplay, SPI_HandleTypeDef* pSPI){
	const uint8_t n = sizeof(initBuffer_g)/sizeof(initBuffer_g[0]) ;

	// select OLED
	GPIOD->ODR &= ~(1 << 8) ;	// CS low

	// write command to turn on display
	GPIOB->ODR &= ~(1 << 14) ;	// DC low
	GPIOB->ODR |= (1 << 12) ; 	// RES high

	HAL_SPI_Transmit(pSPI, initBuffer_g, n, HAL_MAX_DELAY) ;
}



void testOLED(struct display* pDisplay, SPI_HandleTypeDef* pSPI){
	//static uint8_t dataArr[10] = { [0 ... 9] = 0 } ;
	const uint8_t n = sizeof(horizMode_g)/sizeof(horizMode_g[0]) ;

	// write to OLED
	writeCommandOLED_buffer(pDisplay, pSPI, n, horizMode_g) ;

	// test data write
	pDisplay->periphOLED.registerData = 0 ;	// half pixels on, half off (0xAA --> 0xFF --> 0x00)
	for(uint8_t i = 0; i < 12; i++){
		writeDataOLED(pDisplay, pSPI, 1) ;	// single byte
		//writeDataOLED_buffer(pDisplay, pSPI, n, dataArr) ;
	}
}



void clearOLED(struct display* pDisplay, SPI_HandleTypeDef* pSPI){
	// init buffer to transmit (command buffer)
	static uint8_t buf[3] = { PAGE_START, 0x00, 0x10 } ;
	// buffer for commands
	static uint8_t cmdBuf[10] ;
	// buffer for clearing screen
	static uint8_t clearBuf[WIDTH] = { [0 ... (WIDTH - 1)] = 0 } ;

	const uint8_t n = sizeof(buf)/sizeof(buf[0]) ;
	const uint8_t m = sizeof(cmdBuf)/sizeof(cmdBuf[0]) ;

	// address mode (horizontal)
	cmdBuf[0] = MEM_ADDR_MODE ;
	cmdBuf[1] = 0 ;
	// turn display on
	cmdBuf[2] = DISPLAY_ON_RES ;
	cmdBuf[3] = TURN_ON_OLED ;
	// set address range
	cmdBuf[4] = PAGE_ADDR ;		// 2 byte command
	cmdBuf[5] = 0 ;
	cmdBuf[6] = 7 ;
	// set column range
	cmdBuf[7] = COLUMN_ADDR ;	// 2 byte command
	cmdBuf[8] = 0 ;
	cmdBuf[9] = 127 ;

	writeCommandOLED_buffer(pDisplay, pSPI, m, cmdBuf) ;

	// start at page 0xB0 (page 0)
	// start at lower column 0x00 (column 0) ... (0x00 thru 0x0F)
	// end at upper column 0x10 (column 16) ... (0x10 thru 0x1F)

	for(uint8_t i = 0; i < (HEIGHT >> 3); i++){	// (HEIGHT >> 3) <--> (HEIGHT / 8)
		// write command
		writeCommandOLED_buffer(pDisplay, pSPI, n, buf) ;
		// write data buffer
		writeDataOLED_buffer(pDisplay, pSPI, WIDTH, clearBuf) ;
		buf[0]++ ;	// increment by 1
	}

	// reset buf[0]
	buf[0] = PAGE_START ;
}



OLED_ERR_MSG createBorderOLED(struct display* pDisplay, SPI_HandleTypeDef* pSPI){
	static OLED_ERR_MSG ERR_MSG = OLED_OK ;							// error message
	static uint8_t border[WIDTH] = { [0 ... (WIDTH - 1)] = 0x40 } ;	// GCC exclusive
	static uint8_t borderSetupArr[10] = {
			MEM_ADDR_MODE, 0, DISPLAY_ON_RES, TURN_ON_OLED,
			PAGE_ADDR, 0, 7, COLUMN_ADDR, 0, 127
			} ;

	static uint8_t n = sizeof(borderSetupArr)/sizeof(borderSetupArr[0]) ;

	if(writeCommandOLED_buffer(pDisplay, pSPI, n, borderSetupArr) != OLED_OK){
		ERR_MSG = OLED_SPI_ERR ;
		return ERR_MSG ;
	}

	writeDataOLED_buffer(pDisplay, pSPI, WIDTH, border) ;

	for(uint8_t i = 0; i < 3; i++){
		OLED_setCursor(pDisplay, pSPI, (i + 1), 0) ;
		// symbolsBME280 + (8 * i) returns incorrect data with a pointer
		horizMode_g[5] = (i + 1) ;	// set start to i-th row
		horizMode_g[6] = (i + 1) ;	// set end to i-th row
		writeCommandOLED_buffer(pDisplay, pSPI, 10, horizMode_g) ;
		writeDataOLED_buffer(pDisplay, pSPI, 8, &symbolsBME280[i + 12][0]) ;	// i + 10 --> i + 12
	}

	return ERR_MSG ;
}



OLED_ERR_MSG OLED_setCursor(struct display* pDisplay, SPI_HandleTypeDef* pSPI, uint8_t row, uint8_t col){
	OLED_ERR_MSG ret = OLED_OK ;
	uint8_t cursor[8] = { [0 ... 7] = 0xFF } ;	// GCC cheat code for initializing arr
	const static uint8_t n_cursor = sizeof(cursor)/sizeof(cursor[0]) ;

	// determine if row and/or column is out of range
	if((col > 15) || (col < 0) || (row > 7) || (row < 0)){
		ret = OLED_NUM_OOR ;
	    return ret ;
	}

	horizMode_g[5] = row ;
	horizMode_g[8] = col << 3 ;	// multiply by 8

	if(writeCommandOLED_buffer(pDisplay, pSPI, 10, horizMode_g) != OLED_OK){
		ret = OLED_SPI_ERR ;
	}

	writeDataOLED_buffer(pDisplay, pSPI, n_cursor, cursor) ;

	return ret ;
}



OLED_ERR_MSG OLED_printStr(struct display* pDisplay, SPI_HandleTypeDef* pSPI, int8_t* ptr[]){
	OLED_ERR_MSG ret = OLED_OK ;

	if(OLED_setCursor(pDisplay, pSPI, 1, 2) != OLED_OK){	// row 1, column 2
		ret = OLED_SPI_ERR ;
		return ret ;
	}

	horizMode_g[5] = 1 ;			// starting page
	horizMode_g[6] = 2 ;			// ending page
	horizMode_g[8] = (2 << 3) - 1 ;	// starting column	(cursor col 2, subtract 1 to account for starting col)
	// ((4 << 3) - 1) --> ((5 << 3) - 1)
	horizMode_g[9] = (6 << 3) - 1 ;	// ending column	(cursor col 6, subtract 1 to account for ending col)

	if(writeCommandOLED_buffer(pDisplay, pSPI, HORIZ_MODE_LEN, horizMode_g) != OLED_OK){
		ret = OLED_SPI_ERR ;
		return ret ;
	}

	for(uint8_t i = 0; i < 2; i++){	// only write humidity and temperature for now
		while(**ptr != '\n'){
			// [**ptr - ASCII_OFFSET] --> [**ptr - (ASCII_OFFSET - 2)] ... 2 is the num of non-digit vals in LUT
			writeDataOLED_buffer(pDisplay, pSPI, 8, &symbolsBME280[**ptr - (ASCII_OFFSET - 2)][0]) ;
			ptr[0]++ ;
		}
		ptr++ ;	// once last element of ptr[2] has been reached, this goes into an error (hard fault)
	}

	horizMode_g[5] = 3 ;			// starting page
	horizMode_g[5] = 5 ;			// ending page
	horizMode_g[8] = (2 << 3) - 1 ;	// starting column
	horizMode_g[9] = (7 << 3) - 1 ;	// ending column

	OLED_setCursor(pDisplay, pSPI, 3, 2) ;

	if(writeCommandOLED_buffer(pDisplay, pSPI, 10, horizMode_g) != OLED_OK){
		ret = OLED_SPI_ERR ;
		return ret ;
	}

	while(**ptr != '\n'){	// write pressure
		writeDataOLED_buffer(pDisplay, pSPI, 8, &symbolsBME280[**ptr - (ASCII_OFFSET - 2)][0]) ;
		ptr[0]++ ;
	}

	return ret ;
}



void OLED_writeSensorData(struct display* pDisplay, SPI_HandleTypeDef* pSPI){
	//uint32_t * ptr = &pDisplay->writeData ;
	uint8_t dataArr[24] = { 0x00, 0x44, 0x2E, 0x14, 0x28, 0x74, 0x22, 0x00,  // percent sign %
						   0x0C, 0x12, 0x12, 0x0C, 0x00, 0x3C, 0x42, 0x42,	// degrees celcius sign
						   0x3C, 0x42, 0x81, 0x99, 0x99, 0x85, 0x42, 0x3C	// pressure sign
						} ;

	writeCommandOLED_buffer(pDisplay, pSPI, 10, horizMode_g) ;

	writeDataOLED_buffer(pDisplay, pSPI, 24, dataArr) ;
}



OLED_ERR_MSG writeCommandOLED_buffer(struct display* pDisplay, SPI_HandleTypeDef* pSPI, uint8_t n, uint8_t* arr){
	static OLED_ERR_MSG ERR_MSG = OLED_OK ;	// oled error driver message
	static uint8_t *p ;			// used in debugging
	p = arr ;

#if 0	// remove if #if when file is modified to only use pDataBuf
	// check if address of OLED is null
	if(pDisplay->pDataBuf == NULL){
		ERR_MSG = OLED_NUM_OOR ;
		return ERR_MSG ;
	}
#endif

	// check if arr is NULL
	if(p == NULL){
		ERR_MSG = OLED_NUM_OOR ;
		return ERR_MSG ;
	}

	// turn off DC
	GPIOB->ODR &= ~(1 << 14) ;

	if(HAL_SPI_Transmit(pSPI, p, n, HAL_MAX_DELAY) != HAL_OK){
		ERR_MSG = OLED_SPI_ERR ;
	}

	// turn on DC
	GPIOB->ODR |= (1 << 14) ;

	return ERR_MSG ;
}



OLED_ERR_MSG writeDataOLED_buffer(struct display* pDisplay, SPI_HandleTypeDef* pSPI, uint8_t n, uint8_t* arr){
	static OLED_ERR_MSG ERR_MSG = OLED_OK ;
	static uint8_t *p ;			// used in debugging
	p = arr ;

#if 0	// remove if #if when file is modified to only use pDataBuf
	// check if address of OLED is null
	if(pDisplay->pDataBuf == NULL){
		ERR_MSG = OLED_NULL ;
		return ERR_MSG ;
	}
#endif

	// check if arr is NULL
	if(p == NULL){
		ERR_MSG = OLED_NULL ;
		return ERR_MSG ;
	}

	// turn on DC
	GPIOB->ODR |= (1 << 14) ;

	if(HAL_SPI_Transmit(pSPI, p, n, HAL_MAX_DELAY) != HAL_OK){
		ERR_MSG = OLED_SPI_ERR ;
	}

	// turn off DC
	GPIOB->ODR &= ~(1 << 14) ;

	return ERR_MSG ;
}



void OLED_changeContrast(struct display* pDisplay, SPI_HandleTypeDef* pSPI, uint8_t dimmingVal_g){
	// buffer to change dimming on OLED panel
	uint8_t buf[2] = { CONTRAST, dimmingVal_g } ;
	const uint8_t n = sizeof(buf)/sizeof(buf[0]) ;

	// send commands to OLED panel
	// select OLED
	GPIOD->ODR &= ~(1 << 8) ;	// CS low

	// write command to turn on display
	GPIOB->ODR &= ~(1 << 14) ;	// DC low
	GPIOB->ODR |= (1 << 12) ; 	// RES high

	// send data to OLED panel
	HAL_SPI_Transmit(pSPI, buf, n, HAL_MAX_DELAY) ;
}
