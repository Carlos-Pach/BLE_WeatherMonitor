/*
 * OLED.h
 *
 *  Created on: Jun 2, 2021
 *      Author: CarlosPach
 */

#ifndef INC_OLED_H_
#define INC_OLED_H_

#include "main.h"

// OLED address registers
#define PAGE_START		0xB0
#define OLED_ADDR		0x78

#define COLUMN_ADDR		0x21
#define PAGE_ADDR		0x22
#define HORIZ_MODE		0x00
#define OLED_CLK		0xD5
#define OLED_FREQ		0x80
#define OLED_REFRESH	0xF0
#define MUX_MODE		0xA8
#define MUX_HEIGHT		0x3F	// (64 - 1) --> 0x3F
#define OLED_OFFSET		0xD3
#define START_LINE		0x40
#define CHARGE_PUMP		0x8D
#define ENABLE_PUMP		0x14
#define MEM_ADDR_MODE	0x20
#define SEGMENT_MAP		0xA0	// used in rotating x axis
#define SEGMENT_MAP_INV	0xA1	// used in rotating x axis
#define COM_SCAN		0xC0	// used in rotating y axis
#define COM_SCAN_DEC	0xC8	// used in rotating y axis
#define COM_PINS		0xDA
#define CONTRAST		0x81
#define CONTRAST_RES	0x7F
#define PRE_CHARGE		0xD9
#define COM_DETECT		0xD8
#define VCOM_H			0xDB	// adjusts Vcomh regulator

#define DISPLAY_ON_RES	0xA4	// outputs RAM contents
#define DISPLAY_ON_ALL	0xA5
#define DISPLAY_NORM	0xA6
#define TURN_OFF_OLED	0xAE
#define TURN_ON_OLED	0xAF

// misc defines
#define WIDTH			(128UL)
#define HEIGHT			(64UL)
#define NUM_SYMBOLS		(15UL)
#define ASCII_OFFSET	(0x30)
#define HORIZ_MODE_LEN	(10UL)



typedef enum {
	OLED_OK,			// OK
	OLED_NUM_OOR,		// number out of range
	OLED_SPI_ERR,		// SPI error
	OLED_INVALID_BME,	// BME280 data invalid
	OLED_NULL, 			// value of OLED struct member is NULL
	OLED_ERR_UNKNOWN	// unknown error
} OLED_ERR_MSG ;



struct display{
	struct Peripheral periphOLED ;				// inherits Peripheral struct
	uint8_t *pDataBuf, resetData ;				// OLED exclusive to point to write data buffer
} ;




// OLED function prototypes
void initOLED_buffer(struct display* pDisplay, SPI_HandleTypeDef* pSPI) ;
void initOLEDParams(struct display* pDisplay) ;
void resetOLED(struct display* pDisplay, SPI_HandleTypeDef* pSPI) ;
OLED_ERR_MSG writeCommandOLED(struct display* pDisplay, SPI_HandleTypeDef* pSPI, uint8_t n) ;
OLED_ERR_MSG writeCommandOLED_buffer(struct display* pDisplay, SPI_HandleTypeDef* pSPI,
									 uint8_t n, uint8_t* arr) ;
OLED_ERR_MSG createBorderOLED(struct display* pDisplay, SPI_HandleTypeDef* pSPI) ;
OLED_ERR_MSG writeDataOLED(struct display* pDisplay, SPI_HandleTypeDef* pSPI, uint8_t n) ;
OLED_ERR_MSG writeDataOLED_buffer(struct display* pDisplay, SPI_HandleTypeDef* pSPI, uint8_t n, uint8_t* arr) ;
void testOLED(struct display* pDisplay, SPI_HandleTypeDef* pSPI) ;
void clearOLED(struct display* pDisplay, SPI_HandleTypeDef* pSPI) ;
OLED_ERR_MSG OLED_setCursor(struct display* pDisplay, SPI_HandleTypeDef* pSPI, uint8_t row, uint8_t col) ;
void OLED_writeSensorData(struct display* pDisplay, SPI_HandleTypeDef* pSPI) ;
OLED_ERR_MSG OLED_printStr(struct display* pDisplay, SPI_HandleTypeDef* pSPI, int8_t* ptr[]) ;
void OLED_changeContrast(struct display* pDisplay, SPI_HandleTypeDef* pSPI, uint8_t dimmingVal_g) ;

#endif /* INC_OLED_H_ */
