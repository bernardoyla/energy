/**
 * @file BMP280.h
 * @brief Header file for the BMP280 sensor library.
 * 
 * This file contains the declarations of constants, enums, structs, and functions
 * related to the BMP280 sensor library. It provides communication bus configuration,
 * register addresses, and various settings for the BMP280 sensor.
 */

#ifndef _BMP280_H_
#define _BMP280_H_

#include "bmp280_dev/BMP280_DEV.h"

/** 
 * Communications bus: I2C or SPI
 */
#define BMP280_I2C

/** 
 * Clock configuration
 */
#define BMP280_I2C_SCL 400000

/**
 * Chip select pin
 */
#define BMP_SPI_CS 10

/**
 * SPI clock speed
 */
#define BMP_SPI_CLOCK_SPEED 1000000

/**
 * Write/read mask for SPI communication
 */
static const uint8_t WRITE_MASK = 0x7F;
static const uint8_t READ_MASK = 0x80;

/* ------------------------ ------------------------ */

/* BME280 register */

#define	BMP280_REG_TRIM_PARAMS	0x88	/* Trim parameter registers' base sub-address */
#define	BMP280_REG_DEVICE_ID 	0xD0	/* Device ID register sub-address */
#define	BMP280_REG_RESET 		0xE0	/* Reset register sub-address */
#define	BMP280_REG_STATUS     	0xF3	/* Status register sub-address */ 
#define	BMP280_REG_CTRL_MEAS  	0xF4	/* Control and measurement register sub-address */
#define	BMP280_REG_CONFIG     	0xF5	/* Configuration register sub-address */
#define	BMP280_REG_PRES_MSB   	0xF7	/* Pressure Most Significant Byte (MSB) register sub-address */
#define	BMP280_REG_PRES_LSB   	0xF8	/* Pressure Least Significant Byte (LSB) register sub-address */ 
#define	BMP280_REG_PRES_XLSB  	0xF9	/* Pressure eXtended Least Significant Byte (XLSB) register sub-address */
#define	BMP280_REG_TEMP_MSB   	0xFA	/* Pressure Most Significant Byte (MSB) register sub-address */
#define	BMP280_REG_TEMP_LSB   	0xFB  	/* Pressure Least Significant Byte (LSB) register sub-address */ 
#define	BMP280_REG_TEMP_XLSB  	0xFC	/* Pressure eXtended Least Significant Byte (XLSB) register sub-address */


/* Infinite Impulse Response (IIR) filter bit field in the configuration register */

#define	BMP280_IIR_FILTER_OFF	0x00
#define	BMP280_IIR_FILTER_2		0x01
#define	BMP280_IIR_FILTER_4		0x02
#define	BMP280_IIR_FILTER_8 	0x03
#define	BMP280_IIR_FILTER_16	0x04


#define BMP280_IIR_FILTER BMP280_IIR_FILTER_OFF

/* Time standby bit field in the configuration register */

#define	BMP280_TIME_STANDBY_05MS  		0x00
#define	BMP280_TIME_STANDBY_62MS  		0x01
#define	BMP280_TIME_STANDBY_125MS 		0x02
#define	BMP280_TIME_STANDBY_250MS 		0x03
#define	BMP280_TIME_STANDBY_500MS 		0x04
#define	BMP280_TIME_STANDBY_1000MS		0x05
#define	BMP280_TIME_STANDBY_2000MS		0x06
#define	BMP280_TIME_STANDBY_4000MS		0x07


#define BMP280_TIME_STANDBY BMP280_TIME_STANDBY_05MS

/**
 * Device mode bitfield in the control and measurement register
 */
#define	BMP280_SLEEP_MODE 		0x00
#define	BMP280_FORCED_MODE		0x01
#define	BMP280_NORMAL_MODE		0x03

#define	BMP280_MASK_MODE		0x03


#define BMP280_MODE BMP280_SLEEP_MODE

/** 
 * Oversampling bit fields in the control and measurement register 
 */
#define	BMP280_OVERSAMPLING_SKIP	0x00
#define	BMP280_OVERSAMPLING_X1  	0x01
#define	BMP280_OVERSAMPLING_X2  	0x02
#define	BMP280_OVERSAMPLING_X4  	0x03
#define	BMP280_OVERSAMPLING_X8  	0x04
#define	BMP280_OVERSAMPLING_X16 	0x05

#define BMP280_OVERSAMPLING_TEMP BMP280_OVERSAMPLING_X2
#define BMP280_OVERSAMPLING_PRESS BMP280_OVERSAMPLING_X16

/** 
 * The BMP280 compensation trim parameters (coefficients) 
 */
struct {
	uint16_t dig_T1;
	int16_t  dig_T2;
	int16_t  dig_T3;
	uint16_t dig_P1;
	int16_t  dig_P2;
	int16_t  dig_P3;
	int16_t  dig_P4;
	int16_t  dig_P5;
	int16_t  dig_P6;
	int16_t  dig_P7;
	int16_t  dig_P8;
	int16_t  dig_P9;
} params;

/** 
 * Copy of the BMP280's configuration register
 */
union {
	struct {
		uint8_t spi3w_en : 1;
		uint8_t 		 : 1;
		uint8_t filter 	 : 3;
		uint8_t t_sb	 : 3;
	} bit;
	uint8_t reg;
} config = { .reg = 0 };

/** 
 * Copy of the BMP280's control and measurement register
 */
union {
	struct {
		uint8_t mode   : 2;
		uint8_t osrs_p : 3;
		uint8_t osrs_t : 3;
	} bit;
	uint8_t reg;

} ctrl_meas = { .reg = 0 };

/** 
 * Copy of the BMP280's status register
 */
/* union {
	struct {
		uint8_t im_update 	: 1;
		uint8_t				: 2;
		uint8_t measuring 	: 1;
	} bit;
	uint8_t reg;
} status = { .reg = 0 }; */
#define BMP280_STATUS_MEASURING 0b1000
#define BMP280_STATUS_IM_UPDATE 0b0001


/** 
 * Sea level pressure 
 */
#define SEA_LEVEL_PRESSURE 1013.23f



uint8_t bmp280_init(void);

float bmp280_temperature(void);

int bmp280_measurements(float * temperature, float * pressure, float * altitude);

#endif /* _BMP280_H_ */