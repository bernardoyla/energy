/**
 * 
 * 
 * 
 * 
 */

#ifndef _SERIAL_ARCH_H_
#define _SERIAL_ARCH_H_

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>



/** 
 * The device I2C address
 */
uint8_t address;

/** 
 * The SPI chip select pin
 */
uint8_t cs;


#ifdef ARDUINO_ARCH_ESP32
/** 
 * SPI port type VSPI or HSPI
 */
uint8_t spiPort;
#endif



/** 
 * Pointer to the SPI class
 */
SPIClass * spi;

/** 
 * The SPI clock speed
 */
uint32_t spiClockSpeed;


#if defined ARDUINO_ARCH_ESP8266 || defined ARDUINO_ARCH_ESP32
/** 
 * Software I2C SDA and SCL pins 
 */
uint8_t sda, scl;
#endif

#endif /* _SERIAL_ARCH_H_ */