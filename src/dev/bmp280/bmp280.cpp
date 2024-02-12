/** 
 * 
 * 
 * 
 */

//#include "bmp280_dev/BMP280_DEV.h"
#include "bmp280.h"


/** 
 * Definition of the BMP280 device communication type
 */
#ifdef BMP280_I2C

#include <Wire.h>
static TwoWire * bmp280_i2c;

#else /* BMP280_SPI */

#include <SPI.h>
static SPIClass * spi;

#endif /* BMP280_SPI or BMP280_I2C */



#include "../serial_arch.h"



/** 
 * 
 *
 */
static BMP280_DEV bmp280;
int trying_to;


/** @todo ni puta idea que es esta variable pero se usa de forma global
 * para compensar algo en la temperatura
 */
int32_t t_fine;	

/* ------------------------------------------------------------------------------------------- */


void serial_write_byte(uint8_t bmP280_reg, uint8_t data) {

	#ifdef BMP280_I2C
	bmp280_i2c->beginTransmission(BMP280_I2C_ADDR);
	bmp280_i2c->write(bmP280_reg);          
	bmp280_i2c->write(data);                 
	bmp280_i2c->endTransmission();

	#else /* BMP280_SPI */
	spi->beginTransaction(SPISettings(spiClockSpeed, MSBFIRST, SPI_MODE0));
	digitalWrite(cs, LOW);
	spi->transfer(bmP280_reg & WRITE_MASK);
	spi->transfer(data);
	digitalWrite(cs, HIGH);
	spi->endTransaction();
	#endif /* BMP280_I2C or BMP280_SPI */
}

uint8_t serial_read_byte(uint8_t bmP280_reg) {

	/* I2C */
	uint8_t data = 0x00;

	#ifdef BMP280_I2C
	bmp280_i2c->beginTransmission(BMP280_I2C_ADDR);         
	bmp280_i2c->write(bmP280_reg);                  
	bmp280_i2c->endTransmission(false);             
	bmp280_i2c->requestFrom(BMP280_I2C_ADDR, (uint8_t)1);	 
	data = bmp280_i2c->read();                      
	
	#else /* BMP280_SPI */
	spi->beginTransaction(SPISettings(spiClockSpeed, MSBFIRST, SPI_MODE0));
	digitalWrite(cs, LOW);
	spi->transfer(bmP280_reg | READ_MASK);
	data = spi->transfer(data);
	digitalWrite(cs, HIGH);
	spi->endTransaction();	
	#endif /* BMP280_I2C or BMP280_SPI */

  return data;

}

void serial_read_bytes(uint8_t bmP280_reg, uint8_t* data, uint16_t count) {  

	#ifdef BMP280_I2C
	bmp280_i2c->beginTransmission(BMP280_I2C_ADDR);          
	bmp280_i2c->write(bmP280_reg);                   
	bmp280_i2c->endTransmission(false);              
	uint8_t i = 0;
	bmp280_i2c->requestFrom(BMP280_I2C_ADDR, (uint8_t)count);  
	while (bmp280_i2c->available()) {
		data[i++] = bmp280_i2c->read();          
	}

	#else /* BMP280_SPI */	
	spi->beginTransaction(SPISettings(spiClockSpeed, MSBFIRST, SPI_MODE0));	// Read "count" bytes into the "data" buffer using SPI
	digitalWrite(cs, LOW);
	spi->transfer(bmP280_reg | READ_MASK);
	spi->transfer(data, count);
	digitalWrite(cs, HIGH);
	spi->endTransaction();
	#endif /* BMP280_I2C or BMP280_SPI */
}


/* ------------------------------------------------------------------------------------------- */

uint8_t bmp280_init(void) {

	/** 
	 * Initialise I2C communication and set the SCL clock 
	 */
	#ifdef BMP280_I2C
		bmp280_i2c->begin();
		bmp280_i2c->setClock(BMP280_I2C_SCL);

	#if defined ARDUINO_ARCH_ESP8266 || defined ARDUINO_ARCH_ESP32
		//no esta terminado
		bmp280_i2c->begin(sda, scl);
		bmp280_i2c->setClock(BMP280_I2C_SCL);

	#endif /* defined ARDUINO_ARCH_ESP8266 || defined ARDUINO_ARCH_ESP32 */
	#endif /* BMP280_I2C */

	#ifdef BMP280_SPI
		digitalWrite(cs, HIGH);		// Pull the chip select (CS) pin high
		pinMode(cs, OUTPUT);			// Set-up the SPI chip select pin

	#ifdef ARDUINO_ARCH_ESP32
			if (spiPort == HSPI)			// Set-up spi pointer for VSPI or HSPI communications
			{
				spi->begin(14, 27, 13, 2);		// Start HSPI on SCK 14, MOSI 13, MISO 24, SS CS (GPIO2 acts as dummy pin)
			}
			else
			{
				spi = &SPI;					// Start VSPI on SCK 5, MOSI 18, MISO 19, SS CS
				spi->begin();
			}							
	#endif /* ARDUINO_ARCH_ESP32 */
			spi = &SPI;						// Set-up spi pointer for SPI communications
			spi->begin();

	#endif /* BMP280_SPI */

	/* Reading device ID */
	if (serial_read_byte(BMP280_DEVICE_ID) != DEVICE_ID) //Aqui falta controlar porque fallo
    	return 0;

	/* Soft reset */
  	serial_write_byte(BMP280_RESET, RESET_CODE);                     									
 	delay(10);                                                            
  	
	/** 
	 * Read the trim parameters into the params structure 
	 */
	serial_read_bytes(BMP280_TRIM_PARAMS, (uint8_t*)&params, sizeof(params));
	
	/** 
	 * Set the initial configuration and control and measurement register
	 * @todo esto puede quedar mas elegante
	 */
	config.reg = BMP280_TIME_STANDBY << 5 | BMP280_IIR_FILTER << 2;
	ctrl_meas.reg = BMP280_OVERSAMPLING_TEMP << 5 | BMP280_OVERSAMPLING_PRESS << 2 | BMP280_MODE;	
	serial_write_byte(BMP280_REG_CONFIG, config.reg);
	serial_write_byte(BMP280_REG_CTRL_MEAS, ctrl_meas.reg);    

	return 1;

}



uint8_t bmp280_set_mode(uint8_t mode) {

	/* Si el modo solicitado es force... */
	if(mode == FORCED_MODE) {
		/* ...hay que verificar que este en sleep */
		if ((serial_read_byte(BMP280_CTRL_MEAS) & BMP280_MASK_MODE) != SLEEP_MODE)
			return 0;
	}

	ctrl_meas.bit.mode = mode;
	serial_write_byte(BMP280_CTRL_MEAS, (ctrl_meas.bit.mode));


	return ctrl_meas.bit.mode;
}

/** 
 * @todo esta funcion esta sin revisar y no se bienq ue hace
 */
int32_t bmp280_compensate_T_int32(int32_t adc_T) {
	int32_t var1, var2, T;
	var1 = ((((adc_T >> 3) - ((int32_t)params.dig_T1 << 1))) * ((int32_t)params.dig_T2)) >> 11;
	var2 = (((((adc_T >> 4) - ((int32_t)params.dig_T1)) * ((adc_T >> 4) - ((int32_t)params.dig_T1))) >> 12) *
	((int32_t)params.dig_T3)) >> 14;
	t_fine = var1 + var2;
	T = (t_fine * 5 + 128) >> 8;
	return T;
}

float bmp280_temperature(void) {

	/* Preparando el registro para medir solo la temperatura */
	ctrl_meas.bit.osrs_p = BMP280_OVERSAMPLING_SKIP;
	ctrl_meas.bit.osrs_t = BMP280_OVERSAMPLING_TEMP;

	/* Activando la conversion */
	bmp280_set_mode(FORCED_MODE);

	/** @todo las dos operaciones de chequeo de espera por la conversion 
	 * y la copia en memoria no creo que funciones bien, hay que mejorarlas
	 */
	/* Esperando que la conversion termine */
	while (serial_read_byte(BMP280_STATUS) & BMP280_STATUS_MEASURING);
	/* Esperando a que la copia en memoria termine */
	while (serial_read_byte(BMP280_STATUS) & BMP280_STATUS_IM_UPDATE);

	/* Leyendo el registro de temperatura */

	uint8_t data[3];                                                  // Create a data buffer
	serial_read_bytes(BMP280_TEMP_MSB, &data[0], 3);             							// Read the temperature and pressure data
	int32_t adcTemp = (int32_t)data[0] << 12 | (int32_t)data[1] << 4 | (int32_t)data[2] >> 4;  // Copy the temperature and pressure data into the adc variables
	int32_t temp = bmp280_compensate_T_int32(adcTemp);                // Temperature compensation (function from BMP280 datasheet)
	return ((float)temp / 100.0f);                               // Calculate the temperature in degrees Celsius

}





int bmp280_measurements(float * temperature, float * pressure, float * altitude) {    
	
	delay(1000);

	Serial.println(trying_to);
	trying_to++;

	/* Start BMP280 forced conversion (if we're in SLEEP_MODE) */
	bmp280.startForcedConversion();

	/* Check if the measurement is complete */

	if (bmp280.getMeasurements(*temperature, *pressure, *altitude))
		return(1);

	return(0);
}
