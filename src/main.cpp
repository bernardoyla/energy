// Example testing sketch for various DHT humidity/temperature sensors
// Written by ladyada, public domain

// REQUIRES the following Arduino libraries:
// - DHT Sensor Library: https://github.com/adafruit/DHT-sensor-library
// - Adafruit Unified Sensor Lib: https://github.com/adafruit/Adafruit_Sensor

#include "dev/dht11/dht11.h"




void setup() {
	Serial.begin(9600);
	Serial.println(F("DHTxx test!"));

	dht11_init();

}

void loop() {
  // Wait a few seconds between measurements.
  	delay(2000);

	float h, t , f = 0.0;

  	Serial.print(F("Humidity: "));
  	Serial.print(h);
  	Serial.print(F("%  Temperature: "));
  	Serial.print(t);
  	Serial.print(F("°C "));
  	Serial.print(f);
  	Serial.print(F("°F  Heat index: "));
  	Serial.print(hic);
  	Serial.print(F("°C "));
  	Serial.print(hif);
  	Serial.println(F("°F"));
}
