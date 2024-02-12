/** 
 * 
 * 
 * 
 */


#include "dht11.h"

static DHT dht(DHTPIN, DHTTYPE);


void dht11_init(void) {
    dht.begin();
}

void dht11_read(float *h, float *t, float *f, float *hic, float *hif) {
    *h = dht.readHumidity();
    *t = dht.readTemperature();
    *f = dht.readTemperature(true);
    *hic = dht.computeHeatIndex(*t, *h, false);
    *hif = dht.computeHeatIndex(*f, *h);
}

  // Reading temperature or humidity takes about 250 milliseconds!
  // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
  float h = dht.readHumidity();
  // Read temperature as Celsius (the default)
  float t = dht.readTemperature();
  // Read temperature as Fahrenheit (isFahrenheit = true)
  float f = dht.readTemperature(true);

  // Check if any reads failed and exit early (to try again).
  if (isnan(h) || isnan(t) || isnan(f)) {
    Serial.println(F("Failed to read from DHT sensor!"));
    return;
  }

  // Compute heat index in Fahrenheit (the default)
  float hif = dht.computeHeatIndex(f, h);
  // Compute heat index in Celsius (isFahreheit = false)
  float hic = dht.computeHeatIndex(t, h, false);