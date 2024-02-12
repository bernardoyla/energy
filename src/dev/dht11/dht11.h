/** 
 * @file dht11.h
 * 
 * 
 * 
 * 
 */

#ifndef DHT11_H
#define DHT11_H

#include "dht/DHT.h"

#define DHTPIN 2     // Digital pin connected to the DHT sensor

#define DHTTYPE DHT11   // DHT 11
//#define DHTTYPE DHT22   // DHT 22  (AM2302), AM2321
//#define DHTTYPE DHT21   // DHT 21 (AM2301)

void dht11_init(void);


#endif /* DHT11_H */ 