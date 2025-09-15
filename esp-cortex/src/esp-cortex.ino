#include "WiFi.h"
#include "WiFiUdp.h"
#include "parse_console.h"
#include "nvs.h"
#include "PPP.h"
#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>


#define NRST_PIN 2
#define LED_PIN 32
#define BOOT_PIN 22


void setup() {
  // put your setup code here, to run once:
  // pinMode(NRST_PIN, INPUT); //input for HI-Z
  pinMode(NRST_PIN, OUTPUT);
  digitalWrite(NRST_PIN, 1);

  pinMode(BOOT_PIN, OUTPUT); //input for HI-Z
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, 1);
  digitalWrite(BOOT_PIN, 0);
  init_prefs(&preferences, &gl_prefs);

  Serial.begin(2000000);  //this can stay 2mbps. WHICH IS CRAZY omg
  Serial2.begin(2000000, SERIAL_8N1, 16, 17);  //once you have a working system, try pushing this way higher (2MBPS is supported by ESP32!!)

}

uint32_t ledts = 0; 
uint8_t led_state = 0;

void loop() 
{
	uint32_t tick = millis();
	if(tick - ledts > 250)
	{
		ledts = tick;
		digitalWrite(LED_PIN, led_state);
		led_state = ~led_state & 1;
	}
	if(Serial.available())
	{
		int v = Serial.read();
		Serial2.write((uint8_t)v);
	}
	if(Serial2.available())
	{
		int v = Serial2.read();
		Serial.write((uint8_t)v);
	}
}
