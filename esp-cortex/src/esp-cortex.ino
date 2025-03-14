#include "WiFi.h"
#include "WiFiUdp.h"
#include "parse_console.h"
#include "nvs.h"
#include "PPP.h"
#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include "esp_wifi.h"
#include "tcpip_adapter.h"  // For tcpip_adapter functions


#define NRST_PIN 2
#define LED_PIN 32
#define BOOT_PIN 22

/*
Tested with 3.0.3 esp32 arduino board package by Espressif Systems.
*/

#define IPV4_ADDR_ANY   0x00000000UL

enum {PERIOD_CONNECTED = 50, PERIOD_DISCONNECTED = 1000};

WiFiUDP udp;


IPAddress local_ip(192,168,33,1);
IPAddress gateway(192,168,33,1);
IPAddress subnet(255,255,255,0);

void printConnectedClients() 
{
  // Create a structure to hold the list of connected stations (by MAC address)
  wifi_sta_list_t staList;
  // Structure to hold the client info including IP addresses
  tcpip_adapter_sta_list_t adapterList;

  // Get the list of connected stations (their MAC addresses)
  esp_wifi_ap_get_sta_list(&staList);

  // Get the IP addresses for the connected stations
  tcpip_adapter_get_sta_list(&staList, &adapterList);

  Serial.print("Number of connected clients: ");
  Serial.println(adapterList.num);

  // Loop through each connected client and print its IP address
  for (int i = 0; i < adapterList.num; i++) {
    tcpip_adapter_sta_info_t station = adapterList.sta[i];
    Serial.print("Client ");
    Serial.print(i + 1);
    Serial.print(" - IP Address: ");
    Serial.println(ip4addr_ntoa((const ip4_addr_t *)&station.ip));
  }
}

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
//   Serial2.begin(2000000, SERIAL_8N1, 16, 17);  //once you have a working system, try pushing this way higher (2MBPS is supported by ESP32!!)

	/*Begin wifi connection*/
	WiFi.softAPConfig(local_ip, gateway, subnet);
	WiFi.softAP(gl_prefs.ssid, gl_prefs.password);
	Serial.println("Access Point Started");
	Serial.print("AP IP address: ");
  	Serial.println(WiFi.softAPIP());
//   WiFi.mode(WIFI_AP);  
//   WiFi.begin((const char *)gl_prefs.ssid, (const char *)gl_prefs.password);
}


int cmd_match(const char * in, const char * cmd)
{
  int i = 0;
  for(i = 0; cmd[i] != '\0'; i++)
  {
    if(in[i] == '\0')
      return -1;
    if(in[i] != cmd[i])
      return -1;   
  }
  return i;
}

#define UNSTUFFING_BUFFER_SIZE 256
#define PAYLOAD_BUFFER_SIZE ((UNSTUFFING_BUFFER_SIZE - 2)/2)  //max cap based on unstuffing buffer size
uint8_t gl_unstuffing_buffer[UNSTUFFING_BUFFER_SIZE] = {0};
uint8_t gl_pld_buffer[PAYLOAD_BUFFER_SIZE] = {0};


void loop() 
{
  // put your main code here, to run repeatedly:
  uint32_t led_ts = 0;
  uint8_t led_state = 1;
  uint8_t stm32_enabled = 0;
  uint32_t blink_per = PERIOD_DISCONNECTED;
  uint8_t udp_pkt_buf[256] = {0};
  int ppp_stuffing_bidx = 0;

  while(1)
  {
    uint32_t ts = millis();
	
    get_console_lines();
    if(gl_console_cmd.parsed == 0)
    {
      uint8_t match = 0;
      uint8_t save = 0;
      int cmp = -1;

      //////////////////////////////////////////////////////////////////////////////////////////////////////
      cmp = strcmp((const char *)gl_console_cmd.buf,"ipconfig\r");
      if(cmp == 0)
      {
        match = 1;
		Serial.println(WiFi.softAPIP());
		Serial.println(WiFi.soft)		
      }
      
      //////////////////////////////////////////////////////////////////////////////////////////////////////
      cmp = cmd_match((const char *)gl_console_cmd.buf,"setssid ");
      if(cmp > 0)
      {
        match = 1;
        const char * arg = (const char *)(&gl_console_cmd.buf[cmp]);
        /*Set the ssid*/
        for(int i = 0; i < WIFI_MAX_SSID_LEN; i++)
        {
          gl_prefs.ssid[i] = '\0';
        }
        for(int i = 0; arg[i] != '\0'; i++)
        {
          if(arg[i] != '\r' && arg[i] != '\n')  //copy non carriage return characters
          {
            gl_prefs.ssid[i] = arg[i];
          }
        }
        Serial.printf("Changing ssid to: %s\r\n", gl_prefs.ssid);
        save = 1;
      }
      //////////////////////////////////////////////////////////////////////////////////////////////////////
      cmp = cmd_match((const char *)gl_console_cmd.buf,"setname ");
      if(cmp > 0)
      {
        match = 1;
        const char * arg = (const char *)(&gl_console_cmd.buf[cmp]);
        /*Set the ssid*/
        for(int i = 0; i < DEVICE_NAME_LEN; i++)
        {
          gl_prefs.name[i] = '\0';
        }
        for(int i = 0; arg[i] != '\0'; i++)
        {
          if(arg[i] != '\r' && arg[i] != '\n')  //copy non carriage return characters
          {
            gl_prefs.name[i] = arg[i];
          }
        }
        Serial.printf("Changing device name to: %s\r\n", gl_prefs.name);
        save = 1;
      }

      //////////////////////////////////////////////////////////////////////////////////////////////////////
      cmp = cmd_match((const char *)gl_console_cmd.buf,"setpwd ");
      if(cmp > 0)
      {
        match = 1;
        const char * arg = (const char *)(&gl_console_cmd.buf[cmp]);
        /*Set the password*/
        for(int i = 0; i < WIFI_MAX_PWD_LEN; i++)
        {
          gl_prefs.password[i] = '\0';
        }
        for(int i = 0; arg[i] != '\0'; i++)
        {
          if(arg[i] != '\r' && arg[i] != '\n')
          {
            gl_prefs.password[i] = arg[i];
          }
        }
        Serial.printf("Changing pwd to: %s\r\n",gl_prefs.password);
        save = 1;
      }

      //////////////////////////////////////////////////////////////////////////////////////////////////////
      cmp = cmd_match((const char *)gl_console_cmd.buf,"readcred");
      if(cmp > 0)
      {
        match = 1;
        Serial.printf("SSID: \'");
        for(int i = 0; gl_prefs.ssid[i] != 0; i++)
        {
          char c = gl_prefs.ssid[i];
          if(c >= 0x1f && c <= 0x7E)
          {
            Serial.printf("%c",c);
          }
          else
          {
            Serial.printf("%0.2X",c);
          }
        }
        Serial.printf("\'\r\n");

        Serial.printf("Password: \'");
        for(int i = 0; gl_prefs.password[i] != 0; i++)
        {
          char c = gl_prefs.password[i];
          if(c >= 0x1f && c <= 0x7E)
          {
            Serial.printf("%c",c);
          }
          else
          {
            Serial.printf("%0.2X",c);
          }
        }
        Serial.printf("\'\r\n");
      }

      //////////////////////////////////////////////////////////////////////////////////////////////////////
      cmp = cmd_match((const char *)gl_console_cmd.buf,"restart\r");
      if(cmp > 0)
      {
        Serial.printf("restarting chip...\r\n");
        ESP.restart();
      }

      /********************************Parsing over, cleanup*************************************/
      if(match == 0)
      {
        Serial.printf("Failed to parse: %s\r\n", gl_console_cmd.buf);
      }
      if(save != 0)
      {
        int nb = preferences.putBytes("settings", &gl_prefs, sizeof(nvs_settings_t));
        Serial.printf("Saved %d bytes\r\n", nb);
      }

      for(int i = 0; i < BUFFER_SIZE; i++)
      {
        gl_console_cmd.buf[i] = 0; 
      }
      gl_console_cmd.parsed = 1;
    }


    if(WiFi.status() != WL_CONNECTED)
    {
      blink_per = PERIOD_DISCONNECTED;
    }
    else
    {
      blink_per = PERIOD_CONNECTED;
    }

    if(ts - led_ts > blink_per)
    {
      led_ts = ts;
      digitalWrite(LED_PIN, led_state);
      led_state = ~led_state & 1;
    }
  }
}
