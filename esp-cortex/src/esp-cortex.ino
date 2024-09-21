#include "WiFi.h"
#include "WiFiUdp.h"
#include "parse_console.h"
#include "nvs.h"
#include "PPP.h"
#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include "MSP.h"
#include "MSP_OSD.h"
#include "OSD_positions_config.h"


#define NRST_PIN 2
#define LED_PIN 32
#define BOOT_PIN 22

/*
Tested with 3.0.3 esp32 arduino board package by Espressif Systems.
*/

#define IPV4_ADDR_ANY	 0x00000000UL

enum {PERIOD_CONNECTED = 50, PERIOD_DISCONNECTED = 1000};

WiFiUDP udp;

#define ANALOG_IN								A0		// Voltage Read pin (notice this is now Pin 0, instead of Pin 1)
#define VOLT_DIVIDER						 48		// Set to 1024/full scale voltage
#define DEBUG													//uncomment to see diagnostics from USB serial

#define FC_FIRMWARE_NAME					"Betaflight"
#define FC_FIRMWARE_IDENTIFIER		"BTFL"

HardwareSerial &mspSerial = Serial2;
MSP msp;

// Arm Logic
uint32_t unarmedMillis = 3000;	 // number of milliseconds to wait before arming, after AU is initialized. Recommend at least 3000 (3 seconds).

//Voltage and Battery Reading
float averageVoltage = 0;
int sampleVoltageCount = 0;
int lastCount = 0;

boolean lightOn = true;

uint8_t *messageID;
void *payload;
uint8_t maxSize;
uint8_t *recvSize;

//Other
char fcVariant[5] = "BTFL";
char craftname[15] = "Craft";
uint32_t previousMillis_MSP = 0;
uint32_t activityDetectedMillis_MSP = 0;
bool activityDetected = false;
const uint32_t next_interval_MSP = 100;
uint32_t general_counter = next_interval_MSP;
uint32_t custom_mode = 0;	//flight mode
uint8_t vbat = 0;
uint32_t flightModeFlags = 0x00000002;
uint8_t batteryCellCount = 3;
uint32_t previousFlightMode = custom_mode;

msp_battery_state_t battery_state = { 0 };
msp_name_t name = { 0 };
msp_fc_version_t fc_version = { 0 };
msp_fc_variant_t fc_variant = { 0 };
//msp_status_BF_t status_BF = {0};
msp_status_DJI_t status_DJI = { 0 };
msp_analog_t analog = { 0 };


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

	Serial2.begin(115200, SERIAL_8N1, 16, 17);	//once you have a working system, try pushing this way higher (2MBPS is supported by ESP32!!)
	while (!Serial2);

	msp.begin(mspSerial);
	status_DJI.cycleTime = 0x0080;
	status_DJI.i2cErrorCounter = 0;
	status_DJI.sensor = 0x23;
	status_DJI.configProfileIndex = 0;
	status_DJI.averageSystemLoadPercent = 7;
	status_DJI.accCalibrationAxisFlags = 0;
	status_DJI.DJI_ARMING_DISABLE_FLAGS_COUNT = 20;
	status_DJI.djiPackArmingDisabledFlags = (1 << 24);
	flightModeFlags = 0x00000002;
	
	Serial2.end();

	Serial.begin(2000000);	//this can stay 2mbps. WHICH IS CRAZY omg
	Serial2.begin(2000000, SERIAL_8N1, 16, 17);	//once you have a working system, try pushing this way higher (2MBPS is supported by ESP32!!)

	int connected = 0;
	Serial.printf("\r\n\r\n Trying \'%s\' \'%s\'\r\n",gl_prefs.ssid, gl_prefs.password);
	/*Begin wifi connection*/
	WiFi.mode(WIFI_STA);	
	WiFi.begin((const char *)gl_prefs.ssid, (const char *)gl_prefs.password);
	//connected = WiFi.waitForConnectResult();
	if (connected != WL_CONNECTED) {
		Serial.printf("Connection to network %s failed for an unknown reason\r\n", (const char *)gl_prefs.ssid);
	}

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
#define PAYLOAD_BUFFER_SIZE ((UNSTUFFING_BUFFER_SIZE - 2)/2)	//max cap based on unstuffing buffer size
uint8_t gl_unstuffing_buffer[UNSTUFFING_BUFFER_SIZE] = {0};
uint8_t gl_pld_buffer[PAYLOAD_BUFFER_SIZE] = {0};


void arm_the_stupid_thing(void)
{
	
  if (!activityDetected) {
    #ifdef DEBUG
      Serial.println("Waiting for AU...");
    #endif
    // digitalWrite(LED_BUILTIN, LOW);

    // Wait for Air Unit to send data
    while(!msp.activityDetected()) {    
    };
    activityDetected = true;
    activityDetectedMillis_MSP = millis();    
    #ifdef DEBUG
      Serial.println("AU Detected, waiting (unarmedMillis) time till arm");
    #endif
  }

//   digitalWrite(LED_BUILTIN, HIGH);
  
  uint32_t currentMillis_MSP = millis();

  if ((uint32_t)(currentMillis_MSP - previousMillis_MSP) >= next_interval_MSP) {
    previousMillis_MSP = currentMillis_MSP;

    if (general_counter % 300 == 0) {  // update the altitude and voltage values every 300ms
      getVoltageSample();
      lightOn = !lightOn;
    }

    if (currentMillis_MSP < (activityDetectedMillis_MSP + unarmedMillis)) {
      set_flight_mode_flags(false);
    } else {
      set_flight_mode_flags(true);
    }

#ifdef DEBUG
    //debugPrint();
#endif
    send_msp_to_airunit();
    general_counter += next_interval_MSP;
  }
  if (custom_mode != previousFlightMode) {
    previousFlightMode = custom_mode;
    display_flight_mode();
  }

  if (batteryCellCount == 0 && vbat > 0) {
    set_battery_cells_number();
  }

  //display flight mode every 10s
  if (general_counter % 10000 == 0) {
    display_flight_mode();
  }
}

void loop() 
{
	while(1)
	{
		arm_the_stupid_thing();
	}
	
	IPAddress server_address((uint32_t)IPV4_ADDR_ANY); //note: may want to change to our local IP, to support multiple devices on the network
	udp.begin(server_address, gl_prefs.port);

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

		int len = udp.parsePacket();
		if(len != 0)
		{
			int len = udp.read(udp_pkt_buf,255);
			
			int cmp = -1;
			cmp = cmd_match((const char *)udp_pkt_buf,"WHO_GOES_THERE");
			if(cmp > 0)
			{
				int len = strlen(gl_prefs.name);
				udp.beginPacket(udp.remoteIP(),udp.remotePort()+gl_prefs.reply_offset);
				udp.write((uint8_t*)gl_prefs.name,len);
				udp.endPacket();
			}
			
			Serial2.write(udp_pkt_buf,len);
			for(int i = 0; i < len; i++)
				udp_pkt_buf[i] = 0;
		}

		uint8_t serial_pkt_sent = 0;
		while(Serial2.available())
		{
			 uint8_t new_byte = Serial2.read();
			 int pld_len = parse_PPP_stream(new_byte, gl_pld_buffer, PAYLOAD_BUFFER_SIZE, gl_unstuffing_buffer, UNSTUFFING_BUFFER_SIZE, &ppp_stuffing_bidx);
			 if(pld_len != 0)
			 {
					if(gl_prefs.en_fixed_target == 0)
					{
						udp.beginPacket(udp.remoteIP(), udp.remotePort()+gl_prefs.reply_offset);
					}
					else
					{
						IPAddress remote_ip(gl_prefs.remote_target_ip);
						udp.beginPacket(remote_ip, gl_prefs.port+gl_prefs.reply_offset);
					}
					udp.write((uint8_t*)gl_pld_buffer, pld_len);
					udp.endPacket();			
					serial_pkt_sent = 1;
			 }
		}

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
				if(WiFi.status() == WL_CONNECTED)
				{
					Serial.printf("Connected to: %s\r\n", gl_prefs.ssid);
				}
				else
				{
					Serial.printf("Not connected to: %s\r\n", gl_prefs.ssid);
				}
				Serial.printf("UDP server on port: %d\r\n", gl_prefs.port);
				Serial.printf("Server Response Offset: %d\r\n", gl_prefs.reply_offset);
				Serial.printf("IP address is: %s\r\n", WiFi.localIP().toString().c_str());
		
			}

			//////////////////////////////////////////////////////////////////////////////////////////////////////
			cmp = cmd_match((const char *)gl_console_cmd.buf,"udpconfig\r");
			if(cmp > 0)
			{
				match = 1;
				Serial.printf("UDP server on port: %d\r\n", gl_prefs.port);
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
					if(arg[i] != '\r' && arg[i] != '\n')	//copy non carriage return characters
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
					if(arg[i] != '\r' && arg[i] != '\n')	//copy non carriage return characters
					{
						gl_prefs.name[i] = arg[i];
					}
				}
				Serial.printf("Changing device name to: %s\r\n", gl_prefs.name);
				save = 1;
			}
			//////////////////////////////////////////////////////////////////////////////////////////////////////
			cmp = cmd_match((const char *)gl_console_cmd.buf,"settargetip ");
			if(cmp > 0)
			{
				match = 1;
				const char * arg = (const char *)(&gl_console_cmd.buf[cmp]);
				char copy[15] = {0};
				Serial.printf("Raw str arg: ");
				for(int i = 0; arg[i] != 0 && arg[i] != '\r' && arg[i] != '\n'; i++)
				{
					copy[i] = arg[i];
					Serial.printf("%0.2X",arg[i]);
				}
				Serial.printf(": %s\r\n", copy);
				IPAddress addr;
				if(addr.fromString((const char *)copy) == true)
					Serial.printf("Parsed IP address successfully\r\n");
				else
					Serial.printf("Invalid IP string entered\r\n");
				gl_prefs.remote_target_ip = (uint32_t)addr;
				Serial.printf("%X\r\n",gl_prefs.remote_target_ip);
				IPAddress parseconfirm(gl_prefs.remote_target_ip);
				Serial.printf("Target IP: %s\r\n", parseconfirm.toString().c_str());
				save = 1;
			}
			//////////////////////////////////////////////////////////////////////////////////////////////////////
			/*
				Usage:
					fixedtarget enable
					fixedtarget disable
			 */
			cmp = cmd_match((const char *)gl_console_cmd.buf,"fixedtarget ");
			if(cmp > 0)
			{
				match = 1;
				const char * arg = (const char *)(&gl_console_cmd.buf[cmp]);
				int argcmp = cmd_match( arg, "enable");
				if(argcmp > 0)
				{
					gl_prefs.en_fixed_target = 1;
					IPAddress parseconfirm(gl_prefs.remote_target_ip);
					Serial.printf("Enabling Fixed Target: %s\r\n", parseconfirm.toString().c_str());
				}
				argcmp = cmd_match( arg, "disable");
				if(argcmp > 0)
				{
					gl_prefs.en_fixed_target = 0;
					Serial.printf("Disabling Fixed Target\r\n");
				}
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
			cmp = cmd_match((const char *)gl_console_cmd.buf,"setport ");
			if(cmp > 0)
			{
				match = 1;
				const char * arg = (const char *)(&gl_console_cmd.buf[cmp]);
				char * tmp;
				int port = strtol(arg, &tmp, 10);
				Serial.printf("Changing port to: %d\r\n",port);
				/*Set the port*/
				gl_prefs.port = port;
				save = 1;
			}
		
			//////////////////////////////////////////////////////////////////////////////////////////////////////
			cmp = cmd_match((const char *)gl_console_cmd.buf,"setTXoff ");
			if(cmp > 0)
			{
				match = 1;
				const char * arg = (const char *)(&gl_console_cmd.buf[cmp]);
				char * tmp;
				int offset = strtol(arg, &tmp, 10);
				Serial.printf("Setting port offset to: %d\r\n",offset);
				gl_prefs.reply_offset = offset;
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
			cmp = cmd_match((const char *)gl_console_cmd.buf,"reconnect\r");
			if(cmp > 0)
			{
				match = 1;
				Serial.printf("restarting wifi connection...\r\n");
				/*Try to connect using modified ssid and password. for convenience, as a restart will fulfil the same functionality*/
				WiFi.disconnect();
				WiFi.begin((const char *)gl_prefs.ssid,(const char *)gl_prefs.password);
				udp.begin(server_address, gl_prefs.port);
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
			if(WiFi.status() != WL_CONNECTED)
			{
				//WiFi.reconnect();
				WiFi.disconnect();
				WiFi.begin((const char *)gl_prefs.ssid,(const char *)gl_prefs.password);
				udp.begin(server_address, gl_prefs.port);

			}

		}
	}
}


msp_osd_config_t msp_osd_config = { 0 };

void send_osd_config() {

#ifdef IMPERIAL_UNITS
  msp_osd_config.units = 0;
#else
  msp_osd_config.units = 1;
#endif

  msp_osd_config.osd_item_count = 56;
  msp_osd_config.osd_stat_count = 24;
  msp_osd_config.osd_timer_count = 2;
  msp_osd_config.osd_warning_count = 16;  // 16
  msp_osd_config.osd_profile_count = 1;   // 1
  msp_osd_config.osdprofileindex = 1;     // 1
  msp_osd_config.overlay_radio_mode = 0;  //  0

  msp_osd_config.osd_rssi_value_pos = osd_rssi_value_pos;
  msp_osd_config.osd_main_batt_voltage_pos = osd_main_batt_voltage_pos;
  msp_osd_config.osd_crosshairs_pos = osd_crosshairs_pos;
  msp_osd_config.osd_artificial_horizon_pos = osd_artificial_horizon_pos;
  msp_osd_config.osd_horizon_sidebars_pos = osd_horizon_sidebars_pos;
  msp_osd_config.osd_item_timer_1_pos = osd_item_timer_1_pos;
  msp_osd_config.osd_item_timer_2_pos = osd_item_timer_2_pos;
  msp_osd_config.osd_flymode_pos = osd_flymode_pos;
  msp_osd_config.osd_craft_name_pos = osd_craft_name_pos;
  msp_osd_config.osd_throttle_pos_pos = osd_throttle_pos_pos;
  msp_osd_config.osd_vtx_channel_pos = osd_vtx_channel_pos;
  msp_osd_config.osd_current_draw_pos = osd_current_draw_pos;
  msp_osd_config.osd_mah_drawn_pos = osd_mah_drawn_pos;
  msp_osd_config.osd_gps_speed_pos = osd_gps_speed_pos;
  msp_osd_config.osd_gps_sats_pos = osd_gps_sats_pos;
  msp_osd_config.osd_altitude_pos = osd_altitude_pos;
  msp_osd_config.osd_roll_pids_pos = osd_roll_pids_pos;
  msp_osd_config.osd_pitch_pids_pos = osd_pitch_pids_pos;
  msp_osd_config.osd_yaw_pids_pos = osd_yaw_pids_pos;
  msp_osd_config.osd_power_pos = osd_power_pos;
  msp_osd_config.osd_pidrate_profile_pos = osd_pidrate_profile_pos;
  msp_osd_config.osd_warnings_pos = osd_warnings_pos;
  msp_osd_config.osd_avg_cell_voltage_pos = osd_avg_cell_voltage_pos;
  msp_osd_config.osd_gps_lon_pos = osd_gps_lon_pos;
  msp_osd_config.osd_gps_lat_pos = osd_gps_lat_pos;
  msp_osd_config.osd_debug_pos = osd_debug_pos;
  msp_osd_config.osd_pitch_angle_pos = osd_pitch_angle_pos;
  msp_osd_config.osd_roll_angle_pos = osd_roll_angle_pos;
  msp_osd_config.osd_main_batt_usage_pos = osd_main_batt_usage_pos;
  msp_osd_config.osd_disarmed_pos = osd_disarmed_pos;
  msp_osd_config.osd_home_dir_pos = osd_home_dir_pos;
  msp_osd_config.osd_home_dist_pos = osd_home_dist_pos;
  msp_osd_config.osd_numerical_heading_pos = osd_numerical_heading_pos;
  msp_osd_config.osd_numerical_vario_pos = osd_numerical_vario_pos;
  msp_osd_config.osd_compass_bar_pos = osd_compass_bar_pos;
  msp_osd_config.osd_esc_tmp_pos = osd_esc_tmp_pos;
  msp_osd_config.osd_esc_rpm_pos = osd_esc_rpm_pos;
  msp_osd_config.osd_remaining_time_estimate_pos = osd_remaining_time_estimate_pos;
  msp_osd_config.osd_rtc_datetime_pos = osd_rtc_datetime_pos;
  msp_osd_config.osd_adjustment_range_pos = osd_adjustment_range_pos;
  msp_osd_config.osd_core_temperature_pos = osd_core_temperature_pos;
  msp_osd_config.osd_anti_gravity_pos = osd_anti_gravity_pos;
  msp_osd_config.osd_g_force_pos = osd_g_force_pos;
  msp_osd_config.osd_motor_diag_pos = osd_motor_diag_pos;
  msp_osd_config.osd_log_status_pos = osd_log_status_pos;
  msp_osd_config.osd_flip_arrow_pos = osd_flip_arrow_pos;
  msp_osd_config.osd_link_quality_pos = osd_link_quality_pos;
  msp_osd_config.osd_flight_dist_pos = osd_flight_dist_pos;
  msp_osd_config.osd_stick_overlay_left_pos = osd_stick_overlay_left_pos;
  msp_osd_config.osd_stick_overlay_right_pos = osd_stick_overlay_right_pos;
  msp_osd_config.osd_display_name_pos = osd_display_name_pos;
  msp_osd_config.osd_esc_rpm_freq_pos = osd_esc_rpm_freq_pos;
  msp_osd_config.osd_rate_profile_name_pos = osd_rate_profile_name_pos;
  msp_osd_config.osd_pid_profile_name_pos = osd_pid_profile_name_pos;
  msp_osd_config.osd_profile_name_pos = osd_profile_name_pos;
  msp_osd_config.osd_rssi_dbm_value_pos = osd_rssi_dbm_value_pos;
  msp_osd_config.osd_rc_channels_pos = osd_rc_channels_pos;

  msp.send(MSP_OSD_CONFIG, &msp_osd_config, sizeof(msp_osd_config));
}


void invert_pos(uint16_t *pos1, uint16_t *pos2) {
  uint16_t tmp_pos = *pos1;
  *pos1 = *pos2;
  *pos2 = tmp_pos;
}

void set_flight_mode_flags(bool arm) {
    if ((flightModeFlags == 0x00000002) && arm) {
      flightModeFlags = 0x00000003;    // arm
      #ifdef DEBUG
        Serial.println("ARMING");
      #endif
    } else if ((flightModeFlags == 0x00000003) && !arm) {        
      flightModeFlags = 0x00000002;    // disarm 
      #ifdef DEBUG
        Serial.println("DISARMING");
      #endif
    }
}
void show_text(char (*text)[15]) {
  memcpy(craftname, *text, sizeof(craftname));
}

void display_flight_mode() {
  show_text(&craftname);
}

void send_msp_to_airunit() {
  
  //MSP_FC_VARIANT
  memcpy(fc_variant.flightControlIdentifier, fcVariant, sizeof(fcVariant));
  msp.send(MSP_FC_VARIANT, &fc_variant, sizeof(fc_variant));

  //MSP_FC_VERSION
  fc_version.versionMajor = 4;
  fc_version.versionMinor = 1;
  fc_version.versionPatchLevel = 1;
  msp.send(MSP_FC_VERSION, &fc_version, sizeof(fc_version));

  //MSP_NAME
  memcpy(name.craft_name, craftname, sizeof(craftname));
  msp.send(MSP_NAME, &name, sizeof(name));

  //MSP_STATUS
  status_DJI.flightModeFlags = flightModeFlags;
  status_DJI.armingFlags = 0x0303;
  msp.send(MSP_STATUS_EX, &status_DJI, sizeof(status_DJI));
  status_DJI.armingFlags = 0x0000;
  msp.send(MSP_STATUS, &status_DJI, sizeof(status_DJI));

  //MSP_BATTERY_STATE
  battery_state.amperage = 0;
  battery_state.batteryVoltage = vbat * 10;
  battery_state.mAhDrawn = 0;
  battery_state.batteryCellCount = batteryCellCount;
  battery_state.batteryCapacity = 0;
  battery_state.batteryState = 0;
  battery_state.legacyBatteryVoltage = vbat;
  msp.send(MSP_BATTERY_STATE, &battery_state, sizeof(battery_state));
 
  //MSP_OSD_CONFIG
  send_osd_config();
}

void set_battery_cells_number() {
  if (vbat < 43) batteryCellCount = 1;
  else if (vbat < 85) batteryCellCount = 2;
  else if (vbat < 127) batteryCellCount = 3;
  else if (vbat < 169) batteryCellCount = 4;
  else if (vbat < 211) batteryCellCount = 5;
  else if (vbat < 255) batteryCellCount = 6;
}


/*Hard load 12VDC*/
void getVoltageSample() {
  vbat = 120;	//analogRead(ANALOG_IN)*10/VOLT_DIVIDER;
}
