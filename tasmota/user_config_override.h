#ifndef _USER_CONFIG_OVERRIDE_H_
#define _USER_CONFIG_OVERRIDE_H_

/*****************************************************************************************************\
 * USAGE:
 *   To modify the stock configuration without changing the my_user_config.h file:
 *   (1) copy this file to "user_config_override.h" (It will be ignored by Git)
 *   (2) define your own settings below
 *
 ******************************************************************************************************
 * ATTENTION:
 *   - Changes to SECTION1 PARAMETER defines will only override flash settings if you change define CFG_HOLDER.
 *   - Expect compiler warnings when no ifdef/undef/endif sequence is used.
 *   - You still need to update my_user_config.h for major define USE_MQTT_TLS.
 *   - All parameters can be persistent changed online using commands via MQTT, WebConsole or Serial.
\*****************************************************************************************************/

// Examples :

// -- Master parameter control --------------------
#undef  CFG_HOLDER
#define CFG_HOLDER        4617                   // [Reset 1] Change this value to load SECTION1 configuration parameters to flash

// -- Setup your own Wifi settings  ---------------
#undef  STA_SSID1
#ifdef KARLSRUHE
#define STA_SSID1         "UPC4654669"             // [Ssid1] Wifi SSID
#undef  STA_PASS1
#define STA_PASS1         "e8tpJsjzzypa"     // [Password1] Wifi password
#else

#define STA_SSID1         "Palauzovi"             // [Ssid1] Wifi SSID

#undef  STA_PASS1
#define STA_PASS1         "pluwane9"     // [Password1] Wifi password

#undef  STA_SSID2
#define STA_SSID2         "Palauzovi2"             // [Ssid2] Wifi SSID

#undef  STA_PASS2
#define STA_PASS2         "pluwane9"     // [Password2] Wifi password
#endif

// -- Setup your own MQTT settings  ---------------
#ifdef KARLSURHE
#define USE_MQTT_TLS                             // Use TLS for MQTT connection (+34.5k code, +7.0k mem and +4.8k additional during connection handshake)
#undef  MQTT_TLS_ENABLED
#define MQTT_TLS_ENABLED       true              // [SetOption103] Enable TLS mode (requires TLS version)

#define BR_MAX_EC_SIZE 384
#define USE_MQTT_TLS_CA_CERT
#define USE_MQTT_TLS_FORCE_EC_CIPHER
#endif

#undef  MQTT_HOST
#ifdef KARLSRUHE
#define MQTT_HOST         "95.43.114.153"
#undef  MQTT_PORT
#define MQTT_PORT         8883
#else
#define MQTT_HOST         "192.168.0.176" // [MqttHost]
#endif

#undef  MQTT_USER
#define MQTT_USER         "a"         // [MqttUser] Optional user

#undef  MQTT_PASS
#define MQTT_PASS         "123"         // [MqttPassword] Optional password

#ifndef SUB_PREFIX
#define SUB_PREFIX             "cmnd"            // [Prefix1] Tasmota devices subscribe to %prefix%/%topic% being SUB_PREFIX/MQTT_TOPIC and SUB_PREFIX/MQTT_GRPTOPIC
#endif

// You might even pass some parameters from the command line ----------------------------
// Ie:  export PLATFORMIO_BUILD_FLAGS='-DUSE_CONFIG_OVERRIDE -DMY_IP="192.168.1.99" -DMY_GW="192.168.1.1" -DMY_DNS="192.168.1.1"'
#ifdef MY_IP
#define MY_GW	"192.168.0.1"
#define MY_DNS	"192.168.0.1"
#define MY_DNS2	"8.8.8.8"
#undef  WIFI_IP_ADDRESS
#define WIFI_IP_ADDRESS     MY_IP                // Set to 0.0.0.0 for using DHCP or enter a static IP address
#endif

#ifdef MY_GW
#undef  WIFI_GATEWAY
#define WIFI_GATEWAY        MY_GW                // if not using DHCP set Gateway IP address
#endif

#ifdef MY_DNS
#undef  WIFI_DNS
#define WIFI_DNS            MY_DNS               // If not using DHCP set DNS IP address (might be equal to WIFI_GATEWAY)
#endif

#ifdef MY_DNS2
#undef  WIFI_DNS2
#define WIFI_DNS2           MY_DNS2              // If not using DHCP set DNS IP address (might be equal to WIFI_GATEWAY)
#endif

// !!! Remember that your changes GOES AT THE BOTTOM OF THIS FILE right before the last #endif !!!
// */
#undef MDNS_ENABLED
#define MDNS_ENABLED           true             // [SetOption55] Use mDNS (false = Disable, true = Enable)
#undef NTP_SERVER1
#undef NTP_SERVER2
#undef NTP_SERVER3
#define NTP_SERVER1      "0.bg.pool.ntp.org"        // [NtpServer1] Select first NTP server by name or IP address (135.125.104.101, 2001:418:3ff::53)
#define NTP_SERVER2      "1.bg.pool.ntp.org" // [NtpServer2] Select second NTP server by name or IP address (192.36.143.134, 2a00:2381:19c6::100)
#define NTP_SERVER3      "2.bg.pool.ntp.org"     // [NtpServer3] Select third NTP server by name or IP address (46.249.42.13, 2603:c022:c003:c900::4)
#undef TIME_DST_OFFSET
#undef TIME_STD_OFFSET
#define TIME_DST_OFFSET        +180              // Offset from UTC in minutes (-780 to +780)
#define TIME_STD_OFFSET        +120               // Offset from UTC in minutes (-780 to +780)

// -- Location ------------------------------------
#undef LATITUDE
#define LATITUDE               43.3             // [Latitude] Your location to be used with sunrise and sunset
#undef LONGITUDE
#define LONGITUDE              27.9147          // [Longitude] Your location to be used with sunrise and sunset
#undef APP_TIMEZONE
#define APP_TIMEZONE           99                // [Timezone] +1 hour (Amsterdam) (-13 .. 14 = hours from UTC, 99 = use TIME_DST/TIME_STD)
#define MY_LANGUAGE            bg_BG           // Bulgarian in Bulgaria
#define USE_ARDUINO_OTA                          // Add optional support for Arduino OTA with ESP8266 (+13k code)
// #define USE_IPV6                                 // Enable IPv6 support (if the underlying esp-idf is also configured to support it)
                                                 // Code size increase: ESP8266: +34.5kb
                                                 // Enabled by default on ESP32 and variants
#define USE_WEBSEND_RESPONSE                   // Enable command WebSend response message (+1k code)
#define USE_WEBGETCONFIG                       // Enable restoring config from external webserver (+0k6)
#define USE_PING                                 // Enable Ping command (+2k code)
#undef USE_RULES                                // Add support for rules (+8k code)


#undef CODE_IMAGE_STR
#define CODE_IMAGE_STR "vn"

#undef USE_EMULATION
#undef USE_EMULATION_HUE                         // Disable Hue emulation - only for lights and relays
#undef USE_EMULATION_WEMO                        // Disable Wemo emulation - only for relays

#undef USE_DOMOTICZ                              // Disable Domoticz
#undef USE_HOME_ASSISTANT                        // Disable Home Assistant
#undef USE_KNX                                   // Disable KNX IP Protocol Support
//#undef USE_CUSTOM                                // Disable Custom features
//#undef USE_TIMERS                                // Disable support for up to 16 timers
//#undef USE_TIMERS_WEB                            // Disable support for timer webpage
//#undef USE_SUNRISE                               // Disable support for Sunrise and sunset tools
//#undef USE_RULES                                 // Disable support for rules
#undef USE_DISCOVERY                             // Disable mDNS for the following services (+8k code or +23.5k code with core 2_5_x, +0.3k mem)

// -- IR options ----------------------------

// #define USE_IR_HVAC
#define _IR_ENABLE_DEFAULT_ false
#define SEND_RAW true
// #define USE_IR_REMOTE                            // Enable IR remote commands using library IRremoteESP8266
#define USE_IR_REMOTE_FULL                       // Support all IR protocols from IRremoteESP8266
// #define USE_IR_RECEIVE 
#define DECODE_HASH  true
#define SEND_GLOBALCACHE true
#define DECODE_TOSHIBA_AC true
#define SEND_TOSHIBA_AC true
#define DECODE_DAIKIN true
#define SEND_DAIKIN true
#ifdef KARLSRUHE
#define SEND_LG true
#else
#undef USE_IR_SEND_NEC                        // Support IRsend NEC protocol
#endif

#undef USE_IR_SEND_RC5                        // Support IRsend Philips RC5 protocol
#undef USE_IR_SEND_RC6                        // Support IRsend Philips RC6 protocol
#undef USE_MODBUS_BRIDGE



// -- Optional modules ----------------------------
#undef ROTARY_V1                                 // Disable support for MI Desk Lamp
#undef USE_SONOFF_RF                             // Disable support for Sonoff Rf Bridge (+3k2 code)
  #undef USE_RF_FLASH                            // Disable support for flashing the EFM8BB1 chip on the Sonoff RF Bridge. C2CK must be connected to GPIO4, C2D to GPIO5 on the PCB
#undef USE_SONOFF_SC                             // Disable support for Sonoff Sc (+1k1 code)
#undef USE_TUYA_MCU                              // Disable support for Tuya Serial MCU
#undef USE_ARMTRONIX_DIMMERS                     // Disable support for Armtronix Dimmers (+1k4 code)
#undef USE_PS_16_DZ                              // Disable support for PS-16-DZ Dimmer and Sonoff L1 (+2k code)
#undef USE_SONOFF_IFAN                           // Disable support for Sonoff iFan02 and iFan03 (+2k code)
#undef USE_BUZZER                                // Disable support for a buzzer (+0k6 code)
#undef USE_ARILUX_RF                             // Disable support for Arilux RF remote controller
#undef USE_SHUTTER                               // Disable Shutter support for up to 4 shutter with different motortypes (+6k code)
#undef USE_DEEPSLEEP                             // Disable support for deepsleep (+1k code)
#undef USE_EXS_DIMMER                            // Disable support for EX-Store WiFi Dimmer
#undef USE_HOTPLUG                               // Disable support for HotPlug
#undef USE_DEVICE_GROUPS                         // Disable support for device groups (+3k5 code)
#undef USE_PWM_DIMMER                            // Disable support for MJ-SD01/acenx/NTONPOWER PWM dimmers (+4k5 code)
#undef USE_PWM_DIMMER_REMOTE                     // Disbale support for remote switches to PWM Dimmer
#undef USE_KEELOQ                                // Disable support for Jarolift rollers by Keeloq algorithm (+4k5 code)
#undef USE_SONOFF_D1                             // Disable support for Sonoff D1 Dimmer (+0k7 code)

// -- Optional light modules ----------------------
#undef USE_LIGHT                                 // Also disable all Dimmer/Light support
#undef USE_WS2812                                // Disable WS2812 Led string using library NeoPixelBus (+5k code, +1k mem, 232 iram) - Disable by //
#undef USE_MY92X1                                // Disable support for MY92X1 RGBCW led controller as used in Sonoff B1, Ailight and Lohas
#undef USE_SM16716                               // Disable support for SM16716 RGB LED controller (+0k7 code)
#undef USE_SM2135                                // Disable support for SM2135 RGBCW led control as used in Action LSC (+0k6 code)
#undef USE_SM2335                                // Disable support for SM2335 RGBCW led control as used in Switchbot Bulb
#undef USE_BP5758D                               // Disable support for BP5758D RGBCW led control as used in some Tuya lightbulbs (+0k8 code)
#undef USE_SONOFF_L1                             // Disable support for Sonoff L1 led control
#undef USE_ELECTRIQ_MOODL                        // Disable support for ElectriQ iQ-wifiMOODL RGBW LED controller
#undef USE_LIGHT_PALETTE                         // Disable support for color palette (+0k9 code)
#undef USE_SHELLY_DIMMER                         // Disable support for Shelly Dimmer (+3k code)

#undef USE_COUNTER                               // Disable counters
#define USE_ADC_VCC                              // Display Vcc in Power status. Disable for use as Analog input on selected devices
#undef USE_DS18x20                               // Disable DS18x20 sensor

#undef USE_ENERGY_SENSOR                         // Disable energy sensors (-14k code)
  #undef USE_PZEM004T                            // Disable PZEM004T energy sensor
  #undef USE_PZEM_AC                             // Disable PZEM014,016 Energy monitor
  #undef USE_PZEM_DC                             // Disable PZEM003,017 Energy monitor
  #undef USE_MCP39F501                           // Disable MCP39F501 Energy monitor as used in Shelly 2
  #undef USE_SDM72                               // Disable support for Eastron SDM72-Modbus energy meter
  #undef USE_SDM120                              // Disable support for Eastron SDM120-Modbus energy meter
  #undef USE_SDM230                              // Disable support for Eastron SDM230-Modbus energy monitor (+?? code)
  #undef USE_SDM630                              // Disable support for Eastron SDM630-Modbus energy monitor (+0k6 code)
  #undef USE_DDS2382                             // Disable support for Hiking DDS2382 Modbus energy monitor (+0k6 code)
  #undef USE_DDSU666                             // Disable support for Chint DDSU666 Modbus energy monitor (+0k6 code)
  #undef USE_SOLAX_X1                            // Disable support for Solax X1 series Modbus log info (+3k1 code)
  #undef USE_LE01MR                              // Disable support for F&F LE-01MR Modbus energy meter (+2k code)
  #undef USE_TELEINFO                            // Disable support for French Energy Provider metering telemetry
  #undef USE_IEM3000                             // Disable support for Schneider Electric iEM3000-Modbus series energy monitor (+0k8 code)
  #undef USE_WE517                               // Disable support for Orno WE517-Modbus energy monitor (+1k code)
  #undef USE_MODBUS_ENERGY                       // Disable support for generic modbus energy monitor using a user file in rule space (+5k)

//#undef USE_DS18x20                               // Disable support for DS18x20 sensors with id sort, single scan and read retry (+1k3 code)

#undef USE_I2C                                   // Disable all I2C sensors
#undef USE_SPI                                   // Disable all SPI devices

#undef USE_DISPLAY                               // Disable support for displays

#undef USE_MHZ19                                 // Disable support for MH-Z19 CO2 sensor
#undef USE_SENSEAIR                              // Disable support for SenseAir K30, K70 and S8 CO2 sensor
#undef USE_PMS5003                               // Disable support for PMS5003 and PMS7003 particle concentration sensor
#undef USE_NOVA_SDS                              // Disable support for SDS011 and SDS021 particle concentration sensor
#undef USE_HPMA                                  // Disable support for Honeywell HPMA115S0 particle concentration sensor
#undef USE_SR04                                  // Disable support for HC-SR04 ultrasonic devices (+1k code)
#undef USE_ME007                                 // Disable support for ME007 ultrasonic devices (+1k5 code)
#undef USE_DYP                                   // Disable support for DYP ME-007 ultrasonic distance sensor, serial port version (+0k5 code)
#undef USE_SERIAL_BRIDGE                         // Disable support for software Serial Bridge
#undef USE_MODBUS_BRIDGE                         // Disable support for software Modbus Bridge (+3k code)
#undef USE_MP3_PLAYER                            // Disable DFPlayer Mini MP3 Player RB-DFR-562 commands: play, volume and stop
#undef USE_AZ7798                                // Disable support for AZ-Instrument 7798 CO2 datalogger
#undef USE_PN532_HSU                             // Disable support for PN532 using HSU (Serial) interface (+1k8 code, 140 bytes mem)
#undef USE_ZIGBEE                                // Disable serial communication with Zigbee CC2530 flashed with ZNP
#undef USE_RDM6300                               // Disable support for RDM6300 125kHz RFID Reader (+0k8)
#undef USE_IBEACON                               // Disable support for bluetooth LE passive scan of ibeacon devices (uses HM17 module)
#undef USE_GPS                                   // Disable support for GPS and NTP Server for becoming Stratus 1 Time Source (+ 3.1kb flash, +132 bytes RAM)
#undef USE_HM10                                  // (ESP8266 only) Disable support for HM-10 as a BLE-bridge for the LYWSD03 (+5k1 code)
#undef USE_BLE_ESP32                             // (ESP32 only) Disable support for native BLE on ESP32 - use new driver
#undef USE_MI_ESP32                              // (ESP32 only) Disable support for ESP32 as a BLE-bridge (+9k2 mem, +292k flash)
#undef USE_HRXL                                  // Disable support for MaxBotix HRXL-MaxSonar ultrasonic range finders (+0k7)
#undef USE_TASMOTA_CLIENT                        // Disable support for Arduino Uno/Pro Mini via serial interface including flashing (+2k3 code, 44 mem)
#undef USE_OPENTHERM                             // Disable support for OpenTherm (+15k code)
#undef USE_MIEL_HVAC                             // Disable support for Mitsubishi Electric HVAC serial interface (+5k code)
#undef USE_PROJECTOR_CTRL                        // Disable support for LCD/DLP Projector serial control interface
#undef USE_LOX_O2                                // Disable support for LuminOx LOX O2 Sensor

#ifdef SKIP_DHT
#undef USE_DHT                                   // Disable support for DHT11, AM2301 (DHT21, DHT22, AM2302, AM2321) and SI7021 Temperature and Humidity sensor
#endif
#undef USE_MAX31855                              // Disable MAX31855 K-Type thermocouple sensor using softSPI
#undef USE_MAX31865                              // Disable support for MAX31865 RTD sensors using softSPI
#undef USE_TM1638                                // Disable support for TM1638 switches copying Switch1 .. Switch8
#undef USE_HX711                                 // Disable support for HX711 load cell
#undef USE_TX20_WIND_SENSOR                      // Disable support for La Crosse TX20 anemometer
#undef USE_TX23_WIND_SENSOR                      // Disable support for La Crosse TX23 anemometer
#undef USE_WINDMETER                             // Disable support for analog anemometer (+2k2 code)
#undef USE_RC_SWITCH                             // Disable support for RF transceiver using library RcSwitch
#undef USE_RF_SENSOR                             // Disable support for RF sensor receiver (434MHz or 868MHz) (+0k8 code)
#undef USE_HRE                                   // Disable support for Badger HR-E Water Meter (+1k4 code)
#undef USE_A4988_STEPPER                         // Disable support for A4988_Stepper
#undef USE_THERMOSTAT                            // Disable support for Thermostat
#undef DEBUG_THEO                                // Disable debug code
#undef USE_DEBUG_DRIVER                          // Disable debug code
#undef USE_TASMOTA_DISCOVERY

#endif  // _USER_CONFIG_OVERRIDE_H_
