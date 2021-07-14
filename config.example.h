/**********************************************************************

config.h
COPYRIGHT (c) 2020 Fred Decker

The configuration file for DCC-EX Command Station

**********************************************************************/

/////////////////////////////////////////////////////////////////////////////////////
//  NOTE: Before connecting these boards and selecting one in this software
//        check the quick install guides!!! Some of these boards require a voltage
//        generating resitor on the current sense pin of the device. Failure to select
//        the correct resistor could damage the sense pin on your Arduino or destroy
//        the device.
//
// DEFINE MOTOR_SHIELD_TYPE BELOW ACCORDING TO THE FOLLOWING TABLE:
//
//  STANDARD_MOTOR_SHIELD : Arduino Motor shield Rev3 based on the L298 with 18V 2A per channel
//  POLOLU_MOTOR_SHIELD   : Pololu MC33926 Motor Driver (not recommended for prog track)
//  FUNDUMOTO_SHIELD      : Fundumoto Shield, no current sensing (not recommended, no short protection)
//  FIREBOX_MK1           : The Firebox MK1                    
//  FIREBOX_MK1S          : The Firebox MK1S
//  IBT_2_WITH_ARDUINO    : Arduino Motor Shield for PROG and IBT-2 for MAIN
//   |
//   +-----------------------v
//
#define MOTOR_SHIELD_TYPE STANDARD_MOTOR_SHIELD
/////////////////////////////////////////////////////////////////////////////////////
//
// The IP port to talk to a WIFI or Ethernet shield.
//
#define IP_PORT 2560

/////////////////////////////////////////////////////////////////////////////////////
//
// NOTE: Only supported on Arduino Mega
// Set to false if you not even want it on the Arduino Mega
//
#define ENABLE_WIFI true

/////////////////////////////////////////////////////////////////////////////////////
//
// DEFINE WiFi Parameters (only in effect if WIFI is on)
//
// If DONT_TOUCH_WIFI_CONF is set, all WIFI config will be done with
// the <+> commands and this sketch will not change anything over
// AT commands and the other WIFI_* defines below do not have any effect.
//#define DONT_TOUCH_WIFI_CONF
//
// WIFI_SSID is the network name IF you want to use your existing home network.
// Do NOT change this if you want to use the WiFi in Access Point (AP) mode. 
//
// If you do NOT set the WIFI_SSID and do NOT set the WIFI_PASSWORD,
// then the WiFi chip will first try to connect to the previously
// configured network and if that fails fall back to Access Point mode.
// The SSID of the AP will be automatically set to DCCEX_*.
// If you DO set the WIFI_SSID then the WiFi chip will try to connect
// to that (home) network in station (client) mode. If a WIFI_PASSWORD
// is set (recommended), that password will be used for AP mode.
// The AP mode password must be at least 8 characters long.
//
// Your SSID may not contain ``"'' (double quote, ASCII 0x22).
#define WIFI_SSID "Your network name"
//
// WIFI_PASSWORD is the network password for your home network or if
// you want to change the password from default AP mode password
// to the AP password you want. 
// Your password may not conain ``"'' (double quote, ASCII 0x22).
#define WIFI_PASSWORD "Your network passwd"
//
// WIFI_HOSTNAME: You probably don't need to change this
#define WIFI_HOSTNAME "dccex"
//
// WIFI_CHANNEL: If the line "#define ENABLE_WIFI true" is uncommented, 
// WiFi will be enabled (Mega only). The default channel is set to "1" whether
// this line exists or not. If you need to use an alternate channel (we recommend
// using only 1,6, or 11) you may change it here.
#define WIFI_CHANNEL 1

/////////////////////////////////////////////////////////////////////////////////////
//
// ENABLE_ETHERNET: Set to true if you have an Arduino Ethernet card (wired). This
// is not for Wifi. You will then need the Arduino Ethernet library as well
//
//#define ENABLE_ETHERNET true


/////////////////////////////////////////////////////////////////////////////////////
//
// DEFINE STATIC IP ADDRESS *OR* COMMENT OUT TO USE DHCP
//
//#define IP_ADDRESS { 192, 168, 1, 200 }


/////////////////////////////////////////////////////////////////////////////////////
//
// DEFINE LCD SCREEN USAGE BY THE BASE STATION
//
// Note: This feature requires an I2C enabled LCD screen using a PCF8574 based chipset.
//       or one using a Hitachi  HD44780.
//       OR an I2C Oled screen.
// To enable, uncomment one of the lines below

// define LCD_DRIVER for I2C LCD address 0x3f,16 cols, 2 rows
// #define LCD_DRIVER  0x3F,16,2

//OR define OLED_DRIVER width,height in pixels (address auto detected)
// 128x32 or 128x64 I2C SSD1306-based devices are supported.
// Also 132x64 I2C SH1106 devices.
// #define OLED_DRIVER 128,32


/////////////////////////////////////////////////////////////////////////////////////
//
// DEFINE S88_MEGA IF YOU WANT TO USE THE S88 BUS FOR SENSORS. TO CONFIGURE ALL THE
// OTHER OPTIONS, PLEASE CHANGE THE DEFINES IN THE FILE S88_MEGA.H
//
#define S88_MEGA
// You can use a timer for reading the sensors (preferred) or you can use the standard-loop
#define S88_USE_TIMER
//Here you can define how many sensors are connected at each bus
#define S88_BUS0_LENGTH 16
#define S88_BUS1_LENGTH 16
#define S88_BUS2_LENGTH 16
#define S88_BUS3_LENGTH 0



/////////////////////////////////////////////////////////////////////////////////////

