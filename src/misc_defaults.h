//
// Created by swood on 11/28/18.
//

#ifndef ESP32DAC_MISC_DEFAULTS_H
#define ESP32DAC_MISC_DEFAULTS_H

#ifndef DEFDELAY
  #define DEFDELAY 500
#endif

#ifndef SERIAL_BAUD
  #define SERIAL_BAUD 115200
#endif

#ifndef OTA_HOSTNAME
  #define OTA_HOSTNAME "esp32ota"
#endif

#ifndef OTA_PASSWORD
  #define OTA_PASSWORD "21232f297a57a5a743894a0e4a801fc3"
#endif

#ifndef ADMIN_PASSWORD
  #define ADMIN_PASSWORD "admin"
#endif

#ifndef SSID_NAME
  #define SSID_NAME "esp32ap"
#endif

#ifndef SSID_PASSWORD
  #define SSID_PASSWORD "esp32connect"
#endif

// digital analog converter pins
#define DAC_WLCK 25
#define DAC_BLCK 26
// examples often show data as 22 but that is the i2c SCL pin
//#define DAC_DOUT 22
#define DAC_DOUT 27

// pin used for eyeball LED fader
#define EYELEDPIN 2
#define EYELEDPWM 0

// EasyVR voice module serial pins
#define EASYVR_RX_PIN 16
#define EASYVR_TX_PIN 17
#define EASYVRBAUD 66 // cmd 1 for 115200 (equivalent to 'B')
// servo defaults and ranges
// Recommended PWM GPIO pins on the ESP32 include 2,4,12-19,21-23,25-27,32-33
#define SERVOPIN 0
// standard 50 hz servo
#define SERVOFREQ 50
#define SERVOMIN 150
#define SERVOMAX 350
#define SERVOWGT 5
#define SERVODIV 8

#endif //ESP32DAC_MISC_DEFAULTS_H
