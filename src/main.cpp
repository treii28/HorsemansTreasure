//
// Created by swood on 10/27/18.
//

//<editor-fold desc="includes">
#ifndef ESP32DAC_MISC_DEFAULTS_H
#include "misc_defaults.h"
#endif


#if defined(ARDUINO)
  #if ARDUINO < 100
    #include <WProgram.h>
  #else
    #include <Arduino.h>
  #endif
#else
  #include <Arduino.h>
#endif

#ifdef ESP32
  #include <WiFi.h>
  #include <HTTPClient.h>
#else
  #include <ESP8266WiFi.h>
#endif
#ifndef _WIFIUDP_H_
  #include <WiFiUdp.h>
#endif
#ifndef _ESPAsyncWebServer_H_
  #include "ESPAsyncWebServer.h"
#endif
#ifndef SPIFFSEditor_H_
  #include <SPIFFSEditor.h>
#endif

#ifndef FS_H
  #include "FS.h"
#endif
#ifndef _SPIFFS_H_
  #include "SPIFFS.h"
#endif
#ifndef _SPI_H_INCLUDED
  #include "SPI.h"
#endif
#ifndef _SD_H_
  #include "SD.h"
#endif

#ifndef ESP32MDNS_H
  #include <ESPmDNS.h>
#endif
#ifndef __ARDUINO_OTA_H
  #include <ArduinoOTA.h>
#endif
#ifndef _AUDIOFILESOURCESD_H
  #include "AudioFileSourceSD.h"
#endif
#ifndef _AUDIOFILESOURCEID3_H
  #include "AudioFileSourceID3.h"
#endif
#ifndef _AUDIOGENERATORMP3_H
  #include "AudioGeneratorMP3.h"
#endif
#ifndef _AUDIOGENERATORWAV_H
  #include "AudioGeneratorWAV.h"
#endif

#ifndef _AUDIOAOUTPUTI2SAVE_H
  #include "AudioOutputI2SAve.h"
#endif

#ifndef TwoWire_h
  #include <Wire.h>
#endif

#ifndef _ADAFRUIT_PWMServoDriver_H
  #include <Adafruit_PWMServoDriver.h>
#endif

#ifndef _EASYVR_LIBRARY_H
  #include "EasyVR.h"
#endif

#ifndef HardwareSerial_h
  #include <HardwareSerial.h>
#endif

//</editor-fold>

//<editor-fold desc="varinit">
AsyncWebServer httpserver(80);
AsyncWebSocket ws("/ws"); // access at ws://[esp ip]/ws
AsyncEventSource events("/events"); // event source (Server-Sent events)

// define the I2S pins for the DAC

// handles for ESP8266Audio objects
AudioGeneratorMP3 *mp3;
AudioGeneratorWAV *wav;
AudioFileSource *file;
AudioOutputI2SAve *dac;
AudioFileSourceID3 *id3;

// second hardware serial to EasyVR
HardwareSerial evSerial(1);
EasyVR easyvr(evSerial);
// use negative group for wordsets
int8_t group, idx;

// used to queue one sound file ahead
String queued = "";

// switches to notify what filesystems are attached
bool hasSDCard  = false;
bool hasSPIFFS  = false;
bool hasStorage = false;

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(&Wire, 0x40);
int eAve = 0;
int oPos = 0;
int nPos = 0;
//</editor-fold>

//<editor-fold desc="filesystemfunctions">
/**
 * print some information about the SPIFFS filesystem
 */
void printSPIFFSInfo() {
    if(hasSPIFFS) {
        // NOTE: this is ESP32 method. See esp8266 core for SPIFFS.info() example
        Serial.println("SPIFFS info...");
        Serial.print("Total Bytes: ");
        Serial.println(SPIFFS.totalBytes());
        Serial.print("Used Bytes: ");
        Serial.println(SPIFFS.usedBytes());
        Serial.print("Bytes Free: ");
        Serial.println(SPIFFS.totalBytes() - SPIFFS.usedBytes());
    } else {
        Serial.println("SPIFFS not enabled");
    }
}
/**
 * print some information about the SD card
 */
void printSDCardInfo() {
    if(hasSDCard) {
        uint8_t cardType = SD.cardType();

        if(cardType == CARD_NONE){
            Serial.println("No SD card attached");
            return;
        }

        Serial.print("SD Card Type: ");
        if(cardType == CARD_MMC){
            Serial.println("MMC");
        } else if(cardType == CARD_SD){
            Serial.println("SDSC");
        } else if(cardType == CARD_SDHC){
            Serial.println("SDHC");
        } else {
            Serial.print("UNKNOWN:");
            Serial.println(cardType);
        }

        uint64_t cardSize = SD.cardSize() / (1024 * 1024);
        Serial.printf("SD Card Size: %lluMB\n", cardSize);
    } else {
        Serial.println("no SD Card mounted!");
    }
}
/**
 * print information about any mounted filesystems
 */
void printStorageInfo() {
    if(hasStorage) {
        printSPIFFSInfo();
        printSDCardInfo();
    }
}
//</editor-fold>

//<editor-fold desc="soundfunctions">
/**
 * test to see if a given file is a directory
 *
 * @param {const char*} filename
 * @return {bool}
 */
bool fileIsDirectory(const char *filename) {
    if(!SD.exists(filename)) return false;
    File sdfile = SD.open(filename);
    bool fileIsDir = sdfile.isDirectory();
    sdfile.close();
    return fileIsDir;
}
/**
 * crude check by extension to see if a file is a wav file
 *
 * @param {const char*} filename
 * @return {bool}
 */
bool isWavFile(String filename) {
    if(!SD.exists(filename.c_str())) return false;
    if(fileIsDirectory(filename.c_str())) return false;
    return filename.endsWith(".wav");
}
/**
 * Called when a metadata event occurs (i.e. an ID3 tag, an ICY block, etc.
 *
 * @param {void} cbData
 * @param {const char*} type
 * @param {bool} isUnicode
 * @param {const char*} string
 */
void MDCallback(void *cbData, const char *type, bool isUnicode, const char *string) {
    (void)cbData;
    Serial.printf("ID3 callback for: %s = '", type);

    if (isUnicode) {
        string += 2;
    }

    while (*string) {
        char a = *(string++);
        if (isUnicode) {
            string++;
        }
        Serial.printf("%c", a);
    }
    Serial.printf("'\n");
    Serial.flush();
}
/**
 * crude check by extension to see if a file is a mp3 file
 *
 * @param {const char*} filename
 * @return {bool}
 */
bool isMP3File(String filename) {
    if(!SD.exists(filename.c_str())) return false;
    if(fileIsDirectory(filename.c_str())) return false;
    return filename.endsWith(".mp3");
}
/**
 * play a wav file off of the SD card by name
 *   if a sound is already playing, queue the file to be played next
 *   (overrides any previously queued sound)
 * @param {const char*} filename
 */
void playSoundFile(const char *filename) {
    if(!SD.exists(filename)) {
        Serial.println("file not found!");
        return;
    } else if(fileIsDirectory(filename)) {
        Serial.println("file specified is a directory!");
        return;
    }

    if (wav->isRunning() || mp3->isRunning()) {
        Serial.println("Sound file is already playing. Queueing file");
        queued = filename;
        return;
    }

    if(isWavFile(filename)) {
        file = new AudioFileSourceSD(filename);
        wav->begin(file, dac);
    } else if(isMP3File(filename)) {
        file = new AudioFileSourceSD(filename);
        id3 = new AudioFileSourceID3(file);
        id3->RegisterMetadataCB(MDCallback, (void*)"ID3TAG");
        mp3->begin(id3, dac);
    } else {
        Serial.println("can only play files of type WAV!");
    }
}
//</editor-fold>

//<editor-fold desc="httphandlers">
/**
 * empty https success handler
 *
 * @param {AsyncWebServerRequest} request
 */
void returnOK(AsyncWebServerRequest *request) { request->send(200, "text/plain", ""); }
/**
 * generic http 500 failure message
 *
 * @param {AsyncWebServerRequest} request
 * @param {String} msg
 */
void returnFail(AsyncWebServerRequest *request, String msg) {request->send(500, "text/plain", msg + "\r\n");}
/**
 * fallback handler for non-specified paths
 *   issues a 404 "Not Found"
 *
 * @param {AsyncWebServerRequest} request
 */
void handleNotFound(AsyncWebServerRequest *request) {
    request->send(404, "text/plain", "Not found");
}
/**
 * set the servo position from a web request
 *
 * @param {AsyncWebServerRequest} request
 */
void handleServo(AsyncWebServerRequest *request) {
    if (request->hasParam("pos")) {
        int pulselen = request->getParam("pos")->value().toInt();

        if((pulselen >= SERVOMIN) && (pulselen <= SERVOMAX)) {
            pwm.setPWM(SERVOPIN, 0, pulselen);
            request->send(200, "text/html", "Setting new servo position to " + String(pulselen));
        } else {
            returnFail(request, "Position value is out of range! (0-4095");
        }
    } else {
        returnFail(request, "No position parameter given!");
    }
}
/**
 * play a sound based on the wav GET parameter from a web request
 *
 * @param {AsyncWebServerRequest} request
 */
void handlePlay(AsyncWebServerRequest *request) {
    if (request->hasParam("snd")) {
        String sndfile = request->getParam("snd")->value();
        if (isWavFile(sndfile) || isMP3File(sndfile)) {
            Serial.print("Playing file: ");
            Serial.println(sndfile);
            playSoundFile(sndfile.c_str());
            request->send(200, "text/html", "Playing sound file");
            return;
        } else {
            returnFail(request, "file does not appear to be a sound file");
        }
    } else {
        returnFail(request, "to file specified");
    }
}
/**
 * crude directory listing for the SD card
 *
 * @param {AsyncWebServerRequest} request
 */
void listSDFiles(AsyncWebServerRequest *request) {
    String dirHtml = "<html><head><title>SD Card Files</title></head><body>";
    File root = SD.open("/");
    if(!root){
        dirHtml += "<p>Failed to open directory</p>";
        return;
    }
    if(!root.isDirectory()){
        dirHtml += "<p>Not a directory</p>";
        return;
    }

    File file = root.openNextFile();
    dirHtml += "<ul>";
    while(file){
        if(file.isDirectory()){
            dirHtml += "<li>  DIR : " + String(file.name()) + "</li>";
        } else {
            dirHtml += "<li>  FILE: " + String(file.name()) + "</br>  SIZE: " + String(file.size()) + "</li>";
        }
        file = root.openNextFile();
    }
    dirHtml += "</ul></body></html>";
    request->send(200, "text/html", dirHtml);
}
//</editor-fold>

/**
 * scan i2c bus looking for devices and their addresses
 */
void i2cscan() {

    byte error, address;
    int nDevices;

    Serial.println("Scanning i2c bus...");
    nDevices = 0;
    for (address = 1; address < 127; address++) {
        // The i2c_scanner uses the return value of
        // the Write.endTransmisstion to see if
        // a device did acknowledge to the address.
        Wire.beginTransmission(address);
        error = Wire.endTransmission();

        if (error == 0) {
            Serial.print("I2C device found at address 0x");
            if (address < 16)
                Serial.print("0");
            Serial.print(address, HEX);
            Serial.println("  !");

            nDevices++;
        } else if (error == 4) {
            Serial.print("Unknown error at address 0x");
            if (address < 16)
                Serial.print("0");
            Serial.println(address, HEX);
        }
    }
    if (nDevices == 0)
        Serial.println("---=== No I2C devices found ===---\n");
    else
        Serial.println("done\n");
}
/**
 * use the audio envelope value (0-1) to set the servo positon
 *
 * @param {float} env
 */
void moveServo(float env) {
    int range = SERVOMAX - SERVOMIN;
    int div = range / SERVODIV;
    eAve = ((eAve * SERVOWGT) +  (int)(env * range)) / (SERVOWGT + 1);
    nPos = div * (eAve / div);
    if(oPos != nPos) {
        oPos = nPos;
        int pulselen = range - nPos + SERVOMIN;
        //Serial.printf("New Position: %d\n", pulselen);
        pwm.setPWM(SERVOPIN, 0, pulselen);
    }
}

//<editor-fold desc="init functions">
/**
 * initialize the 2-wire I2C bus
 */
void initI2C() {
    Serial.printf("Initializing 2-Wire I2C interface with pins SDA=%d and SCL=%d\n", SDA, SCL);
    Wire.begin(SDA,SCL);
    i2cscan();
}
/**
 * initialize the PCA9685 PWM extender
 */
void initPWM() {
    Serial.println("Setting up PCA9685 I2C PWM driver");
    pwm.begin();
    pwm.setPWMFreq(SERVOFREQ);
    delay(DEFDELAY);
    pwm.setPWM(SERVOPIN, 0, SERVOMAX);
}
/**
 * initialize ESP32 second hardware serial at 115200 baud
 */
void initHWSerial() {
    Serial.printf("Initializing second ESP32 HardwareSerial RX: %d TX:%d at 115200 baud\n", EASYVR_RX_PIN, EASYVR_TX_PIN);
    evSerial.begin(115200, SERIAL_8N1, EASYVR_RX_PIN, EASYVR_TX_PIN);
    delay(100);
}
/**
 * try changing the EasyVR baud rate from default 9600 to 115200
 */
void easyvrBaud() {
    Serial.println("Trying at EasyVR default of 9600... ");
    evSerial.begin(9600, SERIAL_8N1, EASYVR_RX_PIN, EASYVR_TX_PIN);
    delay(100);
    if(easyvr.detect()) {
        Serial.print("EasyVR detected, changing to 115200...");
        if(easyvr.changeBaudrate(EasyVR::Baudrate::B115200)) {
            Serial.println("Success!");
        } else {
            Serial.println("Failed!");
        }
    } else {
        Serial.println("EasyVR not detected");
    }
    delay(100);
    initHWSerial();
}
/**
 * initialize and detect the EasyVR Voice Recognition module
 */
 void initEasyVR() {
    Serial.println("Initializing EasyVR Voice Recognition");
    initHWSerial();

    // initialize EasyVR
    // do an initial check to see if easyvr is detected
    if(!easyvr.detect()) {
        Serial.println("EasyVR not detected at 115200 baud");
        // try changing the baud of easyvr
        easyvrBaud();

        // try detecting again
        if(!easyvr.detect())
        {
            Serial.println("EasyVR not detected!");
            return;
        }
    }

    Serial.print(F("EasyVR detected, version "));
    Serial.print(easyvr.getID());

    if (easyvr.getID() < EasyVR::EASYVR3)
        easyvr.setPinOutput(EasyVR::IO1, LOW); // Shield 2.0 LED off

    if (easyvr.getID() < EasyVR::EASYVR)
        Serial.print(F(" = VRbot module"));
    else if (easyvr.getID() < EasyVR::EASYVR2)
        Serial.print(F(" = EasyVR module"));
    else if (easyvr.getID() < EasyVR::EASYVR3)
        Serial.print(F(" = EasyVR 2 module"));
    else
        Serial.print(F(" = EasyVR 3 module"));
    Serial.print(F(", FW Rev."));
    Serial.println(easyvr.getID() & 7);

    Serial.println("setting EasyVR defaults!");
    easyvr.setDelay(0); // speed-up replies

    easyvr.setTimeout(5);
    easyvr.setLanguage(0); //<-- same language set on EasyVR Commander when code was generated

    group = EasyVR::TRIGGER; //<-- start group (customize)
}
/**
 * initialize wifi station and access point modes based on defined parameters
 *
 * @return {bool} returns true on success
 */
bool initWiFi() {
    Serial.println("Connecting to WiFi...");
    WiFi.mode(WIFI_AP_STA);
    Serial.print("Starting AP: ");
    Serial.println(OTA_HOSTNAME);
    WiFi.softAP(OTA_HOSTNAME, ADMIN_PASSWORD);
    Serial.print("Connecting to network: ");
    Serial.println(SSID_NAME);
    WiFi.begin(SSID_NAME, SSID_PASSWORD);
    while (WiFi.waitForConnectResult() != WL_CONNECTED) {
        Serial.println("Connection Failed!");
        return false;
    }
    Serial.println(WiFi.localIP());
    return true;
}
/**
 * initialize Asyncronous http server on port 80 complete with handler definitions
 */
void initHttp() {
    Serial.println("initializing HTTP Server");
    // attach AsyncWebSocket
    //ws.onEvent(onEvent);
    //httpserver.addHandler(&ws);
    // attach AsyncEventSource
    //httpserver.addHandler(&events);
    httpserver.addHandler(new SPIFFSEditor(SPIFFS, "admin",ADMIN_PASSWORD));
    // attach filesystem root at URL /fs
    httpserver.serveStatic("/sdcard", SD, "/");
    httpserver.serveStatic("/spiffs", SPIFFS, "/");


    httpserver.on("/", HTTP_GET, listSDFiles);
    // send a file when /index is requested
    httpserver.on("/index.html", HTTP_ANY, [](AsyncWebServerRequest *request){
        request->send(SD, "/index.htm");
    });
    httpserver.on("/servo", HTTP_GET, handleServo);
    httpserver.on("/play", HTTP_GET, handlePlay);
    httpserver.onNotFound(handleNotFound);
    httpserver.begin();
    Serial.println("HTTP Server started");
}
/**
 * initialize mDNS service and add http on port 80
 */
void initMDNS() {
    Serial.println("initializing mDNS Server");
    MDNS.addService("http","tcp",80);
}
/**
 * initialize over-the-air (OTA) update listener
 */
void initOTA() {
    Serial.println("Initializing OTA listener...");
    // Port defaults to 3232
    // ArduinoOTA.setPort(3232);

    // Hostname defaults to esp3232-[MAC]
    ArduinoOTA.setHostname(OTA_HOSTNAME);
    //ArduinoOTA.setPassword(ADMIN_PASSWORD);
    // Password can be set with it's md5 value as well
    // MD5(admin) = 21232f297a57a5a743894a0e4a801fc3
    ArduinoOTA.setPasswordHash(OTA_PASSWORD);

    ArduinoOTA
            .onStart([]() {
                String type;
                if (ArduinoOTA.getCommand() == U_FLASH)
                    type = "sketch";
                else // U_SPIFFS
                    type = "filesystem";

                // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
                Serial.println("Start updating " + type);
            })
            .onEnd([]() {
                Serial.println("\nEnd");
            })
            .onProgress([](unsigned int progress, unsigned int total) {
                Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
            })
            .onError([](ota_error_t error) {
                Serial.printf("Error[%u]: ", error);
                if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
                else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
                else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
                else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
                else if (error == OTA_END_ERROR) Serial.println("End Failed");
            });

    ArduinoOTA.begin();
}
/**
 * initialize SPIFFS filesystem
 *
 * @return {bool} returns true on success
 */
bool initSPIFFS() {
    Serial.println("Starting SPIFFS file system...");
    if(!SPIFFS.begin()){
        Serial.println("Starting SPIFFS Failed! Rebooting...");
        delay(5000);
        return false;
    }
    return true;
}
/**
 * initialize SD card adapter
 *
 * @return {bool} returns true on success
 */
bool initSDCard() {
    Serial.println("Starting SD card reader...");
    // enable pullup on MOSI pin
    pinMode(MOSI,INPUT_PULLUP);
    if(!SD.begin()){
        Serial.println("Card Mount Failed! Rebooting...");
        return false;
    }
    return true;
}
/**
 * initialize filesystems
 *
 * @return {bool} returns true if any filesystems succeeded
 */
bool initStorage() {
    hasSPIFFS = initSPIFFS();
    hasSDCard = initSDCard();
    hasStorage = hasSPIFFS || hasSDCard;

    if(hasStorage) {
        printStorageInfo();
    }

    return hasStorage;
}
/**
 * initialize I2S bus Digital analog converter
 */
void initDAC() {
    Serial.println("initializing I2S Dac");
    dac = new AudioOutputI2SAve();
    dac->SetPinout(DAC_BLCK,DAC_WLCK,DAC_DOUT);
    dac->SetOutputModeMono(true);
    wav = new AudioGeneratorWAV();
    mp3 = new AudioGeneratorMP3();
}
//</editor-fold>

void setup()
{
    Serial.begin(SERIAL_BAUD);
    delay(DEFDELAY);

    initI2C();
    delay(DEFDELAY);

    initPWM();
    delay(DEFDELAY);

    Serial.println("Setting up LED Fade pins");
    ledcSetup(EYELEDPWM, 5000, 8);
    ledcAttachPin(EYELEDPIN, EYELEDPWM);
    delay(DEFDELAY);

    initEasyVR();

    initPWM();

    hasStorage = initStorage();
    delay(DEFDELAY);

    initWiFi();
    delay(DEFDELAY);

    initOTA();
    delay(DEFDELAY);

    initHttp();
    delay(DEFDELAY);

    initMDNS();
    delay(DEFDELAY);

    initDAC();
    delay(DEFDELAY);

    // play startup file
    if(hasStorage) {
        Serial.println("playing test startup file...");
        playSoundFile("/startup.wav");
    } else {
        Serial.println("no storage, cannot play startup");
    }
}
void loop()
{
    float env = dac->getEnvelope();
    // show check status of wav file playback, show when done and queue any files as necessary
    if(hasSDCard) {
        if (wav->isRunning()) {
            ledcWrite(EYELEDPWM, (int) (255 * env));
            moveServo(env);
            if (!wav->loop()) {
                wav->stop();
                dac->stop();
                Serial.println(" WAV done");
            }
        } else if (mp3->isRunning()) {
            if (!mp3->loop()) {
                mp3->stop();
                Serial.println(" MP3 done");
            }
        } else {
            // see if any attempts to queue have been given
            if(queued != "") {
                playSoundFile(queued.c_str());
                queued = "";
            }
        }
    }

    // handle any OTA requests
    ArduinoOTA.handle();
}
