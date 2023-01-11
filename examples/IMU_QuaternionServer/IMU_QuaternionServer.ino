#include "I2Cdev.h"
#define DEBUG
#include "Domo_IMU.h"
#include <ESPAsyncWebServer.h>
#include <Ticker.h>

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

#define INTERRUPT_PIN 4  // use pin 2 on Arduino Uno & most boards

#include <Arduino_JSON.h>
#include "webpage.h"

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}
#define SERIALDEBUG
//Domo_IMU mpu();
Domo_IMU mpu( INTERRUPT_PIN , dmpDataReady );

Ticker IMUTicker;

const char *ssid = "SSID";
const char *password = "PASSWORD";

// Create AsyncWebServer object on port 80
AsyncWebServer server(80);

// Create an Event Source on /events
AsyncEventSource events("/events");

// Json Variable to Hold Sensor Readings
JSONVar readings;

// Timer variables
unsigned long IMU_time_interval = 40;
#include "WiFi_Controller.h"

void mpusend(){
  
  events.send(getGyroReadings().c_str(), "gyro_readings", millis());
  #ifdef SERIALDEBUG
    serialReadings();
  #endif
}

void setup() {
    Serial.begin(115200);
    while (!Serial); 

    initWiFi();
    initWebServer();
    mpu.setGyroOffsets( 220, 76, -85 );
    mpu.setAccelOffsets(0,0, 1788); // 1688 factory default for my test chip
    mpu.calibrateAcc(6);
    mpu.calibrateGyro(6);
    mpu.init();
    IMUTicker.attach_ms( IMU_time_interval, mpusend );
}

void loop() {
  mpu.update();
}
