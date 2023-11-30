#include "esp_camera.h"
#include "modules.h"
#include <Arduino.h>


#define WHEELS_LAYOUT 0

#if WHEELS_LAYOUT

// GPIO Setting
extern int gpLb = 12; // Left Wheel Back
extern int gpLf = 13; // Left Wheel Forward
extern int gpRb = 15; // Right Wheel Back
extern int gpRf = 14; // Right Wheel Forward

#else

extern int gpLb = 13; // Left Wheel Back
extern int gpLf = 12; // Left Wheel Forward
extern int gpRb = 15; // Right Wheel Back
extern int gpRf = 14; // Right Wheel Forward

#endif

extern int gpLed =  4; // Light

void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println();


  pinMode(gpLb, OUTPUT); //Left Backward
  pinMode(gpLf, OUTPUT); //Left Forward
  pinMode(gpRb, OUTPUT); //Right Forward
  pinMode(gpRf, OUTPUT); //Right Backward
  pinMode(gpLed, OUTPUT); //Light
  // pinMode(gpIR, INPUT); //Light

  //initialize
  digitalWrite(gpLb, LOW);
  digitalWrite(gpLf, LOW);
  digitalWrite(gpRb, LOW);
  digitalWrite(gpRf, LOW);
  digitalWrite(gpLed, LOW);

  // initFS();

  initCamera();
   //try connect BT controller
  bool bt_connected = startPs3InputWithTimeout(5);

   //if no controller connected, start web interface
  if(!bt_connected) {
      startWebServicesAndWifi();
  }

}

void loop() {
  // vTaskDelay(50/ portTICK_PERIOD_MS );
}
