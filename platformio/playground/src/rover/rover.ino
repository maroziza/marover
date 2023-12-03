#include "esp_camera.h"
#include "modules.h"
#include <Arduino.h>
#include "config.h"
#include "web_server.h"

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


void stats(String msg)
{
    uint32_t heap =  ESP.getFreeHeap() * 100 / ESP.getHeapSize();
    u_int32_t minHeap = ESP.getMinFreeHeap() * 100 / ESP.getHeapSize();
    uint32_t psRam =  ESP.getFreePsram() * 100 / ESP.getPsramSize();
    uint16_t maxHeap = ESP.getMaxAllocHeap()* 100 / ESP.getHeapSize();
    Serial.print(msg);
    Serial.printf(" -> Stats%: free heap: %u - min free heap: %u - max free heap block: %u - free psram: %u\n", heap, minHeap, maxHeap,  psRam);
}

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

  stats("Initial");
  initFS();
  stats("FS");

#if CONFIG_MAROVER_QUICK_JS
  startJS();
  stats("JS");
#endif  

  initCamera();
  stats("Camera");
   //try connect BT controller
  bool bt_connected = false;

#if CONFIG_MAROVER_PS3_BT  
  startPs3InputWithTimeout(5);
  stats("PS3 BT");
#endif

   //if no controller connected, start web interface
  if(!bt_connected) {
      startWifi();
      stats("WiFi");

      AsyncWebServer * server = web_server_init();
      stats("Init Webserver");

      initCameraStream(server);
      stats("Init Camera stream");

      initControlEndpoints(server);
      stats("Init Control endpoints");

      web_server_start();
      stats("Web Server Started");
  }

}

void loop() {

#if CONFIG_MAROVER_QUICK_JS
  loopJS(); 
#endif  

// For timer, async, etc.
  vTaskDelay(50/ portTICK_PERIOD_MS );
}


