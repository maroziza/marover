#include "esp_camera.h"
#include "modules.h"
#include <Arduino.h>
#include "config.h"
#include "web_server.h"
#include "chasis.h"
#include "soc/rtc_wdt.h"


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
  //https://stackoverflow.com/questions/66278271/task-watchdog-got-triggered-the-tasks-did-not-reset-the-watchdog-in-time
  // rtc_wdt_protect_off();    // Turns off the automatic wdt service
  // rtc_wdt_enable();         // Turn it on manually
  // rtc_wdt_set_time(RTC_WDT_STAGE0, 20000); 
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.print("setup() running on core ");
  Serial.println(xPortGetCoreID());
  Serial.println();

  initLight();
#if CONFIG_MAROVER_CHASIS_MODE == CHASIS_MODE_DUMMY  
  initChasis();
  stats("Chasis DUMMY");
#elif CONFIG_MAROVER_CHASIS_MODE == CHASIS_MODE_PWM 
  initChasisPWMChannels();
  stats("Chasis PWM");
#endif


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

      initFileAsyncEndpoints(server);
      stats("Init Fileserver endpoints");

      web_server_start();
      stats("Web Server Started");
  }

}

void loop() {

#if CONFIG_MAROVER_QUICK_JS
  loopJS(); 
#endif  

	if (IsRebootRequired) {
		Serial.println("Rebooting ESP32: "); 
		delay(1000); // give time for reboot page to load
		ESP.restart();
	}

#if CONFIG_MAROVER_CHASIS_MODE == CHASIS_MODE_PWM
  chasis_pwm_idle();
#endif 
// For timer, async, etc.
  vTaskDelay(50/ portTICK_PERIOD_MS );
}


