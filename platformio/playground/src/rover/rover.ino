#include "esp_camera.h"
#include "modules.h"
#include <Arduino.h>
#include "config.h"
#include "web_server.h"
#include "chasis.h"
#include "soc/rtc_wdt.h"
#include <Preferences.h>

// Auto control modes for storage in preferences
#define STARTUP_CONTROL_MODE_BT_AUTO 1
#define STARTUP_CONTROL_MODE_WIFI_AUTO 2
#define STARTUP_CONTROL_MODE_BT_FORCED 10
#define STARTUP_CONTROL_MODE_WIFI_FORCED 20

Preferences preferences;

void stats(String msg)
{
  uint32_t heap = ESP.getFreeHeap() * 100 / ESP.getHeapSize();
  u_int32_t minHeap = ESP.getMinFreeHeap() * 100 / ESP.getHeapSize();
  uint32_t psRam = ESP.getFreePsram() * 100 / ESP.getPsramSize();
  uint16_t maxHeap = ESP.getMaxAllocHeap() * 100 / ESP.getHeapSize();
  Serial.print(msg);
  Serial.printf(" -> Stats%: free heap: %u - min free heap: %u - max free heap block: %u - free psram: %u\n", heap, minHeap, maxHeap, psRam);
}

void setup()
{
  // https://stackoverflow.com/questions/66278271/task-watchdog-got-triggered-the-tasks-did-not-reset-the-watchdog-in-time
  //  rtc_wdt_protect_off();    // Turns off the automatic wdt service
  //  rtc_wdt_enable();         // Turn it on manually
  //  rtc_wdt_set_time(RTC_WDT_STAGE0, 20000);
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.print("setup() running on core ");
  Serial.println(xPortGetCoreID());
  preferences.begin("startup_prefs", false);

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
  // try connect BT controller

  uint startup_control_mode = preferences.getUInt("boot_mode", STARTUP_CONTROL_MODE_BT_AUTO);
  Serial.printf("Read startup mode preference: %d\n", startup_control_mode);
  if (startup_control_mode == STARTUP_CONTROL_MODE_BT_FORCED)
  {
    Serial.println("Boot mode: STARTUP_CONTROL_MODE_BT_FORCED. Will wait for BT connection.");
    startPs3Input();
    stats("PS3 BT");
  }
  else if (startup_control_mode == STARTUP_CONTROL_MODE_BT_AUTO)
  {
    Serial.println("Boot mode: STARTUP_CONTROL_MODE_BT_AUTO. Will try BT connection. If failed fallback to WIFI on reboot.");
    bool bt_connected = startPs3InputWithTimeout(CONFIG_MAROVER_BT_CONNECTION_TIMEOUT);
    stats("PS3 BT");
    if (!bt_connected)
    {
      preferences.putUInt("boot_mode", STARTUP_CONTROL_MODE_WIFI_AUTO);
      Serial.println("Set boot mode to STARTUP_CONTROL_MODE_WIFI_AUTO. Rebooting.");
      delay(1000);
      ESP.restart();
    }
  }
  else if (startup_control_mode == STARTUP_CONTROL_MODE_WIFI_AUTO || startup_control_mode == STARTUP_CONTROL_MODE_WIFI_FORCED)
  {

    if (startup_control_mode == STARTUP_CONTROL_MODE_WIFI_AUTO)
    {
      Serial.println("Boot mode: STARTUP_CONTROL_MODE_WIFI_AUTO. Will reset to STARTUP_CONTROL_MODE_BT_AUTO on reboot.");
      preferences.putUInt("boot_mode", STARTUP_CONTROL_MODE_BT_AUTO);
    }
    else if (startup_control_mode == STARTUP_CONTROL_MODE_WIFI_FORCED)
    {
      Serial.println("Boot mode: STARTUP_CONTROL_MODE_WIFI_FORCED. Creating web services.");
    }
    startWifi(CONFIG_MAROVER_WIFI_STATION_TIMEOUT);
    stats("WiFi");

    AsyncWebServer *server = web_server_init();
    stats("Init Webserver");

#if CONFIG_MAROVER_WEB_STREAMING_ASYNC
    initCameraStream(server);
    stats("Init Async Camera stream");
#else
    startLegacyStreamServer();
    stats("Init Legacy stream server");
#endif

    initFileAsyncEndpoints(server);
    stats("Init Fileserver endpoints");

    initWebHTML(server);
    stats("Init static web resources");

    initWebSocketControlsV1(server);
    stats("Init Web Sockets V1 (async)");

    web_server_start();
    stats("Web Server Started");
  }
  else
  {
    Serial.printf("Boot mode unknown: %d. Will reset to STARTUP_CONTROL_MODE_BT_AUTO on reboot.\n", startup_control_mode);
    preferences.putUInt("boot_mode", STARTUP_CONTROL_MODE_BT_AUTO);
  }
}

void loop()
{

#if CONFIG_MAROVER_QUICK_JS
  loopJS();
#endif

  if (IsRebootRequired)
  {
    Serial.println("Rebooting ESP32: ");
    delay(1000); // give time for reboot page to load
    ESP.restart();
  }

  // For timer, async, etc.
  vTaskDelay(50 / portTICK_PERIOD_MS);
  // todo control
  webSocketControlsV1Loop();
}
