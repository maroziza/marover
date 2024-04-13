#include "esp_camera.h"
#include "modules.h"
#include <Arduino.h>
#include "config.h"
#include "web_server.h"
#include "chasis.h"
#include "soc/rtc_wdt.h"
#include <Preferences.h>
#include "crsf.h"
#include "esp32js.h"
#include "duk_config.h"
#include "duktape.h"
#include <ESPmDNS.h>
#define HOSTNAME    "marover" // todo add board cash of some kind



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
  uint32_t psRam = (1+ESP.getFreePsram()) * 100 / (1+ESP.getPsramSize());
  uint16_t maxHeap = ESP.getMaxAllocHeap() * 100 / ESP.getHeapSize();
  Serial.print(msg);
  Serial.printf(" -> Stats%: free heap: %u - min free heap: %u - max free heap block: %u - free psram: %u\r\n", heap, minHeap, maxHeap, psRam);
}


duk_context *ctx;

static duk_ret_t js_native_print(duk_context *ctx) {
  duk_push_string(ctx, " ");
  duk_insert(ctx, 0);
  duk_join(ctx, duk_get_top(ctx) - 1);
  Serial.printf("%s", duk_to_string(ctx, -1));

  return 0;
}
void startJS()
{
  ctx = duk_create_heap_default();

  duk_push_c_function(ctx, js_native_print, DUK_VARARGS);
  duk_put_global_string(ctx, "print");
}

void js_eval (char *code)
{
  Serial.printf("\n>>> Eval Code: %s\n", code);
  duk_push_string(ctx, code);
  duk_int_t rc = duk_peval(ctx);

  if (rc != 0) {
    duk_safe_to_stacktrace(ctx, -1);
  } else {
    duk_safe_to_string(ctx, -1);
  }

  String res = duk_get_string(ctx, -1);

  Serial.printf("\n>>> Reslt: %s\n", res ? res : "null");

  duk_pop(ctx);
//  duk_destroy_heap(ctx);
}




#define RXD2 16
#define TXD2 17
#define SBUS_BUFFER_SIZE 25
uint8_t _rcs_buf[25] {};
uint16_t _raw_rc_values[RC_INPUT_MAX_CHANNELS] {};
uint16_t _raw_rc_count{};

void js_eval (char *code);

void fileChanged(const char* name) {
	js_eval("print('Hello, world!');");
}  

void setup()
{
  // https://stackoverflow.com/questions/66278271/task-watchdog-got-triggered-the-tasks-did-not-reset-the-watchdog-in-time
  rtc_wdt_protect_off();    // Turns off the automatic wdt service
  //  rtc_wdt_enable();         // Turn it on manually
  //  rtc_wdt_set_time(RTC_WDT_STAGE0, 20000);
  
  Serial2.begin(420000, SERIAL_8N1, TXD2, TXD2);
  crsf_parse(&_rcs_buf[0], SBUS_BUFFER_SIZE, &_raw_rc_values[0], &_raw_rc_count, RC_INPUT_MAX_CHANNELS );


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
  esp32_javascript_init();
  startJS();
  fileChanged("");
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
    MDNS.begin(HOSTNAME);

    stats("WiFi");

    AsyncWebServer *server = web_server_init();
    stats("Init Webserver");

#if CONFIG_MAROVER_WEB_STREAMING_ASYNC
   // initCameraStream(server);
    stats("Init Async Camera stream");
#else
   // startLegacyStreamServer();
    stats("Init Legacy stream server");
#endif

//    initFileAsyncEndpoints(server);
    stats("Init Fileserver endpoints");

 //   initWebHTML(server);
    stats("Init static web resources");

 //   initWebSocketControlsV1(server);
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
