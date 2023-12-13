// Copyright 2015-2016 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
// #include "esp_http_server.h"
#include "Arduino.h"
#include "ESPAsyncWebServer.h"
#include "FS.h"
#include <LittleFS.h>
#include "config.h"

extern String WiFiAddr;

// Replaces placeholders
static String processor(const String& var) {
  if (var == "WIFI_ADDR") {
    return WiFiAddr;
  } else if (var == "STREAM_PORT") {
#if CONFIG_MAROVER_WEB_STREAMING_ASYNC    
    return String(CONFIG_MAROVER_WEB_PORT);
#else
    return String(CONFIG_MAROVER_WEB_PORT+1);
#endif
  } 
  return String();
}


void initWebHTML(AsyncWebServer *server)
{
  server -> serveStatic("/", LittleFS, "/").setDefaultFile("index.html").setTemplateProcessor(processor);
}
