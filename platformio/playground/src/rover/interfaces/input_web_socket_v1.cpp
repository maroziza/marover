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
#include "esp_timer.h"
#include "Arduino.h"
#include "chasis.h"
#include "ESPAsyncWebServer.h"

#include "FS.h"
#include <LittleFS.h>
#include "modules.h"
#include "config.h"
#include <ArduinoJson.h>
#include <AsyncJson.h>

extern String WiFiAddr;
// Create a WebSocket object
AsyncWebSocket ws("/ws"); 


static void led_handler()
{
  toggle_light();
  stats("led");
}

static void axis_handler(int *i_axs, int min, int max)
{
}

void handleWebSocketMessage(void *arg, uint8_t *data, size_t len)
{
  AwsFrameInfo *info = (AwsFrameInfo *)arg;
  if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT)
  {
    data[len] = 0;
    String message = (char *)data;
    // Serial.print("WS msg: ");
    // Serial.println(message);
    StaticJsonDocument<200> doc;
    DeserializationError error = deserializeJson(doc, message);
    // Test if parsing succeeds.
    if (error)
    {
      Serial.print(F("deserializeJson() failed: "));
      Serial.println(error.f_str());
      return;
    }

    const char *type = doc["type"];
    if (strcmp(type, "led") == 0)
    {
      led_handler();
    }
    else if (strcmp(type, "axis") == 0)
    {
      JsonArray ax = doc["axis"].as<JsonArray>();
      int i_axs[CONFIG_MAROVER_AXIS_NUMBER];
      copyArray(ax, i_axs);
      int min = doc["min"];
      int max = doc["max"];
#if CONFIG_MAROVER_CHASIS_MODE == CHASIS_MODE_DUMMY
      chasis_axis(i_axs, min, max);
#elif CONFIG_MAROVER_CHASIS_MODE == CHASIS_MODE_PWM
      chasis_pwm_axis(i_axs, min, max);
#endif
    }
  }
}

void onEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len)
{
  switch (type)
  {
  case WS_EVT_CONNECT:
    Serial.printf("WebSocket client #%u connected from %s\n", client->id(), client->remoteIP().toString().c_str());
    stats("WS Connect");
    break;
  case WS_EVT_DISCONNECT:
    Serial.printf("WebSocket client #%u disconnected\n", client->id());
    stats("WS Disconnect");
    break;
  case WS_EVT_DATA:
    handleWebSocketMessage(arg, data, len);
    break;
  case WS_EVT_PONG:
  case WS_EVT_ERROR:
    break;
  }
}

void initWebSocketControlsV1(AsyncWebServer *server)
{
  ws.onEvent(onEvent);
  server->addHandler(&ws);
}

void webSocketControlsV1Loop(){
  ws.cleanupClients();
}
