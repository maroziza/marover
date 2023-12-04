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
#include "esp_camera.h"
#include "img_converters.h"
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


static void go_handler(AsyncWebServerRequest *request){
    chasis_forward();
    stats("go");
    request->send(200, "text/html", "OK");
}

static void back_handler(AsyncWebServerRequest *request){
    chasis_back();
    stats("back");
    request->send(200, "text/html", "OK");
}

static void left_handler(AsyncWebServerRequest *request){
    chasis_left();
    stats("left");
    request->send(200, "text/html", "OK");
}
static void right_handler(AsyncWebServerRequest *request){
    chasis_right();
    stats("right");
    request->send(200, "text/html", "OK");
}



static void stop_handler(AsyncWebServerRequest *request){
    chasis_stop();
    stats("stop");
    request->send(200, "text/html", "OK");
}

static void led_handler(AsyncWebServerRequest *request){
    chasis_light();
    stats("led");
    request->send(200, "text/html", "OK");
}

static void battery_handler(AsyncWebServerRequest *request){
    // static int t=0;
    // Serial.println("battery:");
    request->send(200, "text/html", "Battery: TBD");    
}


// Replaces placeholders
static String processor(const String& var) {
  if (var == "WIFI_ADDR") {
    return WiFiAddr;
  } else if (var == "STREAM_PORT") {
    return CONFIG_MAROVER_WEB_STREAMING_PORT;
  } 
  return String();
}



void initControlEndpoints(AsyncWebServer * server){

    server -> on("/go", HTTP_GET, go_handler);
    server -> on("/stop", HTTP_GET, stop_handler);
    server -> on("/back", HTTP_GET, back_handler);
    server -> on("/left", HTTP_GET, left_handler);
    server -> on("/right", HTTP_GET, right_handler);
    server -> on("/led", HTTP_GET, led_handler);

    server -> on("/battery", HTTP_GET, battery_handler); 


    server -> addHandler(new AsyncCallbackJsonWebHandler("/axis", [](AsyncWebServerRequest *request, JsonVariant &json) {
            if (json.is<JsonObject>()){
                JsonObject root = json.as<JsonObject>();
                JsonArray ax =  root["axis"].as<JsonArray>();
                // size_t axsize = ax.size();


                int i_axs[CONFIG_MAROVER_AXIS_NUMBER];
                copyArray(ax, i_axs);
                int min = root["min"];
                int max = root["max"];
                chasis_axis(i_axs, min, max);

                String out;
                serializeJson(root, out);
                request->send(200, "application/json", out);
            } else {
                request->send(400, "application/json", "{\"error\": \"Bad Request\"}");
            }
    }));


    // server -> on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    //     request->send(LittleFS, "/index.html", "text/html", false, processor);
    // });

    
    server -> serveStatic("/", LittleFS, "/").setDefaultFile("index.html").setTemplateProcessor(processor);
}
