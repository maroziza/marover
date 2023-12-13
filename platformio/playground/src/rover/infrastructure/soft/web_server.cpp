#include "Arduino.h"
#include "ESPAsyncWebServer.h"
#include "FS.h"
#include <LittleFS.h>

#include "config.h"


extern String WiFiAddr;
AsyncWebServer *server = NULL;  
        

static File SpiffsFile;




// Make size of files human readable
// source: https://github.com/CelliesProjects/minimalUploadAuthESP32
String server_ui_size(const size_t bytes) {
  if (bytes < 1024) return String(bytes) + " B";
  else if (bytes < (1024 * 1024)) return String(bytes / 1024.0) + " KB";
  else if (bytes < (1024 * 1024 * 1024)) return String(bytes / 1024.0 / 1024.0) + " MB";
  else return String(bytes / 1024.0 / 1024.0 / 1024.0) + " GB";
}


void server_not_found(AsyncWebServerRequest *request) {
  String logmessage = "Client:" + request->client()->remoteIP().toString() + " " + request->url() + " - Not found";
  Serial.println(logmessage);
  request->send(404, "text/plain", "Not found");
}
  

AsyncWebServer * web_server_init() {
  server = new AsyncWebServer(CONFIG_MAROVER_WEB_PORT);
  return server;
}


void web_server_start(){
    Serial.println("Starting Webserver ...");
    server->begin();
    Serial.print("Web http://");
    Serial.print(WiFiAddr);
    Serial.printf(":%d/", CONFIG_MAROVER_WEB_PORT);
    Serial.println();
}
