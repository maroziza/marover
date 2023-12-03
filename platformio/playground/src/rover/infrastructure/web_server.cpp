#include "Arduino.h"
#include "ESPAsyncWebServer.h"
#include "FS.h"
#include "config.h"


typedef struct SERVER_CONFIG_ {
//   String ssid;               // wifi ssid
//   String wifipassword;       // wifi password
  String httpuser;           // username to access web admin
  String httppassword;       // password to access web admin
  int webserverporthttp;     // http port number for web admin
} SERVER_CONFIG;

extern String WiFiAddr;
AsyncWebServer *server = NULL;  
        

static SERVER_CONFIG config;    
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
  

// used by server.on functions to discern whether a user has the correct httpapitoken OR is authenticated by username and password
bool server_authenticate(AsyncWebServerRequest * request) {
  bool isAuthenticated = false;

  if (request->authenticate(config.httpuser.c_str(), config.httppassword.c_str())) {
    Serial.println("is authenticated via username and password");
    isAuthenticated = true;
  }
  return isAuthenticated;
}


AsyncWebServer * web_server_init() {
  config.httpuser = CONFIG_MAROVER_WEB_USER;
  config.httppassword = CONFIG_MAROVER_WEB_PASSWORD;
  config.webserverporthttp = CONFIG_MAROVER_WEB_PORT;
  server = new AsyncWebServer(config.webserverporthttp);
  return server;
}


void web_server_start(){
    Serial.println("Starting Webserver ...");
    server->begin();
    Serial.print("Web http://");
    Serial.print(WiFiAddr);
    Serial.printf(":%d/", config.webserverporthttp);
    Serial.println();
}
