#include <WiFi.h>
#include "config.h"

#define STA_MODE CONFIG_MAROVER_WIFI_STA_MODE

#if STA_MODE
const char* ssid = CONFIG_MAROVER_WIFI_SSID;
const char* password = CONFIG_MAROVER_WIFI_PWD;
#else
const char* ssid = CONFIG_MAROVER_AP_WIFI_SSID;
const char* password = CONFIG_MAROVER_AP_WIFI_PWD;
#endif

extern String WiFiAddr = "";

void startWifi(){
Serial.print("Startting WiFI: ");    
#if STA_MODE
  WiFi.begin(ssid, password);
  Serial.print("connecting to ");
  Serial.print(ssid);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }
  
#else
  Serial.print("start AP ");
  Serial.print(ssid);
  WiFi.softAP(ssid, password);
#endif


#if STA_MODE
  WiFiAddr = WiFi.localIP().toString();
#else
  WiFiAddr = WiFi.softAPIP().toString();
#endif
Serial.println(". Done!");
}


// void stopWifi(){
//     WiFi.disconnect(true, true);
//     Serial.println("WiFi stopped.");
// }
