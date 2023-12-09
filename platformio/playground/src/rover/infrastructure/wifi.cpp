#include <WiFi.h>
#include "config.h"

#define STA_MODE CONFIG_MAROVER_WIFI_STA_MODE

extern String WiFiAddr = "";

void startWifi(int seconds)
{
  bool connected = false;
  if (CONFIG_MAROVER_WIFI_STATION_ENABLED)
  {
    int cnt = 0;
    WiFi.begin(CONFIG_MAROVER_WIFI_STATION_SSID, CONFIG_MAROVER_WIFI_STATION_SSID);
    Serial.print("connecting to ");
    Serial.print(CONFIG_MAROVER_WIFI_STATION_SSID);
    while (!connected && cnt++ < seconds)
    {
      delay(1000);
      connected = (WiFi.status() == WL_CONNECTED);
      Serial.print(".");
    }
  }

  if (connected)
  {
    WiFiAddr = WiFi.localIP().toString();
    Serial.println("done!");
  }
  if (!connected)
  {
    Serial.printf(" No connection within %ds timeout! Enabling AP ", seconds);
    Serial.print(CONFIG_MAROVER_WIFI_AP_SSID);
    Serial.print(WiFi.disconnect(true, true) && WiFi.softAP(CONFIG_MAROVER_WIFI_AP_SSID, CONFIG_MAROVER_WIFI_AP_PWD) ? ": done.\n" : ": failed.\n");
    WiFiAddr = WiFi.softAPIP().toString();
  }
}
