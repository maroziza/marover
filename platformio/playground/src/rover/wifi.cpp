#include <WiFi.h>

#define STA_MODE 0

#if STA_MODE
const char* ssid = "YOUR SSID";
const char* password = "YOUR PASSWD";
#else
const char* ssid = "esp32-sv";
const char* password = "qwer1234";
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
