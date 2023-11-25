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
Serial.println("Startting WiFI");    
#if STA_MODE
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }
#else
   
  WiFi.softAP(ssid, password);
#endif


#if STA_MODE
  WiFiAddr = WiFi.localIP().toString();
#else
  WiFiAddr = WiFi.softAPIP().toString();
#endif
  Serial.print("WiFi Ready! Use 'http://");
  Serial.print(WiFiAddr);
  Serial.println("' to connect");
}


// void stopWifi(){
//     WiFi.disconnect(true, true);
//     Serial.println("WiFi stopped.");
// }
