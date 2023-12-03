#include <ESPAsyncWebServer.h>

extern AsyncWebServer *server;
// extern bool IsRebootRequired;





static void server_not_found(AsyncWebServerRequest *request);
  
bool server_authenticate(AsyncWebServerRequest * request);

AsyncWebServer * web_server_init() ;

void initCameraStream(AsyncWebServer * server);
void initControlEndpoints(AsyncWebServer * server);

void web_server_start();