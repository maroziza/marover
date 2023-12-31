#include <ESPAsyncWebServer.h>

extern AsyncWebServer *server;

extern bool IsRebootRequired;


//util methods
void server_not_found(AsyncWebServerRequest *request);  
bool server_authenticate(AsyncWebServerRequest * request);
String server_ui_size(const size_t bytes) ;




//init methods
AsyncWebServer * web_server_init() ;

void initCameraStream(AsyncWebServer * server);
void initFileAsyncEndpoints(AsyncWebServer * server);
void initWebHTML(AsyncWebServer *server);
void initWebSocketControlsV1(AsyncWebServer *server);
void webSocketControlsV1Loop();



void web_server_start();