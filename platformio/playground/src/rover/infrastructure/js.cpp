#include "esp32/QuickJS.h"
#include "modules.h"

ESP32QuickJS qjs;
static const char *jscode = R"CODE(
  console.log('Hello, JavaScript!');
)CODE"; //todo load


void startJS(){
  qjs.begin();
  qjs.exec(jscode);
}

void loopJS(){
    qjs.loop();
}