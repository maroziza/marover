
#include "Arduino.h"


static int gpLed =  4; 
static bool lightState = false;


void initLight(){
  pinMode(gpLed, OUTPUT); //Light
  digitalWrite(gpLed, LOW);
}


void toggle_light() {
    if (lightState) {
        digitalWrite(gpLed, LOW);
        lightState = false;
    } else {
        digitalWrite(gpLed, HIGH);
        lightState = true;
    }
    Serial.print("Light: ");
    Serial.println(lightState);
}


