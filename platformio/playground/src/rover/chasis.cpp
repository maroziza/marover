
#include "Arduino.h"

extern int gpLb;
extern int gpLf;
extern int gpRb;
extern int gpRf;
extern int gpLed;

bool lightState = false;

void WheelAct(int nLf, int nLb, int nRf, int nRb)
{
 digitalWrite(gpLf, nLf);
 digitalWrite(gpLb, nLb);
 digitalWrite(gpRf, nRf);
 digitalWrite(gpRb, nRb);
}

void chasis_light() {
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


void chasis_right(){
    WheelAct(HIGH, LOW, LOW, HIGH);
    Serial.println("Right");
}

void chasis_left(){
    WheelAct(LOW, HIGH, HIGH, LOW);
    Serial.println("Left");
}


void chasis_back() {
    WheelAct(LOW, HIGH, LOW, HIGH);
    Serial.println("Back");
}

void chasis_stop() {
    WheelAct(LOW, LOW, LOW, LOW);
    // Serial.print(".");
}

void chasis_forward(){
    WheelAct(HIGH, LOW, HIGH, LOW);
    Serial.println("Forward");
}

