
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
    Serial.print(".");
}

void chasis_forward(){
    WheelAct(HIGH, LOW, HIGH, LOW);
    Serial.println("Forward");
}

//todo analog
void chasis_axis(int *asix, int min,  int max){
    if (sizeof(asix) >=2){
        int x = map(asix[0], min, max, -100, 100);
        int y = map(asix[1],  min, max, -100, 100);
        int xMod = abs(x);
        int yMod = abs(y);
        if (xMod > 10 || yMod > 10){
            if (xMod > yMod) {
                if(x > 0) {
                    chasis_right();
                } else {
                    chasis_left();
                }
            } else {
                if (y < 0){
                    chasis_forward();
                } else {
                    chasis_back();
                }
            }
        } else {
            chasis_stop();
        }
    }
}
