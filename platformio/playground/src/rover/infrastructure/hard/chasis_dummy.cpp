
#include "Arduino.h"


#define WHEELS_LAYOUT 0


#if WHEELS_LAYOUT

// GPIO Setting
static int gpLb = 12; // Left Wheel Back
static int gpLf = 13; // Left Wheel Forward
static int gpRb = 15; // Right Wheel Back
static int gpRf = 14; // Right Wheel Forward

#else

static int gpLb = 13; // Left Wheel Back
static int gpLf = 12; // Left Wheel Forward
static int gpRb = 15; // Right Wheel Back
static int gpRf = 14; // Right Wheel Forward

#endif



void initChasis(){
  pinMode(gpLb, OUTPUT); //Left Backward
  pinMode(gpLf, OUTPUT); //Left Forward
  pinMode(gpRb, OUTPUT); //Right Forward
  pinMode(gpRf, OUTPUT); //Right Backward


  //initialize
  digitalWrite(gpLb, LOW);
  digitalWrite(gpLf, LOW);
  digitalWrite(gpRb, LOW);
  digitalWrite(gpRf, LOW);
}



static void WheelAct(int nLf, int nLb, int nRf, int nRb)
{
 digitalWrite(gpLf, nLf);
 digitalWrite(gpLb, nLb);
 digitalWrite(gpRf, nRf);
 digitalWrite(gpRb, nRb);
}



static void chasis_right(){
    WheelAct(HIGH, LOW, LOW, HIGH);
    Serial.println("Right");
}

static void chasis_left(){
    WheelAct(LOW, HIGH, HIGH, LOW);
    Serial.println("Left");
}


static void chasis_back() {
    WheelAct(LOW, HIGH, LOW, HIGH);
    Serial.println("Back");
}

static void chasis_stop() {
    WheelAct(LOW, LOW, LOW, LOW);
    Serial.print(".");
}

static void chasis_forward(){
    WheelAct(HIGH, LOW, HIGH, LOW);
    Serial.println("Forward");
}


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
