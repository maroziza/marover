#include <Ps3Controller.h>
#include "chasis.h"



static void  notify()
{
    //--------------- Digital D-pad button events --------------

    if(Ps3.data.button.up) {
        chasis_forward();
    } else  if(Ps3.data.button.down) {
        chasis_back();
    } else if (Ps3.data.button.left) {
        chasis_left();
    } else if (Ps3.data.button.right) {
        chasis_right();
    }


    if (Ps3.data.button.l1 || Ps3.data.button.r1 || Ps3.data.button.l2  || Ps3.data.button.r2 ) {
        int nLf = Ps3.data.button.l1 ? HIGH : LOW;
        int nLb = Ps3.data.button.l2 && !Ps3.data.button.l1 ? HIGH : LOW;
        int nRf = Ps3.data.button.r1 ? HIGH : LOW;
        int nRb = Ps3.data.button.r2 && !Ps3.data.button.r1 ? HIGH : LOW;

        WheelAct(nLf, nLb, nRf, nRb);
         
        if( nLf > 0 ){
            Serial.println("Left Track: Forward");
        } 
        if( nLb > 0 ){
            Serial.println("Left Track: Back");
        } 

        if( nRf > 0 ){
            Serial.println("Right Track: Forward");
        } 

        if( nRb > 0 ){
            Serial.println("Right Track: Back");
        } 
    }

}

static void controllerBattery(){
    ps3_status_battery battery = Ps3.data.status.battery;
    Serial.print("The controller battery is ");
    if( battery == ps3_status_battery_charging )      Serial.println("charging");
    else if( battery == ps3_status_battery_full )     Serial.println("FULL");
    else if( battery == ps3_status_battery_high )     Serial.println("HIGH");
    else if( battery == ps3_status_battery_low)       Serial.println("LOW");
    else if( battery == ps3_status_battery_dying )    Serial.println("DYING");
    else if( battery == ps3_status_battery_shutdown ) Serial.println("SHUTDOWN");
    else Serial.println("UNDEFINED");
} 

static void onConnect(){
    Serial.println("Connected.");
    controllerBattery();
}



void startPs3Input(){
    Ps3.attach(notify);
    Ps3.attachOnConnect(onConnect);
    Ps3.begin();
    String address = Ps3.getAddress();
    Serial.print("BT Ready for connection. Mac: ");
    Serial.println(address);
}