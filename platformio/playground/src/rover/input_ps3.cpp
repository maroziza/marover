#include <Ps3Controller.h>
#include "chasis.h"
#include "modules.h"



static void  notify()
{
    int nLf = LOW;
    int nLb = LOW;
    int nRf = LOW;
    int nRb = LOW;

    //--------------- Digital D-pad button events --------------
    if(Ps3.data.button.up) {
        nLf = HIGH;
        nLb = LOW;
        nRf = HIGH;
        nRb = LOW;
    } else  if(Ps3.data.button.down) {
        nLf = LOW;
        nLb = HIGH;
        nRf = LOW;
        nRb = HIGH;
    } else if (Ps3.data.button.left) {
        nLf = LOW;
        nLb = HIGH;
        nRf = HIGH;
        nRb = LOW;
    } else if (Ps3.data.button.right) {
        nLf = HIGH;
        nLb = LOW;
        nRf = LOW;
        nRb = HIGH;
    } else if (Ps3.data.button.l1 || Ps3.data.button.r1 || Ps3.data.button.l2  || Ps3.data.button.r2 ) {
        nLf = Ps3.data.button.l1 ? HIGH : LOW;
        nLb = Ps3.data.button.l2 && !Ps3.data.button.l1 ? HIGH : LOW;
        nRf = Ps3.data.button.r1 ? HIGH : LOW;
        nRb = Ps3.data.button.r2 && !Ps3.data.button.r1 ? HIGH : LOW;
    }



    int x = abs(Ps3.data.analog.stick.lx) > abs(Ps3.data.analog.stick.rx) ? Ps3.data.analog.stick.lx : Ps3.data.analog.stick.rx ;
    int y = abs(Ps3.data.analog.stick.ly) > abs(Ps3.data.analog.stick.ry) ? Ps3.data.analog.stick.ly : Ps3.data.analog.stick.ry ; 

    int xMod = abs(x);
    int yMod = abs(y);
    if (xMod > 30 || yMod > 30){
        if (xMod > yMod) {
            if(x > 0) {
                //right
                nLf = HIGH;
                nLb = LOW;
                nRf = LOW;
                nRb = HIGH; 
            } else {
                //left
                nLf = LOW;
                nLb = HIGH;
                nRf = HIGH;
                nRb = LOW;
            }
        } else {
            if (y < 0){
                nLf = HIGH;
                nLb = LOW;
                nRf = HIGH;
                nRb = LOW;
            } else {
                nLf = LOW;
                nLb = HIGH;
                nRf = LOW;
                nRb = HIGH;
            }

        }
    }


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
    Serial.println("Connected BT controller.");
    controllerBattery();
}


static void onDisconnect(){
    Serial.println("Disconnected BT controller.");
}




void startPs3Input(){
    Ps3.attach(notify);
    Ps3.attachOnConnect(onConnect);
    Ps3.attachOnDisconnect(onDisconnect);
    Ps3.begin();
    String address = Ps3.getAddress();
    Serial.print("BT Ready for connection. Mac: ");
    Serial.println(address);
}

bool startPs3InputWithTimeout(int seconds){
    startPs3Input();
    bool connected = false;
    int cnt = 0;
    Serial.print("BT waiting for connection:  ");
    while(!(connected = Ps3.isConnected()) && cnt++ < seconds) {
        delay(1000);
        Serial.print(".");
    }

    if(!connected) {
        Serial.print(" No connection within timeout! Disabling BT: ");
        Serial.println(Ps3.end() && btStop() ? "done." : "failed.");
    }
    return connected;
}

