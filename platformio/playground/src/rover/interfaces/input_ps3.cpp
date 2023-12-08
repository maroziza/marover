#include <Ps3Controller.h>
#include "chasis.h"
#include "modules.h"

static const int PS3_MIN_ANALOG = -255;
static const int PS3_MAX_ANALOG = 255;


static void  notify(){

    int x = abs(Ps3.data.analog.stick.lx) > abs(Ps3.data.analog.stick.rx) ? Ps3.data.analog.stick.lx : Ps3.data.analog.stick.rx ;
    int y = abs(Ps3.data.analog.stick.ly) > abs(Ps3.data.analog.stick.ry) ? Ps3.data.analog.stick.ly : Ps3.data.analog.stick.ry ; 


    int xMod = abs(x);
    int yMod = abs(y);
    int axis[2] = {0, 0};

    if(Ps3.event.button_down.triangle){
        toggle_light();
    }else if (xMod > 30 || yMod > 30){
        axis[0] = x;
        axis[1] = y;     
    } else if(Ps3.data.button.up) {
        axis[0] = 0;
        axis[1] = -Ps3.data.analog.button.up;   
    } else if(Ps3.data.button.down) {
        axis[0] = 0;
        axis[1] = Ps3.data.analog.button.down;   
    } else if (Ps3.data.button.left) {
        axis[0] = -Ps3.data.analog.button.left;
        axis[1] = 0;  
    } else if (Ps3.data.button.right) {
        axis[0] = Ps3.data.analog.button.right;
        axis[1] = 0;
    } 
        
    chasis_pwm_axis(axis, PS3_MIN_ANALOG, PS3_MAX_ANALOG);

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

