#include <HardwareSerial.h>
#include <Stream.h>
#include <string.h>
#include "config.h"
#include <queue>

#define RC_CHANS 18
class RoverEvent {public:
    String name, data;
    RoverEvent(String n, String d) {name=n; data=d;}
};

class Marover { public:
    int inChannels = RC_CHANS;
    uint16_t rc[RC_CHANS] {};

    int outChannels = 3;
    // todo out
    std::queue<RoverEvent*> events ;

void fireEvent(String name, String data) { events.push(new RoverEvent(name, data)); }

};

class Module {    public:
    virtual void setup(Marover *m) = 0;
    virtual void loop() = 0;
};


void startWifi(int seconds);
void initCamera();
void startLegacyStreamServer();

void initLight();
void initChasis();
void initChasisPWMChannels();
#if CONFIG_MAROVER_PS3_JOY
void startPs3Input();
bool startPs3InputWithTimeout(int seconds);
#endif
void initFS();

void stats(String msg);
#if CONFIG_MAROVER_QUICK_JS
Module *initDuktape(HardwareSerial *s);
#endif

Module *initCrsf(HardwareSerial *s);
