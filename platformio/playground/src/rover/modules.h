#include <HardwareSerial.h>
#include <Stream.h>
#include <string.h>

#define RC_CHANS 18

 class Marover { public:
    int inChannels = RC_CHANS;
    uint16_t rc[RC_CHANS] {};

    int outChannels = 3;
    // todo out
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

void startPs3Input();
bool startPs3InputWithTimeout(int seconds);

void initFS();

void stats(String msg);
Module *initDuktape(Stream *s);
Module *initCrsf(HardwareSerial *s);
