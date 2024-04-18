#include <HardwareSerial.h>
#include <Stream.h>
#include <string.h>

class Module {    public:
        virtual void setup() = 0;
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
