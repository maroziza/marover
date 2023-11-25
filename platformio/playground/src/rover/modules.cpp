#include "modules.h"

void startWebServicesAndWifi(){
    startWifi();

    startWebControlServer();
    startStreamServer();
    // startAsyncStreamServer();
}
