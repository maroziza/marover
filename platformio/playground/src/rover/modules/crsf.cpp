#include <modules.h>
#include "crsf.h"
#include <stdlib.h>
#include "esp_log.h"


#define SBUS_BUFFER_SIZE 128
#define RXD2 16
#define TXD2 17
char *tag = "crsf";

class CrsfModule : public Module { public:
    Marover *rover;
    HardwareSerial *in;
    CrsfModule(HardwareSerial *s) { in = s; }


    uint16_t _raw_rc_count{};

    virtual void setup(Marover*ro) {
        rover = ro;
        in->begin(420000, SERIAL_8N1, TXD2, TXD2);
        stats("Input - CRSF");
    }

    virtual void loop() {
            uint8_t _rcs_buf[SBUS_BUFFER_SIZE] {};

        if (!in->available()) return;
        size_t numBytesRead = in->read(_rcs_buf, SBUS_BUFFER_SIZE);
        if(numBytesRead <= 0) return;
        crsf_parse(&_rcs_buf[0], SBUS_BUFFER_SIZE, rover->rc,
                   &_raw_rc_count, RC_INPUT_MAX_CHANNELS );
       // ESP_LOGW(tag, "rc1:%d rc3:%d", rover->rc[0], rover->rc[2]);
        //rover->rc[0]++;
    }
};

Module* initCrsf(HardwareSerial *s) {
    return new CrsfModule(s);
}
