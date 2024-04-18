#include <modules.h>
#include "crsf.h"
#include <stdlib.h>

#define SBUS_BUFFER_SIZE 128
#define RXD2 16
#define TXD2 17


class CrsfModule : public Module { public:
HardwareSerial *in;
CrsfModule(HardwareSerial *s) { in = s; }


uint8_t _rcs_buf[SBUS_BUFFER_SIZE] {};
uint16_t _raw_rc_values[RC_INPUT_MAX_CHANNELS] {};
uint16_t _raw_rc_count{};

virtual void setup() {
    in->begin(420000, SERIAL_8N1, TXD2, TXD2);
    stats("JS");
}

virtual void loop() {
    if (!in->available()) return;
    size_t numBytesRead = in->readBytes(_rcs_buf, SBUS_BUFFER_SIZE);
    if(numBytesRead == 0) return;

    crsf_parse(&_rcs_buf[0], SBUS_BUFFER_SIZE, &_raw_rc_values[0], &_raw_rc_count, RC_INPUT_MAX_CHANNELS );


}

};

Module* initCrsf(HardwareSerial *s) {
    return new CrsfModule(s);
}
