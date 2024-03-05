const CRSF_CRC_POLY = 0xd5;
const CRSF_SYNC_BYTE = 0xC8;
const CRSF_PAYLOAD_SIZE_MAX = 62;

const CRSF_FRAMETYPE = {
    '02': 'GPS',
    '07': 'VARIO',
    '08': 'BATTERY_SENSOR',
    '09': 'BARO_ALTITUDE',
    '14': 'LINK_STATISTICS',
    '10': 'OPENTX_SYNC',
    '3A': 'RADIO_ID',
    '16': 'RC_CHANNELS_PACKED',
    '1E': 'ATTITUDE',
    '21': 'FLIGHT_MODE',
    // Extended Header Frames, range: 0x28 to 0x96
    '28': 'DEVICE_PING',
    '29': 'DEVICE_INFO',
    '2B': 'PARAMETER_SETTINGS_ENTRY',
    '2C': 'PARAMETER_READ',
    '2D': 'PARAMETER_WRITE',

    //'2E': 'ELRS_STATUS', ELRS good/bad packet count and status flags

    '32': 'COMMAND',
    // KISS frames
    '78': 'KISS_REQ',
    '79': 'KISS_RESP',
    // MSP commands
    '7A': 'MSP_REQ',   // response request using msp sequence as command
    '7B': 'MSP_RESP',  // reply with 58 byte chunked binary
    '7C': 'MSP_WRITE', // write with 8 byte chunked binary (OpenTX outbound telemetry buffer limit)
    // Ardupilot frames
    '80': 'ARDUPILOT_RESP',
};


function getCRC8(buf) {
    var crc8 = 0x00;
    var size = buf.length;
    for (var i=0; i<size; i++) {
        crc8 ^= buf[i];

        for (var j = 0; j < 8; j++) {
            if (crc8 & 0x80) {
                crc8 <<= 1;
                crc8 ^= CRSF_CRC_POLY;
            } else {
                crc8 <<= 1;
            }
        }
    }
    return crc8;
}

export default function CRFS() { return {
    update: function(buf) {

    }
};}
