
const STATUS    = 0x0b;
const RAW_ANGLE = 0x0c;
const ANGLE     = 0x0e;


export function AS5600(device) {
    return {
        typical: [0x36],
        reader: device.blockReader,
        angle: function(reg=0x0c) {
            var buf = new Uint8Array(2);
            this.reader(RAW_ANGLE, buf.buffer);
            return (buf[0]<<8)|buf[1];
        },
        scaledAngle: function() {
            return angle(0x0e);
        },
        status: function() {
            var status_registers = new Uint8Array(4);
            this.reader(STATUS, status_registers.buffer);
            return {
                magnetDetected: (status_registers[0] & 0b00100000) != 0,
                magnetTooWeak: (status_registers[0] & 0b00010000) != 0,
                magnetTooStrong: (status_registers[0] & 0b00001000) != 0,
                automaticGainControl: status_registers[1],
                magnitude: status_registers[2] << 8 | status_registers[3]
            }
        }
    };
}

