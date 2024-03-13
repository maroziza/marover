/**
 * Futaba SBus implementation
 *
 * link is Inverted 8E2 100kbod or 200kbod for fastlink
 *
 * max packet size is 25 bytes
 *
 * A header byte (0x0F);
 * 16 x 11 bit channels (total of 22 bytes);
 * 1 flag byte, which includes two digital channels [0 0 0 0 fail_safe frame_lost ch18 ch17]; and
 * 1 footer byte (0x00).
 */
const MAX = 24;
export function unpackChannels(packet, out) {
        out[0]  = ((packet[1] | (packet[2] << 8)) & 0x07FF);
        out[1]  = ((packet[2] >>> 3) | ((packet[3] << 5) & 0x07FF));
        out[2]  = ((packet[3] >> 6) |  (packet[4] << 2) |  ((packet[5] << 10) & 0x07FF));
        out[3]  = ((packet[5] >> 1) | ((packet[6] << 7) & 0x07FF));
        out[4]  = ((packet[6] >> 4) | ((packet[7] << 4) & 0x07FF));
        out[5]  = ((packet[7] >> 7) |  (packet[8] << 1) |  ((packet[9] << 9) & 0x07FF));
        out[6]  = ((packet[9] >> 2) | ((packet[10] << 6) & 0x07FF));
        out[7]  = ((packet[10] >> 5) |((packet[11] << 3) & 0x07FF));
        out[8]  =  (packet[12] |      ((packet[13] << 8) & 0x07FF));
        out[9]  = ((packet[13] >> 3) |((packet[14] << 5) & 0x07FF));
        out[10] = ((packet[14] >> 6) | (packet[15] << 2) | ((packet[16] << 10) & 0x07FF));
        out[11] = ((packet[16] >> 1) |((packet[17] << 7) & 0x07FF));
        out[12] = ((packet[17] >> 4) |((packet[18] << 4) & 0x07FF));
        out[13] = ((packet[18] >> 7) | (packet[19] << 1) | ((packet[20] << 9) & 0x07FF));
        out[14] = ((packet[20] >> 2) |((packet[21] << 6) & 0x07FF));
        out[15] = ((packet[21] >> 5) |((packet[22] << 3) & 0x07FF));
};


export default function SBUS(gotPacket, failedPacket=undefined) { return {
    state: new Uint8Array(MAX),

    parsePacket: function(packet) {
        var out = new Uint16Array(16);
        unpackChannels(packet, out);
        gotPacket(out, packet[23]);
    },
    update: function(buf) {
        var state = this.state;
        var count = this.state[0];

        for(var i = 0; i<buf.length;i++) {
            console.log(count, buf[i]);

            switch(count) {
                case 0:
                    if(buf[i]==0xf) count++;
                    break;
                case MAX:
                    if(buf[i]==0) this.parsePacket(state);
                    else if(failedPacket) failedPacket(state);
                    count=0;
                    break;
                default:// todo we can update all the channesl as they come
                    state[count++]=buf[i];
            }

        }
        state[0]=count;
    }

};}
