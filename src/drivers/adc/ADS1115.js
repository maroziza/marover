/**


0 : Conversion register
1 : Config register
2 : Lo_thresh register
3 : Hi_thresh register
*/
export default function ADS1115(dev) {
return {
    addresses: [0x48],

    channel: function(n) {
        //n=3;
        var arr = Uint8Array.from([0x48, 1, 0xC3 | ((0x3&n)<<4), 0xF3]);
        var buf = arr.buffer;

        return function() {
            dev.blockWriter(buf);
            while(dev.byteReader() < 0x80);

            return dev.wordRegReader(0);
        }
    }
};}
