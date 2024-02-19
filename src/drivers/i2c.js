export function I2Cbus() {
    return {
        find: function(driver) {
            for(a in driver.typical) {

                }
            },
         device: function(addr) {
            return {
                writeToAddress: function() { return (addr<<1)+1; },
                byteWriter: function(byte) {
                    byte = 0xFF &byte;
                    console.log("i2c byteWrite",
                    (addr>>>0).toString(16),": ",
                    (byte>>>0).toString(2), "(0x",(byte>>>0).toString(16),")");

                },
                blockWriter: function(block) {
                    // todo set or check block address
                    // todo write to bus
                    block[0]>>>=1;
                    console.log("sudo i2cset -y 13",block.toString().replaceAll(',',' '), "i #", addr===block[0]);
                    }
                }
            }

        }}
