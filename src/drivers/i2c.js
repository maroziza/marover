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
                    console.log("i2c byteWrite", addr,": ",block);

                    },
                blockWriter: function(block) {
                    // todo set or check block address
                    // todo write to bus
                    console.log("i2c blockWrite", addr,": ",block);
                    }
                }
            }

        }}
