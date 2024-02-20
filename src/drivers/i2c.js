import * as os from "os";
import {ioctl} from "ioctl.so";

export function I2Cbus() {
    return {
        find: function(driver) {
            for(a in driver.typical) {

                }
            },
         device: function(addr) {
            var file =  os.open("/dev/i2c-13", os.O_RDWR);
            var c = ioctl (file , 0x0703, addr);
            console.log("open",file, c);

            return {
                file,
                writeToAddress: function() { return (addr<<1)+1; },
                byteWriter: function(byte) {
                    byte = 0xFF &byte;
                    console.log("i2c byteWrite",
                    (addr>>>0).toString(16),": ",
                    (byte>>>0).toString(2), "(0x",(byte>>>0).toString(16),")");

                },
                blockWriter: function(block) {
                    // for linux we should specify
                    os.write(file,block, 1, block.byteLength-1);
console.log(file, block.byteLength);
                    }
                }
            }

        }}
