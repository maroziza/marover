import * as os from "os";
import * as i2c from "../ioctl.so";
/**
 * Linux kernel i2c driver using /dev/i2c device and ioctl.so lib
 *
 * TODO enumerate linux i2c buses and use last one
 *
 */
export default function LinuxI2Cbus(num="13") {
    return {
        find: function(driver) {
            for(a in driver.typical) {
// TODO try to find device by it's typical address
                }
            },
         device: function(addr) {
            var file =  os.open("/dev/i2c-13", os.O_RDWR);
            var c = i2c.ioctl(file , 0x0703, addr);
            console.log("i2c open", file, "<=", addr.toString(16),'==', addr.toString(16));

            return {
                file,
                writeToAddress: function() { return (addr<<1)+1; },

                byteWriter: function(byte) {
                    byte = 0xFF &byte;
                    console.log("i2c byteWrite",(addr>>>0).toString(16),": ", (byte>>>0).toString(2), "(0x",(byte>>>0).toString(16),")");
                    const buf = Uint8Array.from([byte]).buffer;
                    os.write(file, buf,0,1);
                },
                blockWriter: function(block) {
                    // for linux we should specify address at ioctl, so ommiting here
                    if(block instanceof ArrayBuffer) {

                        if(block.byteLength > 27)
                            console.log("i2c big block fd:", file, block.byteLength);
                        else {
                //            console.log("i2c block write req fd:", file, block.byteLength);
                            return os.write(file, block, 1, block.byteLength-1);
                        }
                    } else if (block instanceof Array) {
                        for(var i = 0; i < block.length; i++ ) {
                            os.write(file, block[i], 1, block[i].byteLength-1);
                            //console.log("i2c part block fd:", file, block[i].byteLength);

                        }
                    } else {
                        console.log("esle");
                        throw {"Invalid":block};
                    }
                }
                ,
                blockRegReader: function(cmd, block) {
                    return i2c.i2c_read_block(file, addr, cmd, block);
                },

                blockReader: function(cmd, block) {
                    return i2c.i2c_read_block(file, addr, cmd, block);
                },
                wordRegReader: function(cmd) {
                    const buf = new Uint8Array(2);
                    this.blockReader(cmd, buf.buffer);
                    return buf[0]<<8 | buf[1];
                },
                byteRegReader: function(cmd) {
                    const buf = new Uint8Array(1);
                    this.blockReader(cmd, buf.buffer);
                    return buf[0];
                },
                wordReader: function(cmd) { return null;} ,
                byteReader: function() {
                    const buf = new ArrayBuffer(1);
                    const res = os.read(file,buf,0,1);
                    return new Uint8Array(buf)[0];
                },
                byteRegWriter: function(reg, val) {
                    this.blockWriter(new Uint8Array([this.writeToAddress(), reg, val]).buffer);
                }
            }}

    }
}
