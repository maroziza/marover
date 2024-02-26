import * as os from "os";
import * as i2c from "ioctl.so";

export function I2Cbus() {
    return {
        find: function(driver) {
            for(a in driver.typical) {

                }
            },
         device: function(addr) {
            var file =  os.open("/dev/i2c-13", os.O_RDWR);
            var c = i2c.ioctl(file , 0x0703, addr);
            console.log("open",file, c);

            return {
                file,
                writeToAddress: function() { return (addr<<1)+1; },
                byteWriter: function(byte) {
                    byte = 0xFF &byte;
                    console.log("i2c byteWrite",
                    (addr>>>0).toString(16),": ",
                    (byte>>>0).toString(2), "(0x",(byte>>>0).toString(16),")");
                    const buf = Uint8Array.from([byte]).buffer;
                    os.write(file, buf,0,1);
                },
                blockWriter: function(block) {
                    // for linux we should specify address at ioctl, so ommiting here
                    if(block instanceof ArrayBuffer) {
                        if(block.byteLength > 27)
                            console.log("i2c big block fd:", file, block.byteLength);
                        else
                            os.write(file, block, 1, block.byteLength-1);
                    } else if (block instanceof Array) {
                        for(var i = 0; i < block.length; i++ ) {
                            os.write(file, block[i], 1, block[i].byteLength-1);
                         //   console.log("i2c part block fd:", file, block[i].byteLength);

                        }
                    } else {
                        throw {"Invalid":block};
                    }
                }
                ,
                blockReader: function(cmd, block) {
                    return i2c.i2c_read_block(file, addr, cmd, block);
                },
                wordRegReader: function(cmd) { return null;} ,
                byteRegReader: function(cmd) { return blockReader(cmd);},
                wordReader: function(cmd) { return null;} ,
                byteReader: function() {
                    const buf = new ArrayBuffer(1);
                    const res = os.read(file,buf,0,1);
                    return new Uint8Array(buf)[0];
                }
            }}

    }
}
