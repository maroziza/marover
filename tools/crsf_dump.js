#!/usr/local/bin/qjs
import * as os from 'os';
import * as std from 'std';
import * as i2c from '../src/ioctl.so';
import I2Cbus from '../src/drivers/i2c.js';
import {SSD1306} from '../src/drivers/i2c/SSD1306.js';
import Proto from '../src/proto/crsf.js';

var fd = os.open("/dev/ttyS0", "rw");
var pro = new Proto();

var bus = new I2Cbus(1);
var dev = bus.device(0x3c);
var scr = new SSD1306(dev, 128,64, false, true);
scr.reinit();



const log = std.fdopen(2,"w");

var buf = new Uint8Array(128);
var t = 0;
while(true) {
    var io2 = i2c.ioctl(fd, 0x541B);
    if(io2 < 1) {os.sleep(10); continue;}
    if(io2 > 128) io2 = 128;
    var rd = os.read(fd, buf.buffer,0,io2);
    pro.update(buf, io2);
    if((t++%1000)==0) console.log(JSON.stringify(pro.stats()));
    log.puts(pro.channels().toString()+"\r");
    scr.position(pro.channels()[2]>>>2);

}


