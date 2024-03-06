//home/giver/Source/esp32quickjs/qjs $0 $1 ; exit
import * as os from 'os';
import * as std from 'std';
import * as i2c from '../src/ioctl.so';
var fd = os.open("/dev/ttyUSB0", "rw");

import Proto from '../src/proto/crsf.js';
var test  = [0,76,60,213,137,200,24,22,182,0,95,245,192,247,139,242,149,15,224,229,43,95,249,202,7,0,0,76,60,213,137,
200,24,22,182,0,95,245,192,247,139,242,149,15,224,229,43,95,249,202,7,0,0,76,60,213,137,200,24,22,182,0,95,245,192,247,139,242,149,15,224,229,43,95,249,202,7,0,0,76,60,213,137,
200,24,22,182,0,95,245,192,247,139,242,149,15,224,229,43,95,249,202,7,0,0,76,60,213,137,
200,24,22,182,0,95,245,192,247,139,242,149,15,224,229,43,95,249,202,7,0,0,76,60,213,137,200,24,22,182,0,95,245,192,247,139,242,149,15,224,229,43,95,249,202,7,0,0,76,60,213,137
];
function callback() {}
var pro = new Proto(callback);
pro.update(test);

const s = 32;
while(true) {

var io2 = i2c.ioctl(fd, 0x541B);
if(io2 < 1) {os.sleep(10); continue;}
var buf = new Uint8Array(io2);
var rd = os.read(fd, buf.buffer,0,io2);


//console.log(fd, rd, io2);

console.log(buf);


}


