#!/home/giver/Focus/quickjs/qjs
//home/giver/Source/esp32quickjs/qjs $0 $1 ; exit

import I2Cbus from '../src/drivers/i2c.js';
import PCA from '../src/drivers/pwm/PCA9685.js';
import * as os from 'os';

var i2c = new I2Cbus(13);
var dev = i2c.device(0x40);
var pca = new PCA(dev);
console.log("ok");
pca.init(1400);
while (true) {
for(var i = 0; i < 4196;i+=17) {
pca.channel(1)(i);
os.sleep(10);
}

}

