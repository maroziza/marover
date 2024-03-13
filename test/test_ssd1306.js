#!/home/giver/Focus/quickjs/qjs
//home/giver/Source/esp32quickjs/qjs $0 $1 ; exit

import I2Cbus from '../src/drivers/i2c.js';
import {SSD1306} from '../src/drivers/i2c/SSD1306.js';
import * as os from 'os';

var i2c = new I2Cbus(13);
var dev = i2c.device(0x3c);
var scr = new SSD1306(dev, 128,64, true, true);
scr.reinit();
var i = 0;
while(true) {
    os.sleep(10);
    scr.position(Math.sin(i++/140)*0xFF);
}
