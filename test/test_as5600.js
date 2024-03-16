#!/usr/local/bin/qjs

import I2Cbus from '../src/drivers/i2c.js';
import {AS5600} from '../src/drivers/i2c/AS5600.js';
import * as os from 'os';

var i2c = new I2Cbus(13);
var dev = i2c.device(0x36);
console.log("ok");
var as = new AS5600(dev);

while(true) {

    console.log("status is",
    as.angle(),
    JSON.stringify(as.status()));
    os.sleep(100);

}
