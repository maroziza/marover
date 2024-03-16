#!/home/giver/Focus/quickjs/qjs

//home/giver/Source/esp32quickjs/qjs $0 $1 ; exit
import * as os from 'os';
// loading device drivers
import {SSD1306} from "drivers/i2c/SSD1306.js";
import {PCF8574} from "drivers/i2c/PCF8574.js";
import {AS5600} from "drivers/i2c/AS5600.js";
import PCA from "drivers/pwm/PCA9685.js";
import {PCF8591} from "drivers/adc/PCF8591.js";
import Compas from "drivers/compass/QMC5883L.js";

// loading BUS drivers
import I2Cbus from "drivers/i2c.js";
import RC from "proto/crsf.js";

// loading UI

import Helvetica from "fonts/reduced/CourierMR8.json";
import Bold from "fonts/reduced/CourierBR8.json";
import Schoolbook from "fonts/reduced/TerminusMR32.json";
import Icons from 'fonts/SijiMR10.json'

// loading controllers
import {RelayDrive} from "control/drive/relay.js";
import * as std from "std";

// SSD1306.find(i2c) // try to find by default addresses

var i2c = new I2Cbus()
, screen = new SSD1306(i2c.device(0x3c), 128,32, true)
//, relays = new PCF8574(i2c.device(0x20))
//, angle = new AS5600(i2c.device(0x36))
//, pwm = new PCA9685(i2c.device(0x40))
;

screen.init();
//pwm.init(500);
console.log("Device init done");


// UI
var regular = screen.prepareFont(Helvetica, 1, 0, 2);
var icons8 = screen.prepareFont(Icons, 1, 0, -2, 1);
Object.assign(regular, regular, icons8);
var bold = screen.prepareFont(Bold,1,0,-3);
var roman = screen.prepareFont(Schoolbook, 3 ,0x800000, 6,0);
var icons16 = screen.prepareFont(Icons, 3, 0x8001, -6,-2);
Object.assign(roman, icons16, roman);

//roman.lines = 4;
//var
//motorRelays = relays.nextBlock(3), // forward, backward, fullThrottle
//motorPwm = screen.label(0, regular, "motor: "),
//motorDrive = RelayDrive(motorPwm, motorRelays),
//steerPwm = screen.label(6, roman, "steer: ")
//lightRelay = relays.nextBlock(2) // high beam, low beam
//leds = pwm.channel(61.5)
;
/*var control= {
    motorRelays, motorPwm, motorDrive
}*/

// hobby servo sg90 = 3150 - 3860

var i2c = new I2Cbus(13);
var dev = i2c.device(0x40);
var pca = new PCA(dev);
console.log("ok");
pca.init(100);
const start = Date.now();
var t = 0;
while (true) {
//for(var i = 3150; i  < 3860; i+=1) {
const ca = pca.channel(15);
const cb = pca.channel(0);
const i = Math.round((1+Math.sin(t/100))*((3860-3150)/2)+3150);
    screen.gotoPage(roman, 0);
    screen.drawLetters(roman, i.toString().padStart(8,"0"));
    ca(i);
    cb((1+Math.sin(t/50))*2048);

  //  os.sleep(100);

    if(t++%1000==0) console.log((Date.now()-start)/t," ms");

}

