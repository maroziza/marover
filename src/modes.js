//home/giver/Source/esp32quickjs/qjs $0 $1 ; exit

// loading device drivers
import {SSD1306} from "drivers/i2c/SSD1306.js";
import {PCF8574} from "drivers/i2c/PCF8574.js";
import {AS5600} from "drivers/i2c/AS5600.js";

// loading BUS drivers
import {I2Cbus} from "drivers/i2c.js";
// loading UI

import Helvetica from "fonts/reduced/CourierMR8.json";
import Bold from "fonts/reduced/CourierBR8.json";
import Schoolbook from "fonts/reduced/TimesMR24.json";
import Icons from 'fonts/SijiMR10.json'

// loading controllers
import {RelayDrive} from "control/drive/relay.js";


// SSD1306.find(i2c) // try to find by default addresses
var
i2c = new I2Cbus(),
screen = new SSD1306(i2c.device(0x3c), 128,64, false),
relays = new PCF8574(i2c.device(0x20)),
angle = new AS5600(i2c.device(0x36));


var regular = screen.prepareFont(Helvetica, 1, 0, 2);
var icons8 = screen.prepareFont(Icons, 1, 0, -2, 1);
Object.assign(regular, regular, icons8);

var bold = screen.prepareFont(Bold,1,0,-3);

var roman = screen.prepareFont(Schoolbook, 4 ,0x80000001, -11,0);
//var icons16 = screen.prepareFont(Icons, 3, 0x8001, -6,-2);
//Object.assign(roman, icons16, roman);

//roman.lines = 3;

var
motorRelays = relays.nextBlock(3), // forward, backward, fullThrottle
//motorPwm = screen.label(0, regular, "motor: "),
//motorDrive = RelayDrive(motorPwm, motorRelays),
steerPwm = screen.label(6, roman, "steer: "),
lightRelay = relays.nextBlock(2) // high beam, low beam
;
/*var control= {
    motorRelays, motorPwm, motorDrive
}*/
/*
var buf = new ArrayBuffer(27);
var imu =  i2c.device(0x68)
imu.blockWriter(new Uint8Array([0x74,1]).buffer);
for (var i =0; i<10;i++) {
var res = imu.blockReader(0x74, buf);
console.log(new Uint8Array(buf), res);
}
*/
screen.init();
screen.gotoPage(roman, 0);
//screen.drawLetters(roman, "Hello \uE0ebbramfaktura!!")

screen.drawLetters(roman, "202%studio")
screen.gotoPage(roman, 4);
screen.drawLetters(roman, "ADHD LAB")



screen.gotoPage(regular, 2);
screen.drawLetters(regular, "Never \uE032 underestimate your enemy please ");
screen.drawLetters(bold,"never");
screen.drawLetters(regular," forgive forget be vigilant. привітики вам з криївки передає япручок ґї       \ue006");

//screen.showFont(icons8);


