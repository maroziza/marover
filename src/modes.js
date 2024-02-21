//home/giver/Source/esp32quickjs/qjs $0 $1 ; exit
import {SSD1306} from "drivers/i2c/SSD1306.js";
import {PCF8574} from "drivers/i2c/PCF8574.js";
import {RelayDrive} from "control/drive/relay.js";
import {I2Cbus} from "drivers/i2c.js";
import Helvetica from "fonts/TimesMR8.json";
import Bold from "fonts/HelveticaBR8.json";
import Schoolbook from "fonts/TimesBR14.json";
import Sm4tik from 'fonts/sm4tikMR16.json'


// SSD1306.find(i2c) // try to find by default addresses
var
i2c = new I2Cbus(),
screen = new SSD1306(i2c.device(0x3c), 128,64, false),
relays = new PCF8574(i2c.device(0x20));

var regular = screen.prepareFont(Helvetica, 1, 0, 1);
var icons8 = screen.prepareFont(Sm4tik, 1, 0, 4);
Object.assign(regular, regular, icons8);

var bold = screen.prepareFont(Bold,1,0,1,1);

var roman = screen.prepareFont(Schoolbook, 2 ,0x8001, 1);
var icons16 = screen.prepareFont(Sm4tik, 2, 0x8001, 0);
Object.assign(roman, icons16, roman);

roman.lines=2;
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


screen.init();
screen.gotoPage(roman, 0);
screen.drawLetters(roman, "Hello \uE015bramfaktura!!")
screen.gotoPage(regular, 2);
screen.drawLetters(regular, "Never \uE015 underestimate your enemy please ");
screen.drawLetters(bold,"never");
//screen.drawLetters(regular," forgive forget be vigilant. привітики вам з криївки передає япручок ґї       \ue006");
screen.drawLetters(regular, JSON.stringify(screen));


