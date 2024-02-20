import {SSD1306} from "drivers/i2c/SSD1306.js";
import {PCF8574} from "drivers/i2c/PCF8574.js";
import {RelayDrive} from "control/drive/relay.js";
import {I2Cbus} from "drivers/i2c.js";
import Schoolbook from "fonts/MediumR/Schoolbook.json";



// SSD1306.find(i2c) // try to find by default addresses
var
i2c = new I2Cbus(),
screen = new SSD1306(i2c.device(0x3c), 128,64, false),
relays = new PCF8574(i2c.device(0x20));

var regular = screen.prepareFont(Schoolbook);


var
motorRelays = relays.nextBlock(3), // forward, backward, fullThrottle
motorPwm = screen.nextLabel(regular, "motor"),
motorDrive = RelayDrive(motorPwm, motorRelays),
steerPwm = screen.nextLabel(regular, "steer"),
lightRelay = relays.nextBlock(2) // high beam, low beam
;
var control= {
    motorRelays, motorPwm, motorDrive
}
screen.init();
//lightRelay(0);
screen.drawLetters(regular, "wuwuzell@");
