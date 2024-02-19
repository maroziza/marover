import {SSD1306} from "drivers/i2c/SSD1306.js";
import {PCF8574} from "drivers/i2c/PCF8574.js";
import {RelayDrive} from "control/drive/relay.js";
import {I2Cbus} from "drivers/i2c.js";

var helv = undefined; // todo fonts

// SSD1306.find(i2c) // try to find by default addresses

var i2c = new I2Cbus(),
 screen = new SSD1306(i2c.device(0x3c), 128,64, false),
 relays = new PCF8574(i2c.device(0x20));

var motorRelays = relays.nextBlock(3),
motorPwm = screen.nextLabel(helv, "motor"),
motorDrive = RelayDrive(motorPwm, motorRelays),
steerPwm = screen.nextLabel(helv, "steer")

;
var control= {
    motorRelays, motorPwm, motorDrive
}
