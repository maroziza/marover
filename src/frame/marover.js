import {SSD1306} from "drivers/osd/SH1106.js";
import {PCF8574} from "drivers/i2c/PCF8574.js";
import {AS5600} from "drivers/i2c/AS5600.js";
import PWM from "drivers/pwm/PCA9685.js";
import ADC from "drivers/adc/ADS1115.js";


export default function(i2c, rc) {
    const devices = {
        gpio: new PCF8574(i2c.device(0x20)),
        pwm: new PWM(i2c.device(0x40)),
        adc: new ADC(i2c.device(0x48)),
        osd: new SSD1306(i2c.device(0x3c), 128,64, false,false)
    };
    devices.pwm.init(120);
    devices.osd.init();



    const inputs = {
        throttle: devices.adc.channel(3),
        steer: devices.adc.channel(2),
        mode: devices.adc.channel(1),

//        current: adc.channel(0)
//        voltage: adc.channel(0)
    },
    outputs = {
        throttle: devices.pwm.channel(0),
        steer: devices.pwm.channel(15),
    };
    return {devices, inputs, outputs};
}
