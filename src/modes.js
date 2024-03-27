#!/usr/local/bin/qjs

import * as os from 'os';
// loading device drivers

//import {PCF8591} from "drivers/adc/PCF8591.js";
//import Compas from "drivers/compass/QMC5883L.js";

// loading BUS drivers
import I2Cbus from "drivers/i2c.js";
import RC from "proto/crsf.js";

// loading UI

import Helvetica from "fonts/reduced/CourierMR8.json";
import Bold from "fonts/reduced/CourierBR8.json";
import Schoolbook from "fonts/reduced/TerminusMR32.json";
import Icons from 'fonts/SijiMR10.json'

// loading controllers
import Scroll from "control/osd/ScrollPane.js";
import {RelayDrive} from "control/drive/relay.js";
import * as std from "std";
import Marover from 'frame/marover.js';
// SSD1306.find(i2c) // try to find by default addresses

var i2c = new I2Cbus()
//, relays = new PCF8574(i2c.device(0x20))
//, angle = new AS5600(i2c.device(0x36))
//, pwm = new PCA9685(i2c.device(0x40))
;
var frame = Marover(i2c, undefined);

var pwm = frame.devices.pwm;
var screen = frame.devices.osd;
console.log("Device init done");


// UI
var small = screen.prepareFont(Helvetica, 1, 0, 2);
var icons8 = screen.prepareFont(Icons, 1, 0, -2, 1);
Object.assign(small, small, icons8);
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

const start = Date.now();
var t = 0;
function form(i) { return i.toString().padStart(8,"0"); }
var scroll = new Scroll(screen);
var str = `Олександра — викладачка з вокального мистецтва музичної школи DIM. Після дослідження різних практик з тілом та голосом вона зібрала вправи з мистецтва імпровізації, акторства, йоги та вокалу в сесію ТЕГ. Ми запрошуємо тебе разом поринути в цей експіріенс, полегшити емоційний тягар та наповнити себе творчими благими емоціями.
Тіло є містищем нашого життєвого досвіду. Всі емоції народжуються, живуть і осідають в ньому. Стримання злості під час нічних обстрілів, брак лайливих слів і сил на те, щоб гніватись щоразу.
Емоційна втома в налаштуваннях нашого тіла — вже за замовчуванням. Тому сесія ТЕГ направлена на людину, яка дійсно морально втомлена.
Розкритий голос — це швидке проявлення внутрішнього, адже йде зсередини. Разом із роботою тіла це впливає на затиски усередині на поліпшення психоемоційного фону. Тіло|Емоції|Голос – це вправи на дихання та звукоутворення, а не співу в класичному розумінні. Рухлива голосова імпровізація, що допомагає вивільнити застійні емоції.
Ми будемо працювати в парі, бо в комунікації з іншими ми краще пізнаємо себе. А основною задачею є чесність, прийняття всіх своїх проявів та пошук нерозкритого.
Ми чекаємо тебе відкриватись разом із DIM.`;
var strs =[];
var k = 0;
while(k<str.length)strs.push(str.substring(k,k+=25));

while (true) {
var c = frame.inputs.throttle()/57;
if(c>32000)c=0;
    scroll.position(0);
if(c<2) {
    screen.height(64);
} else {
   // scroll.position(c-2);
    screen.height(56);
    frame.outputs.steer(frame.inputs.steer() >>> 2);
    frame.outputs.throttle(frame.inputs.throttle() >>> 4);

}
//screen.height(adc.channel(2)()>>>8);

var inputs = Object.values(frame.inputs);
   for(var i = 0; i < inputs.length; i++) {
//for (var i = 0; i < 8;i++) {
        scroll.gotoPage(small, i);
        scroll.drawLetters(small, inputs[i]()+"");
//          os.sleep();

    }
        if(t++%200==0) {
            console.log((Date.now()-start)/t," ms");

        }
}
//{
while(false)
for(var i = 3150; i  < 3860; i+=1) {
    const ca = pwm.channel(15);
    const cb = pwm.channel(0);
    const i = Math.round((1+Math.sin(t/100))*((3860-3150)/2)+3150);
    scroll.gotoPage(small, i>>>3);
    scroll.drawLetters(small, i.toString().padStart(8,"0")+" The E70 community");
    ca(i);
    cb((1+Math.sin(t/50))*2048);

  //  os.sleep(100);

    if(t++%1000==0) console.log(i, (Date.now()-start)/t," ms");

}

