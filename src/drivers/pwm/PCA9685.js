import {sleep} from '../../service/time.js';

const oscillatorFrequency = .25e8;
const reg_PreScale = 0xfe;
const reg_Mode1 = 0x00;
const reg_Leds = 0x06;
const MODE1_RESTART = 1 << 7, MODE1_EXTCLK = 1 << 6, MODE1_AI = 1 << 5, MODE1_SLEEP = 1 << 4;
const MODE1 = "ALLCALL,SUB1,SUB2,SUB3,SLEEP,AI,RESTART".split(",");

function bits(arr, n) {
    return n.toString(2).split("").reverse().map((a,b)=>a==1?arr[b]:undefined);
}

export default function PCA9685(dev) {
    var mode1 = dev.byteRegReader(reg_Mode1);
    mode1 = (mode1 | MODE1_AI);
    dev.byteRegWriter(reg_Mode1, mode1);


    const write = dev.blockWriter;
    return {
        addresses: [0x40],
        init: function(freq) {
             const prescale_value = Math.round(oscillatorFrequency / (4096. * freq)) - 1.;
            if (prescale_value < 0x03 || prescale_value > 0xff)
                throw  {error:"Requested PWM frequency out of range"};

            // The PRE_SCALE register can only be set when the SLEEP bit of MODE1 register is set to logic 1.
            // See section 7.3.1.1 Restart mode of PCA9685 datasheet for the explanation of the following sequence.
            const old_mode1 = dev.byteRegReader(reg_Mode1);
            console.log(bits(MODE1, dev.byteRegReader(reg_Mode1)));

            dev.byteRegWriter(reg_Mode1, old_mode1 | MODE1_SLEEP);
            sleep(100);

            dev.byteRegWriter(reg_PreScale, prescale_value);
            dev.byteRegWriter(reg_Mode1, old_mode1);
            sleep(500);
            dev.byteRegWriter(reg_Mode1, (old_mode1 | MODE1_RESTART) & ~MODE1_SLEEP );
            console.log(bits(MODE1, dev.byteRegReader(reg_Mode1)));
            console.log(prescale_value, dev.byteRegReader(reg_PreScale));

        },
        // todo init MODE1
        channel: function(n) {
            const array = new Uint8Array([dev.writeToAddress(), n*4+reg_Leds, 0, 0, 0, 0]);
            const buf = array.buffer;
            return function(value) {
                array[2] = (value)&0xFF;
                array[3] = (value>>>8)&0xF;
                write(buf);
                var b = new Uint8Array(4);
//                dev.blockRegReader(reg_Leds+n*4, b.buffer);
//                console.log(b);
            }
        }
};}
