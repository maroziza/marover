const SHOW_ALL = 0xA4;
const SHOW_RAM = 0xA5;
const CONTRAST = 0x81;
const INVERSE_ON = 0xA7;
const INVERSE_OFF = 0xA6;
const ON = 0xAF;
const OFF = 0xAE;
const ADDRESS = 0x20;
const MODE_HORIZONTAL = 0;
const MODE_VERTICAL = 1;
const MODE_PAGE =  2;
const MUX_RATIO = 0xA8; // effectively display height -1
const DIRECTION_FORWARD = 0xC0;
const DIRECTION_BACKWARD = 0xC8;
const START_DISPLAY = 0xD3;
const LINE_START = 0x40;
const OSC_FREQ = 0xD5;
const CHARGE_PUMP = 0xD9;
const VOLTAGE_SELECT = 0xDB;
//scroll commands


export function SSD1306(dev, w=128, h=64, flipped = false) {
    return {
        typical: [0x3c, 0x3d],
        write: dev.blockWriter,
        // todo flip
        initData: new Uint8Array([dev.writeToAddress(), 0,
                OFF, LINE_START,        // AF, 40
                MUX_RATIO, h-1,         // A8 3F (for h=64)
                START_DISPLAY, 0x00,    // D3 00
                ADDRESS, MODE_PAGE,     // 20 02
                OSC_FREQ, 0x80,         // D5 80
                0x8d, 0x14, ON      // magic numbers from datasheet
        ]).buffer,
        init: function () { this.write(this.initData); },
        gotoPage: function(addr) {
            console.log("addr" ,addr);
            this.write(Uint8Array.from([dev.writeToAddress(), 0, 0xB0 | (0x7&(addr>>>1)),
            (addr & 1) ? 0x10 : 0x14, 0]).buffer);
        },
        drawLetters(font, text) {
    //       if(! text instanceof String) return;
            for(var i =0; i<text.length;i++) {
                const c = font[text.codePointAt(i)];
  //              if(!c) { console.log(font, text.codePointAt(i)); continue;}
                this.write(c);
            }
        },





        label(position, font, lab) {
            var self = this;
            return function(text) {
                self.gotoPage(position);
                self.drawLetters(font, lab); // this only once needed
                // todo we can optimise this by just updating value
                self.drawLetters(font, text);
            }
        },

        allocatedLabel: 0,
        nextLabel: function(font, caption)  {
            return this.label(this.allocatedLabel++, font, caption);

        },
        prepareFont: function(font) {
            var out = {};
            for(var k in font.chars) {
                var c = font.chars[k];
                var bmp = c.bitmap.split(" ").map(
                (x)=>x.match(/../g).map((r)=>Number
                    .parseInt(r,16)
                    .toString(2)
                    .padStart(8,"0")

                ).join('')).reverse();
var bytes = [];
if(c.bbx[2]>0)
for (var xpad = Math.max(0, c.bbx[2]); xpad > 0; xpad--) bytes.push(0);
for (var x = 0; x < c.bbx[0]; x++) {

var line= Number.parseInt(
bmp.map((str)=>str.charAt(x)).join("")
,2)

<< Math.max(0, 7-c.bbx[1]+c.bbx[3])
;

bytes.push(    line);
}
                out[k] = new Uint8Array([dev.writeToAddress(), 0x40, ... bytes ]).buffer;
            };
           out[" ".codePointAt(0)] = new Uint8Array([dev.writeToAddress(), 0x40, 0,0,0]).buffer;
            return out;
        }
    }
}
