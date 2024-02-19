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
const OSC_FREQ = 0xD5;
const CHARGE_PUMP = 0xD9;
const VOLTAGE_SELECT = 0xDB;
//scroll commands


export function SSD1306(dev, w=128, h=64, flipped = false) {
    return {
        typical: [0x3c, 0x3d],
        wri: dev.blockWriter,
        // todo flip
        initData: [dev.writeToAddress(), 0,
                OFF,                // AF
                MUX_RATIO, h-1,     // A8, 3F (for h=64)
                START_DISPLAY, 0x00,
                ADDRESS, MODE_PAGE, //
                OSC_FREQ, 0x80, // D5 80
                0x8d, 0x14, ON      // magic numbers from datasheet
        ],
        init: function () { this.wri(this.initData); },
        gotoPage: function() {

        },



        label(font) {
            return function(position, text) {
                this.gotoPage(position);
                this.outLetters(font, text);
            }
        },

        allocatedLabel: 0,
        nextLabel: function(font, caption)  {
            return this.label(this.allocatedLabels++, font, caption);

        }
    }
}
