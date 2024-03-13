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
const COLUMN_RANGE = 0x21;
const PAGE_RANGE = 0x22;
//scroll commands


export function SSD1306(dev, w=128, h=64, flipped = false, alternate = undefined) {
    return {
        typical: [0x3c, 0x3d],
        read: dev.byteReader,
        write: dev.blockWriter,

        bufferData: function(cmds) {
            // todo current linux driver is limited to 26 bytes, good place to check
            const chunkSize = 25;

            if(cmds.length<chunkSize)
                return Uint8Array.from([dev.writeToAddress(), ...cmds]).buffer;

            var out = [];
            for (let i = 1; i < cmds.length; i += chunkSize) {
                out.push( Uint8Array.from([
                    dev.writeToAddress(), cmds[0],
                    ...cmds.slice(i, i + chunkSize)]).buffer);
            }

            return out;
        },
        initData: [0,
                OFF, LINE_START,        // AF, 40
                MUX_RATIO, h-1,         // A8 3F (for h=64)
                START_DISPLAY, 0x00,    // D3 00
                ADDRESS, MODE_VERTICAL,     // 20 02
                OSC_FREQ, 0xf0,         // D5 80
                0xda, (alternate !== undefined) ? alternate : (h>32 ? 0x10: 0x2), // DA 10 or 2 for alternate caps
                CONTRAST, 0xa0,
                flipped ? 0xC8 : 0xC0,
                flipped ? 0xA1 : 0xA0,
                0x8d, 0x14, ON      // magic numbers from datasheet
        ],
        reinit: function () {
                this.write(this.bufferData(this.initData));
        },
        init: function () {
            if(this.read()>10) this.reinit();// init only if not inited
        },
        gotoPage: function(font, addr) {
           // console.log("addr" ,addr);
            this.write(this.bufferData([0,
            ADDRESS, font.lines == 1 ? MODE_HORIZONTAL : MODE_VERTICAL,
            PAGE_RANGE,addr, font.lines == 1 ? 7 : addr+font.lines-1,
            COLUMN_RANGE, 0, 0x7F]));
        },
        showFont(font) {
            for(var a in font)
                this.write(font[a]);
        },
        drawLetters(font, text) {
    //       if(! text instanceof String) return;
            for(var i =0; i<text.length;i++) {
                const c = font[text.codePointAt(i)];
                if(!c) { console.log("Char not found", text.charAt(i) ); continue;}
                this.write(c);
            }
        },



        label(position, font, lab) {
            var self = this;
            return function(text) {
                self.gotoPage(font, position);
                self.drawLetters(font, lab); // this only once needed
                // todo we can optimise this by just updating value
                self.drawLetters(font, text);
            }
        },

        allocatedLabel: 0,
        nextLabel: function(font, caption)  {
            return this.label(this.allocatedLabel++, font, caption);

        },
        position: function(pos) {
            this.write(this.bufferData([0, 0xD3, pos]));
        },
/**
 * prepares json bdf font to direct SSD1306 commands
 *
 * lines -  font can span multiple pages vertically, lines select how
 *          many lines to prepare for use. check window size for correct
 *          render
 * mask -   is xored to every line, can be used to underline (1),
 *          strikethrough(8), boxing(0x81), inverting (0xFF),
 */
        prepareFont: function(font, lines=0, mask = 0, baseline = 7, spacing = 0) {
            // todo lines autodetec
            if(lines == 0) lines = 1

            var fbbx = font.head.FONTBOUNDINGBOX
                    .split(" ")
                    .map((x)=>Number.parseInt(x,10));
            var bl = baseline+fbbx[1]-Number.parseInt(font.head.FONT_DESCENT,10);
            console.log(bl);

            function prepareChar(c) {
                var bmp = c.bitmap.split(" ").map(
                (x)=>x.match(/../g).map((r)=>Number
                    .parseInt(r,16)
                    .toString(2)
                    .padStart(8,"0")

                ).join('')).reverse();

                var bytes = [0x40];
                if((c.bbx[2]+spacing)>0)
                    for(var xpad = spacing + c.bbx[2]; xpad > 0; xpad--)
                       for(var y = 0; y < lines; y++)
                            bytes.push(0xFF&(mask>>>(8*y)));

                for (var x = 0; x < c.bbx[0]; x++) {
                    var col = bmp.map((str)=>str.charAt(x))
                        .join("")

                    var line = Number.parseInt(col,2)
                               <<  Math.max(0, -bl+c.bbx[1]+c.bbx[3] )
                               >>> -Math.min(0, -bl+c.bbx[1]+c.bbx[3] )
                               ^ mask;

//                    bytes.push(0xFF&(line));
  //                  if(lines>1)
                        for(var y = 0; y < lines; y++)
                            bytes.push(0xFF & (line >>> (8*y)));
                }
                return bytes;
            }



            var out = {};
            for(var k in font.chars) {
                var c = font.chars[k];
                var bytes = prepareChar(c, );
                out[k] = this.bufferData(bytes);
            };
            // fix space to 3 pixels, how to adjust, use some props??
            var space = [0x40];
            for(var xpad = spacing + fbbx[0]/4; xpad > 0; xpad--)
                for(var y = 0; y < lines; y++)
                    space.push(0xFF&(mask>>>(8*y)));

            out[" ".codePointAt(0)] = this.bufferData(space);
            // todo default char
            out.lines = lines;
            return out;
        }
    }
}
