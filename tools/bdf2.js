//home/giver/Source/esp32quickjs/qjs $0 $1 ; exit
//home/giver/Focus/quickjs/qjs $0 $1 ; exit
import * as std from "std";
import * as os from "os";
var fn = scriptArgs[1];

var f = std.open(fn, "r");
var font = f.readAsString(Number.MAX_SAFE_INTEGER);
f.close();
var cre = /STARTCHAR (.*)\n([^~]*?)\nBITMAP\n([0-9A-F\n]+)\nENDCHAR\n+/g;
var header = font.replaceAll(cre, "");
var head = {};
header.split("\n").forEach((x)=>{
    var p = x.split(" ");
    head[p.shift()] = p.join(" ").replaceAll("\"","");
});
//console.log(JSON.stringify(head));

function optimizeBitmap(gly) {
    if(gly.bitmap.match(/^[0 ]+$/))return;
var arr = gly.bitmap.split(" ");
var el;

while(true) { el = arr.pop(); if(!el.match(/^0+$/)) break; gly.bbx[1]--;  }
arr.push(el);

while(true) { el = arr.shift(); if(!el.match(/^0+$/)) break; gly.bbx[1]--; gly.bbx[3]++;  }
arr.unshift(el);

gly.bitmap = arr.join(" ");
}

var iter = font.matchAll(cre), match=iter.next();
var glyphs = {};
while(!match.done) {
    var props = {};
    match.value[2].split("\n")
    .map((x)=>x.split(" "))
    .forEach((x)=>props[x.shift()] = x.map((r)=>Number.parseInt(r)));

    var gly = {
//        props, // do we really need them now?
        id: props.ENCODING[0],
        name: match.value[1],
        bbx: props.BBX,
        bitmap: match.value[3].replaceAll("\n"," ")// .map((r)=>Number.parseInt(r,16))
    };
    optimizeBitmap(gly);
    glyphs[gly.id] = gly;
//    delete gly.props.BBX;
//    console.log(JSON.stringify(gly));

    match = iter.next();
//    break;
}
var dir = "";//fonts/MediumR/";
var data = JSON.stringify({head, chars:glyphs}).replaceAll("},","},\n");
os.mkdir(dir)
fn = head.FAMILY_NAME.split(" ").pop() + head.WEIGHT_NAME[0] + head.SLANT + head.PIXEL_SIZE;
var out = std.open(dir+fn+".json","w");
out.puts("export default\n");
out.puts(data);
out.puts(";\n");
out.close();

console.log("Glyph count: ",Object.keys(glyphs).length," Font:", head.FAMILY_NAME , head.WEIGHT_NAME, head.SLANT , head.PIXEL_SIZE );

