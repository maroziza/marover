import * as std from "std";
import * as os from "os";
var f = std.open("font.bdf", "r");
var font = f.readAsString(Number.MAX_SAFE_INTEGER);
f.close();
var cre = /STARTCHAR (.*)\n([^~]*?)\nBITMAP\n([0-9A-F\n]+)\nENDCHAR\n+/g;
var header = font.replaceAll(cre, "");
//console.log(header);

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
    glyphs[gly.id] = gly;
//    delete gly.props.BBX;
//    console.log(JSON.stringify(gly));

    match = iter.next();
//    break;
}
var dir = "fonts/MediumR";
var data = JSON.stringify({chars:glyphs});
os.mkdir(dir)
var out = std.open(dir+"/Helvetica.json","w");
out.puts("export default\n");
out.puts(data);
out.puts(";\n");
out.close();

console.log(data);


