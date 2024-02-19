import * as std from "std";

var f = std.open("font.bdf", "r");
var font = f.readAsString(Number.MAX_SAFE_INTEGER);
f.close();

console.log(font.replaceAll("\n"," ").replaceAll("ENDCHAR","\n").split("STARTCHAR"));

