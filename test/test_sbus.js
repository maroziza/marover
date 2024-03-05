//home/giver/Source/esp32quickjs/qjs $0 $1 ; exit


import {SBUS} from "../src/proto/sbus.js";

function* reader(arr) {
for (var i = 0; i < arr.length; i++) yield arr[i];
}

var sbus = new SBUS((p,f)=>console.log("ok",f, p), ()=>console.log("fail"));
sbus.update([0,1,2,3,4, 0xf,0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,0,24,25,0xf,27,28]);

