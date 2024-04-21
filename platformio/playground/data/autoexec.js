var next = 0
function chan(pin) {
var me = ++next;
print("LEDC: "+me+" pin:"+pin);
ledcSetup(me, 800, 12);
ledcAttachPin(pin, me);


return function(lev) {
if(lev<0) lev=0;
if(lev>4096)lev=4095;
ledcWrite(me, lev);
}
}
a = chan(13);
b = chan(12);
c = chan(14);
ledcSetup(next, 200, 11);



rcs = function() {
    var out = [];
    for(var i = 0; i<rc.length;i++) 
    out.push("\trc"+(i+1)+": "+rc[i]);
    out.push("\n");
    print(out.join(""));
};
cookie = 15;

function loop() {
var gas = (rc[2]-1010)<<3, diff = (rc[3] -1500)<<3;
if(gas<0) gas = 0;
a(gas+diff);
b(gas-diff);
c(rc[0]); 
print("RC1 ", rc[0],
      "RC2 ", rc[1],
      "RC3 ", rc[2],
      "RC4 ", rc[3], "\t\t",
      "GAS ", gas,
      "DIF ", diff,
      "LE1 ", (gas+diff),
      "LE2 ", (gas-diff), "\n"

);

}