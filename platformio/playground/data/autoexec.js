var next = 0
function chan(pin) {
var me = ++next;
print("LEDC: "+me+" pin:"+pin);
ledcSetup(me, 800, 12);
ledcAttachPin(pin, me);


return function(lev) {
//print("LEDC: "+me+" pin:"+pin+" write:"+lev);

ledcWrite(me, lev);
}
}
a = chan(13);
b = chan(12);
c = chan(14);



rcs = function() {
    var out = [];
    for(var i = 0; i<rc.length;i++) 
    out.push("\trc"+(i+1)+": "+rc[i]);
    out.push("\n");
    print(out.join(""));
};
cookie = 15;

function loop() {
var gas = 2*(rc[2]-1010), diff = (rc[0] -1500)<<3;
if(gas<0) gas = 0;
a(gas+diff);
b(gas-diff);

}