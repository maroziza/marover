use <rover.scad>
//$BOXY=true;
//rotate([0,180,90])
intersection() {
    marover();
    translate([-50,-50, -4.49]) cube([100,100,16]);
}
