use <rover.scad>
//$BOXY=true;
rotate([0,180,90])
intersection() {
    marover();
    translate([-50,-50, -1]) cube([100,100,12]);
}
