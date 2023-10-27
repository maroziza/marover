use <rover.scad>
//$BOXY=true;
rotate([0,0,90])
difference() {
    marover();
    translate([-50,-50, -1]) cube([100,100,12]);
}
