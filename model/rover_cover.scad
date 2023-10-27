use <rover.scad>
$BOXY=true;
rotate([0,0,90])
difference() {

//intersection() {
    marover();
    translate([-50, -50 , -8.5]) cube([100,100,17]);
  //  translate([0,0,-4.7]) cube([100,100,9.4], center=true);
    }
