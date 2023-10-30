use <rover.scad>
$BOXY=true;
//projection(cut=true)
//translate([0,0,laye])
rotate([0,0,90])

intersection() {
    marover();
    translate([-50, -50 , -7.5]) cube([100,100,14]);
}
