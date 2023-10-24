use<lib.scad>

wall = 0.81;
lay = 0.21;
lax = 0.41;

/**
 *  M3 x 20 screw for hex head
 */
module screw() {
    cylinder(d=3,h=20, center=true);
    translate([0,0,8]) cylinder(d=5.5,h=2, $fn=6);
}


module eng() {
    cylinder(h=60,d=3.4, center=true, $fn=28); // shaft
    #cylinder(h=28,d=6.5, $fn=6, center= true); // screw head

    translate([0,0,17.2])
        hull() {
            cylinder(h=30,d=3.1, $fn=22);
            translate([10,0])cylinder(h=30,d=3.1, $fn=22);
        }
        translate([-7,-5.25,-12])#cube([12.2,10.5,30]);
}

/**
 * Battery section
 * designed to use one 18650 or double 17400
 */ 
module bat() {
    rotate([00,90,0]) hull()  {
        translate([5,0]) cylinder(d=19,h=66,center=true);
        translate([0,0]) cylinder(d=19,h=66,center=true);
    }
}


/**
 * Module for esp32-cam, sipeed m1s dock or default 2xmotor driver
 * modelled to be compatible for all of this
 */
module esp() {
    cube([28,6.4,20]);
    translate([-3.5,3,10]) screw();
    translate([31.5,3,10]) screw();
}

module maincube() {
    translate([-20,-16,-6]) cube([40,78+8,12]);
}

$fn=64;
// 40 is total inside, 26 is motor itself
rotate([0,180,0]){
    difference() {
        union() {
           translate([0,28,0]) difference() {
               outerWalls(1.2)  bat();
               rotate([0,90,0]) {
                   //  translate([0,-9.4])cylinder(d=3,h=70,center=true);
                   translate([0,0]) cylinder(d=3,h=70,center=true);
               }
           }
           difference() {
               //%maincube();
               minkowski() {maincube();sphere(1.2, $fn=8);}
               translate([-14,62.2,-10]) esp();
               translate([-14,42.2,-10]) esp();
               translate([-14,6.2,-10]) esp();
               #translate([-14,-15.55,-10]) esp();


               translate([0,28,0]) bat();
               translate([0,-2,0]) rotate([180,90]) eng();
               translate([0,56,0]) rotate([0,90]) eng();
           }
       }
       translate([0,28,0]) bat();
       translate([0,0,-10]) scale([5,5,1]) maincube();
       translate([0,0,13.108]) scale([5,5,1]) maincube();
       translate([11,0,1]) rotate([-90,0,0]) cylinder(d=4,h=35);
       translate([11,76,1]) rotate([-90,0,180]) #cylinder(d=4,h=10);
       translate([-11,76,1]) rotate([-90,0,180]) #cylinder(d=4,h=45);
    }
}
