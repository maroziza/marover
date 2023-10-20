/*rotate([-90,0,0]) {
*translate([-100,0,0])
import("modellib/pcb/Ai-Thinker/ESP-CAM/esp32-cam.stl");
//ESP-CAM_Body>;
cube([27.2,40,1.5],center=true);
translate()
cylinder(h=10,d=0.5);

rotate([180,00,0])

}

*/
use<lib.scad>

module eng() {
    cylinder(h=60,d=3.4, center=true, $fn=28); // shaft
    #cylinder(h=28,d=6.5, $fn=6, center= true); // screw head

%translate([74,-133.4,-30])import("sv_tank/PLA_sv_wheels_2x.stl");

translate([0,0,17.2])        
    hull(){
        cylinder(h=30,d=3.1, $fn=22); 
        translate([10,0])cylinder(h=30,d=3.1, $fn=22); 
    }
    translate([-7,-5.25,-12])#cube([12.2,10.5,30]);
}
module bat() {
 //batterys
rotate([00,90,0]) hull()  {
translate([5,0])cylinder(d=19,h=66,center=true);
translate([0,0]) cylinder(d=19,h=66,center=true);
}

}
wall = 0.81;
lay = 0.21;
lax = 0.41;
module esp() {
  cube([28,6.4,40]); 
#translate([-3.5,3,12]) cylinder(d=3,h=12, center=true);
#translate([31.5,3,12]) cylinder(d=3,h=12, center=true);
}

module maincube() {
translate([-20,-8,-6])cube([40,78,12]);

}
$fn=64;
// 40 is total inside, 26 is motor itself
difference() {
union() {
    translate([0,28,0])difference() {
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

translate([0,28,0]) bat();
translate([0,-2,0]) rotate([180,90]) eng();
translate([0,56,0]) rotate([0,90]) eng();
}}
translate([0,0,-10]) scale([5,5,1])maincube();
translate([0,0,13.108]) scale([5,5,1])maincube();
translate([11,0,1]) rotate([-90,0,0])cylinder(d=4,h=35);
translate([11,76,1]) rotate([-90,0,180])#cylinder(d=4,h=10);
translate([-11,76,1]) rotate([-90,0,180])#cylinder(d=4,h=45);

}