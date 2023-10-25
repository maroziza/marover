use<lib.scadl>

wall = 0.81;
lay = 0.21;
lax = 0.41;

module maincube() {
    translate([-20,-16,-9]) cube([40,78+8,16]);
}


/**
 *  M3 x 20 screw for hex head
 */
module screw() {
    cylinder(d=3,h=20, center=true);
    #translate([0,0,5.5]) cylinder(d=5.7,h=2.3, $fn=6);
    #translate([0,0,-9.3]) cylinder(d=7,h=2.3);

}


module eng() {
    cylinder(h=60,d=3.4, center=true, $fn=28); // shaft
    #cylinder(h=28,d=6.5, $fn=6, center= true); // screw head

    translate([0,0,17.2])
        difference() {
            hull() {
            cylinder(h=30,d=3.1, $fn=22);
            translate([10,0])cylinder(h=30,d=3.1, $fn=22);
        }
        translate([5.6,-5])cube([5,10,10]);
    }
        translate([-7,-5.25,-12])#cube([12.2,10.5,30]);
}

/**
 * Battery section
 * designed to use one 18650 or double 17400
 */ 
module bat() {
    rotate([00,90,0]) hull()  {

       translate([5,0]) 
        difference() { 
            cylinder(d=19,h=68,center=true);
            translate([7,0,0]) cube([14,20,70], center=true);
        }
        translate([-.5,0]) cylinder(d=19,h=68,center=true);
    }
}


/**
 * Module for esp32-cam, sipeed m1s dock or default 2xmotor driver
 * modelled to be compatible for all of this
 */
module esp() {

translate([0,0,2])    #cube([28,6.4,20]);
    translate([-3.5,3,10]) screw();
    translate([31.5,3,10]) screw();
}

module marover() {
$fn=64;
// 40 is total inside, 26 is motor itself
//rotate([0,180,0])
    {
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
               translate([-14,-15.55,-10]) esp();


               translate([0,28,0]) bat();
               translate([0,-2,0]) rotate([180,90]) eng();
               translate([0,56,0]) rotate([0,90]) eng();
           }
       }
       translate([0,28,0]) bat();
       translate([-24,13,1]) rotate([0,90,0]) cylinder(d=4, h=100);
       translate([-24,41.6,1]) rotate([0,90,0]) cylinder(d=4, h=100);
 
       translate([11,-24,1]) rotate([-90,0,]) cylinder(d=4,h=100);
       translate([-11,76,1]) rotate([-90,0,180]) cylinder(d=4,h=100);
 
       #translate([-14,-16.5,-9]) 
        scale([.65,2,1.48])  rotate([90,0]) 
        linear_extrude(.8) text("ROVER",spacing=.88);
       }
}
}

module marover_body() {
difference() {
  marover();
  translate([-50,-20,-17.18]) cube([100,100,12]);
  translate([-50,-20,13.108-5.5]) cube([100,100,12]);
}
}

module marover_bottom() {
difference() {
union() {
intersection() {
  marover();
  translate([-50,-20,-17.18]) cube([100,100,12]);
}
translate([-14,-8,-7]) scale([0.22, 0.22, 0.1]) rotate([0,180,270])cube([128,128,16]); 

}

translate([-50,-20,-21.3]) cube([100,100,12]);
translate([0,70,-8.6]) #rotate([0,180,90]) scale([.3,.3]) linear_extrude(.8){
text("https://github.com/");
translate([0,-15,0]) #text("maroziza/marover");
}
translate([-14,-8,-7.5])
scale([0.22, 0.22, 0.1])  {
if(!$preview)    #surface(file="qr.code.png",invert=true);
else rotate([0,180,270])#cube([128,128,120]); 

}}
}
marover_body();
marover_bottom();