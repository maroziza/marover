use<lib.scadl>
use<components.scadl>
wall = 0.81;
lay = 0.21;
lax = 0.41;
module maincube() {
    //translate([-20,-16,-10]) 
    cube([40,90,18], center=true);
}

$fn=64;
module marover() {
    difference() {
        union() {
           difference() {
               outerWalls(1.19)  bat($fn=64);
               //18650 battery screws
           }
           
           difference() {
               minkowski() {maincube();cylinder(d=4,h=2.2, center=true,$fn=8);}
               
               translate([0,14,0]) {     esp();
                   translate([-14,0,-7]) cube([28,10,32]);
               }
               translate([0,-14,0]) rotate([0,0,180]) { esp();
                   translate([-14,0,-7]) cube([28,10,32]);
               
               }
               translate([0,28+14,0]) rotate([0,0,180]) esp();
               translate([0,-14-28,0]) esp();
               
               translate([0,-31,0]) rotate([0,180,180]) eng();
               translate([0,31,0]) rotate([0,180]) eng();
           }
       }
       bat();

       translate([0,0,4]) {
       rotate([0,90,0]) {
            translate([-1,0,0])incel(3,74,$fn=16); 
       
            translate([0,-19,0]) incel(5, 60);
            translate([0,+19,0]) incel(5, 60);
       }
       translate([11.5,0,0]) rotate([-90,0]) incel(5,100);
       translate([-11.5,0,0]) rotate([-90,0]) incel(5,100);
     }
     translate([-15,-44.3,-8]) 
       scale([.76,2,1.2])  rotate([90,0]) 
         linear_extrude(.8) text("ЯOVER",spacing=.77);
   }
}



marover();