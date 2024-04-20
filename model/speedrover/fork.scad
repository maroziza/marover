H=19;
$fn=39;
difference() {
hull() {
  translate([0,0,15])  cylinder(d1=15, d2=20, h=H);    
  translate([0,-3,10])  rotate([0,90,0]) cylinder(h=20,d=15, center=true);
}
translate([0,0,H+10])
*for(i = [1: 4]) rotate([0,0,-45+i*90]) {
    translate([5,0,0])  cylinder(d=1, h=9);
    translate([8,0,0]) cylinder(d=1, h=9);
}

    cylinder(d=2.8, h=166);    
#translate([0,0,H+11.5]) cylinder(d=5, h=166);    
translate([0,0,H+11-5]) cylinder(d=7, h=5);    

 rotate([-9,0,0])   translate([-20,2,-10]) cube([40,20,160]);

difference() {    
    rotate([90,0,0])    hull() {
           translate([0,3,0])  cylinder(d=16,h=90, center = true);
          translate([0,H+3,0]) cylinder(d=16,h=90, center=true);
          
        }
          translate([0,-3,10])  rotate([0,90,0]) 
        cylinder(h=100,d=5, center=true);

    }
      translate([0,-3,10])  rotate([0,90,0]) cylinder(h=13,d=6, center=true);

  translate([0,-3,10])  rotate([0,90,0]) cylinder(h=100,d=4, center=true);

}

  translate([0,-3,10]) 
rotate([0,90,0])
%cylinder(d=30, h=12,center=true);