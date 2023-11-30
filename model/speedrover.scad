use<lib.scadl>

module eng() {
    rotate([0,90]) cylinder(d=3.4, h=40, $fn=46);

    translate([4,-13/2,-11/2]) { 
cube([30,13, 11]);    
        }
// reductor
//    cube([13,10,10]);

}

//translate([0,-31,0])
difference() { 
translate([-35,-7.2
    ,-6]) cube([70,50,10]);
rotate([0,0,180]) eng();
//translate([0,31,0]) 
rotate([0,0,0]) eng();


}