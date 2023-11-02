$fa = 1;
$fs = 0.4;

active = true;

module disk(disk_width, wheel_radius, hub_radius) {
    cylinder(h = disk_width * 0.7, r1=wheel_radius, r2 = hub_radius * 1.2, center = false);
    translate([0,0,-disk_width * 0.3])
        cylinder(h = disk_width * 0.3, r = wheel_radius, center = false);        
}

module spikes(hub_radius, wheel_width, disk_width) {
for(i=[1:1:12]) {
    rotate([0,0,i*360/12]) translate([hub_radius,0,0]) {
        difference() {
                cube([(hub_radius * 1.2 - hub_radius) * 2, 3, wheel_width - disk_width], center=true);
            rotate([0,0,45])
                translate([2.7,-0.5,0])
                    cube([1, 2, wheel_width - disk_width], center=true);
            rotate([0,0,-45])
                translate([2.7,0.5,0])
                    cube([1, 2, wheel_width - disk_width], center=true);
            
            }
        }
    }
}


// Default values are roughly taken from the stl
module complex_wheel(wheel_radius=14, wheel_width=9.73) {
    // Wheel consists of two disks and a hub between
    // From the upper side the cacke is extruded 
    // to form axis hole and fillets
    disk_width = wheel_width * 0.16;
    hub_radius = ((wheel_width / wheel_radius) * wheel_radius) * 1.1;
        
    disk_offset = (wheel_width - disk_width) / 2 + 1; 
    
    fillet_radius = disk_width * 2;
    union() {
        difference(){
            // Spool
            union() {
                // upper disk   
                translate([0, 0, - disk_offset]) 
                    disk(disk_width, wheel_radius, hub_radius);
                    
                // Hub
                cylinder(h = wheel_width,r = hub_radius, center = true);
                
                // Spikes
                spikes(hub_radius, wheel_width, disk_width);
                    
                // Lower disk
                translate([0, 0, disk_offset]) rotate([180,0,0])
                    disk(disk_width, wheel_radius, hub_radius);              
            }
            
            // I am putting addition 0.1 step somewhere
            // to not to have super thin artifacts
            translate([0,0,-0.1])
            // The "cake" to be extracted from the top of the spoll
            union() {
                // Axle
                intersection() {
                    cylinder(h = wheel_width*2, r = 2, center = true);
                    // Active axile has non-round cross-section
                    if (active) {
                        translate([0.5,0,0])
                            cube([4,4,wheel_width*2], center = true);
                    }
                }
                // Wider cutter
                // Fillet is the union of cylinder and fillet anti-ring
                union() {
                    translate([0,0,-(wheel_width - disk_width) / 2])
                        cylinder(h = fillet_radius, r = hub_radius-fillet_radius*2 + 0.1, center = true);
                    
                    rotate_extrude() {
                    // Magic numbers are from `disk` declaration
                        translate([hub_radius-fillet_radius*2,-disk_offset - (fillet_radius / 2) * 0.3]) {
                            difference() {
                                square([fillet_radius, fillet_radius]);
                                translate([fillet_radius,fillet_radius])
                                    circle(r = fillet_radius);
                            }
                        }
                    }
                }
                
                // Smaller cutter
                translate([0,0,-(wheel_width - disk_width) / 2 + fillet_radius/2])
                    cylinder(h = fillet_radius/2, r = (hub_radius - fillet_radius)/2 +0.1 , center = true);
                
                translate([0,0,-(wheel_width - disk_width) / 2 + fillet_radius/2])
                    rotate_extrude() {
                        translate([(hub_radius - fillet_radius) /2,0]) {
                            intersection() {
                                square([fillet_radius, fillet_radius]);
                                circle(r = fillet_radius/4);
                            }
                        }
                    }
            }
        }
    }
}
rotate([180,0,0])
    complex_wheel();