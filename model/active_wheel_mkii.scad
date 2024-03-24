$fa = 1;
$fs = 0.4;


module hub(hub_radius) {
  cylinder(h = wheel_width, r = hub_radius, center = true);
}

module spikes(min_radius, max_radius, wheel_width, number=12, spike_width = 1.5) {
  spike_angle = min_radius - spike_width;

  // TODO calc it counting on the spike_angle
  // spike_length = sqrt((max_radius) ^ 2 + (spike_angle + spike_width) ^ 2);
  spike_length = max_radius + spike_angle;
  spike_dimensions = [spike_length, spike_width, wheel_width];
  spike_position = [min_radius - spike_angle - spike_width, spike_angle, - wheel_width / 2];
  intersection() {
    for(i=[1:1:number]) {
      rotate([0,0,i*360/number]) {
        translate(spike_position) {
          difference() {
            cube(spike_dimensions);
          }
        }
      }
    }
    // TODO if I can calculate the spike length more acurately
    // I wont need cutting their edges, sticking from the tire outside
    // This is to trim the spikes:
    cylinder(r=max_radius, h=2 * wheel_width, center = true);
  }
}


module rim(min_radius, max_radius, wheel_width) {
  // Profile of the rubber
  module tire_marks(tire_radius, wheel_width, factor=112) {
    for(i=[1:1:factor]) {
      tire_mark_size = wheel_width/2 + 1;
      position = i % 2 == 0 ? 1 : -1;

      rotate([position* 20, position * 2, i*360/factor]) {
        translate([tire_radius, 0, position * tire_mark_size/2]) {
          cube([1, 1, tire_mark_size], center=true);
        }
      }
    }
  }

  difference() {
    union() {

      cylinder(h = wheel_width, r = max_radius, center = true);
    }
    // Cutter
    union(){
      tire_marks(max_radius, min_radius, factor=98);
      cylinder(h = wheel_width * 1.1, r = min_radius, center = true);
    }
  }
}

module fillet_cutter(wheel_radius, wheel_width) {
  min_radius = wheel_radius * 0.65;
  max_radius = wheel_radius * 1.3;

  module cutter(direction = 1) {
    r1 = direction > 0 ? max_radius : min_radius;
    r2 = direction < 0 ? max_radius : min_radius;

    translate([0, 0, direction * wheel_width/2]){
      union() {
        translate([0, 0, direction * wheel_width/4])
          cube([wheel_radius * 2, wheel_radius * 2, wheel_width / 2], center=true);
        cylinder(r1=r1,
                 r2=r2,
                 h=wheel_width,
                 center=true);
      }
    }
  }
  // Upper
  cutter();
  // Lower
  cutter(-1);
}


module hole_cutter(wheel_radius,
                   wheel_width,
                   is_active,
                   axle_d,
                   axle_smaller_d,
                   stopper_bolt_d) {

  module keyseat(wheel_width, axle_d, is_active) {
    intersection() {
        cylinder(h = wheel_width, d = axle_d);
        // Active axile has non-round cross-section
        if (is_active) {
          // TODO formalize it somehow
          // the cut on the axle gives h2.5 instaed of ø3
          translate([0, 0.5, 0])
            cube([axle_d, axle_d, wheel_width*2], center = true);
        }
      }
  }

  module axle() {
    union(){
      keyseat(wheel_width, axle_d, is_active);
      translate([0, 0, - wheel_width])
        keyseat(wheel_width, axle_smaller_d, is_active);

    }
  }

  module stopper() {
    translate([0,0,0]) {
      rotate([90,0,0]){
        cylinder(h=wheel_radius, d=stopper_bolt_d);
      }
    }
  }

  union() {
    // AXLE
    axle();
    // STOPPER BOLT HOLE
    // Not needee for TPU built, as it sits on tension
    if (make_stopper_hole)
      stopper();
  }
}


module base_wheel(wheel_radius, wheel_width, axle_number, spike_width) {
  // Variables
  // Numbers are pure magic. If you need more/less rigid wheel - play here
  hub_radius = wheel_radius * 0.25;
  spikes_radius = wheel_radius * 0.85;
  rim_radius = wheel_radius;

  union() {
    // HUB
    color("#5f9ea0") hub(hub_radius);

    // SPIKES
    color("#8b4513") spikes(hub_radius, spikes_radius, wheel_width, axle_number, spike_width);

    // RIM
    color("#Ffdead") rim(spikes_radius, rim_radius, wheel_width);
  }
}


module complex_wheel(wheel_radius,        // Radius of the wheel       | mm
                     wheel_width,         // Width of the "tyre"	   | mm
                     is_active,           // Active weel has a keyseat | true/false
                     axle_d,              // Ø of your engine axle     | mm
                     axle_smaller_d,      // Ø of small axle           | mm
                     stopper_bolt_d,	  // Ø of the stopper bolt     | mm
                     axle_number,         // Number of spokes          | number
                     spike_width          // width of the spoke        | mm
                     ) {
  intersection(){
    difference() {
      base_wheel(wheel_radius, wheel_width, axle_number, spike_width);
      // Cutter
      hole_cutter(wheel_radius,
                  wheel_width,
                  is_active,
                  axle_d,
                  axle_smaller_d,
                  stopper_bolt_d);

    }
    // FILLET
    fillet_cutter(wheel_radius, wheel_width);
  }
}

// Taken from the original stl
wheel_radius=14;
wheel_width=10;

// Taken from the last hardware rev.
stopper_bolt_d = 2.8;
// We are going to make axle hole smaller on one side to
// be able to fit two kind of motors with one wheel
axle_d = 3;
axle_smaller_d = 2;

// Fittable
axle_number = 4;
make_stopper_hole = false;
spike_width = 1.1;

complex_wheel(wheel_radius, wheel_width, true, axle_d, axle_smaller_d, stopper_bolt_d, axle_number, spike_width);
