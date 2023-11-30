$fn = 50;

// 5.5 diameter
bolt_hole_r = 5.5 / 2;

module GoPro_Connection()
{
	translate([0,3,0])
	{
		difference()
		{
			// tabs
			union()
			{
				cube([2.8,10,10]);
				translate([0,10,5]) rotate([90,0,90]) cylinder(2.8,5,5);
		
				// note: center tab is slightly larger the the other two tabs
				translate([6.8,0,0])
				{
					cube([3,10,10]);
					translate([0,10,5]) rotate([90,0,90]) cylinder(3,5,5);
				}

				translate([12.8,0,0])
				{
					cube([2.8,10,10]);
					translate([0,10,5]) rotate([90,0,90]) cylinder(2.8,5,5);
				}
			}

			// bolt hole
			translate([-7, 10, 5]) rotate([0,90,0]) cylinder(30, bolt_hole_r, bolt_hole_r);
		}
	}

	// connection block
	cube([15.6,3,10]);
}

GoPro_Connection();

