module array(arr) {
    for(a = arr) translate(a) children();
}

module squa(x,y) {
array([[x/2,y/2,0],[x/2,-y/2,0],[-x/2,y/2,0],[-x/2,-y/2,0]])
    children();
}

function trianglePoint(tri)  = let
(c=tri[0], b=tri[1], a=tri[2], xc = (a*a-b*b+c*c)/(2*c)) 
[c/2-xc
,sqrt(a*a-xc*xc)
,0];

module triangle(tri) {
    array([
    [-tri[0]/2,0], 
    [tri[0]/2,0],
    trianglePoint(tri)]) children();
}

module screw(H) {
    cylinder(h=H, d=3.3);
    translate([0,0,H-2]) cylinder(d=6.2, h=10+H);
}

module outerWalls(width=1.2) {
    a = $fn;
    difference() {
        minkowski() {
    $fn =a;

            children();
            sphere(width, $fn=8);
        }
    $fn =a;

        //children();
    }
}
// leaving walls only, this does not work because scale center point is not right, use outerwall for a while
module walls(width=1) {
    
minkowski() {
    difference() {
        children();
    scale([0.99,0.99,0.99]) children();
    }
    cube(width);
}

}
$fn= $preview ? 16 : 160;
module linearr(step=0, labels=[], count = 0) {
    count = (count == 0) ? len(labels) : count;
    for(i = [0 : count-1]) translate([i*step,0,0]){
    $label = labels[i];
    children();
    }
}