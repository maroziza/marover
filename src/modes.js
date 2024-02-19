function Debug(proxy, printer) {
return function(data) {
	printer(data);
	proxy(data);
	return data;
}
}
function DriveWaitNoCurrent( current, proxy) {
	return function (t) {
if(current() >0 ) {
	proxy(0);
	return this;
	} else {
		return proxy;
	}
}
}
const RELAY_FORWARD = 1;
const RELAY_BACKWARD = 2;
const RELAY_FULLTHROTTLE = 4;
function RelayDrive(pwm, relays, toZero = 0.05, toFull=0.95) {
	return function (t) {
		if((t<0.05) && (-0.05 <t)) {
			relays(0);
			pwm(0);
			return 0;
		}

		if(t < 0) {
			if(t<-0.95) {
				relays(RELAY_BACKWARD|RELAY_FULLTHROTTLE);
				pwm(0);
				return -1;
			} else {
				relays(RELAY_BACKWARD);
				pwm(-t);
				return t;
			}
		} else {	
			if(t>0.95) {
				relays(RELAY_FORWARD|RELAY_FULLTHROTTLE);
				pwm(0);
				return 1;
			} else {
				relays(RELAY_FORWARD);
				pwm(t);
				return t;
			}
		}
	}
}
function HBridgeDrive(pwmForward, pwmBackward) {
	return function(t) {
		if(t>0) {
			pwmBackward(0);
			pwmForward(t);
			} else {
			pwmForward(0);
			pwmBackward(-t);
			}
		}
	
	}
/**
var input = readData();
var state = updateState(was, input);
var output = process(state);
*/

var drive = RelayDrive(console.log, console.log)
drive(-.4);



