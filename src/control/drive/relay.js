const RELAY_FORWARD = 1;
const RELAY_BACKWARD = 2;
const RELAY_FULLTHROTTLE = 4;

export function RelayDrive(pwm, relays, toZero = 0.05, toFull=0.95) {
    return function (t) {
        if((t<toZero) && (-toZero <t)) {
            relays(0);
            pwm(0);
            return 0;
        }

        if(t < 0) {
            if(t<-toFull) {
                relays(RELAY_BACKWARD|RELAY_FULLTHROTTLE);
                pwm(0);
                return -1;
            } else {
                relays(RELAY_BACKWARD);
                pwm(-t);
                return t;
            }
        } else {
            if(t>toFull) {
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
