export function HBridgeDrive(pwmForward, pwmBackward) {
    return function(t) {
        if(t>0) {
            pwmBackward(0);
            pwmForward(t);
            } else {
            pwmForward(0);
            pwmBackward(-t);
            }
        }
        return this;

    }
