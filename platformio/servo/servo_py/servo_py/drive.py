import math

class Drive:
    def __init__(self, pwm_driver):
        pwm_driver.channels[0].duty_cycle = 0x0
        pwm_driver.channels[1].duty_cycle = 0x0
        self._pwm_driver = pwm_driver

    def set_throttle(self, throttle: float):
        if math.isfinite(throttle):
            duty_cycle = int(min(math.fabs(throttle), 1.0) * 0xffff)

            # First reset the PWM channel that is idle,
            # then set the PWM channel that drives in the desired direction.
            if throttle > 0.:
                self._pwm_driver.channels[1].duty_cycle = 0
                self._pwm_driver.channels[0].duty_cycle = duty_cycle
            else:
                self._pwm_driver.channels[0].duty_cycle = 0
                self._pwm_driver.channels[1].duty_cycle = duty_cycle
