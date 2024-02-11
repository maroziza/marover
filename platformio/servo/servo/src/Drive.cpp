#include "Drive.hpp"

#include <utility>
#include <cmath>


namespace servo
{
    Drive::Drive(device_kit::pca9685::Pca9685&& pwm)
    :   pwm_ {std::move(pwm)}
    {
    }


    void Drive::throttle(float value)
    {
        if (std::isfinite(value))
        {
            std::uint16_t const duty_cycle = static_cast<std::uint16_t>(
                std::min(std::abs(value), 1.f) * pwm_.countsPerCycle());

            // First reset the PWM channel that is idle,
            // then set the PWM channel that drives in the desired direction.
            if (value > 0.f)
            {
                dutyCycle(0, 0);
                dutyCycle(1, duty_cycle);
            }
            else
            {
                dutyCycle(1, 0);
                dutyCycle(0, duty_cycle);
            }
        }
    }


    void Drive::dutyCycle(unsigned channel, std::uint16_t value)
    {
        pwm_.dutyCycle(channel, value > 0 ? 0 : pwm_.countsPerCycle(), value);
    }
}