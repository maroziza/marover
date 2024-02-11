#pragma once

#include <device_kit/Pca9685.hpp>


namespace servo
{
    /// @brief Controls motor thrust and direction
    ///
    class Drive
    {
    public:
        /// @brief Constructor
        ///
        /// @param pwm PWM driver object
        ///
        explicit Drive(device_kit::pca9685::Pca9685&& pwm);

        /// @brief Set throttle and direction
        ///
        /// @param value throttle value. -1.f is full reverse, 1.f is full forward.
        /// Values outside this range are clipped.
        /// Values which are not finite numbers are ignored.
        ///
        void throttle(float value);

    private:
        void dutyCycle(unsigned channel, std::uint16_t value);

        device_kit::pca9685::Pca9685 pwm_;
    };
}