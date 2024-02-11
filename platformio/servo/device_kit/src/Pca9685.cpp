#include <device_kit/Pca9685.hpp>

#include <stdexcept>
#include <utility>
#include <cmath>
#include <thread>


namespace device_kit :: pca9685
{
    Pca9685::Pca9685(bus_io::I2cSlave&& slave)
    :   slave_ {std::move(slave)}
    {
        // Enable register auto increment
        mode1(mode1() | MODE1_AI);
    }


    void Pca9685::dutyCycle(unsigned channel, std::uint16_t time_on, std::uint16_t time_off)
    {
        if (channel >= channelCount())
            throw std::out_of_range {"Channel number out of range"};

        bool const does_go_off = time_off < countsPerCycle();
        bool const does_go_on = time_on < countsPerCycle();
        std::uint16_t const led_on = does_go_off ? (does_go_on ? time_on : 0) : countsPerCycle();
        std::uint16_t const led_off = does_go_on ? (does_go_off ? time_off : 0) : countsPerCycle();

        // {LED_ON_L, LED_ON_H, LED_OFF_L, LED_OFF_H}
        std::uint8_t const data[4] = {
            static_cast<std::uint8_t>(led_on & 0xff),
            static_cast<std::uint8_t>(led_on >> 8),
            static_cast<std::uint8_t>(led_off & 0xff),
            static_cast<std::uint8_t>(led_off >> 8)
        };

        slave_.write(regLedStart_ + static_cast<std::uint8_t>(4 * channel), data);
    }


    void Pca9685::frequency(float value)
    {
        using namespace std::chrono_literals;

        float const prescale_value = std::round(oscillatorFrequency_ / (4096.f * value)) - 1.f;
        if (prescale_value < 0x03 || prescale_value > 0xff)
            throw std::invalid_argument {"Requested PWM frequency out of range"};

        // The PRE_SCALE register can only be set when the SLEEP bit of MODE1 register is set to logic 1.
        // See section 7.3.1.1 Restart mode of PCA9685 datasheet for the explanation of the following sequence.
        std::uint8_t const old_mode1 = mode1();
        slave_.writeByte(regMode1_, old_mode1 | MODE1_SLEEP);
        slave_.writeByte(regPreScale_, static_cast<std::uint8_t>(prescale_value));
        slave_.writeByte(regMode1_, old_mode1);
        std::this_thread::sleep_for(500us);
        slave_.writeByte(regMode1_, old_mode1 | MODE1_RESTART);
    }


    float Pca9685::frequency() const
    {
        return oscillatorFrequency_ / (4096.f * (slave_.readByte(regPreScale_) + 1));
    }


    std::uint8_t Pca9685::mode1() const
    {
        return slave_.readByte(regMode1_);
    }


    void Pca9685::mode1(std::uint8_t value)
    {
        slave_.writeByte(regMode1_, value);
    }
}