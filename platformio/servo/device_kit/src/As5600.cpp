#include <device_kit/As5600.hpp>

#include <stdexcept>
#include <utility>
#include <cmath>


namespace device_kit
{
    As5600::As5600(bus_io::I2cSlave&& slave)
    :   slave_ {std::move(slave)}
    {
    }


    void As5600::configure(As5600Settings const&)
    {
        // slave_.writeWord(reg_config, r);
    }


    As5600Angle As5600::readAngle()
    {
        std::uint8_t angle_registers[4];
        slave_.read(regRawAngle_, angle_registers);

        As5600Angle angle;
        angle.rawAngle = static_cast<std::uint16_t>(angle_registers[0] << 8 | angle_registers[1]);
        angle.angle = static_cast<std::uint16_t>(angle_registers[2] << 8 | angle_registers[3]);
        angle.angleRadians = 2.f * M_PIf * angle.angle / (1U << bits_);

        return angle;
    }


    As5600Status As5600::readStatus()
    {
        std::uint8_t status_registers[4];
        slave_.read(regStatus_, status_registers);

        return As5600Status {
            .magnetDetected = (status_registers[0] & 0b00100000) != 0,
            .magnetTooWeak = (status_registers[0] & 0b00010000) != 0,
            .magnetTooStrong = (status_registers[0] & 0b00001000) != 0,
            .automaticGainControl = status_registers[1],
            .magnitude = static_cast<std::uint16_t>(status_registers[2] << 8 | status_registers[3])
        };
    }
}