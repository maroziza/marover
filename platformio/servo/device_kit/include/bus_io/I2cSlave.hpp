#pragma once

#include <bus_io/File.hpp>

#include <linux/i2c-dev.h>

#include <utility>
#include <cstdint>


namespace bus_io
{
    /// @brief I2C connection with a slave device
    ///
    class I2cSlave
    {
    public:
        /// @brief Initialize communication with I2C slave device
        ///
        /// @param device_name name of the device file for I2C I/O
        /// @param addr device address
        ///
        ///
        I2cSlave(std::string const& device_name, std::uint8_t addr)
        :   file_ {device_name, O_RDWR}
        {
            file_.ioctl(I2C_SLAVE, static_cast<unsigned long>(addr));
        }

        I2cSlave(I2cSlave const&) = delete;
        I2cSlave(I2cSlave&&) = default;
        I2cSlave& operator=(I2cSlave const&) = delete;
        I2cSlave& operator=(I2cSlave&&) = default;

        /// @brief Write 16-bit unsigned value to a register on I2C device
        ///
        /// @param reg register number
        /// @param data to write
        ///
        /// @throw @a std::runtime_error on failure
        ///
        void writeWord(std::uint8_t reg, std::uint16_t data)
        {
            std::uint8_t const buf[] = {
                reg,
                static_cast<std::uint8_t>(data >> 8),
                static_cast<std::uint8_t>(data & 0x00ff)
            };

            if (file_.write(buf, sizeof(buf)) != sizeof(buf))
                throw std::runtime_error {"Wrong number of bytes written to I2C device"};
        }

        /// @brief Read 16-bit unsigned value from a register on I2C device.
        ///
        /// @param reg register number
        ///
        /// @return the value read from the register
        ///
        /// @throw @a std::runtime_error on failure
        ///
        std::uint16_t readWord(std::uint8_t reg)
        {
            std::uint8_t buf[sizeof(std::uint16_t)];
            read(reg, buf, sizeof(buf));

            return (static_cast<std::uint16_t>(buf[0]) << 8) | static_cast<std::uint16_t>(buf[1]);
        }


        /// @brief Write 8-bit unsigned value to a register on I2C device
        ///
        /// @param reg register number
        /// @param data to write
        ///
        /// @throw @a std::runtime_error on failure
        ///
        void writeByte(std::uint8_t reg, std::uint8_t data)
        {
            std::uint8_t const buf[] = {reg, data};

            if (file_.write(buf, sizeof(buf)) != sizeof(buf))
                throw std::runtime_error {"Wrong number of bytes written to I2C device"};
        }

        /// @brief Read 8-bit unsigned value from a register on I2C device.
        ///
        /// @param reg register number
        ///
        /// @return the value read from the register
        ///
        /// @throw @a std::runtime_error on failure
        ///
        std::uint8_t readByte(std::uint8_t reg)
        {
            std::uint8_t data;
            read(reg, &data, sizeof(data));
            return data;
        }


        /// @brief Read block of registers from an I2C device.
        ///
        /// @tparam N number of bytes to read
        ///
        /// @param reg first register address
        /// @param data output data buffer
        ///
        /// @throw @a std::runtime_error on failure
        ///
        template <unsigned N>
        inline void read(std::uint8_t reg, std::uint8_t (&data)[N])
        {
            read(reg, data, sizeof(data));
        }


        /// @brief Write block of registers to an I2C device.
        ///
        /// @tparam N number of bytes to write
        ///
        /// @param reg first register address
        /// @param data data buffer
        ///
        /// @throw @a std::runtime_error on failure
        ///
        template <unsigned N>
        inline void write(std::uint8_t reg, std::uint8_t const (&data)[N])
        {
            std::uint8_t buf[N + 1];
            buf[0] = reg;
            std::copy_n(data, N, buf + 1);

            if (file_.write(buf, sizeof(buf)) != sizeof(buf))
                throw std::runtime_error {"Wrong number of bytes written to I2C device"};
        }


    private:
        /// @brief Read block of registers from an I2C device.
        ///
        /// @param reg first register address
        /// @param data output data buffer
        /// @param size number of bytes to read
        ///
        /// @throw @a std::runtime_error on failure
        ///
        void read(std::uint8_t reg, void * data, unsigned size)
        {
            if (file_.write(&reg, sizeof(reg)) != sizeof(reg))
                throw std::runtime_error {"Wrong number of bytes written to I2C device"};

            if (file_.read(data, size) != size)
                throw std::runtime_error {"Wrong number of bytes read from I2C device"};
        }


        File file_;
    };
}