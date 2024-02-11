#pragma once

#include <bus_io/I2cSlave.hpp>

#include <cstdint>


namespace device_kit
{
	/***
	 * @brief AS5600 settings
	 **/
	struct As5600Settings
	{

	};


	/***
	 * @brief AS5600 status
	 **/
	struct As5600Status
	{
		bool magnetDetected;
		bool magnetTooWeak;
		bool magnetTooStrong;
		std::uint8_t automaticGainControl;
		std::uint16_t magnitude;
	};


	/***
	 * @brief AS5600 angle reading
	 **/
	struct As5600Angle
	{
		std::uint16_t rawAngle;
		std::uint16_t angle;
		float angleRadians;
	};


	/***
	 * @brief AS5600 magnetic rotary position sensor
	 **/
	class As5600
	{
	public:
		As5600(bus_io::I2cSlave&& slave);

		/***
		 * @brief Configure the chip
		 *
		 * \param settings A struct with the settings.
		 **/
		void configure(As5600Settings const& settings);

		/***
		 * @brief Get current settings
		 *
		 * @return the current settings
		 **/
		As5600Settings settings() const
		{
			throw std::logic_error {"As5600::settings() not implemented"};
		}

		/// @brief Read raw value from ADC
		///
		/// @return Raw ACD value
		///
		As5600Angle readAngle();

		/// @brief Get number of bits in angle reading.
		///
		/// @return number of bits in angle reading.
		///
		std::uint8_t constexpr bits() const
		{
			return bits_;
		}

		As5600Status readStatus();

	private:
		bus_io::I2cSlave slave_;
		static std::uint8_t constexpr bits_ = 12;

		static std::uint8_t constexpr regRawAngle_ = 0x0c;
		static std::uint8_t constexpr regAngle_ = 0x0e;
		static std::uint8_t constexpr regStatus_ = 0x0b;
	};
}