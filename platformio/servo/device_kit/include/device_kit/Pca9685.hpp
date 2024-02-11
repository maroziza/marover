#pragma once

#include <bus_io/I2cSlave.hpp>

#include <cstdint>


namespace device_kit :: pca9685
{
	// PCA9685 MODE1 register bits
	static std::uint8_t constexpr MODE1_RESTART = 1 << 7;
	static std::uint8_t constexpr MODE1_EXTCLK = 1 << 6;
	static std::uint8_t constexpr MODE1_AI = 1 << 5;
	static std::uint8_t constexpr MODE1_SLEEP = 1 << 4;


	/***
	 * @brief PCA9685 PWM modulator
	 **/
	class Pca9685
	{
	public:
		/// @brief Constructor
		///
		/// NOTE: the constructor sets MODE1 AI bit,
		/// because other functions rely on automatic register address increment.
		///
		/// @param slave I2C slave communication object
		///
		Pca9685(bus_io::I2cSlave&& slave);

		/// @brief Move constructor
		///
		Pca9685(Pca9685&&) = default;

		/// @brief Move assignment
		///
		Pca9685& operator=(Pca9685&&) = default;

		/// @brief Set duty cycle for the given channel
		///
		/// @param channel channel number
		/// @param time_on count on which the output goes to logical 1.
		/// Value >= @a countsPerCycle() means "never".
		/// @param time_off count on which the output goes to logical 0.
		/// Value >= @a countsPerCycle() means "never".
		///
		/// NOTE: if @a time_on == @a time_off or both are >= @a countsPerCycle(),
		/// then the behavior is undefined.
		///
		/// @throw @a std::out_of_range if @a channel >= @a channelCount().
		///
		void dutyCycle(unsigned channel, std::uint16_t time_on, std::uint16_t time_off);

		/// @brief Number of counts per cycle.
		///
		/// The duty cycle resulution is 1 count.
		///
		/// @return number of counts per cycle.
		///
		std::uint16_t constexpr countsPerCycle() const
		{
			return 1 << bits_;
		}

		/// @brief Get number of PWM channels
		///
		/// @return the number of PWM channels
		///
		std::uint8_t constexpr channelCount() const
		{
			return 16;
		}

		/// @brief Set PWM frequency
		///
		/// @param value desired PWM frequency in Hz
		///
		/// @throw @a std::invaid_argument if the value is out of supported range
		/// @throw @a std::runtime_error on other failures
		///
		void frequency(float value);

		/// @brief Get PWM frequency
		///
		/// @return current PWM frequency in Hz
		///
		/// @throw @a std::runtime_error on failure
		///
		float frequency() const;

		/// @brief Read MODE1 register
		///
		/// @return MODE1 register value
		///
		/// @throw @a std::system_error on failure
		///
		std::uint8_t mode1() const;

		/// @brief Write MODE1 register
		///
		/// @param value new value for MODE1 register
		///
		/// @throw @a std::system_error on failure
		///
		void mode1(std::uint8_t value);

	private:
		mutable bus_io::I2cSlave slave_;
		static unsigned constexpr bits_ = 12;

		// Oscillator frequency in Hz
		static float constexpr oscillatorFrequency_ = 25e+6f;

		// MODE1
		static unsigned constexpr regMode1_ = 0x00;

		// Start address of LED control registers (LED0_ON_L)
		static unsigned constexpr regLedStart_ = 0x06;

		// PRE_SCALE
		static unsigned constexpr regPreScale_ = 0xfe;
	};
}