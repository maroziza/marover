#pragma once

#include <bus_io/I2cSlave.hpp>


// #define ISR_TIMEOUT 1000

// default address if ADDR is pulled to GND
#define DEFAULT_ADS1115_ADDRESS 0x48


namespace device_kit
{
	/***
	 * @brief ADS1115 settings
	 **/
	struct Ads1115Settings
	{
	public:
		/***
		 * @brief Sampling rate in Hz
		 **/
		unsigned samplingRate = 8;

		/**
		 * Full scale range: 2.048V, 1.024V, 0.512V or 0.256V.
		 **/
		enum PGA {
			FSR2_048 = 2,
			FSR1_024 = 3,
			FSR0_512 = 4,
			FSR0_256 = 5
		};

		/**
		 * Requested full scale range
		 **/
		PGA pgaGain = FSR2_048;

		/**
		 * Input channel
		 **/
		std::uint16_t channel = 0;
	};


	/***
	 * @brief This class reads data from the ADS1115 in the background (separate
	 * thread) and calls a callback function whenever data is available.
	 **/
	class Ads1115
	{
	public:
		Ads1115(bus_io::I2cSlave&& slave);

		Ads1115(Ads1115&&) = default;
		Ads1115& operator=(Ads1115&&) = default;

		/***
		 * @brief Configure the chip
		 *
		 * \param settings A struct with the settings.
		 **/
		void configure(Ads1115Settings const& settings);

		/***
		 * @brief Get current settings
		 *
		 * @return the current settings
		 **/
		Ads1115Settings settings() const
		{
			throw std::logic_error {"Ads1115::settings() not implemented"};
		}

		/// @brief Read raw value from ADC
		///
		/// @return Raw ACD value from the range [-range(), range)
		///
		int readValue();

		/// @brief Range of ADC readings.
		///
		/// @return ADC readings range.
		/// The values returned by @a readValue() are in the range [-range(), range)
		///
		unsigned constexpr range() const noexcept
		{
			return 1 << (bits_ - 1);
		}

	private:
		bus_io::I2cSlave slave_;
		static unsigned constexpr bits_ = 16;

		const uint8_t reg_config = 1;
		const uint8_t reg_lo_thres = 2;
		const uint8_t reg_hi_thres = 3;

		float fullScaleVoltage();
		std::uint8_t samplingRateToCode(unsigned sampling_rate);
	};


	/// @brief Read normalized value from ADC
	///
	/// @return ACD value normalized to the range [-1., 1.)
	///
	inline float readNormalizedValue(Ads1115& adc)
	{
		return static_cast<float>(adc.readValue()) / static_cast<float>(adc.range());
	}
}