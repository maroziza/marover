#include <device_kit/Ads1115.hpp>

#include <stdexcept>
#include <utility>


namespace device_kit
{
    Ads1115::Ads1115(bus_io::I2cSlave&& slave)
    :   slave_ {std::move(slave)}
    {
    }


    void Ads1115::configure(Ads1115Settings const& settings)
    {
        // Enable RDY
        slave_.writeWord(reg_lo_thres, 0x0000);
        slave_.writeWord(reg_hi_thres, 0x8000);

        std::uint16_t r = (0b10000000 << 8); // kick it all off
        r |= (1 << 2) | (1 << 3); // data ready active high & latching
        r |= static_cast<std::uint16_t>(samplingRateToCode(settings.samplingRate) << 5);
        r |= static_cast<std::uint16_t>(settings.pgaGain << 9);
        r |= static_cast<std::uint16_t>((settings.channel & 0b111) << 12 | 1 << 14); // unDSipolar
        slave_.writeWord(reg_config, r);
    }


    int Ads1115::readValue()
    {
        return static_cast<std::int16_t>(slave_.readWord(0));
    }


    float Ads1115::fullScaleVoltage()
    {
        // switch (settings_.pgaGain) {
        // case ADS1115settings::FSR2_048:
        //     return 2.048f;
        // case ADS1115settings::FSR1_024:
        //     return 1.024f;
        // case ADS1115settings::FSR0_512:
        //     return 0.512f;
        // case ADS1115settings::FSR0_256:
        //     return 0.256f;
        // }
        // throw std::logic_error {"Invalid pgaGain value"};
        throw std::logic_error {"Ads1115::fullScaleVoltage() not implemented"};
    }


    std::uint8_t Ads1115::samplingRateToCode(unsigned sampling_rate)
    {
        /**
		 * Sampling rates
		 **/
		enum SamplingRates {
			FS8HZ   = 0,
			FS16HZ  = 1,
			FS32HZ  = 2,
			FS64HZ  = 3,
			FS128HZ = 4,
			FS250HZ = 5,
			FS475HZ = 6,
			FS860HZ = 7
		};

        switch (sampling_rate)
        {
            case 8: return 0;
            case 16: return 1;
            case 32: return 2;
            case 64: return 3;
            case 128: return 4;
            case 250: return 5;
            case 475: return 6;
            case 860: return 7;
            default:
                throw std::invalid_argument {"Unsupported samling rate " + std::to_string(sampling_rate)};
        }
    }
}