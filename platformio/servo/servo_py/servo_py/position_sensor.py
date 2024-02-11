class PositionSensor:
    def __init__(self, adc_channel, bits: int):
        self._adc_channel = adc_channel
        self._adc_max = 2 ** (bits - 1)

    def read(self) -> float:
        return self._adc_channel.value / self._adc_max