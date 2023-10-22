Available examples:

- `blink` - blinks with red led on board
- `wifi` - softAP or connection (from `tank`)
- `tank` - RC vehicle example based on [https://github.com/devinzhang91/esp32_sv_tank](https://github.com/devinzhang91/esp32_sv_tank) and `CameraWevServer`
- `CameraWebServer` -  example for esp32 available in Arduino IDE


To build specific example please update src_dir property at [platformio.ini](platformio.ini)

For example 
```
[platformio]
src_dir = ./src/CameraWebServer
```

