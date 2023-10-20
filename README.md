
Small educational robot platform for $25 base price. Build around 3d print body

![marover](docs/images/look.jpg)

# Features
* skid steer mode of operation (**supported**)
* motor+steering wheel (**pending**)

* WiFi online control (**supported**)
* bluetooth joystick (**pending**)
* CRSF radio (**pending**)
* SBUS radio (**pending**)
* GPS positioning
* i2c peripherials

# BOM
## body
* 1x [body](model/rover.scad)
* 2x wheel set [passive + active](model/sv_tank/PLA_sv_wheels_2x.stl)
* 2x track (eiter [high profile](model/sv_tank/TPU_sv_track2.stl) or [low profile](model/sv_tank/TPU_sv_track1.stl))
* 2x M4 screw
* 2x M4 nuts


## control
Control hardware can be implemented in two flavours, more cheap and common esp32-cam, or more powerfull sipeed m1s dock
### ESP32-Camera
![esp32 cam pinout](docs/pinout/ESP32-CAM.png)
Docs: http://www.ai-thinker.com/pro_view-24.html

### Sipeed M1S dock
![m1s dock pinout](docs/pinout/m1s_dock.png)
Docs: https://wiki.sipeed.com/hardware/en/maix/m1s/m1s_dock.html

## equipment
### laser cannon
* https://arduino.ua/prod2551-servoprivid-tower-pro-mg995-180
* https://arduino.ua/prod1525-lazer-krasnii-krest-3v-650nm-5mvt-9mm



# links
* [https://github.com/devinzhang91/esp32_sv_tank](https://github.com/devinzhang91/esp32_sv_tank)


