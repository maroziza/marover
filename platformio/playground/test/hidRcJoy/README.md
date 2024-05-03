cloned from https://github.com/patolin/rc-receiver-joystick and heavily modified, README is mostly accurate, though

# rc-receiver-joystick
A super simple adapter to connect a RC Receiver, to an USB port, using a Digispark board.

The board uses the PPM output of a 6 channels RC Receiver, to enable a USB Joystick, 
that enable the use of a RC Transmitter in RC simulators on the PC

**Requirements:**

1. Digispark board (http://digistump.com/products/1)
2. Arduino IDE 2+
3. Digistump libraries hack, to enable interrupt on change in IO pins (https://github.com/J-Rios/Digispark_DigiUSB-SoftSerial)
4. 2 Dupont extensions (to connect the Digispark board to the RC receiver)
4. Some soldering
5. A couple of hot drinks ;)

**Schematic:**

```
RC Receiver               Digispark board

PPM Signal -------------- Pin 1
                     |--- Pin 2 *
                     |--- Pin 3 *
                          Pin 4
Vcc        -------------- Digispark +5V 
Gnd        -------------- Digispark GND
```

*Pins 2 and 3 must be short in order to enable USB device (because of the library hack)

Tested with a Hobbyking 3 channel transmitter and receiver, and VRC simulator. Works great, just like the original VRC adapter. Photos and more info (I will upload a video soon) in http://patolin.com/blog/2018/05/29/receptor-rc-digispark/

Also tested with XR9 receiver