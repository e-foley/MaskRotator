# MaskRotator
Arduino code driving a rotation mechanism for telescope aperture masks. All code in this repository except for the TimerOne library was written by Ed Foley.

## Hardware required
* [Arduino Uno](https://store.arduino.cc/usa/arduino-uno-rev3)
* [Arduino Motor Shield](https://store.arduino.cc/usa/arduino-motor-shield-rev3)
* [SparkFun Electronics stepper motor ROB-09238](https://www.sparkfun.com/products/9238)
* [SunFounder Hall switch TS0215](https://www.sunfounder.com/switch-hall-sensor-module.html)
* Compatible USB cable
* Compatible power supply
* Aperture mask rotation mechanism assembly (see Ed Foley's thesis, *A Rotating Aperture Mask for Small Telescopes*, published 2019)

## Software required
* [Arduino IDE](https://www.arduino.cc/en/Main/Software)

## Wiring
### Stepper
Stepper wire | Arduino terminal
------------ | ----------------
A (red) | A+
B (yellow) | B+
C (green) | A-
D (blue) | B-

### Hall switch
Hall switch line | Arduino pin
---------------- | -----------
SIG | 5
VCC | 4
GND | GND

## Running
1. Clone this repository.
2. Set the Arduino sketchbook location to the new MaskRotator directory.
3. Open mask_rotator.ino using the Arduino IDE.
4. Connect the power supply to the Arduino Uno.
5. Connect the computer to the Arduino Uno with a USB cable.
6. Press Ctrl+U to upload the sketch.

## Communicating with the device
Either
* Open the serial monitor with Ctrl+Shift+M and type commands manually (e.g. `rt18000` to spin the mask 180 degrees), or
* Execute a custom program that implements the rotator's communication protocol.

See *A Rotating Aperture Mask for Small Telescopes* for complete information about the communications protocol, or inspect the source of mask_rotator.ino.
