# MaskRotator
Arduino code driving a rotation mechanism for telescope aperture masks.

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
3. Open mask_rotator.ino.
4. Connect to the Arduino with a USB cable.
5. Press Ctrl+U to upload the sketch.
