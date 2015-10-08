# pySX127x.LoRa

This is a python interface to the Semtech SX127x long range, low power transceiver family 
[SX1272, SX1273, SX1276, SX1277, SX1278, SX1279](http://www.semtech.com/wireless-rf/rf-transceivers/).
The SX127x has both [LoRa](https://lora-alliance.org) and FSK capabilities. Here the focus lies on the
LoRa spread spectrum modulation. 

Spread spectrum modulation has a number of intriguing features:
* High interference immunity
* Up to 20dBm link budget advantage (for the SX1276/7/8/9)
* High Doppler shift immunity

Links to some LoRa performance reports can be found in the [references](#references) section below.


# Motivation

Transceiver modules are usually interfaced with microcontroller boards such as the 
[Arduino](https://www.arduino.cc/) and there are already many fine C/C++ libraries for the SX127x family on 
[github](https://github.com/search?q=sx127x) and [mbed.org](https://developer.mbed.org/search/?q=sx127x).

Although C/C++ is the de facto standard for development on microcontrollers, [python](https://www.python.org)
running on a [Raspberry Pi](https://www.raspberrypi.org) is becoming a viable alternative for rapid prototyping.

High level programming languages like python require a full-blown OS like Linux. (There are some exceptions like
[PyMite](https://wiki.python.org/moin/PyMite) and most notably [MicroPython](https://micropython.org).)
Using hardware capable of running Linux contradicts, to some extent, the low power specification of the SX127x family.
Therefore it is clear that in most cases this approach is useful mostly for prototyping and technology testing.

On the other hand prototyping on a full-blown OS using high level programming languages has several advantages:
* Working prototypes can be built quickly 
* Technology testing ist faster
* Proof of concept is easier to achieve
* The application development phase is reached quicker 


# Hardware

The transceiver module is a SX1276 based Modtronix [inAir9B](http://modtronix.com/inair9.html). 
It is mounted on a prototyping board to a Raspberry Pi rev 2 model B.

| Proto board pin | RaspPi GPIO | Direction |
|:----------------|:-----------:|:---------:|
| inAir9B DIO0    | GPIO 21     |    IN     |
| inAir9B DIO1    | GPIO 22     |    IN     |
| inAir9B DIO2    | GPIO 23     |    IN     |
| inAir9B DIO3    | GPIO 24     |    IN     |
| LED             | GPIO 25     |    OUT    |

Todo:
- [ ] Add picture(s)
- [ ] Wire the SX127x reset to a GPIO?


# Code Examples

First import the modules 
```python
from SX127x.LoRa import *
from SX127x.board_config import BOARD
```
then set up the board GPIOs
```python
BOARD.setup()
```
The LoRa object is instantiated and put into the standby mode
```python
lora = LoRa()
lora.set_mode(MODE.STDBY)
```
Registers are queried like so:
```python
print lora.version()        # this prints the sx127x chip version
print lora.get_freq()       # this prints the frequency setting 
```
and setting registers is easy, too
```python
lora.set_freq(433.0)       # Set the frequency to 433 MHz 
```
In applications the `LoRa` class should be subclassed while overriding one or more of the callback functions that
are invoked on successful RX or TX operations, for example.
```python
class MyLoRa(LoRa):

  def __init__(self, verbose=False):
    super(MyLoRa, self).__init__(verbose)
    # setup registers etc.

  def on_rx_done(self):
    payload = self.read_payload(nocheck=True) 
    # etc.
```


# Installation

**pySX127x** requires 
* [RPi.GPIO](https://pypi.python.org/pypi/RPi.GPIO") for accessing the GPIOs and
* [spidev](https://pypi.python.org/pypi/spidev) for controlling SPI


# Class Reference

@todo


# Script reference

### `rcv_cont.py`
The SX127x is put in RXCONT mode and continuously waits for transmissions. Upon a successful read the
payload and the irq flags are printed to screen.

### `beacon.py`
A small payload is transmitted in regular intervals.


# Tests

Execute `test_lora.py` to run a few unit tests. 


# Contributors

Please feel free to comment, report issues, or contribute!

Contact me via my company website [Mayer Analytics](http://mayeranalytics.com) and my private blog
[mcmayer.net](http://mcmayer.net). 

Follow me on twitter [@markuscmayer](https://twitter.com/markuscmayer) and
[@mayeranalytics](https://twitter.com/mayeranalytics).


# Version and future plans

**pySX127x** is still in the development phase. The current version is 0.1.

95% of functions for the Sx127x LoRa capabilities are implemented. Functions will be added when necessary.
The test coverage is rather low but we intend to change that soon.


# References

### Hardware references
* [Semtech SX1276/77/78/79 - 137 MHz to 1020 MHz Low Power Long Range Transceiver](http://www.semtech.com/images/datasheet/sx1276_77_78_79.pdf)
* [Modtronix inAir9](http://modtronix.com/inair9.html)
* [Spidev Documentation](http://tightdev.net/SpiDev_Doc.pdf)
* [Make: Tutorial: Raspberry Pi GPIO Pins and Python](http://makezine.com/projects/tutorial-raspberry-pi-gpio-pins-and-python/)

### LoRa performance tests
* [Extreme Range Links: LoRa 868 / 900MHz SX1272 LoRa module for Arduino, Raspberry Pi and Intel Galileo](https://www.cooking-hacks.com/documentation/tutorials/extreme-range-lora-sx1272-module-shield-arduino-raspberry-pi-intel-galileo/)
* [UK LoRa versus FSK - 40km LoS (Line of Sight) test!](http://www.instructables.com/id/Introducing-LoRa-/step17/Other-region-tests/)

### Spread spectrum modulation theory
* [An Introduction to Spread Spectrum Techniques](http://www.ausairpower.net/OSR-0597.html)
* [Theory of Spread-Spectrum Communications-A Tutorial](http://www.fer.unizg.hr/_download/repository/Theory%20of%20Spread-Spectrum%20Communications-A%20Tutorial.pdf)
(technical paper)


# License

**pySX127x** is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

pySX127x is distributed in the hope that it will be useful,
but **WITHOUT ANY WARRANTY**; without even the implied warranty of
**MERCHANTABILITY** or **FITNESS FOR A PARTICULAR PURPOSE**.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with **pySX127x**.  If not, see <http://www.gnu.org/licenses/>.