## pySX127x

This is a python interface to the Semtech SX127x long range, low power transceiver family 
[SX1272, SX1273, SX1276, SX1277, SX1278, SX12789](http://www.semtech.com/wireless-rf/rf-transceivers/).
The SX127x has both [LoRa](https://lora-alliance.org) and FSK capabilities. Here the focus lies on the
LoRa spread spectrum modulation.


## Motivation

Transceiver modules are usually interfaced with microcontroller boards such as the 
[Arduino](https://www.arduino.cc/) and there are already many fine C/C++ libraries for the SX128x family on 
[github](https://github.com/search?q=sx127x) and [mbed.org](https://developer.mbed.org/search/?q=sx127x).

Although C/C++ is the de facto standard for development on, [python](https://www.python.org)
running on a [Raspberry Pi](https://www.raspberrypi.org) is becoming a viable alternative for rapid prototyping.


## Hardware

The transceiver module is a SX1276 based Modtronix [inAir9B](http://modtronix.com/inair9.html). 
It is mounted on a prototyping board to a [Raspberry Pi](https://www.raspberrypi.org) rev 2 model B.
 
|  board pin   | RaspPi GPIO | Direction |
|--------------|-------------|-----------|
| inAir9B DIO0 | GPIO 21     |    IN     |
| inAir9B DIO1 | GPIO 22     |    IN     |
| inAir9B DIO2 | GPIO 23     |    IN     |
| inAir9B DIO3 | GPIO 24     |    IN     |
| LED          | GPIO 25     |    OUT    |

@todo Add picture(s)


## Code Example

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


## Installation

**pySX127x** requires [RPi.GPIO](https://pypi.python.org/pypi/RPi.GPIO).


## API Reference

@todo


## Script reference

### `rcv_cont.py`
The SX127x is put in RXCONT mode and continuously waits for transmissions. Upon a successful read the
payload and the irq flags are printed to screen.

### `beacon.py`
A small payload is transmitted in regular intervals.


## Tests

Execute `test_lora.py` to run a few unit tests. 


## Contributors

Please feel free to comment, report issues, or even contribute!

Check out my company website [Mayer Analytics](http://mayeranalytics.com) and my private blog
[mcmayer.net](http://mcmayer.net). Follow me on twitter [@markuscmayer](https://twitter.com/markuscmayer) and
[@mayeranalytics](https://twitter.com/mayeranalytics).


## Version

**pySX127x** is still in the development phase. The current version is 0.1.


## License

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
