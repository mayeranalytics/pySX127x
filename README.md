# Overview

This is a python interface to the [Semtech SX1276/7/8/9](http://www.semtech.com/wireless-rf/rf-transceivers/) 
long range, low power transceiver family.

The SX127x have both LoRa and FSK capabilities. Here the focus lies on the
LoRa spread spectrum modulation hence only the LoRa modem interface is implemented so far 
(but see the [roadmap](#roadmap) below for future plans).

Spread spectrum modulation has a number of intriguing features:
* High interference immunity
* Up to 20dBm link budget advantage (for the SX1276/7/8/9)
* High Doppler shift immunity

More information about LoRa can be found on the [LoRa Alliance website](https://lora-alliance.org).
Links to some LoRa performance reports can be found in the [references](#references) section below.


# Motivation

Transceiver modules are usually interfaced with microcontroller boards such as the 
Arduino and there are already many fine C/C++ libraries for the SX127x family available on 
[github](https://github.com/search?q=sx127x) and [mbed.org](https://developer.mbed.org/search/?q=sx127x).

Although C/C++ is the de facto standard for development on microcontrollers, [python](https://www.python.org)
running on a Raspberry Pi (NanoPi, BananaPi, UDOO Neo, BeagleBoard, etc. etc.) is becoming a viable alternative for rapid prototyping.

High level programming languages like python require a full-blown OS such as Linux. (There are some exceptions like
[MicroPython](https://micropython.org) and its fork [CircuitPython](https://www.adafruit.com/circuitpython).)
But using hardware capable of running Linux contradicts, to some extent, the low power specification of the SX127x family.
Therefore it is clear that this approach aims mostly at prototyping and technology testing.

Prototyping on a full-blown OS using high level programming languages has several clear advantages:
* Working prototypes can be built quickly 
* Technology testing ist faster
* Proof of concept is easier to achieve
* The application development phase is reached quicker 


# Hardware

The transceiver module is a SX1276 based Modtronix [inAir9B](https://web.archive.org/web/20200926024317/https://modtronix.com/inair9.html). 
It is mounted on a prototyping board to a Raspberry Pi rev 2 model B.

| Proto board pin | RaspPi GPIO | Direction |
|:----------------|:-----------:|:---------:|
| inAir9B DIO0    | GPIO 22     |    IN     |
| inAir9B DIO1    | GPIO 23     |    IN     |
| inAir9B DIO2    | GPIO 24     |    IN     |
| inAir9B DIO3    | GPIO 25     |    IN     |
| inAir9b Reset   | GPIO ?      |    OUT    |
| LED             | GPIO 18     |    OUT    |
| Switch          | GPIO 4      |    IN     |

Todo:
- [ ] Add picture(s)
- [ ] Wire the SX127x reset to a GPIO?


# Code Examples

### Overview
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
print(lora.version())        # this prints the sx127x chip version
print(lora.get_freq())       # this prints the frequency setting 
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

In the end the resources should be freed properly
```python
BOARD.teardown()
```

### More details
Most functions of `SX127x.Lora` are setter and getter functions. For example, the setter and getter for 
the coding rate are demonstrated here
```python 
print(lora.get_coding_rate())                # print the current coding rate
lora.set_coding_rate(CODING_RATE.CR4_6)     # set it to CR4_6
```

@todo


# Installation

Make sure SPI is activated on you RaspberryPi: [SPI](https://www.raspberrypi.org/documentation/hardware/raspberrypi/spi/README.md)
**pySX127x** requires these two python packages:
* [RPi.GPIO](https://pypi.python.org/pypi/RPi.GPIO") for accessing the GPIOs, it should be already installed on
  a standard Raspian Linux image
* [spidev](https://pypi.python.org/pypi/spidev) for controlling SPI

In order to install spidev download the source code and run setup.py manually:
```bash
wget https://pypi.python.org/packages/source/s/spidev/spidev-3.1.tar.gz
tar xfvz  spidev-3.1.tar.gz
cd spidev-3.1
sudo python setup.py install
```

At this point you may want to confirm that the unit tests pass. See the section [Tests](#tests) below.

You can now run the scripts. For example dump the registers with `lora_util.py`: 
```bash
rasp$ sudo ./lora_util.py
SX127x LoRa registers:
 mode               SLEEP
 freq               434.000000 MHz
 coding_rate        CR4_5
 bw                 BW125
 spreading_factor   128 chips/symb
 implicit_hdr_mode  OFF
 ... and so on ....
```


# Class Reference

The interface to the SX127x LoRa modem is implemented in the class `SX127x.LoRa.LoRa`.
The most important modem configuration parameters are:
 
| Function         | Description                                 |
|------------------|---------------------------------------------|
| set_mode         | Change OpMode, use the constants.MODE class |
| set_freq         | Set the frequency                           |
| set_bw           | Set the bandwidth 7.8kHz ... 500kHz         |
| set_coding_rate  | Set the coding rate 4/5, 4/6, 4/7, 4/8      |
| | |
| @todo            |                              |

Most set_* functions have a mirror get_* function, but beware that the getter return types do not necessarily match 
the setter input types.

### Register naming convention
The register addresses are defined in class `SX127x.constants.REG` and we use a specific naming convention which 
is best illustrated by a few examples:

| Register | Modem | Semtech doc.      | pySX127x                   |
|----------|-------|-------------------| ---------------------------|
| 0x0E     | LoRa  | RegFifoTxBaseAddr | REG.LORA.FIFO_TX_BASE_ADDR |
| 0x0E     | FSK   | RegRssiCOnfig     | REG.FSK.RSSI_CONFIG        |
| 0x1D     | LoRa  | RegModemConfig1   | REG.LORA.MODEM_CONFIG_1    |
| etc.     |       |                   |                            |

### Hardware
Hardware related definition and initialisation are located in `SX127x.board_config.BOARD`.
If you use a SBC other than the Raspberry Pi you'll have to adapt the BOARD class.


# Script references

### Continuous receiver `rx_cont.py`
The SX127x is put in RXCONT mode and continuously waits for transmissions. Upon a successful read the
payload and the irq flags are printed to screen.
```
usage: rx_cont.py [-h] [--ocp OCP] [--sf SF] [--freq FREQ] [--bw BW]
                  [--cr CODING_RATE] [--preamble PREAMBLE]

Continous LoRa receiver

optional arguments:
  -h, --help            show this help message and exit
  --ocp OCP, -c OCP     Over current protection in mA (45 .. 240 mA)
  --sf SF, -s SF        Spreading factor (6...12). Default is 7.
  --freq FREQ, -f FREQ  Frequency
  --bw BW, -b BW        Bandwidth (one of BW7_8 BW10_4 BW15_6 BW20_8 BW31_25
                        BW41_7 BW62_5 BW125 BW250 BW500). Default is BW125.
  --cr CODING_RATE, -r CODING_RATE
                        Coding rate (one of CR4_5 CR4_6 CR4_7 CR4_8). Default
                        is CR4_5.
  --preamble PREAMBLE, -p PREAMBLE
                        Preamble length. Default is 8.
```

### Simple LoRa beacon `tx_beacon.py`
A small payload is transmitted in regular intervals.
```
usage: tx_beacon.py [-h] [--ocp OCP] [--sf SF] [--freq FREQ] [--bw BW]
                    [--cr CODING_RATE] [--preamble PREAMBLE] [--single]
                    [--wait WAIT]

A simple LoRa beacon

optional arguments:
  -h, --help            show this help message and exit
  --ocp OCP, -c OCP     Over current protection in mA (45 .. 240 mA)
  --sf SF, -s SF        Spreading factor (6...12). Default is 7.
  --freq FREQ, -f FREQ  Frequency
  --bw BW, -b BW        Bandwidth (one of BW7_8 BW10_4 BW15_6 BW20_8 BW31_25
                        BW41_7 BW62_5 BW125 BW250 BW500). Default is BW125.
  --cr CODING_RATE, -r CODING_RATE
                        Coding rate (one of CR4_5 CR4_6 CR4_7 CR4_8). Default
                        is CR4_5.
  --preamble PREAMBLE, -p PREAMBLE
                        Preamble length. Default is 8.
  --single, -S          Single transmission
  --wait WAIT, -w WAIT  Waiting time between transmissions (default is 0s)
```


# Tests

Execute `test_lora.py` to run a few unit tests. 


# Contributors

Please feel free to comment, report issues, or contribute!

Contact me via my company website [Mayer Analytics](http://mayeranalytics.com) and my private blog
[mcmayer.net](http://mcmayer.net). 

Follow me on twitter [@markuscmayer](https://twitter.com/markuscmayer) and
[@mayeranalytics](https://twitter.com/mayeranalytics).


# Roadmap

95% of functions for the Sx127x LoRa capabilities are implemented. Functions will be added when necessary.
The test coverage is rather low but we intend to change that soon.

### Semtech SX1272/3 vs. SX1276/7/8/9
**pySX127x** is not entirely compatible with the 1272.
The 1276 and 1272 chips are different and the interfaces not 100% identical. For example registers 0x26/27. 
But the pySX127x library should get you pretty far if you use it with care. Here are the two datasheets:
* [Semtech - SX1276/77/78/79 - 137 MHz to 1020 MHz Low Power Long Range Transceiver](https://semtech.my.salesforce.com/sfc/p/E0000000JelG/a/2R0000001Rbr/6EfVZUorrpoKFfvaF_Fkpgp5kzjiNyiAbqcpqh9qSjE)
* [Semtech SX1272/73 - 860 MHz to 1020 MHz Low Power Long Range Transceiver](https://semtech.my.salesforce.com/sfc/p/E0000000JelG/a/440000001NCE/v_VBhk1IolDgxwwnOpcS_vTFxPfSEPQbuneK3mWsXlU)

### HopeRF transceiver ICs ###
HopeRF has a family of LoRa capable transceiver chips [RFM92/95/96/98](http://www.hoperf.com/)
that have identical or almost identical SPI interface as the Semtech SX1276/7/8/9 family.

### Microchip transceiver IC ###
Likewise Microchip has the chip [RN2483](http://ww1.microchip.com/downloads/en/DeviceDoc/50002346A.pdf)

The [pySX127x](https://github.com/mayeranalytics/pySX127x) project will therefore be renamed to pyLoRa at some point.

# LoRaWAN
LoRaWAN is a LPWAN (low power WAN) and, and  **pySX127x** has almost no relationship with LoRaWAN. Here we only deal with the interface into the chip(s) that enable the physical layer of LoRaWAN networks. If you need a LoRaWAN implementation have a look at [Jeroennijhof](https://github.com/jeroennijhof)s [LoRaWAN](https://github.com/jeroennijhof/LoRaWAN) which is based on pySX127x.

By the way, LoRaWAN is what you need when you want to talk to the [TheThingsNetwork](https://www.thethingsnetwork.org/), a "global open LoRaWAN network". The site has a lot of information and links to products and projects.


# References

### Hardware references
* [Semtech SX1276/77/78/79 - 137 MHz to 1020 MHz Low Power Long Range Transceiver](http://www.semtech.com/images/datasheet/sx1276_77_78_79.pdf)
* [Modtronix inAir9](http://modtronix.com/inair9.html)
* [Spidev Documentation](http://tightdev.net/SpiDev_Doc.pdf)
* [Make: Tutorial: Raspberry Pi GPIO Pins and Python](http://makezine.com/projects/tutorial-raspberry-pi-gpio-pins-and-python/)

### LoRa performance tests
* [Extreme Range Links: LoRa 868 / 900MHz SX1272 LoRa module for Arduino, Raspberry Pi and Intel Galileo](https://www.cooking-hacks.com/documentation/tutorials/extreme-range-lora-sx1272-module-shield-arduino-raspberry-pi-intel-galileo/)
* [UK LoRa versus FSK - 40km LoS (Line of Sight) test!](http://www.instructables.com/id/Introducing-LoRa-/step17/Other-region-tests/)
* [Andreas Spiess LoRaWAN World Record Attempt](https://www.youtube.com/watch?v=adhWIo-7gr4)

### Spread spectrum modulation theory
* [An Introduction to Spread Spectrum Techniques](http://www.ausairpower.net/OSR-0597.html)
* [Theory of Spread-Spectrum Communications-A Tutorial](http://www.fer.unizg.hr/_download/repository/Theory%20of%20Spread-Spectrum%20Communications-A%20Tutorial.pdf)
(technical paper)


# Copyright and License

&copy; 2015 Mayer Analytics Ltd., All Rights Reserved.

### Short version
The license is [GNU AGPL](http://www.gnu.org/licenses/agpl-3.0.en.html).

### Long version
pySX127x is free software: you can redistribute it and/or modify it under the terms of the 
GNU Affero General Public License as published by the Free Software Foundation, 
either version 3 of the License, or (at your option) any later version.

pySX127x is distributed in the hope that it will be useful, 
but WITHOUT ANY WARRANTY; without even the implied warranty of 
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. 
See the GNU Affero General Public License for more details.

You can be released from the requirements of the license by obtaining a commercial license. 
Such a license is mandatory as soon as you develop commercial activities involving 
pySX127x without disclosing the source code of your own applications, or shipping pySX127x with a closed source product.

You should have received a copy of the GNU General Public License
along with pySX127.  If not, see <http://www.gnu.org/licenses/>.

# Other legal boredom
LoRa, LoRaWAN, LoRa Alliance are all trademarks by ... someone.
