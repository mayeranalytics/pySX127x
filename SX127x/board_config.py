""" Defines the BOARD class that contains the board pin mappings. """

# This file is part of pySX127x.
#
# pySX127x is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# pySX127x is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with pySX127x.  If not, see <http://www.gnu.org/licenses/>.

import RPi.GPIO as GPIO


class BOARD:
    """ All the board pin config is kept here. Note that the BCOM numbering for the GPIOs is used.
    """
    DIO0 = 21   # RaspPi GPIO 21
    DIO1 = 22   # RaspPi GPIO 22
    DIO2 = 23   # RaspPi GPIO 23
    DIO3 = 24   # RaspPi GPIO 24
    LED  = 25   # RaspPi GPIO 25 connects to the LED on the proto shield

    @staticmethod
    def setup():
        """ Configure the Raspberry GPIOs
        :rtype : None
        """
        GPIO.setmode(GPIO.BCM)
        # LED
        GPIO.setup(BOARD.LED, GPIO.OUT)
        GPIO.output(BOARD.LED, 0)
        # DIOx
        for gpio_pin in [BOARD.DIO0, BOARD.DIO1, BOARD.DIO2, BOARD.DIO3]:
            GPIO.setup(gpio_pin, GPIO.IN)

    @staticmethod
    def led_on(value=1):
        """ Switch the proto shields LED
        :param value: 0/1 for off/on. Default is 1.
        :return: value
        :rtype : int
        """
        GPIO.output(BOARD.LED, value)
        return value

    @staticmethod
    def led_off():
        """ Switch LED off
        :return: 0
        """
        GPIO.output(BOARD.LED, 0)
        return 0