#!/usr/bin/env python

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

from time import sleep
from SX127x.LoRa import *
from SX127x.board_config import BOARD

BOARD.setup()

lora = LoRa()
lora.set_freq(869)
lora.set_mode(MODE.STDBY)
lora.set_symb_timeout(1023)    # max
print lora

counter = 0
while True:
    lora.reset_ptr_rx([0xff])
    lora.set_mode(MODE.RXSINGLE)
    is_good = False
    print lora
    while not is_good:
        sleep(.3)
        is_good = lora.rx_is_good()
    print lora
    payload = lora.read_payload(nocheck=True)
    print "payload=%s" % payload
