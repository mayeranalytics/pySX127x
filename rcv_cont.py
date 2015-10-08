#!/usr/bin/env python2.7

""" A simple continuous receiver class. """

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
from SX127x.LoRaArgumentParser import LoRaArgumentParser
from SX127x.board_config import BOARD

BOARD.setup()

parser = LoRaArgumentParser("Continous receiver")


class LoRaRcvCont(LoRa):
    def __init__(self, verbose=False):
        super(LoRaRcvCont, self).__init__(verbose)
        self.set_mode(MODE.SLEEP)
        self.set_dio_mapping([0] * 6)

    def on_rx_done(self):
        BOARD.led_on()
        print "\nRxDone"
        print self.get_irq_flags()
        print map(hex, self.read_payload(nocheck=True))
        self.set_mode(MODE.SLEEP)
        self.reset_ptr_rx()
        BOARD.led_off()
        self.set_mode(MODE.RXCONT)

    def on_tx_done(self):
        print "\nTxDone"
        print self.get_irq_flags()

    def on_cad_done(self):
        print "\non_CadDone";
        print self.get_irq_flags()

    def on_rx_timeout(self):
        print "\non_RxTimeout"
        print self.get_irq_flags()

    def on_valid_header(self):
        print "\non_ValidHeader"
        print self.get_irq_flags()

    def on_payload_crc_error(self):
        print "\non_PayloadCrcError"
        print self.get_irq_flags()

    def on_fhss_change_channel(self):
        print "\non_FhssChangeChannel"
        print self.get_irq_flags()

    def start(self):
        self.reset_ptr_rx()
        self.set_mode(MODE.RXCONT)
        while True:
            sleep(.5)
            rssi_value = self.get_rssi_value()
            status = self.get_modem_status()
            sys.stdout.flush()
            sys.stdout.write("\r%d %d %d" % (rssi_value, status['rx_ongoing'], status['modem_clear']))


lora = LoRaRcvCont(verbose=False)
args = parser.parse_args(lora)

lora.set_mode(MODE.STDBY)
lora.set_rx_crc(True)
lora.set_coding_rate(CODING_RATE.CR4_6)
lora.set_pa_config(max_power=0, output_power=0)
lora.set_lna_gain(GAIN.G1)
lora.set_implicit_header_mode(False)
lora.set_low_data_rate_optim(True)
lora.set_pa_ramp(PA_RAMP.RAMP_50_us)
lora.set_agc_auto_on(True)

print lora
assert(lora.get_agc_auto_on() == 1)

raw_input("Press enter to start...")

try:
    lora.start()
except KeyboardInterrupt:
    sys.stdout.flush()
    print
    sys.stderr.write("KeyboardInterrupt\n")
finally:
    sys.stdout.flush()
    print
    lora.set_mode(MODE.SLEEP)
    print lora
    GPIO.cleanup()

