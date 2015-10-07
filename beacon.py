#!/usr/bin/env python

""" A simple beacon transmitter class to send a 1-byte message (0x0f) in regular time intervals. """

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
import RPi.GPIO as GPIO
from SX127x.LoRaArgumentParser import LoRaArgumentParser
from SX127x.board_config import BOARD

BOARD.setup()

parser = LoRaArgumentParser("A simple LoRa beacon")
parser.add_argument('--single', '-S', dest='single', default=False, action="store_true", help="Single transmission")
parser.add_argument('--wait', '-w', dest='wait', default=0, action="store", type=float, help="Waiting time between transmissions (default is 0s)")


class LoRaBeacon(LoRa):

    def __init__(self, verbose=False):
        super(LoRaBeacon, self).__init__(verbose)

    def on_rx_done(self):
        print "\nRxDone"
        print lora.get_irq_flags()
        print map(hex, lora.read_payload(nocheck=True))
        lora.set_mode(MODE.SLEEP)
        lora.reset_ptr_rx()
        lora.set_mode(MODE.RXCONT)

    def on_tx_done(self):
        print "\nTxDone"
        print lora.get_irq_flags()

    def on_cad_done(self):
        print "\non_CadDone";
        print lora.get_irq_flags()

    def on_rx_timeout(self):
        print "\non_RxTimeout"
        print lora.get_irq_flags()

    def on_valid_header(self):
        print "\non_ValidHeader"
        print lora.get_irq_flags()

    def on_payload_crc_error(self):
        print "\non_PayloadCrcError"
        print lora.get_irq_flags()

    def on_fhss_change_channel(self):
        print "\non_FhssChangeChannel"
        print lora.get_irq_flags()

    def start(self):
        global args
        self.set_dio_mapping_1(0b01000000)
        counter = 0
        while True:
            lora.write_payload([0x0f])
            BOARD.led_on()
            lora.set_mode(MODE.TX)
            counter += 1
            sys.stdout.flush()
            sys.stdout.write("\rtx #%d" % counter)
            tx_done = False
            while not tx_done:
                tx_done = lora.get_irq_flags()['tx_done']
                BOARD.led_off()
            if args.single:
                break
            else:
                sleep(args.wait)


lora = LoRaBeacon(verbose=False)
args = parser.parse_args(lora)

lora.set_rx_crc(True)
lora.set_agc_auto_on(True)
lora.set_lna_gain(GAIN.NOT_USED)
lora.set_coding_rate(CODING_RATE.CR4_6)
lora.set_implicit_header_mode(False)
lora.set_pa_config(max_power=0x04, output_power=0x0F)
#lora.set_pa_config(max_power=0x04, output_power=0b01000000)
lora.set_low_data_rate_optim(True)
lora.set_pa_ramp(PA_RAMP.RAMP_50_us)


print lora
#assert(lora.get_lna()['lna_gain'] == GAIN.NOT_USED)
assert(lora.get_agc_auto_on() == 1)

print "simple_beacon parameters:"
print "  Wait\t%f s" % args.wait
print
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
