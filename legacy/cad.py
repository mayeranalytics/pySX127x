#!/usr/bin/env python

# beacon.py
# awaits transmissions in RXSINGLE mode

import sys
from time import sleep
import SX127x
from SX127x.LoRa import *

sx127x = LoRa(verbose=False)
sx127x.set_freq(869)
sx127x.set_mode(MODE.STDBY)
sx127x.set_symb_timeout(100)
sx127x.set_coding_rate(CODING_RATE.CR4_8)
sx127x.set_spreading_factor(7)
sx127x.set_bw(BW.BW125)
sx127x.set_pa_config(max_power=0, output_power=0)
sx127x.set_lna_gain(GAIN.G1)
sx127x.set_implicit_header_mode(True)
sx127x.set_low_data_rate_optim(True)

print sx127x

raw_input("Press enter to start...")

sx127x.set_mode(MODE.CAD)

while sx127x.get_mode() == MODE.CAD:
    sleep(.2)

print MODE.lookup[sx127x.get_mode()]
print sx127x.get_modem_status()
print sx127x.get_irq_flags()

sx127x.reset_ptr_rx()
sx127x.set_mode(MODE.RXCONT)
try:
    rx_ongoing = True
    while rx_ongoing:
        sleep(.1)
        rssi_value = sx127x.get_rssi_value()
        status = sx127x.get_modem_status()
        rx_ongoing = status['rx_ongoing']
        sys.stdout.flush()
        sys.stdout.write("\r%d %d %d" % (rssi_value, rx_ongoing, status['modem_clear']))
    print
    print sx127x.get_irq_flags()
    print map(hex, sx127x.read_payload(nocheck=True))

except KeyboardInterrupt:
    pass
