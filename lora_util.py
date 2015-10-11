#!/usr/bin/env python2.7

""" This is a utility script for the SX127x (LoRa mode). It dumps all registers. """

# Copyright 2015 Mayer Analytics Ltd.
#
# This file is part of pySX127x.
#
# pySX127x is free software: you can redistribute it and/or modify it under the terms of the GNU Affero General Public
# License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later
# version.
#
# pySX127x is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied
# warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU Affero General Public License for more
# details.
#
# You can be released from the requirements of the license by obtaining a commercial license. Such a license is
# mandatory as soon as you develop commercial activities involving pySX127x without disclosing the source code of your
# own applications, or shipping pySX127x with a closed source product.
#
# You should have received a copy of the GNU General Public License along with pySX127.  If not, see
# <http://www.gnu.org/licenses/>.


from SX127x.LoRa import *
from SX127x.board_config import BOARD
import argparse

BOARD.setup()

parser = argparse.ArgumentParser(description='LoRa utility functions')
parser.add_argument('--dump', '-d', dest='dump', default=False, action="store_true", help="dump all registers")
args = parser.parse_args()

lora = LoRa(verbose=False)

if args.dump:

    print("LoRa register dump:\n")
    print("%02s %18s %2s %8s" % ('i', 'reg_name', 'v', 'v'))
    print("-- ------------------ -- --------")
    for reg_i, reg_name, val in lora.dump_registers():
        print("%02X %18s %02X %s" % (reg_i, reg_name, val, format(val, '#010b')[2:]))
    print("")

else:
    print(lora)

BOARD.teardown()