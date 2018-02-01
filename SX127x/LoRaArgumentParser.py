""" Defines LoRaArgumentParser which extends argparse.ArgumentParser with standard config parameters for the SX127x. """
# -*- coding: utf-8 -*-

# Copyright 2018 Mayer Analytics Ltd.
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


import argparse


class LoRaArgumentParser(argparse.ArgumentParser):
    """ This class extends argparse.ArgumentParser.
        Some commonly used LoRa config parameters are defined
        * ocp
        * spreading factor
        * frequency
        * bandwidth
        * preamble
        Call the parse_args with an additional parameter referencing a LoRa object. The args will be used to configure
        the LoRa.
    """

    bw_lookup = dict(BW7_8=0, BW10_4=1, BW15_6=2, BW20_8=3, BW31_25=4, BW41_7=5, BW62_5=6, BW125=7, BW250=8, BW500=9)
    cr_lookup = dict(CR4_5=1, CR4_6=2,CR4_7=3,CR4_8=4)

    def __init__(self, description):
        argparse.ArgumentParser.__init__(self, description=description)
        self.add_argument('--ocp', '-c', dest='ocp', default=100, action="store", type=float,
                          help="Over current protection in mA (45 .. 240 mA)")
        self.add_argument('--sf', '-s', dest='sf', default=7, action="store", type=int,
                          help="Spreading factor (6...12). Default is 7.")
        self.add_argument('--freq', '-f', dest='freq', default=869., action="store", type=float,
                          help="Frequency")
        self.add_argument('--bw', '-b', dest='bw', default='BW125', action="store", type=str,
                          help="Bandwidth (one of BW7_8 BW10_4 BW15_6 BW20_8 BW31_25 BW41_7 BW62_5 BW125 BW250 BW500).\nDefault is BW125.")
        self.add_argument('--cr', '-r', dest='coding_rate', default='CR4_5', action="store", type=str,
                          help="Coding rate (one of CR4_5 CR4_6 CR4_7 CR4_8).\nDefault is CR4_5.")
        self.add_argument('--preamble', '-p', dest='preamble', default=8, action="store", type=int,
                          help="Preamble length. Default is 8.")

    def parse_args(self, lora):
        """ Parse the args, perform some sanity checks and configure the LoRa accordingly.
        :param lora: Reference to LoRa object
        :return: args
        """
        args = argparse.ArgumentParser.parse_args(self)
        args.bw = self.bw_lookup.get(args.bw, None)
        args.coding_rate = self.cr_lookup.get(args.coding_rate, None)
        # some sanity checks
        assert(args.bw is not None)
        assert(args.coding_rate is not None)
        assert(args.sf >=6 and args.sf <= 12)
        # set the LoRa object
        lora.set_freq(args.freq)
        lora.set_preamble(args.preamble)
        lora.set_spreading_factor(args.sf)
        lora.set_bw(args.bw)
        lora.set_coding_rate(args.coding_rate)
        lora.set_ocp_trim(args.ocp)
        return args
