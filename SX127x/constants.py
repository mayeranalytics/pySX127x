""" Defines constants (modes, bandwidths, registers, etc.) needed by SX127x. """
# -*- coding: utf-8 -*-

# Copyright 2015-2018 Mayer Analytics Ltd.
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


def add_lookup(cls):
    """ A decorator that adds a lookup dictionary to the class.
        The lookup dictionary maps the codes back to the names. This is used for pretty-printing. """
    varnames = filter(str.isupper, cls.__dict__.keys())
    lookup = dict(map(lambda varname: (cls.__dict__.get(varname, None), varname), varnames))
    setattr(cls, 'lookup', lookup)
    return cls


@add_lookup
class MODE:
    SLEEP    = 0x80
    STDBY    = 0x81
    FSTX     = 0x82
    TX       = 0x83
    FSRX     = 0x84
    RXCONT   = 0x85
    RXSINGLE = 0x86
    CAD      = 0x87
    FSK_STDBY= 0x01     # needed for calibration


@add_lookup
class BW:
    BW7_8   = 0
    BW10_4  = 1
    BW15_6  = 2
    BW20_8  = 3
    BW31_25 = 4
    BW41_7  = 5
    BW62_5  = 6
    BW125   = 7
    BW250   = 8
    BW500   = 9


@add_lookup
class CODING_RATE:
    CR4_5 = 1
    CR4_6 = 2
    CR4_7 = 3
    CR4_8 = 4


@add_lookup
class GAIN:
    NOT_USED = 0b000
    G1       = 0b001
    G2       = 0b010
    G3       = 0b011
    G4       = 0b100
    G5       = 0b101
    G6       = 0b110


@add_lookup
class PA_SELECT:
    RFO      = 0
    PA_BOOST = 1


@add_lookup
class PA_RAMP:
    RAMP_3_4_ms = 0
    RAMP_2_ms   = 1
    RAMP_1_ms   = 2
    RAMP_500_us = 3
    RAMP_250_us = 4
    RAMP_125_us = 5
    RAMP_100_us = 6
    RAMP_62_us  = 7
    RAMP_50_us  = 8
    RAMP_40_us  = 9
    RAMP_31_us  = 10
    RAMP_25_us  = 11
    RAMP_20_us  = 12
    RAMP_15_us  = 13
    RAMP_12_us  = 14
    RAMP_10_us  = 15


class MASK:
    class IRQ_FLAGS:
        RxTimeout           = 7
        RxDone              = 6
        PayloadCrcError     = 5
        ValidHeader         = 4
        TxDone              = 3
        CadDone             = 2
        FhssChangeChannel   = 1
        CadDetected         = 0


class REG:

    @add_lookup
    class LORA:
        FIFO               = 0x00
        OP_MODE            = 0x01
        FR_MSB             = 0x06
        FR_MID             = 0x07
        FR_LSB             = 0x08
        PA_CONFIG          = 0x09
        PA_RAMP            = 0x0A
        OCP                = 0x0B
        LNA                = 0x0C
        FIFO_ADDR_PTR      = 0x0D
        FIFO_TX_BASE_ADDR  = 0x0E
        FIFO_RX_BASE_ADDR  = 0x0F
        FIFO_RX_CURR_ADDR  = 0x10
        IRQ_FLAGS_MASK     = 0x11
        IRQ_FLAGS          = 0x12
        RX_NB_BYTES        = 0x13
        RX_HEADER_CNT_MSB  = 0x14
        RX_PACKET_CNT_MSB  = 0x16
        MODEM_STAT         = 0x18
        PKT_SNR_VALUE      = 0x19
        PKT_RSSI_VALUE     = 0x1A
        RSSI_VALUE         = 0x1B
        HOP_CHANNEL        = 0x1C
        MODEM_CONFIG_1     = 0x1D
        MODEM_CONFIG_2     = 0x1E
        SYMB_TIMEOUT_LSB   = 0x1F
        PREAMBLE_MSB       = 0x20
        PAYLOAD_LENGTH     = 0x22
        MAX_PAYLOAD_LENGTH = 0x23
        HOP_PERIOD         = 0x24
        FIFO_RX_BYTE_ADDR  = 0x25
        MODEM_CONFIG_3     = 0x26
        PPM_CORRECTION     = 0x27
        FEI_MSB            = 0x28
        DETECT_OPTIMIZE    = 0X31
        INVERT_IQ          = 0x33
        DETECTION_THRESH   = 0X37
        SYNC_WORD          = 0X39
        DIO_MAPPING_1      = 0x40
        DIO_MAPPING_2      = 0x41
        VERSION            = 0x42
        TCXO               = 0x4B
        PA_DAC             = 0x4D
        AGC_REF            = 0x61
        AGC_THRESH_1       = 0x62
        AGC_THRESH_2       = 0x63
        AGC_THRESH_3       = 0x64
        PLL                = 0x70

    @add_lookup
    class FSK:
        LNA                = 0x0C
        RX_CONFIG          = 0x0D
        RSSI_CONFIG        = 0x0E
        PREAMBLE_DETECT    = 0x1F
        OSC                = 0x24
        SYNC_CONFIG        = 0x27
        SYNC_VALUE_1       = 0x28
        SYNC_VALUE_2       = 0x29
        SYNC_VALUE_3       = 0x2A
        SYNC_VALUE_4       = 0x2B
        SYNC_VALUE_5       = 0x2C
        SYNC_VALUE_6       = 0x2D
        SYNC_VALUE_7       = 0x2E
        SYNC_VALUE_8       = 0x2F
        PACKET_CONFIG_1    = 0x30
        FIFO_THRESH        = 0x35
        IMAGE_CAL          = 0x3B
        DIO_MAPPING_1      = 0x40
        DIO_MAPPING_2      = 0x41
