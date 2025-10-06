""" Defines the FSK class and a few utility functions. """
# -*- coding: utf-8 -*-

# Copyright 2015-2025 Mayer Analytics Ltd.
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


import sys
from constants.FSK import *
from constants import LoRa
from .board_config import BOARD
from .utils import set_bit, getter, setter


############################################### Definition of the FSK class ###########################################

class FSK(object):
    """
    FSK/OOK mode driver for SX1276/7/8/9 — styled like your LoRa class.
    Only FSK-mode registers/fields are touched. Shared blocks (PA/LNA/OCP/OP_MODE/FRF) reuse the same helpers.
    """

    spi = BOARD.SpiDev()
    mode = None
    verbose = True
    dio_mapping = [None] * 6  # DIO0..DIO5

    # ---- construction / setup -------------------------------------------------

    def __init__(self, verbose=True, init_defaults=True):
        self.verbose = verbose
        BOARD.add_events(self._dio0, self._dio1, self._dio2, self._dio3, self._dio4, self._dio5)
        self.set_mode(MODE.SLEEP)
        if init_defaults:
            # A sane, minimal FSK baseline (matches datasheet-ish defaults)
            # You can tweak these to your board/profile.
            self.set_mode(MODE.FSK_STDBY)
            # Bit rate ~ 50 kbps (BitRate = FXOSC / BR); with FXOSC=32MHz -> BR=640 -> 50 kbps
            self.set_bitrate_bps(50000.0)
            # Deviation ~ 25 kHz
            self.set_fdev_hz(25000.0)
            # RX BW ~ 83.3 kHz (mant=16, exp=3) – decent for 50k/25k profile
            self.set_rx_bw(mant=16, exp=3, dcc=0b100)
            # AFC BW mirror RX BW
            self.set_afc_bw(mant=16, exp=3, dcc=0b100)
            # OOK disabled; packet mode defaults
            self.set_packet_config_1(
                fixed_len=False, crc_on=True, whitening_on=True, addr_filt=0, pkt_format=0
            )
            self.set_packet_config_2(
                auto_rx_restart_on=False, aes_on=False, data_mode=0
            )
            # Preamble length 5 bytes (arbitrary reasonable)
            self.set_preamble_length(5)
            # Sync word: 3 bytes, 0xC1 0x94 0xC1 (common demo value)
            self.set_sync_config(on=True, size=3, tol=0)
            self.set_sync_values([0xC1, 0x94, 0xC1])
            # FIFO threshold: start Tx when FIFO not empty (tx_start_cond=1), level ~ 32
            self.set_fifo_thresh(tx_start_cond=1, thresh=0x20)
            # RSSI start and thresholds
            self.set_rssi_config(start=0, smpl=0b10)  # default RSSI mode
            self.set_rssi_thresh_dbm(-100.0)
            # ImageCal: leave default; run when you want with LoRa.rx_chain_calibration()
            self.set_mode(MODE.SLEEP)

        # initialize DIO maps cache
        self.get_dio_mapping_1()
        self.get_dio_mapping_2()

    # ---- IRQ callbacks --------------------------------------------------------

    def on_fifo(self):            pass
    def on_sync_addr(self):       pass
    def on_preamble_detect(self): pass
    def on_rssi(self):            pass
    def on_timeout(self):         pass
    def on_packet_sent(self):     pass
    def on_payload_ready(self):   pass

    def _dio0(self, ch):
        # Commonly PayloadReady/PacketSent depending on map.
        m = self.dio_mapping[0]
        if m == 0: self.on_payload_ready()
        elif m == 1: self.on_packet_sent()
        else: self.on_fifo()  # fallback

    def _dio1(self, ch):
        # SyncAddr or FIFO level
        m = self.dio_mapping[1]
        if m == 0: self.on_fifo()
        elif m == 1: self.on_sync_addr()
        else: self.on_fifo()

    def _dio2(self, ch):
        self.on_rssi()

    def _dio3(self, ch):
        self.on_timeout()

    def _dio4(self, ch):
        raise RuntimeError("DIO4 is not used in this profile")

    def _dio5(self, ch):
        raise RuntimeError("DIO5 is not used in this profile")

    # ---- shared low-levels (mode/freq/PA/LNA/etc.) ---------------------------

    def get_mode(self):
        self.mode = self.spi.xfer([REG.OP_MODE, 0])[1]
        return self.mode

    def set_mode(self, mode):
        if mode == self.mode:
            return mode
        if self.verbose:
            sys.stderr.write("Mode <- %s\n" % MODE.lookup[mode])
        self.mode = mode
        return self.spi.xfer([REG.OP_MODE | 0x80, mode])[1]

    # Frequency (FRF) is shared with LoRa
    def get_freq(self):
        msb, mid, lsb = self.spi.xfer([REG.FR_MSB, 0, 0, 0])[1:]
        f = lsb + 256 * (mid + 256 * msb)
        return f / 16384.0

    def set_freq(self, f_mhz):
        assert self.mode in (MODE.SLEEP, MODE.STDBY, MODE.FSK_STDBY)
        i = int(f_mhz * 16384.0)
        msb = i // 65536
        mid = (i - msb * 65536) // 256
        lsb = i - msb * 65536 - mid * 256
        return self.spi.xfer([REG.FR_MSB | 0x80, msb, mid, lsb])

    # PA config / ramp / OCP / LNA — these are the same registers as LoRa; reuse logic
    def get_pa_config(self, convert_dBm=False): return LoRa.get_pa_config(self, convert_dBm)
    def set_pa_config(self, **kw):              return LoRa.set_pa_config(self, **kw)
    get_pa_ramp = LoRa.get_pa_ramp
    set_pa_ramp = LoRa.set_pa_ramp
    def get_ocp(self, convert_mA=False):        return LoRa.get_ocp(self, convert_mA)
    def set_ocp_trim(self, I_mA):               return LoRa.set_ocp_trim(self, I_mA)
    def get_lna(self):                          return LoRa.get_lna(self)
    def set_lna(self, **kw):                    return LoRa.set_lna(self, **kw)
    set_lna_gain = LoRa.set_lna_gain

    # DIO mapping (same addresses)
    get_dio_mapping_1 = LoRa.get_dio_mapping_1
    set_dio_mapping_1 = LoRa.set_dio_mapping_1
    get_dio_mapping_2 = LoRa.get_dio_mapping_2
    set_dio_mapping_2 = LoRa.set_dio_mapping_2
    get_dio_mapping   = LoRa.get_dio_mapping
    set_dio_mapping   = LoRa.set_dio_mapping

    # Version / TCXO / PA_DAC — same addresses
    get_version = LoRa.get_version
    get_tcxo    = LoRa.get_tcxo
    set_tcxo    = LoRa.set_tcxo
    get_pa_dac  = LoRa.get_pa_dac
    set_pa_dac  = LoRa.set_pa_dac

    # ---- FIFO I/O (FSK) ------------------------------------------------------

    def write_payload(self, payload):
        """
        FSK FIFO write. For fixed length, ensure RegPayloadLength is set.
        For variable length, first byte is length.
        """
        self.set_mode(MODE.FSK_STDBY)
        # If variable-length mode: write length byte first
        pc1 = self.get_packet_config_1()
        data = payload[:]
        if pc1['packet_format'] == 0:  # variable length
            if len(data) > 255: raise ValueError("payload too long for variable-length FSK")
            data = [len(data)] + data
        return self.spi.xfer([REG.FSK.FIFO | 0x80] + data)[1:]

    def read_payload(self, max_len=255):
        """
        Read payload from FSK FIFO. For variable length, reads length first.
        """
        pc1 = self.get_packet_config_1()
        if pc1['packet_format'] == 0:  # variable
            ln = self.spi.xfer([REG.FSK.FIFO, 0])[1]
            ln = min(ln, max_len)
            return self.spi.xfer([REG.FSK.FIFO] + [0] * ln)[1:]
        else:  # fixed
            ln = self.get_payload_length()
            ln = min(ln, max_len)
            return self.spi.xfer([REG.FSK.FIFO] + [0] * ln)[1:]

    # ---- Bit rate & deviation -------------------------------------------------

    def _fxosc(self):   # Hz
        return 32_000_000.0

    def _fstep(self):   # Hz
        # Fstep = FXOSC / 2^19
        return self._fxosc() / (1 << 19)

    def get_bitrate_raw(self):
        msb = self.spi.xfer([REG.FSK.BITRATE_MSB, 0])[1]
        lsb = self.spi.xfer([REG.FSK.BITRATE_LSB, 0])[1]
        frac = self.spi.xfer([REG.FSK.BITRATE_FRAC, 0])[1]
        return (msb << 8) | lsb, frac & 0x0F

    def set_bitrate_raw(self, br, frac=0):
        br &= 0xFFFF
        frac &= 0x0F
        self.spi.xfer([REG.FSK.BITRATE_MSB | 0x80, (br >> 8) & 0xFF])
        self.spi.xfer([REG.FSK.BITRATE_LSB | 0x80, br & 0xFF])
        return self.spi.xfer([REG.FSK.BITRATE_FRAC | 0x80, frac])[1]

    def get_bitrate_bps(self):
        br, frac = self.get_bitrate_raw()
        # BitRate = FXOSC / (BR + frac/16)
        denom = br + (frac / 16.0)
        return 0 if denom == 0 else self._fxosc() / denom

    def set_bitrate_bps(self, bps):
        if bps <= 0: raise ValueError("bitrate must be > 0")
        denom = self._fxosc() / float(bps)
        br = int(denom)
        frac = int(round((denom - br) * 16.0))
        if frac == 16:
            br += 1
            frac = 0
        return self.set_bitrate_raw(br, frac)

    def get_fdev_hz(self):
        msb = self.spi.xfer([REG.FSK.FDEV_MSB, 0])[1]
        lsb = self.spi.xfer([REG.FSK.FDEV_LSB, 0])[1]
        val = (msb << 8) | lsb
        return val * self._fstep()

    def set_fdev_hz(self, hz):
        if hz < 0: raise ValueError("fdev must be >= 0")
        val = int(round(hz / self._fstep()))
        val = max(0, min(0x3FFF, val))  # 14 bits effective
        msb = (val >> 8) & 0xFF
        lsb = val & 0xFF
        self.spi.xfer([REG.FSK.FDEV_MSB | 0x80, msb])
        return self.spi.xfer([REG.FSK.FDEV_LSB | 0x80, lsb])[1]

    # ---- Receiver bandwidths --------------------------------------------------

    def _pack_bw(self, mant, exp, dcc):
        mant_bits = {16: 0b00, 20: 0b01, 24: 0b10}.get(mant, None)
        if mant_bits is None: raise ValueError("mant must be 16, 20, or 24")
        if not (0 <= exp <= 7): raise ValueError("exp must be 0..7")
        if not (0 <= dcc <= 7): raise ValueError("dcc must be 0..7")
        return ((dcc & 0x07) << 5) | ((mant_bits & 0x03) << 3) | (exp & 0x07)

    def _unpack_bw(self, v):
        dcc = (v >> 5) & 0x07
        mant_bits = (v >> 3) & 0x03
        exp = v & 0x07
        mant = {0b00: 16, 0b01: 20, 0b10: 24}.get(mant_bits, 16)
        return mant, exp, dcc

    @getter(REG.FSK.RX_BW)
    def get_rx_bw(self, v):
        mant, exp, dcc = self._unpack_bw(v)
        return dict(mant=mant, exp=exp, dcc=dcc)

    def set_rx_bw(self, mant=None, exp=None, dcc=None):
        cur = self.get_rx_bw()
        mant = cur['mant'] if mant is None else mant
        exp  = cur['exp']  if exp  is None else exp
        dcc  = cur['dcc']  if dcc  is None else dcc
        return self.spi.xfer([REG.FSK.RX_BW | 0x80, self._pack_bw(mant, exp, dcc)])[1]

    @getter(REG.FSK.AFC_BW)
    def get_afc_bw(self, v):
        mant, exp, dcc = self._unpack_bw(v)
        return dict(mant=mant, exp=exp, dcc=dcc)

    def set_afc_bw(self, mant=None, exp=None, dcc=None):
        cur = self.get_afc_bw()
        mant = cur['mant'] if mant is None else mant
        exp  = cur['exp']  if exp  is None else exp
        dcc  = cur['dcc']  if dcc  is None else dcc
        return self.spi.xfer([REG.FSK.AFC_BW | 0x80, self._pack_bw(mant, exp, dcc)])[1]

    # ---- RSSI / AGC / thresholds ---------------------------------------------

    @getter(REG.FSK.RSSI_CONFIG)
    def get_rssi_config(self, v):
        return dict(start=(v >> 0) & 1, smpl=(v >> 3) & 0b11)

    def set_rssi_config(self, start=None, smpl=None):
        cur = self.get_rssi_config()
        start = cur['start'] if start is None else start
        smpl  = cur['smpl']  if smpl  is None else smpl
        val = (start & 1) | ((smpl & 0b11) << 3)
        return self.spi.xfer([REG.FSK.RSSI_CONFIG | 0x80, val])[1]

    def get_rssi_value_dbm(self):
        # RSSI (FSK) in dBm = - (RSSI_VALUE / 2)  (datasheet uses 0.5 dB steps)
        raw = self.spi.xfer([REG.FSK.RSSI_VALUE, 0])[1]
        return -0.5 * raw

    def set_rssi_thresh_raw(self, thr):
        thr &= 0xFF
        return self.spi.xfer([REG.FSK.RSSI_THRESH | 0x80, thr])[1]

    def get_rssi_thresh_raw(self):
        return self.spi.xfer([REG.FSK.RSSI_THRESH, 0])[1]

    def set_rssi_thresh_dbm(self, dBm):
        # reverse of -0.5 * raw
        raw = int(max(0, min(255, round(-2.0 * dBm))))
        return self.set_rssi_thresh_raw(raw)

    # ---- Timeouts / delays ----------------------------------------------------

    @getter(REG.FSK.RX_TIMEOUT_1)
    def get_rx_timeout_1(self, v): return v
    @setter(REG.FSK.RX_TIMEOUT_1)
    def set_rx_timeout_1(self, v): return v & 0xFF

    @getter(REG.FSK.RX_TIMEOUT_2)
    def get_rx_timeout_2(self, v): return v
    @setter(REG.FSK.RX_TIMEOUT_2)
    def set_rx_timeout_2(self, v): return v & 0xFF

    @getter(REG.FSK.RX_TIMEOUT_3)
    def get_rx_timeout_3(self, v): return v
    @setter(REG.FSK.RX_TIMEOUT_3)
    def set_rx_timeout_3(self, v): return v & 0xFF

    @getter(REG.FSK.RX_DELAY)
    def get_rx_delay(self, v): return v
    @setter(REG.FSK.RX_DELAY)
    def set_rx_delay(self, v): return v & 0xFF

    # ---- Preamble -------------------------------------------------------------

    def get_preamble_length(self):
        msb = self.spi.xfer([REG.FSK.PREAMBLE_MSB, 0])[1]
        lsb = self.spi.xfer([REG.FSK.PREAMBLE_LSB, 0])[1]
        return (msb << 8) | lsb

    def set_preamble_length(self, nbytes):
        nbytes = int(max(0, min(0xFFFF, nbytes)))
        msb = (nbytes >> 8) & 0xFF
        lsb = nbytes & 0xFF
        self.spi.xfer([REG.FSK.PREAMBLE_MSB | 0x80, msb])
        return self.spi.xfer([REG.FSK.PREAMBLE_LSB | 0x80, lsb])[1]

    # ---- Sync word(s) ---------------------------------------------------------

    @getter(REG.FSK.SYNC_CONFIG)
    def get_sync_config(self, v):
        return dict(
            on=(v >> 7) & 1,
            size=((v >> 3) & 0x07) + 1,  # 1..8 bytes
            tol=v & 0x07
        )

    def set_sync_config(self, on=None, size=None, tol=None):
        cur = self.get_sync_config()
        on   = cur['on']   if on   is None else (1 if on else 0)
        size = cur['size'] if size is None else size
        tol  = cur['tol']  if tol  is None else tol
        if not (1 <= size <= 8): raise ValueError("sync size must be 1..8")
        v = ((on & 1) << 7) | ((((size - 1) & 0x07) << 3)) | (tol & 0x07)
        return self.spi.xfer([REG.FSK.SYNC_CONFIG | 0x80, v])[1]

    def set_sync_values(self, bytes_list):
        if not (1 <= len(bytes_list) <= 8):
            raise ValueError("sync word length must be 1..8")
        addrs = [
            REG.FSK.SYNC_VALUE_1, REG.FSK.SYNC_VALUE_2, REG.FSK.SYNC_VALUE_3, REG.FSK.SYNC_VALUE_4,
            REG.FSK.SYNC_VALUE_5, REG.FSK.SYNC_VALUE_6, REG.FSK.SYNC_VALUE_7, REG.FSK.SYNC_VALUE_8
        ]
        for i, b in enumerate(bytes_list):
            self.spi.xfer([addrs[i] | 0x80, b & 0xFF])
        return True

    # ---- Packet engine --------------------------------------------------------

    @getter(REG.FSK.PACKET_CONFIG_1)
    def get_packet_config_1(self, v):
        # RegPacketConfig1: [7]PacketFormat [6]DcFree1 [5]DcFree0 [4]CrcOn [3]CrcAutoClearOff [2:1]AddrFiltering [0]CrcWhiteningType
        return dict(
            packet_format=(v >> 7) & 1,     # 0: variable, 1: fixed
            dc_free=(v >> 5) & 0b11,        # 00:off 01:Manchester 10:Whitening
            crc_on=(v >> 4) & 1,
            crc_auto_clear_off=(v >> 3) & 1,
            addr_filt=(v >> 1) & 0b11,
            whitening_on=1 if (((v >> 5) & 0b11) == 0b10) else 0
        )

    def set_packet_config_1(self, fixed_len=None, crc_on=None, whitening_on=None, addr_filt=None, pkt_format=None):
        cur = self.get_packet_config_1()
        pkt_format = cur['packet_format'] if pkt_format is None else pkt_format
        dc_free = (0b10 if (whitening_on if whitening_on is not None else cur['whitening_on']) else 0b00)
        crc_on = cur['crc_on'] if crc_on is None else (1 if crc_on else 0)
        addr_filt = cur['addr_filt'] if addr_filt is None else addr_filt
        fixed_len = cur['packet_format'] if fixed_len is None else (1 if fixed_len else 0)
        # crc_auto_clear_off -> leave default 0 (clear CRC status on read)
        v = ((fixed_len & 1) << 7) | ((dc_free & 0b11) << 5) | ((crc_on & 1) << 4) | (0 << 3) | ((addr_filt & 0b11) << 1) | 0
        return self.spi.xfer([REG.FSK.PACKET_CONFIG_1 | 0x80, v])[1]

    @getter(REG.FSK.PACKET_CONFIG_2)
    def get_packet_config_2(self, v):
        # [7]DataMode(0:packet,1:cont) [6]IoHome [5]IoHomeOn [4]BeaconOn [3]AesOn [2]AutoRxRestartOn [1:0]PktMode
        return dict(
            data_mode=(v >> 7) & 1,
            aes_on=(v >> 3) & 1,
            auto_rx_restart_on=(v >> 2) & 1,
            pkt_mode=v & 0b11
        )

    def set_packet_config_2(self, auto_rx_restart_on=None, aes_on=None, data_mode=None, pkt_mode=None):
        cur = self.get_packet_config_2()
        auto_rx_restart_on = cur['auto_rx_restart_on'] if auto_rx_restart_on is None else (1 if auto_rx_restart_on else 0)
        aes_on = cur['aes_on'] if aes_on is None else (1 if aes_on else 0)
        data_mode = cur['data_mode'] if data_mode is None else data_mode
        pkt_mode = cur['pkt_mode'] if pkt_mode is None else pkt_mode
        v = ((data_mode & 1) << 7) | (0 << 6) | (0 << 5) | (0 << 4) | ((aes_on & 1) << 3) | ((auto_rx_restart_on & 1) << 2) | (pkt_mode & 0b11)
        return self.spi.xfer([REG.FSK.PACKET_CONFIG_2 | 0x80, v])[1]

    @getter(REG.FSK.PAYLOAD_LENGTH)
    def get_payload_length(self, v): return v

    @setter(REG.FSK.PAYLOAD_LENGTH)
    def set_payload_length(self, v): return v & 0xFF

    # FIFO threshold (Tx start cond + level)
    @getter(REG.FSK.FIFO_THRESH)
    def get_fifo_thresh(self, v):
        return dict(tx_start_cond=(v >> 7) & 1, thresh=v & 0x7F)

    def set_fifo_thresh(self, tx_start_cond=None, thresh=None):
        cur = self.get_fifo_thresh()
        tsc = cur['tx_start_cond'] if tx_start_cond is None else (1 if tx_start_cond else 0)
        th  = cur['thresh'] if thresh is None else (thresh & 0x7F)
        v = (tsc << 7) | th
        return self.spi.xfer([REG.FSK.FIFO_THRESH | 0x80, v])[1]

    # ---- OOK knobs (optional) -------------------------------------------------

    @getter(REG.FSK.OOK_PEAK)
    def get_ook_peak(self, v): return v
    @setter(REG.FSK.OOK_PEAK)
    def set_ook_peak(self, v): return v & 0xFF

    @getter(REG.FSK.OOK_FIX)
    def get_ook_fix(self, v): return v
    @setter(REG.FSK.OOK_FIX)
    def set_ook_fix(self, v): return v & 0xFF

    @getter(REG.FSK.OOK_AVG)
    def get_ook_avg(self, v): return v
    @setter(REG.FSK.OOK_AVG)
    def set_ook_avg(self, v): return v & 0xFF

    # ---- IRQs (FSK) -----------------------------------------------------------

    def get_irq_flags_1(self):
        v = self.spi.xfer([REG.FSK.IRQ_FLAGS_1, 0])[1]
        # 7:ModeReady 6:RxReady 5:TxReady 4:PLL Lock 3:Rssi 2:Timeout 1:PreambleDetect 0:SyncAddressMatch
        return dict(
            mode_ready=(v >> 7) & 1,
            rx_ready=(v >> 6) & 1,
            tx_ready=(v >> 5) & 1,
            pll_lock=(v >> 4) & 1,
            rssi=(v >> 3) & 1,
            timeout=(v >> 2) & 1,
            preamble_detect=(v >> 1) & 1,
            sync_addr_match=(v >> 0) & 1
        )

    def get_irq_flags_2(self):
        v = self.spi.xfer([REG.FSK.IRQ_FLAGS_2, 0])[1]
        # 7:FIFOFull 6:FIFONotEmpty 5:FIFOLevel 4:FIFOOverrun 3:PacketSent 2:PayloadReady 1:CrcOk 0:LowBat
        return dict(
            fifo_full=(v >> 7) & 1,
            fifo_not_empty=(v >> 6) & 1,
            fifo_level=(v >> 5) & 1,
            fifo_overrun=(v >> 4) & 1,
            packet_sent=(v >> 3) & 1,
            payload_ready=(v >> 2) & 1,
            crc_ok=(v >> 1) & 1,
            low_bat=(v >> 0) & 1
        )

    def clear_fifo_overrun(self):
        # Writing 1 to FIFOOverrun bit clears it (write to RegIrqFlags2)
        v = 1 << 4
        return self.spi.xfer([REG.FSK.IRQ_FLAGS_2 | 0x80, v])[1]

    # ---- Misc helpers ---------------------------------------------------------

    def get_register(self, addr):
        return self.spi.xfer([addr & 0x7F, 0])[1]

    def set_register(self, addr, val):
        return self.spi.xfer([addr | 0x80, val & 0xFF])[1]

    def __del__(self):
        self.set_mode(MODE.SLEEP)
        if self.verbose:
            sys.stderr.write("MODE=SLEEP\n")

    def __str__(self):
        assert self.mode in (MODE.SLEEP, MODE.STDBY, MODE.FSK_STDBY)
        onoff = lambda i: 'ON' if i else 'OFF'

        f_mhz = self.get_freq()
        br = self.get_bitrate_bps()
        fdev = self.get_fdev_hz()
        rx_bw = self.get_rx_bw()
        afc_bw = self.get_afc_bw()
        rssi_dbm = self.get_rssi_value_dbm()
        pc1 = self.get_packet_config_1()
        pc2 = self.get_packet_config_2()
        pa = self.get_pa_config(convert_dBm=True)
        ocp = self.get_ocp(convert_mA=True)
        lna = self.get_lna()

        s  = "SX127x FSK registers:\n"
        s += " mode               %s\n" % MODE.lookup[self.get_mode()]
        s += " freq               %.6f MHz\n" % f_mhz
        s += " bitrate            %.2f bps\n" % br
        s += " fdev               %.1f Hz\n" % fdev
        s += " rx_bw              mant=%d exp=%d dcc=%d\n" % (rx_bw['mant'], rx_bw['exp'], rx_bw['dcc'])
        s += " afc_bw             mant=%d exp=%d dcc=%d\n" % (afc_bw['mant'], afc_bw['exp'], afc_bw['dcc'])
        s += " rssi_value         %.1f dBm\n" % rssi_dbm
        s += " packet_format      %s\n" % ('fixed' if pc1['packet_format'] else 'variable')
        s += " crc_on             %s\n" % onoff(pc1['crc_on'])
        s += " whitening_on       %s\n" % onoff(pc1['whitening_on'])
        s += " payload_length     %d\n" % self.get_payload_length()
        s += " preamble_length    %d\n" % self.get_preamble_length()
        s += " irq_flags_1        %s\n" % self.get_irq_flags_1()
        s += " irq_flags_2        %s\n" % self.get_irq_flags_2()
        s += " pa_select          %s\n" % PA_SELECT.lookup[pa['pa_select']]
        s += " max_power          %.1f dBm\n" % pa['max_power']
        s += " output_power       %.1f dBm\n" % pa['output_power']
        s += " ocp                %s\n"     % onoff(ocp['ocp_on'])
        s += " ocp_trim           %.1f mA\n"  % ocp['ocp_trim']
        s += " lna_gain           %s\n" % GAIN.lookup[lna['lna_gain']]
        s += " lna_boost_lf       %s\n" % bin(lna['lna_boost_lf'])
        s += " lna_boost_hf       %s\n" % bin(lna['lna_boost_hf'])
        s += " dio_mapping 0..5   %s\n" % self.get_dio_mapping()
        s += " tcxo               %s\n" % ['XTAL', 'TCXO'][self.get_tcxo()]
        s += " pa_dac             %s\n" % ['default', 'PA_BOOST'][self.get_pa_dac()]
        s += " version            %#02x\n" % self.get_version()
        return s
