""" Defines the SX127x class and a few utility functions. """

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

import sys
import spidev
from constants import *
import RPi.GPIO as GPIO
from board_config import BOARD


def set_bit(v, index, bit):
    """ Set the index:th bit of v to x, and return the new value.
    :param v: The integer to set the bit in
    :param index: 0-based index
    :param bit: bit to set (0 or 1)
    :return: Changed integer
    """
    mask = 1 << index
    v &= ~mask
    if bit:
        v |= mask
    return v


def Getter(reg_addr):
    def decorator(func):
        def wrapper(self):
            return func(self, self.spi.xfer([reg_addr, 0])[1])
        return wrapper
    return decorator


def Setter(reg_addr):
    def decorator(func):
        def wrapper(self, val):
            return self.spi.xfer([reg_addr | 0x80, func(self, val)])[1]
        return wrapper
    return decorator


class LoRa(object):

    spi = spidev.SpiDev()
    mode = None                       # the mode is backed up here
    backup_registers = []
    verbose = True
    dio_mapping = [None] * 6          # store the dio mapping here

    def __init__(self, verbose=True):
        self.verbose = verbose
        # set the GPIOcallbacks
        GPIO.add_event_detect(BOARD.DIO0, GPIO.RISING, callback=self._dio0)
        GPIO.add_event_detect(BOARD.DIO1, GPIO.RISING, callback=self._dio1)
        GPIO.add_event_detect(BOARD.DIO2, GPIO.RISING, callback=self._dio2)
        GPIO.add_event_detect(BOARD.DIO3, GPIO.RISING, callback=self._dio3)
        # open SPI interface
        self.spi.open(0, 0)
        if self.verbose:
            sys.stderr.write("Opened SPI\n")
        # set mode to sleep
        self.set_mode(MODE.SLEEP)
        self.backup_registers = self.get_all_registers()
        # get the dio mappings, these two calls will populate self.dio_mapping
        self.get_dio_mapping_1()
        self.get_dio_mapping_2()

    # Overridable functions:

    def on_rx_done(self):
        pass

    def on_tx_done(self):
        pass

    def on_cad_done(self):
        pass

    def on_rx_timeout(self):
        pass

    def on_valid_header(self):
        pass

    def on_payload_crc_error(self):
        pass

    def on_fhss_change_channel(self):
        pass

    def _dio0(self, channel):
        # DIO0 00: RxDone
        # DIO0 01: TxDone
        # DIO0 10: CadDone
        if self.dio_mapping[0] == 0:
            self.on_rx_done()
        elif self.dio_mapping[0] == 1:
            self.on_tx_done()
        elif self.dio_mapping[0] == 2:
            self.on_cad_done()
        else:
            raise RuntimeError("unknown dio0mapping!")

    def _dio1(self, channel):
        # DIO1 00: RxTimeout
        # DIO1 01: FhssChangeChannel
        # DIO1 10: CadDetected
        print "dio1"
        if self.dio_mapping[1] == 0:
            self.on_rx_timeout()
        elif self.dio_mapping[1] == 1:
            self.on_fhss_change_channel()
        elif self.dio_mapping[1] == 2:
            self.on_CadDetected()
        else:
            raise RuntimeError("unknown dio1mapping!")

    def _dio2(self, channel):
        # DIO2 00: FhssChangeChannel
        # DIO2 01: FhssChangeChannel
        # DIO2 10: FhssChangeChannel
        self.on_fhss_change_channel()

    def _dio3(self, channel):
        # DIO3 00: CadDone
        # DIO3 01: ValidHeader
        # DIO3 10: PayloadCrcError
        print "dio3"
        if self.dio_mapping[3] == 0:
            self.on_cad_done()
        elif self.dio_mapping[3] == 1:
            self.on_valid_header()
        elif self.dio_mapping[3] == 2:
            self.on_payload_crc_error()
        else:
            raise RuntimeError("unknown dio3 mapping!")

    def get_mode(self):
        self.mode = self.spi.xfer([REG.OP_MODE, 0])[1]
        return self.mode

    def set_mode(self, mode):
        # the mode is backe up in self.mode
        if mode == self.mode:
            return mode
        if self.verbose:
            sys.stderr.write("Mode <- %s\n" % MODE.lookup[mode])
        self.mode = mode
        return self.spi.xfer([REG.OP_MODE | 0x80, mode])[1]

    def write_payload(self, payload):
        self.set_mode(MODE.STDBY)
        base_addr = self.get_fifo_tx_base_addr()
        self.set_fifo_addr_ptr(base_addr)
        return self.spi.xfer([REG.FIFO | 0x80] + payload)[1:]

    def reset_ptr_rx(self):
        """ Get FIFO ready for RX: Set FifoAddrPtr to FifoRxBaseAddr """
        self.set_mode(MODE.STDBY)
        base_addr = self.get_fifo_rx_base_addr()
        self.set_fifo_addr_ptr(base_addr)

    def rx_is_good(self):
        flags = self.get_irq_flags()
        return not any([flags[s] for s in ['valid_header', 'crc_error', 'rx_done', 'rx_timeout']])

    def read_payload(self , nocheck = False):
        if not nocheck and not self.rx_is_good():
            return None
        rx_nb_bytes = self.get_rx_nb_bytes()
        fifo_rx_current_addr = self.get_fifo_rx_current_addr()
        self.set_fifo_addr_ptr(fifo_rx_current_addr)
        payload = self.spi.xfer([REG.FIFO] + [0] * rx_nb_bytes)[1:]
        return payload

    def get_freq(self):
        # returns freq in MHz
        msb, mid, lsb = self.spi.xfer([REG.FR_MSB, 0, 0, 0])[1:]
        f = lsb + 256*(mid + 256*msb)
        return f / 16384.

    def set_freq(self, f):
        # f is a float in MHz
        assert self.mode == MODE.SLEEP or self.mode == MODE.STDBY
        i = int(f * 16384.)    # choose floor
        msb = i // 65536
        i -= msb * 65536
        mid = i // 256
        i -= mid * 256
        lsb = i
        self.spi.xfer([REG.FR_MSB | 0x80, msb, mid, lsb])

    def get_pa_config(self, convert_dBm=False):
        v = self.spi.xfer([REG.PA_CONFIG, 0])[1]
        pa_select    = v >> 7
        max_power    = v >> 4 & 0b111
        output_power = v & 0b1111
        if convert_dBm:
            max_power = max_power * .6 + 10.8
            output_power = max_power - (15 - output_power)
        return dict(
                pa_select    = pa_select,
                max_power    = max_power,
                output_power = output_power
            )

    def set_pa_config(self, pa_select=None, max_power=None, output_power=None):
        assert pa_select is None or pa_select == 0    # the inAir9 uses the RFO pin, so I don't allow to switch to PA_BOOST
        loc = locals()
        current = self.get_pa_config()
        loc = {s: current[s] if loc[s] is None else loc[s] for s in loc}
        val = (loc['pa_select'] << 7) | (loc['max_power'] << 4) | (loc['output_power'])
        return self.spi.xfer([REG.PA_CONFIG | 0x80, val])[1]

    @Getter(REG.PA_RAMP)
    def get_pa_ramp(self, val):
        return val & 0b1111

    @Setter(REG.PA_RAMP)
    def set_pa_ramp(self, val):
        return val & 0b1111

    def get_ocp(self, convert_mA=False):
        v = self.spi.xfer([REG.OCP, 0])[1]
        ocp_on = v >> 5 & 0x01
        ocp_trim = v & 0b11111
        if convert_mA:
            if ocp_trim <= 15:
                ocp_trim = 45. + 5. * ocp_trim
            elif ocp_trim <= 27:
                ocp_trim = -30. + 10. * ocp_trim
            else:
                assert ocp_trim <= 27
        return dict(
                ocp_on   = ocp_on,
                ocp_trim = ocp_trim
                )

    def set_ocp_trim(self, I_mA):
        assert(I_mA >= 45 and I_mA <= 240)
        ocp_on = self.spi.xfer([REG.OCP, 0])[1] >> 5 & 0x01
        if I_mA <= 120:
            v = int(round((I_mA-45.)/5.))
        else:
            v = int(round((I_mA+30.)/10.))
        v = set_bit(v, 5, ocp_on)
        return self.spi.xfer([REG.OCP | 0x80, v])[1]

    def get_lna(self):
        v = self.spi.xfer([REG.LNA, 0])[1]
        return dict(
                lna_gain     = v >> 5,
                lna_boost_lf = v >> 3 & 0b11,
                lna_boost_hf = v & 0b11
            )

    def set_lna(self, lna_gain=None, lna_boost_lf=None, lna_boost_hf=None):
        assert lna_boost_hf is None or lna_boost_hf == 0b00 or lna_boost_hf == 0b11
        self.set_mode(MODE.STDBY)
        if lna_gain is not None:
            # Apparently agc_auto_on must be 0 in order to set lna_gain
            self.set_agc_auto_on(lna_gain == GAIN.NOT_USED)
        loc = locals()
        current = self.get_lna()
        loc = {s: current[s] if loc[s] is None else loc[s] for s in loc}
        val = (loc['lna_gain'] << 5) | (loc['lna_boost_lf'] << 3) | (loc['lna_boost_hf'])
        retval = self.spi.xfer([REG.LNA | 0x80, val])[1]
        if lna_gain is not None:
            # agc_auto_on must track lna_gain: GAIN=NOT_USED -> agc_auto=ON, otherwise =OFF
            self.set_agc_auto_on(lna_gain == GAIN.NOT_USED)
        return retval

    def set_lna_gain(self, lna_gain):
        self.set_lna(lna_gain=lna_gain)

    def get_fifo_addr_ptr(self):
        return self.spi.xfer([REG.FIFO_ADDR_PTR, 0])[1]

    def set_fifo_addr_ptr(self, ptr):
        return self.spi.xfer([REG.FIFO_ADDR_PTR | 0x80, ptr])[1]

    def get_fifo_tx_base_addr(self):
        return self.spi.xfer([REG.FIFO_TX_BASE_ADDR, 0])[1]

    def set_fifo_tx_base_addr(self, ptr):
        return self.spi.xfer([REG.FIFO_TX_BASE_ADDR | 0x80, ptr])[1]

    def get_fifo_rx_base_addr(self):
        return self.spi.xfer([REG.FIFO_RX_BASE_ADDR, 0])[1]

    def set_fifo_rx_base_addr(self, ptr):
        return self.spi.xfer([REG.FIFO_RX_BASE_ADDR | 0x80, ptr])[1]

    def get_fifo_rx_current_addr(self):
        return self.spi.xfer([REG.FIFO_RX_CURR_ADDR, 0])[1]

    def get_fifo_rx_byte_addr(self):
        return self.spi.xfer([REG.FIFO_RX_BYTE_ADDR, 0])[1]

    def get_irq_flags_mask(self):
        v = self.spi.xfer([REG.IRQ_FLAGS_MASK, 0])[1]
        return dict(
                rx_timeout     = v >> 7 & 0x01,
                rx_done        = v >> 6 & 0x01,
                crc_error      = v >> 5 & 0x01,
                valid_header   = v >> 4 & 0x01,
                tx_done        = v >> 3 & 0x01,
                cad_done       = v >> 2 & 0x01,
                fhss_change_ch = v >> 1 & 0x01,
                cad_detected   = v >> 0 & 0x01,
            )

    def set_irq_flags_mask(self, rx_timeout=None, rx_done=None, crc_error=None, valid_header=None, tx_done=None,
                   cad_done=None, fhss_change_ch=None, cad_detected=None):
        loc = locals()
        v = self.spi.xfer([REG.IRQ_FLAGS_MASK, 0])[1]
        for i, s in enumerate(['cad_detected', 'fhss_change_ch', 'cad_done', 'tx_done', 'valid_header',
                               'crc_error', 'rx_done', 'rx_timeout']):
            this_bit = locals()[s]
            if this_bit is not None:
                v = set_bit(v, i, this_bit)
        return self.spi.xfer([REG.IRQ_FLAGS_MASK | 0x80, v])[1]

    def get_irq_flags(self):
        v = self.spi.xfer([REG.IRQ_FLAGS, 0])[1]
        return dict(
                rx_timeout     = v >> 7 & 0x01,
                rx_done        = v >> 6 & 0x01,
                crc_error      = v >> 5 & 0x01,
                valid_header   = v >> 4 & 0x01,
                tx_done        = v >> 3 & 0x01,
                cad_done       = v >> 2 & 0x01,
                fhss_change_ch = v >> 1 & 0x01,
                cad_detected   = v >> 0 & 0x01,
            )

    def set_irq_flags(self, rx_timeout=None, rx_done=None, crc_error=None, valid_header=None, tx_done=None,
                   cad_done=None, fhss_change_ch=None, cad_detected=None):
        v = self.spi.xfer([REG.IRQ_FLAGS, 0])[1]
        for i, s in enumerate(['cad_detected', 'fhss_change_ch', 'cad_done', 'tx_done', 'valid_header',
                               'crc_error', 'rx_done', 'rx_timeout']):
            this_bit = locals()[s]
            if this_bit is not None:
                v = set_bit(v, i, this_bit)
        return self.spi.xfer([REG.IRQ_FLAGS | 0x80, v])[1]

    def get_rx_nb_bytes(self):
        return self.spi.xfer([REG.RX_NB_BYTES, 0])[1]

    def get_rx_header_cnt(self):
        msb, lsb = self.spi.xfer([REG.RX_HEADER_CNT_MSB, 0, 0])[1:]
        return lsb + 256 * msb

    def get_rx_packet_cnt(self):
        msb, lsb = self.spi.xfer([REG.RX_PACKET_CNT_MSB, 0, 0])[1:]
        return lsb + 256 * msb

    def get_modem_status(self):
        status = self.spi.xfer([REG.MODEM_STAT, 0])[1]
        return dict(
                rx_coding_rate    = status >> 5 & 0x03,
                modem_clear       = status >> 4 & 0x01,
                header_info_valid = status >> 3 & 0x01,
                rx_ongoing        = status >> 2 & 0x01,
                signal_sync       = status >> 1 & 0x01,
                signal_detected   = status >> 0 & 0x01
            )

    def get_pkt_snr_value(self):
        v = self.spi.xfer([REG.PKT_SNR_VALUE, 0])[1]
        return float(256-v) / 4.

    def get_pkt_rssi_value(self):
        v = self.spi.xfer([REG.PKT_RSSI_VALUE, 0])[1]
        return v - 157

    def get_rssi_value(self):
        v = self.spi.xfer([REG.RSSI_VALUE, 0])[1]
        return v - 157

    def get_hop_channel(self):
        v = self.spi.xfer([REG.HOP_CHANNEL, 0])[1]
        return dict(
                pll_timeout          = v >> 7,
                crc_on_payload       = v >> 6 & 0x01,
                fhss_present_channel = v >> 5 & 0b111111
            )

    def get_modem_config_1(self):
        val = self.spi.xfer([REG.MODEM_CONFIG_1, 0])[1]
        return dict(
                bw = val >> 4 & 0x0F,
                coding_rate = val >> 1 & 0x07,
                implicit_header_mode = val & 0x01
            )
        
    def set_modem_config_1(self, bw=None, coding_rate=None, implicit_header_mode=None):
        loc = locals()
        current = self.get_modem_config_1()
        loc = {s: current[s] if loc[s] is None else loc[s] for s in loc}
        val = loc['implicit_header_mode'] | (loc['coding_rate'] << 1) | (loc['bw'] << 4)
        return self.spi.xfer([REG.MODEM_CONFIG_1 | 0x80, val])[1]

    def set_bw(self, bw):
        self.set_modem_config_1(bw=bw)

    def set_coding_rate(self, coding_rate):
        self.set_modem_config_1(coding_rate=coding_rate)

    def set_implicit_header_mode(self, implicit_header_mode):
        self.set_modem_config_1(implicit_header_mode=implicit_header_mode)
        
    def get_modem_config_2(self, include_symb_timout_lsb=False):
        val = self.spi.xfer([REG.MODEM_CONFIG_2, 0])[1]
        d = dict(
                spreading_factor = val >> 4 & 0x0F,
                tx_cont_mode = val >> 3 & 0x01,
                rx_crc = val >> 2 & 0x01,
            )
        if include_symb_timout_lsb:
            d['symb_timout_lsb'] = val & 0x03
        return d
        
    def set_modem_config_2(self, spreading_factor=None, tx_cont_mode=None, rx_crc=None):
        loc = locals()
        # RegModemConfig2 contains the SymbTimout MSB bits. We tack the back on when writing this register.
        current = self.get_modem_config_2(include_symb_timout_lsb=True)
        loc = {s: current[s] if loc[s] is None else loc[s] for s in loc}
        val = (loc['spreading_factor'] << 4) | (loc['tx_cont_mode'] << 3) | (loc['rx_crc'] << 2) | current['symb_timout_lsb']
        return self.spi.xfer([REG.MODEM_CONFIG_2 | 0x80, val])[1]

    def set_spreading_factor(self, spreading_factor):
        self.set_modem_config_2(spreading_factor=spreading_factor)

    def set_rx_crc(self, rx_crc):
        self.set_modem_config_2(rx_crc=rx_crc)

    def get_modem_config_3(self):
        val = self.spi.xfer([REG.MODEM_CONFIG_3, 0])[1]
        return dict(
                low_data_rate_optim = val >> 3 & 0x01,
                agc_auto_on = val >> 2 & 0x01
            )

    def set_modem_config_3(self, low_data_rate_optim=None, agc_auto_on=None):
        loc = locals()
        current = self.get_modem_config_3()
        loc = {s: current[s] if loc[s] is None else loc[s] for s in loc}
        val = (loc['low_data_rate_optim'] << 3) | (loc['agc_auto_on'] << 2)
        return self.spi.xfer([REG.MODEM_CONFIG_3 | 0x80, val])[1]
        
    def get_agc_auto_on(self):
        return self.get_modem_config_3()['agc_auto_on']

    def set_agc_auto_on(self, agc_auto_on):
        self.set_modem_config_3(agc_auto_on=agc_auto_on)

    def get_low_data_rate_optim(self):
        return self.set_modem_config_3()['low_data_rate_optim']

    def set_low_data_rate_optim(self, low_data_rate_optim):
        self.set_modem_config_3(low_data_rate_optim=low_data_rate_optim)

    def get_symb_timeout(self):
        SYMB_TIMEOUT_MSB = REG.MODEM_CONFIG_2
        msb, lsb = self.spi.xfer([SYMB_TIMEOUT_MSB, 0, 0])[1:]    # the MSB bits are stored in REG.MODEM_CONFIG_2
        msb = msb & 0b11
        return lsb + 256 * msb

    def set_symb_timeout(self, timeout):
        bkup_reg_modem_config_2 = self.spi.xfer([REG.MODEM_CONFIG_2, 0])[1]
        msb = timeout >> 8 & 0b11    # bits 8-9
        lsb = timeout - 256 * msb    # bits 0-7
        reg_modem_config_2 = bkup_reg_modem_config_2 & 0xFC | msb    # bits 2-7 of bkup_reg_modem_config_2 ORed with the two msb bits
        old_msb = self.spi.xfer([REG.MODEM_CONFIG_2, reg_modem_config_2])[1] & 0x03
        old_lsb = self.spi.xfer([REG.SYMB_TIMEOUT_LSB, lsb])[1]
        return old_lsb + 256 * old_msb

    def get_preamble(self):
        msb, lsb = self.spi.xfer([REG.PREAMBLE_MSB, 0, 0])[1:]
        return lsb + 256 * msb

    def set_preamble(self, preamble):
        msb = preamble >> 8
        lsb = preamble - msb * 256
        old_msb, old_lsb = self.spi.xfer([REG.PREAMBLE_MSB | 0x80, msb, lsb])[1:]
        return old_lsb + 256 * old_msb
        
    @Getter(REG.PAYLOAD_LENGTH)
    def get_payload_length(self, val):
        return val

    @Setter(REG.PAYLOAD_LENGTH)
    def set_payload_length(self, payload_length):
        return payload_length

    @Getter(REG.MAX_PAYLOAD_LENGTH)
    def get_max_payload_length(self, val):
        return val

    @Setter(REG.MAX_PAYLOAD_LENGTH)
    def set_max_payload_length(self, max_payload_length):
        return max_payload_length

    @Getter(REG.HOP_PERIOD)
    def get_hop_period(self, val):
        return val

    @Setter(REG.HOP_PERIOD)
    def set_hop_period(self, hop_period):
        return hop_period

    def get_fei(self):
        msb, mid, lsb = self.spi.xfer([REG.FEI_MSB, 0, 0, 0])[1:]
        msb &= 0x0F
        freq_error = lsb + 256 * (mid + 256 * msb)
        return freq_error

    @Getter(REG.DETECT_OPTIMIZE)
    def get_detect_optimize(self, val):
        return val & 0b111

    @Setter(REG.DETECT_OPTIMIZE)
    def set_detect_optimize(self, val):
        return val & 0b111

    @Getter(REG.DETECTION_THRESH)
    def get_detection_threshold(self, val):
        return val

    @Setter(REG.DETECTION_THRESH)
    def set_detection_threshold(self, val):
        return val

    @Getter(REG.SYNC_WORD)
    def get_sync_word(self, sync_word):
        return sync_word

    @Setter(REG.SYNC_WORD)
    def set_sync_word(self, sync_word):
        return sync_word

    @Getter(REG.DIO_MAPPING_1)
    def get_dio_mapping_1(self, mapping):
        self.dio_mapping = [mapping>>6 & 0x03, mapping>>4 & 0x03, mapping>>2 & 0x03, mapping>>0 & 0x03] + self.dio_mapping[4:6]
        return mapping

    @Setter(REG.DIO_MAPPING_1)
    def set_dio_mapping_1(self, mapping):
        self.dio_mapping = [mapping>>6 & 0x03, mapping>>4 & 0x03, mapping>>2 & 0x03, mapping>>0 & 0x03] + self.dio_mapping[4:6]
        return mapping

    @Getter(REG.DIO_MAPPING_2)
    def get_dio_mapping_2(self, mapping):
        self.dio_mapping = self.dio_mapping[0:4] + [mapping>>6 & 0x03, mapping>>4 & 0x03]
        return mapping

    @Setter(REG.DIO_MAPPING_2)
    def set_dio_mapping_2(self, mapping):
        assert mapping & 0b00001110 == 0
        self.dio_mapping = self.dio_mapping[0:4] + [mapping>>6 & 0x03, mapping>>4 & 0x03]
        return mapping

    @Getter(REG.VERSION)
    def get_version(self, version):
        return version

    @Getter(REG.TCXO)
    def get_tcxo(self, tcxo):
        return tcxo

    @Setter(REG.TCXO)
    def set_tcxo(self, tcxo):
        assert tcxo & 0b11101111 == 0b00001001
        return tcxo

    @Getter(REG.PA_DAC)
    def get_pa_dac(self, pa_dac):
        return pa_dac

    @Setter(REG.PA_DAC)
    def set_pa_dac(self, pa_dac):
        assert pa_dac & 0b1111100 == 0b00010000
        return pa_dac

    def dump_registers(self):
        # returns a list of [reg_addr, reg_name, reg_value] tuples
        self.set_mode(MODE.SLEEP)
        values = self.get_all_registers()
        skip_set = set([REG.FIFO])
        result_list = []
        for i, s in REG.lookup.iteritems():
            if i in skip_set:
                continue
            v = values[i]
            result_list.append((i, s, v))
        return result_list

    def get_register(self, reg_addr):
        return self.spi.xfer([reg_addr & 0x7F, 0])[1]

    def set_register(self, reg_addr, val):
        return self.spi.xfer([reg_addr | 0x80, val])[1]

    def get_all_registers(self):
        # read all registers
        reg = [0] + self.spi.xfer([1]+[0]*0x3E)[1:]
        self.mode = reg[1]
        return reg

    def __del__(self):
        self.set_mode(MODE.SLEEP)
        if self.verbose:
            sys.stderr.write("MODE=SLEEP\n")
        self.spi.close()
        if self.verbose:
            sys.stderr.write("Closed SPI\n")

    def __str__(self):
        # don't use __str__ while in any mode other that SLEEP or STDBY
        assert(self.mode == MODE.SLEEP or self.mode == MODE.STDBY)

        onoff = lambda i: 'ON' if i else 'OFF'
        f = self.get_freq()
        cfg1 = self.get_modem_config_1()
        cfg2 = self.get_modem_config_2()
        cfg3 = self.get_modem_config_3()
        pa_config = self.get_pa_config(convert_dBm=True)
        ocp = self.get_ocp(convert_mA=True)
        lna = self.get_lna()
        s =  "SX127x LoRa registers:\n"
        s += " mode               %s\n" % MODE.lookup[self.get_mode()]
        s += " freq               %f MHz\n" % f
        s += " coding_rate        %s\n" % CODING_RATE.lookup[cfg1['coding_rate']]
        s += " bw                 %s\n" % BW.lookup[cfg1['bw']]
        s += " spreading_factor   %s chips/symb\n" % (1 << cfg2['spreading_factor'])
        s += " implicit_hdr_mode  %s\n" % onoff(cfg1['implicit_header_mode'])
        s += " rx_payload_crc     %s\n" % onoff(cfg2['rx_crc'])
        s += " tx_cont_mode       %s\n" % onoff(cfg2['tx_cont_mode'])
        s += " preamble           %d\n" % self.get_preamble()
        s += " low_data_rate_opti %s\n" % onoff(cfg3['low_data_rate_optim'])
        s += " agc_auto_on        %s\n" % onoff(cfg3['agc_auto_on'])
        s += " symb_timeout       %s\n" % self.get_symb_timeout()
        s += " freq_hop_period    %s\n" % self.get_hop_period()
        s += " hop_channel        %s\n" % self.get_hop_channel()
        s += " payload_length     %s\n" % self.get_payload_length()
        s += " max_payload_length %s\n" % self.get_max_payload_length()
        s += " irq_flags_mask     %s\n" % self.get_irq_flags_mask()
        s += " irq_flags          %s\n" % self.get_irq_flags()
        s += " rx_nb_byte         %d\n" % self.get_rx_nb_bytes()
        s += " rx_header_cnt      %d\n" % self.get_rx_header_cnt()
        s += " rx_packet_cnt      %d\n" % self.get_rx_packet_cnt()
        s += " pkt_snr_value      %f\n" % self.get_pkt_snr_value()
        s += " pkt_rssi_value     %d\n" % self.get_pkt_rssi_value()
        s += " rssi_value         %d\n" % self.get_rssi_value()
        s += " fei                %d\n" % self.get_fei()
        s += " pa_select          %s\n" % PA_SELECT.lookup[pa_config['pa_select']]
        s += " max_power          %f dBm\n" % pa_config['max_power']
        s += " output_power       %f dBm\n" % pa_config['output_power']
        s += " ocp                %s\n"     % onoff(ocp['ocp_on'])
        s += " ocp_trim           %f mA\n"  % ocp['ocp_trim']
        s += " lna_gain           %s\n" % GAIN.lookup[lna['lna_gain']]
        s += " lna_boost_lf       %s\n" % bin(lna['lna_boost_lf'])
        s += " lna_boost_hf       %s\n" % bin(lna['lna_boost_hf'])
        s += " detect_optimize    %#02x\n" % self.get_detect_optimize()
        s += " detection_thresh   %#02x\n" % self.get_detection_threshold()
        s += " sync_word          %#02x\n" % self.get_sync_word()
        s += " dio_mapping_1      %#02x\n" % self.get_dio_mapping_1()
        s += " dio_mapping_2      %#02x\n" % self.get_dio_mapping_2()
        s += " tcxo               %#02x\n" % self.get_tcxo()
        s += " pa_dac             %#02x\n" % self.get_pa_dac()
        s += " fifo_addr_ptr      %#02x\n" % self.get_fifo_addr_ptr()
        s += " fifo_tx_base_addr  %#02x\n" % self.get_fifo_tx_base_addr()
        s += " fifo_rx_base_addr  %#02x\n" % self.get_fifo_rx_base_addr()
        s += " fifo_rx_curr_addr  %#02x\n" % self.get_fifo_rx_current_addr()
        s += " fifo_rx_byte_addr  %#02x\n" % self.get_fifo_rx_byte_addr()
        s += " status             %s\n" % self.get_modem_status()
        s += " dio_mapping        %s\n" % self.dio_mapping
        s += " version            %#02x\n" % self.get_version()
        return s

if __name__ == '__main__':
    lora = LoRa()
    print(lora)
