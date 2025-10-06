from . import add_lookup

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

@add_lookup
class REG:
    FIFO               = 0x00  # RegFifo
    OP_MODE            = 0x01  # RegOpMode

    # Carrier frequency
    FR_MSB             = 0x06  # RegFrfMsb
    FR_MID             = 0x07  # RegFrfMid
    FR_LSB             = 0x08  # RegFrfLsb

    # TX path / PA / protections
    PA_CONFIG          = 0x09  # RegPaConfig
    PA_RAMP            = 0x0A  # RegPaRamp
    OCP                = 0x0B  # RegOcp
    LNA                = 0x0C  # RegLna

    # LoRa FIFO page
    FIFO_ADDR_PTR      = 0x0D  # RegFifoAddrPtr
    FIFO_TX_BASE_ADDR  = 0x0E  # RegFifoTxBaseAddr
    FIFO_RX_BASE_ADDR  = 0x0F  # RegFifoRxBaseAddr
    FIFO_RX_CURR_ADDR  = 0x10  # RegFifoRxCurrentAddr

    # IRQs / status / counters
    IRQ_FLAGS_MASK     = 0x11  # RegIrqFlagsMask
    IRQ_FLAGS          = 0x12  # RegIrqFlags
    RX_NB_BYTES        = 0x13  # RegRxNbBytes
    RX_HEADER_CNT_MSB  = 0x14  # RegRxHeaderCntValueMsb
    RX_HEADER_CNT_LSB  = 0x15  # RegRxHeaderCntValueLsb   <-- missing before
    RX_PACKET_CNT_MSB  = 0x16  # RegRxPacketCntValueMsb
    RX_PACKET_CNT_LSB  = 0x17  # RegRxPacketCntValueLsb   <-- missing before
    MODEM_STAT         = 0x18  # RegModemStat
    PKT_SNR_VALUE      = 0x19  # RegPktSnrValue
    PKT_RSSI_VALUE     = 0x1A  # RegPktRssiValue
    RSSI_VALUE         = 0x1B  # RegRssiValue
    HOP_CHANNEL        = 0x1C  # RegHopChannel

    # PHY configuration
    MODEM_CONFIG_1     = 0x1D  # RegModemConfig1
    MODEM_CONFIG_2     = 0x1E  # RegModemConfig2
    SYMB_TIMEOUT_LSB   = 0x1F  # RegSymbTimeoutLsb
    PREAMBLE_MSB       = 0x20  # RegPreambleMsb
    PREAMBLE_LSB       = 0x21  # RegPreambleLsb           <-- missing before
    PAYLOAD_LENGTH     = 0x22  # RegPayloadLength
    MAX_PAYLOAD_LENGTH = 0x23  # RegMaxPayloadLength
    HOP_PERIOD         = 0x24  # RegHopPeriod
    FIFO_RX_BYTE_ADDR  = 0x25  # RegFifoRxByteAddr
    MODEM_CONFIG_3     = 0x26  # RegModemConfig3
    PPM_CORRECTION     = 0x27  # RegPpmCorrection

    # Frequency error indication / RSSI wideband / IF freq (LoRa errata additions)
    FEI_MSB            = 0x28  # RegFeiMsb
    FEI_MID            = 0x29  # RegFeiMid                 <-- missing before
    FEI_LSB            = 0x2A  # RegFeiLsb                 <-- missing before
    RSSI_WIDEBAND      = 0x2C  # RegRssiWideband           <-- missing before
    IF_FREQ2           = 0x2F  # RegIfFreq2                <-- missing before
    IF_FREQ1           = 0x30  # RegIfFreq1                <-- missing before

    # LoRa detection / IQ invert
    DETECT_OPTIMIZE    = 0x31  # RegDetectOptimize
    INVERT_IQ          = 0x33  # RegInvertIQ
    HIGH_BW_OPTIMIZE1  = 0x36  # RegHighBwOptimize1        <-- missing before
    DETECTION_THRESH   = 0x37  # RegDetectionThreshold
    SYNC_WORD          = 0x39  # RegSyncWord
    HIGH_BW_OPTIMIZE2  = 0x3A  # RegHighBwOptimize2        <-- missing before
    INVERT_IQ2         = 0x3B  # RegInvertIQ2              <-- missing before

    # DIOs / version
    DIO_MAPPING_1      = 0x40  # RegDioMapping1
    DIO_MAPPING_2      = 0x41  # RegDioMapping2
    VERSION            = 0x42  # RegVersion

    # Misc analog / trims
    TCXO               = 0x4B  # RegTcxo
    PA_DAC             = 0x4D  # RegPaDac

    # AGC (LF/HF pages, shared)
    AGC_REF            = 0x61  # RegAgcRef
    AGC_THRESH_1       = 0x62  # RegAgcThresh1
    AGC_THRESH_2       = 0x63  # RegAgcThresh2
    AGC_THRESH_3       = 0x64  # RegAgcThresh3

    PLL                = 0x70  # RegPll
