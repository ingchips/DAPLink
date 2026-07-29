#include "swd_spi.h"
#include "DAP_config.h"
#include "DAP.h"
#include "IO_Config.h"
#include "peripheral_ssp.h"
#include "peripheral_sysctrl.h"
#include "peripheral_pinctrl.h"
#include "ram_code.h"

//**************************************************************************************************
// SPI Hardware SWD Implementation for ING916
//
// The ing916 SPI peripheral (AHB_SSP0) is configured with bidirectional MOSI mode,
// allowing a single GPIO wire (SWDIO) to serve as both data-out and data-in.
// SPI0_CLK is mapped to SWCLK, SPI0_MOSI is mapped to SWDIO (bidirectional).
//**************************************************************************************************

#define SPI_MIC_CLK             GIO_GPIO_7
#define SPI_MIC_MOSI            GIO_GPIO_8
#define SPI_MODE                AHB_SSP0
#define SPI_PORT                SPI_PORT_0
#define SPI_CLK_ITEM            SYSCTRL_ITEM_AHB_SPI0
#define SWD_SPI_BUSY_TIMEOUT    100000U

static volatile uint8_t swd_spi_timeout;

#define spi_wait_idle() \
    do { \
        uint32_t timeout = SWD_SPI_BUSY_TIMEOUT; \
        while ((SPI_MODE->Status & 1U) != 0U) { \
            if (--timeout == 0U) { \
                swd_spi_timeout = 1U; \
                break; \
            } \
        } \
    } while (0)

//**************************************************************************************************
// SPI Macros for SWD bit-level operations
//**************************************************************************************************

// Write with direction switch: sets MOSI to output mode first
#define spi_dir_write(data_bit, data_val) \
    do { \
        SPI_MODE->TransCtrl = (SPI_MODE->TransCtrl & 0xF0E00FFF) | 0x1000000; \
        SPI_MODE->TransFmt  = (SPI_MODE->TransFmt  & 0xFFFFE0FF) | (((data_bit) - 1) << bsSPI_TRANSFMT_DATALEN); \
        SPI_MODE->Cmd = 0; \
        SPI_MODE->Data = (data_val); \
        spi_wait_idle(); \
    } while (0)

// Fast write (no direction switch, keeps current MOSI direction)
#define spi_fast_write_bit(data_bit, data_val) \
    do { \
        SPI_MODE->TransFmt  = (SPI_MODE->TransFmt  & 0xFFFFE0FF) | (((data_bit) - 1) << bsSPI_TRANSFMT_DATALEN); \
        SPI_MODE->Cmd = 0; \
        SPI_MODE->Data = (data_val); \
        spi_wait_idle(); \
    } while (0)

// Read with direction switch: sets MOSI to input mode
#define spi_dir_read(data_bit, data_val) \
    do { \
        SPI_MODE->TransCtrl = (SPI_MODE->TransCtrl & 0xF0FFFE00) | 0x2000000; \
        SPI_MODE->TransFmt  = (SPI_MODE->TransFmt  & 0xFFFFE0FF) | (((data_bit) - 1) << bsSPI_TRANSFMT_DATALEN); \
        SPI_MODE->Cmd = 0; \
        (data_val) = SPI_MODE->Data; \
        spi_wait_idle(); \
    } while (0)

// Fast read (no direction switch)
#define spi_fast_read_bit(data_bit, data_val) \
    do { \
        SPI_MODE->TransFmt  = (SPI_MODE->TransFmt  & 0xFFFFE0FF) | (((data_bit) - 1) << bsSPI_TRANSFMT_DATALEN); \
        SPI_MODE->Cmd = 0; \
        (data_val) = SPI_MODE->Data; \
        spi_wait_idle(); \
    } while (0)

//**************************************************************************************************
// SPI SWD Initialization
//**************************************************************************************************

void swd_spi_init(void)
{
    apSSP_sDeviceControlBlock pParam;

    // Enable clocks
    SYSCTRL_ClearClkGateMulti((1 << SPI_CLK_ITEM) | (1 << SYSCTRL_ITEM_APB_PinCtrl));
    SYSCTRL_SelectSpiClk(SPI_PORT, 0);  // Select PLL clock for SPI0

    // Configure SPI pins: CLK -> SWCLK, MOSI -> SWDIO (bidirectional)
    PINCTRL_SelSpiIn(SPI_PORT, SPI_MIC_CLK, IO_NOT_A_PIN, IO_NOT_A_PIN,
                     IO_NOT_A_PIN, IO_NOT_A_PIN, SPI_MIC_MOSI);
    GIO_SetDirection(SPI_MIC_MOSI, GIO_DIR_BOTH);
    PINCTRL_SetDriveStrength(SPI_MIC_MOSI, PINCTRL_DRIVE_12mA);
    PINCTRL_SetDriveStrength(SPI_MIC_CLK, PINCTRL_DRIVE_12mA);

    // Configure SPI peripheral
    pParam.eSclkDiv         = 0xFF;
    pParam.eSCLKPolarity    = SPI_CPOL_SCLK_HIGH_IN_IDLE_STATES;
    pParam.eSCLKPhase       = SPI_CPHA_EVEN_SCLK_EDGES;
    pParam.eLsbMsbOrder     = SPI_LSB_LEAST_SIGNIFICANT_BIT_FIRST;
    pParam.eDataSize        = 4;     // 5 bits per transfer (SPI_DATALEN_4_BITS)
    pParam.eMasterSlaveMode = SPI_SLVMODE_MASTER_MODE;
    pParam.eReadWriteMode   = SPI_TRANSMODE_WRITE_ONLY;
    pParam.eQuadMode        = SPI_DUALQUAD_REGULAR_MODE;
    pParam.eWriteTransCnt   = 0;
    pParam.eReadTransCnt    = 0;
    pParam.eAddrEn          = SPI_ADDREN_DISABLE;
    pParam.eCmdEn           = SPI_CMDEN_DISABLE;
    pParam.RxThres          = 4;
    pParam.TxThres          = 4;
    pParam.SlaveDataOnly    = SPI_SLVDATAONLY_DISABLE;
    pParam.eAddrLen         = SPI_ADDRLEN_1_BYTE;
    pParam.eInterruptMask   = 0;
    pParam.eMOSI_Dir        = SPI_MOSI_BI_DIR_MODE;  // Bidirectional MOSI

    apSSP_DeviceParametersSet(SPI_MODE, &pParam);

    // Configure nRESET pin
    #ifdef nRESET_PIN
    PINCTRL_SetPadMux(nRESET_PIN, IO_SOURCE_GPIO);
    GIO_SetDirection(nRESET_PIN, GIO_DIR_INPUT);
    PINCTRL_Pull(nRESET_PIN, PINCTRL_PULL_UP);
    #endif

    swd_spi_timeout = 0U;
}

void swd_spi_set_clock(uint32_t clock_hz)
{
    apSSP_SetTimingSclkDiv(SPI_MODE, ((10000000/clock_hz - 2)&0xff));
}

void Set_Clock_Delay(uint32_t clock)
{
    DAP_Data.fast_clock = 1U;
    DAP_Data.clock_delay = 1U;
    swd_spi_set_clock(clock);
}

static RAM_CODE void swd_spi_idle_cycles(uint32_t count)
{
    uint32_t chunk;

    if (count == 0U) {
        return;
    }
    chunk = (count > 32U) ? 32U : count;
    spi_dir_write(chunk, 0U);
    count -= chunk;
    while (count != 0U) {
        chunk = (count > 32U) ? 32U : count;
        spi_fast_write_bit(chunk, 0U);
        count -= chunk;
    }
}

//**************************************************************************************************
// SWD Transfer (hardware SPI accelerated)
//**************************************************************************************************

RAM_CODE uint8_t SWD_Transfer(uint32_t request, uint32_t *data)
{
    uint32_t parity;
    uint32_t ack;
    uint32_t rw_data;
    uint32_t read_data;
    uint32_t read_parity;

    parity = 0;
    rw_data = 0;
    ack = 0;
    swd_spi_timeout = 0U;

    // Request phase: pack and send 8-bit SWD request
    // Start bit(1) + APnDP + RnW + A[2:3] + Parity + Park(1) + Stop(0)
    parity = __builtin_parity(request & 0xf);
    rw_data = 0x81 | (parity << 5) | ((request & 0xf) << 1);
    spi_dir_write(8, rw_data);
    if (swd_spi_timeout != 0U) {
        return DAP_TRANSFER_ERROR;
    }

    // Ack phase: read 3-bit ack + turnaround
    spi_dir_read(3 + DAP_Data.swd_conf.turnaround, ack);
    if (swd_spi_timeout != 0U) {
        return DAP_TRANSFER_ERROR;
    }
    ack = (ack >> DAP_Data.swd_conf.turnaround) & 0x7;

    if (ack == DAP_TRANSFER_OK) {
        if (request & DAP_TRANSFER_RnW) {
            // Read data phase: 16 bits + 16 bits + parity + turnaround
            rw_data = 0;
            parity = 0;
            spi_dir_read(16, rw_data);
            read_data = rw_data;
            spi_fast_read_bit(17 + DAP_Data.swd_conf.turnaround, rw_data);
            read_data |= ((rw_data & 0xFFFF) << 16);
            parity = __builtin_parity(read_data);
            read_parity = (rw_data >> 16) & 0x1;
            if ((parity ^ read_parity) & 1U) {
                ack = DAP_TRANSFER_ERROR;
            }
            if (data) { *data = read_data; }
        } else {
            // Write data phase: 16 bits + turnaround + 16 bits + parity
            rw_data = *data;
            parity = 0;
            spi_dir_write(16 + DAP_Data.swd_conf.turnaround,
                          rw_data << DAP_Data.swd_conf.turnaround);
            parity = __builtin_parity(rw_data);
            spi_fast_write_bit(17, rw_data >> 16 | (parity << 16));
        }

        // Capture Timestamp
        if (request & DAP_TRANSFER_TIMESTAMP) {
            DAP_Data.timestamp = TIMESTAMP_GET();
        }
        swd_spi_idle_cycles(DAP_Data.transfer.idle_cycles);
        if (swd_spi_timeout != 0U) {
            return DAP_TRANSFER_ERROR;
        }
        return (swd_spi_timeout != 0U) ? DAP_TRANSFER_ERROR : (uint8_t)ack;
    }

    if ((ack == DAP_TRANSFER_WAIT) || (ack == DAP_TRANSFER_FAULT)) {
        // Drain data phase
        if (DAP_Data.swd_conf.data_phase && ((request & DAP_TRANSFER_RnW) != 0U)) {
            spi_dir_read(16, rw_data);
            spi_fast_read_bit(17, rw_data);
        }
        spi_dir_read(DAP_Data.swd_conf.turnaround, rw_data);
        if (DAP_Data.swd_conf.data_phase && ((request & DAP_TRANSFER_RnW) == 0U)) {
            spi_dir_write(16, 0);
            spi_fast_write_bit(17, 0);
        }
        return ((uint8_t)ack);
    }

    // Protocol error
    spi_dir_read(16, rw_data);
    spi_fast_read_bit(17 + DAP_Data.swd_conf.turnaround, rw_data);

    return (swd_spi_timeout != 0U) ? DAP_TRANSFER_ERROR : (uint8_t)ack;
}

//**************************************************************************************************
// SWJ Sequence (hardware SPI accelerated)
//**************************************************************************************************

RAM_CODE void SWJ_Sequence(uint32_t count, const uint8_t *data)
{
    uint32_t bytes = count / 8;
    uint32_t rem_bits = count % 8;

    if (bytes) {
        spi_dir_write(8, *data++);
        bytes--;
        while (bytes--) {
            spi_fast_write_bit(8, *data++);
        }
    }
    if (rem_bits) {
        spi_dir_write(rem_bits, *data);
    }
}

//**************************************************************************************************
// SWD Sequence (hardware SPI accelerated)
//**************************************************************************************************

RAM_CODE void SWD_Sequence(uint32_t info, const uint8_t *swdo, uint8_t *swdi)
{
    uint32_t val;
    uint32_t bit_len;

    bit_len = info & SWD_SEQUENCE_CLK;
    if (bit_len == 0) {
        bit_len = 64U;
    }

    if (info & SWD_SEQUENCE_DIN) {
        if (bit_len / 8) {
            spi_dir_read(8, val);
            *swdi++ = (uint8_t)val;
            bit_len -= 8U;
            while (bit_len / 8U) {
                spi_fast_read_bit(8, val);
                *swdi++ = (uint8_t)val;
                bit_len -= 8U;
            }
        }
        if (bit_len != 0U) {
            if ((info & SWD_SEQUENCE_CLK) == bit_len) {
                spi_dir_read(bit_len, val);
            } else {
                spi_fast_read_bit(bit_len, val);
            }
            *swdi = (uint8_t)val;
        }
    } else {
        if (bit_len / 8) {
            spi_dir_write(8, *swdo++);
            bit_len -= 8U;
            while (bit_len / 8U) {
                spi_fast_write_bit(8, *swdo++);
                bit_len -= 8U;
            }
        }
        if (bit_len != 0U) {
            if ((info & SWD_SEQUENCE_CLK) == bit_len) {
                spi_dir_write(bit_len, *swdo);
            } else {
                spi_fast_write_bit(bit_len, *swdo);
            }
        }
    }
}

uint32_t swd_target_reset_aircr(void)
{
    uint32_t data;

    data = 0U;
    if (SWD_Transfer(DP_SELECT, &data) != DAP_TRANSFER_OK) {
        return 0U;
    }
    data = 0x23000052U;
    if (SWD_Transfer(DAP_TRANSFER_APnDP, &data) != DAP_TRANSFER_OK) {
        return 0U;
    }
    data = 0xE000ED0CU;
    if (SWD_Transfer(DAP_TRANSFER_APnDP | DAP_TRANSFER_A2, &data) != DAP_TRANSFER_OK) {
        return 0U;
    }
    data = 0x05FA0004U;
    if (SWD_Transfer(DAP_TRANSFER_APnDP | DAP_TRANSFER_A2 | DAP_TRANSFER_A3, &data) != DAP_TRANSFER_OK) {
        return 0U;
    }
    return (SWD_Transfer(DP_RDBUFF | DAP_TRANSFER_RnW, &data) == DAP_TRANSFER_OK) ? 1U : 0U;
}
