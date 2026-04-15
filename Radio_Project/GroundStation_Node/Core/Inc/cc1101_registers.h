/*
 * cc1101_registers.h
 *
 *  Created on: Mar 24, 2026
 *      Author: andreasodontopoulos
 */

/*
 * cc1101_registers.h
 * HeliosLink Project — CC1101 Register Map
 *
 * This file is a pure lookup table. Every register address, every command
 * strobe, and every status register the CC1101 exposes over SPI lives here.
 * Nothing in this file does anything — it just gives human-readable names
 * to raw hex addresses so the driver code is readable.
 *
 * CC1101 SPI address byte format (always the first byte of any transaction):
 *
 *   bit 7 | bit 6 | bits 5:0
 *   R/W   | Burst | Address
 *
 *   R/W  = 0 → write to chip
 *          1 → read from chip
 *   Burst= 0 → single byte transaction
 *          1 → multi-byte burst OR access a status register (0x30–0x3D)
 *
 * That Burst bit dual-use is the most important thing to understand here.
 * It means you CANNOT read a status register with the READ_SINGLE flag —
 * the chip will interpret it as a command strobe instead.
 */

#ifndef INC_CC1101_REGISTERS_H_
#define INC_CC1101_REGISTERS_H_

/* =========================================================================
 * Configuration Registers (address 0x00–0x2E, read/write)
 * These hold the radio's operating parameters: frequency, data rate,
 * modulation, packet format, etc. You configure these in CC1101_Init().
 * ========================================================================= */
#define CC1101_IOCFG2       0x00  // GDO2 output pin function select
#define CC1101_IOCFG1       0x01  // GDO1 output pin function select
#define CC1101_IOCFG0       0x02  // GDO0 output pin function select
#define CC1101_FIFOTHR      0x03  // TX/RX FIFO threshold for GDO signals
#define CC1101_SYNC1        0x04  // Sync word, high byte  (default 0xD3)
#define CC1101_SYNC0        0x05  // Sync word, low byte   (default 0x91)
#define CC1101_PKTLEN       0x06  // Max packet length in variable-length mode
#define CC1101_PKTCTRL1     0x07  // Packet control: address check, status append
#define CC1101_PKTCTRL0     0x08  // Packet control: format (FIFO/direct), CRC, length mode
#define CC1101_ADDR         0x09  // Device address for optional address filtering
#define CC1101_CHANNR       0x0A  // Channel number (multiplied by channel spacing)
#define CC1101_FSCTRL1      0x0B  // IF frequency (intermediate frequency before baseband)
#define CC1101_FSCTRL0      0x0C  // Frequency offset for fine-tuning
#define CC1101_FREQ2        0x0D  // Carrier frequency word, high byte
#define CC1101_FREQ1        0x0E  // Carrier frequency word, middle byte
#define CC1101_FREQ0        0x0F  // Carrier frequency word, low byte
#define CC1101_MDMCFG4      0x10  // Modem config: channel filter bandwidth + data rate exponent
#define CC1101_MDMCFG3      0x11  // Modem config: data rate mantissa
#define CC1101_MDMCFG2      0x12  // Modem config: modulation format + sync word mode
#define CC1101_MDMCFG1      0x13  // Modem config: FEC enable + preamble length + channel spacing exponent
#define CC1101_MDMCFG0      0x14  // Modem config: channel spacing mantissa
#define CC1101_DEVIATN      0x15  // FSK/GFSK frequency deviation
#define CC1101_MCSM2        0x16  // State machine: RX timeout settings
#define CC1101_MCSM1        0x17  // State machine: which state to go to after TX/RX
#define CC1101_MCSM0        0x18  // State machine: auto-calibration settings
#define CC1101_FOCCFG       0x19  // Frequency offset compensation config
#define CC1101_BSCFG        0x1A  // Bit synchronisation (clock recovery) config
#define CC1101_AGCCTRL2     0x1B  // AGC: max LNA gain, target amplitude
#define CC1101_AGCCTRL1     0x1C  // AGC: carrier sense threshold
#define CC1101_AGCCTRL0     0x1D  // AGC: filter length and hysteresis
#define CC1101_WOREVT1      0x1E  // Wake-on-radio event timeout, high byte
#define CC1101_WOREVT0      0x1F  // Wake-on-radio event timeout, low byte
#define CC1101_WORCTRL      0x20  // Wake-on-radio control
#define CC1101_FREND1       0x21  // RX front-end config (TI-recommended for 868 MHz)
#define CC1101_FREND0       0x22  // TX front-end: which PATABLE entry to use
#define CC1101_FSCAL3       0x23  // Frequency synthesiser calibration value (from SmartRF)
#define CC1101_FSCAL2       0x24  // Frequency synthesiser calibration value
#define CC1101_FSCAL1       0x25  // Frequency synthesiser calibration value
#define CC1101_FSCAL0       0x26  // Frequency synthesiser calibration value
#define CC1101_RCCTRL1      0x27  // RC oscillator config
#define CC1101_RCCTRL0      0x28  // RC oscillator config
#define CC1101_TEST2        0x2C  // Test register (TI-recommended value required)
#define CC1101_TEST1        0x2D  // Test register (TI-recommended value required)
#define CC1101_TEST0        0x2E  // Test register (TI-recommended value required)

/* =========================================================================
 * Command Strobes (address 0x30–0x3D, write only, single byte)
 * These are not registers — they are one-byte commands. Writing any value
 * to one of these addresses triggers an instant action inside the radio.
 * You use CC1101_SendCmd() for these, never CC1101_WriteReg().
 * ========================================================================= */
#define CC1101_SRES         0x30  // Software reset: restores all registers to defaults
#define CC1101_SFSTXON      0x31  // Enable + calibrate frequency synthesiser (stay in FSTXON)
#define CC1101_SXOFF        0x32  // Power down crystal oscillator
#define CC1101_SCAL         0x33  // Calibrate synthesiser then power it down
#define CC1101_SRX          0x34  // Go to RX mode (calibrates first if MCSM0 says so)
#define CC1101_STX          0x35  // Go to TX mode (from IDLE — calibrates if needed)
#define CC1101_SIDLE        0x36  // Exit RX/TX, turn off synthesiser, go to IDLE
#define CC1101_SWOR         0x38  // Start wake-on-radio automatic RX polling
#define CC1101_SPWD         0x39  // Enter power-down mode when CS goes high
#define CC1101_SFRX         0x3A  // Flush the RX FIFO — only safe when not in RX
#define CC1101_SFTX         0x3B  // Flush the TX FIFO — only safe when not in TX
#define CC1101_SWORRST      0x3C  // Reset the wake-on-radio timer
#define CC1101_SNOP         0x3D  // No operation — just returns the status byte

/* =========================================================================
 * Status Registers (address 0x30–0x3D, read only)
 *
 * CRITICAL: These share address space with the command strobes above.
 * The CC1101 uses bit 6 of the SPI address byte (the "burst" bit) to tell
 * them apart: Burst=0 → command strobe. Burst=1 → status register read.
 *
 * This is why you MUST use CC1101_ReadStatus() for these, not CC1101_ReadReg().
 * CC1101_ReadStatus() sets the 0xC0 flag (read + burst bits), while
 * CC1101_ReadReg() only sets 0x80 (read only).
 *
 * If you use CC1101_ReadReg(CC1101_VERSION), the chip sees address 0x31
 * with burst=0 and executes the SFSTXON command instead of returning version!
 * ========================================================================= */
#define CC1101_PARTNUM      0x30  // Part number — always reads 0x00 on CC1101
#define CC1101_VERSION      0x31  // Silicon revision — reads 0x14 on genuine CC1101
#define CC1101_FREQEST      0x32  // Frequency offset estimate from demodulator
#define CC1101_LQI          0x33  // Link Quality Indicator — demodulator signal quality
#define CC1101_RSSI         0x34  // Received Signal Strength Indication (raw, needs conversion)
#define CC1101_MARCSTATE    0x35  // Current state machine state (see CC1101_STATE_* below)
#define CC1101_TXBYTES      0x3A  // Number of bytes currently sitting in the TX FIFO
#define CC1101_RXBYTES      0x3B  // Number of bytes currently sitting in the RX FIFO

/*
 * MARCSTATE values — what the radio is doing right now
 * Read with: CC1101_ReadStatus(CC1101_MARCSTATE) & 0x1F
 * (mask to lower 5 bits — upper bits are don't-care)
 */
#define CC1101_STATE_IDLE               0x01  // Doing nothing, ready for commands
#define CC1101_STATE_RX                 0x0D  // Actively listening for a packet
#define CC1101_STATE_TX                 0x13  // Actively transmitting a packet
#define CC1101_STATE_RXFIFO_OVERFLOW    0x11  // RX FIFO got too full — must flush with SFRX
#define CC1101_STATE_TXFIFO_UNDERFLOW   0x16  // TX FIFO ran dry during TX — must flush with SFTX

/* =========================================================================
 * FIFO Access
 * Both TX and RX FIFOs live at address 0x3F.
 * The chip routes automatically: write → TX FIFO, read → RX FIFO.
 * Always use burst mode (0x40 or 0xC0) to write/read multiple bytes at once.
 * ========================================================================= */
#define CC1101_FIFO         0x3F  // TX write / RX read — always use with burst flag

/* =========================================================================
 * PA Power Table
 * The PATABLE holds up to 8 TX power levels you can switch between.
 * For our project we only use entry [0] (one fixed TX power).
 * ========================================================================= */
#define CC1101_PATABLE      0x3E  // PA power table — 0xC0 ≈ +12 dBm at 868 MHz

/* =========================================================================
 * SPI Flags — ORed onto the address byte to set R/W and Burst bits
 * ========================================================================= */
#define CC1101_WRITE_SINGLE 0x00  // Write, single byte  — usually omitted (it's just 0)
#define CC1101_WRITE_BURST  0x40  // Write, multiple bytes
#define CC1101_READ_SINGLE  0x80  // Read, single byte   — use for config registers 0x00–0x2E
#define CC1101_READ_BURST   0xC0  // Read, multiple bytes OR read a status register (0x30–0x3D)

#endif /* INC_CC1101_REGISTERS_H_ */


/* INC_CC1101_REGISTERS_H_ */
