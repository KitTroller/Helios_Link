/*
 * cc1101.c
 *
 *  Created on: Mar 25, 2026
 *      Author: andreasodontopoulos
 */

/*
 * cc1101.c
 * HeliosLink Project — CC1101 Driver Implementation
 *
 * This file owns all SPI communication with the CC1101.
 * Nothing outside this file should ever call HAL_SPI_* directly.
 *
 * SPI wiring for Nucleo F446RE (confirmed from your CubeMX .ioc file):
 *   SPI2_SCK   → PB10
 *   SPI2_MOSI  → PC1
 *   SPI2_MISO  → PC2   ← IMPORTANT: not the default PB14
 *   CS (manual)→ PB12
 */

#include "cc1101.h"

/* The SPI handle is created and owned by main.c (via CubeMX MX_SPI2_Init).
 * We declare it extern here so this file can use it without owning it.
 * This is standard HAL practice for peripheral sharing across files. */
extern SPI_HandleTypeDef hspi2;

/* CS pin macros — pulling CS LOW selects the CC1101 and begins a transaction.
 * CS HIGH deselects it and ends the transaction. The CC1101 ignores all SPI
 * traffic while CS is high, so every function must bracket its SPI calls
 * with CS_LOW() ... CS_HIGH(). */
#define CS_LOW()  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET)
#define CS_HIGH() HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET)

/* MISO pin — used only during the reset handshake to detect when the
 * CC1101's crystal oscillator has stabilised. After reset, the chip holds
 * MISO low until it's ready. This is confirmed from your CubeMX: PC2. */
#define MISO_PIN() HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_2)

/* =========================================================================
 * Static (private) helper functions
 * These are internal to this file — not exposed in the header.
 * The 'static' keyword enforces this at compile time.
 * ========================================================================= */

/*
 * CC1101_WriteFIFO()
 * Burst-writes 'len' bytes from 'data' into the CC1101 TX FIFO.
 *
 * Why burst mode? The FIFO is a multi-byte register. In the CC1101 SPI
 * protocol, writing a multi-byte register requires the WRITE_BURST flag
 * (bit 6 = 1). Without it, only the first byte lands correctly and subsequent
 * bytes are written to wrong addresses.
 *
 * Address byte sent: 0x3F | 0x40 = 0x7F (FIFO address + burst write flag)
 */
static void CC1101_WriteFIFO(uint8_t *data, uint8_t len) {
    uint8_t addr = CC1101_FIFO | CC1101_WRITE_BURST;  // 0x7F
    CS_LOW();
    HAL_SPI_Transmit(&hspi2, &addr, 1, 100);   // Tell chip: "I'm going to write the FIFO"
    HAL_SPI_Transmit(&hspi2, data, len, 100);   // Write all bytes in one burst
    CS_HIGH();
}

/*
 * CC1101_ReadFIFO()
 * Burst-reads 'len' bytes from the CC1101 RX FIFO into 'buf'.
 *
 * Address byte sent: 0x3F | 0xC0 = 0xFF (FIFO address + read + burst flags)
 * The chip auto-routes: write to 0x3F goes to TX FIFO, read from 0x3F
 * comes from RX FIFO. Same address, separate physical memories.
 */
static void CC1101_ReadFIFO(uint8_t *buf, uint8_t len) {
    uint8_t addr = CC1101_FIFO | CC1101_READ_BURST;   // 0xFF
    CS_LOW();
    HAL_SPI_Transmit(&hspi2, &addr, 1, 100);   // Tell chip: "I'm going to read the FIFO"
    HAL_SPI_Receive(&hspi2, buf, len, 100);     // Read all bytes in one burst
    CS_HIGH();
}

/* =========================================================================
 * Public low-level register access
 * ========================================================================= */

/*
 * CC1101_WriteReg()
 * Writes a single byte to a configuration register (addresses 0x00–0x2E).
 * The address byte has bit 7 = 0 (write) and bit 6 = 0 (single byte).
 * So the address byte is just the raw register address unchanged.
 */
void CC1101_WriteReg(uint8_t regAddr, uint8_t data) {
    CS_LOW();
    HAL_SPI_Transmit(&hspi2, &regAddr, 1, 100);  // Address byte (write flag implicit = 0)
    HAL_SPI_Transmit(&hspi2, &data, 1, 100);     // Data byte
    CS_HIGH();
}

/*
 * CC1101_ReadReg()
 * Reads a single byte from a configuration register (addresses 0x00–0x2E).
 * Sets bit 7 of the address (READ_SINGLE = 0x80) to signal a read.
 *
 * DO NOT use this for status registers (0x30–0x3D) — use CC1101_ReadStatus().
 */
uint8_t CC1101_ReadReg(uint8_t regAddr) {
    uint8_t received = 0;
    uint8_t read_addr = regAddr | CC1101_READ_SINGLE;  // Set bit 7 = read
    CS_LOW();
    HAL_SPI_Transmit(&hspi2, &read_addr, 1, 100);
    HAL_SPI_Receive(&hspi2, &received, 1, 100);
    CS_HIGH();
    return received;
}

/*
 * CC1101_ReadStatus()
 * Reads a status register (addresses 0x30–0x3D).
 *
 * Sets BOTH bit 7 (read) AND bit 6 (burst) → READ_BURST = 0xC0.
 * This is the only difference from CC1101_ReadReg, but it's critical:
 *
 *   CC1101_ReadReg(CC1101_VERSION)   → address sent = 0x31 | 0x80 = 0xB1
 *     Chip sees: read, single, address=0x31 → interprets as command strobe SFSTXON!
 *     Returns garbage and fires up the synthesiser. Wrong.
 *
 *   CC1101_ReadStatus(CC1101_VERSION) → address sent = 0x31 | 0xC0 = 0xF1
 *     Chip sees: read, burst, address=0x31 → status register read. Returns 0x14. Correct.
 */
uint8_t CC1101_ReadStatus(uint8_t regAddr) {
    uint8_t received = 0;
    uint8_t read_addr = regAddr | CC1101_READ_BURST;   // Set bits 7 and 6
    CS_LOW();
    HAL_SPI_Transmit(&hspi2, &read_addr, 1, 100);
    HAL_SPI_Receive(&hspi2, &received, 1, 100);
    CS_HIGH();
    return received;
}

/*
 * CC1101_SendCmd()
 * Sends a one-byte command strobe. No data byte follows — the address
 * byte IS the entire transaction.
 *
 * The CC1101 executes the command immediately (reset, start TX, flush FIFO, etc.)
 * and returns the status byte on MISO during the same clock cycle.
 * We discard the status byte here — check MARCSTATE explicitly when needed.
 */
void CC1101_SendCmd(uint8_t cmd) {
    CS_LOW();
    HAL_SPI_Transmit(&hspi2, &cmd, 1, 100);
    CS_HIGH();
}

/* =========================================================================
 * Initialisation
 * ========================================================================= */

/*
 * CC1101_Init()
 * Full initialisation sequence. This must be called on boot before any TX/RX.
 *
 * Register values chosen by TI's SmartRF Studio for:
 *   Carrier:    868.35 MHz (EU ISM band)
 *   Modulation: GFSK (Gaussian-shaped FSK — cleaner spectrum than plain FSK,
 *               required for EU type approval, and better for GNU Radio decoding)
 *   Data rate:  1.2 kbps (minimum rate = maximum range = best sensitivity ~-112 dBm)
 *   Deviation:  5.157 kHz (narrow GFSK, maximises sensitivity at this data rate)
 *   Channel BW: 58.5 kHz (fits our signal with plenty of margin)
 *   TX power:   +12 dBm (maximum for CC1101)
 *   Packet mode: variable length, CRC-16 enabled, data whitening enabled
 */
void CC1101_Init(void) {

    /* --- Step 1: Reset sequence (CC1101 datasheet section 10.1) ---
     *
     * On power-up, the CC1101's SPI interface may be in an undefined state
     * (especially if the MCU reset without cutting power to the radio).
     * The CS pulse sequence forces it into a known state before we issue SRES.
     *
     * Then SRES restores all internal registers to their power-on defaults.
     * After SRES, the chip holds MISO LOW until its crystal oscillator
     * has locked and it's ready for SPI commands. We wait for that signal
     * rather than using a fixed delay — this is the correct hardware handshake.
     */
    CS_HIGH(); HAL_Delay(1);
    CS_LOW();  HAL_Delay(1);
    CS_HIGH(); HAL_Delay(1);
    CS_LOW();

    /* Wait for MISO to go low: CC1101 acknowledges it is alive */
    uint32_t timeout = HAL_GetTick() + 100;
    while (MISO_PIN() != GPIO_PIN_RESET) {
        if (HAL_GetTick() > timeout) break;  // Hardware missing — CC1101_Verify() will catch this
    }

    /* Send the software reset command */
    HAL_SPI_Transmit(&hspi2, (uint8_t[]){CC1101_SRES}, 1, 100);
    CS_HIGH();
    HAL_Delay(10);  // CC1101 datasheet: wait at least 41 µs after SRES

    /* Wait again for MISO low before writing any configuration */
    CS_LOW();
    timeout = HAL_GetTick() + 100;
    while (MISO_PIN() != GPIO_PIN_RESET) {
        if (HAL_GetTick() > timeout) break;
    }
    CS_HIGH();

    /* --- Step 2: Carrier frequency — 868.35 MHz ---
     *
     * Formula: FREQ = F_carrier × 2^16 / F_xosc = 868.35e6 × 65536 / 26e6 = 2,188,918
     * In hex: 0x216276
     * Split across three bytes: FREQ2=0x21, FREQ1=0x62, FREQ0=0x76
     */
    CC1101_WriteReg(CC1101_FREQ2, 0x21);
    CC1101_WriteReg(CC1101_FREQ1, 0x62);
    CC1101_WriteReg(CC1101_FREQ0, 0x76);

    /* --- Step 3: Modem configuration ---
     *
     * MDMCFG4 = 0xF5:
     *   bits [7:6] = 11 → channel filter bandwidth = 58.5 kHz
     *   bits [3:0] = 5  → data rate exponent (DRATE_E)
     *
     * MDMCFG3 = 0x83:
     *   bits [7:0] = 131 → data rate mantissa (DRATE_M)
     *   Combined: DR = 26e6 × (256 + 131) × 2^(5–28) = 1199 bps ≈ 1.2 kbps
     *
     * MDMCFG2 = 0x13:
     *   bits [6:4] = 001 → GFSK modulation
     *   bit  [3]   = 0   → Manchester encoding off
     *   bits [2:0] = 011 → 30/32 sync word detection (tolerates 1 bit error in sync word)
     *
     * MDMCFG1 = 0x22:
     *   bit  [7]   = 0   → Forward Error Correction disabled (we use CRC-16 instead)
     *   bits [5:4] = 10  → 4 preamble bytes (0xAA 0xAA 0xAA 0xAA before sync)
     *   bits [1:0] = 2   → channel spacing exponent
     *
     * MDMCFG0 = 0xF8: channel spacing mantissa (200 kHz spacing)
     */
    CC1101_WriteReg(CC1101_MDMCFG4, 0xF5);
    CC1101_WriteReg(CC1101_MDMCFG3, 0x83);
    CC1101_WriteReg(CC1101_MDMCFG2, 0x13);
    CC1101_WriteReg(CC1101_MDMCFG1, 0x22);
    CC1101_WriteReg(CC1101_MDMCFG0, 0xF8);

    /* DEVIATN = 0x15: FSK frequency deviation
     * bits [6:4] = 1 → exponent, bits [2:0] = 5 → mantissa
     * Deviation = 26e6 × (8 + 5) × 2^(1-17) = 5157 Hz ≈ 5.16 kHz
     * Narrow deviation at 1.2 kbps = good sensitivity. */
    CC1101_WriteReg(CC1101_DEVIATN, 0x15);

    /* --- Step 4: Packet engine configuration ---
     *
     * PKTCTRL1 = 0x04:
     *   bit [2] = 1 → APPEND_STATUS: the chip appends 2 status bytes after each
     *                 received packet in the RX FIFO: [RSSI_raw][LQI | CRC_OK]
     *                 This gives us free RSSI and link quality data per packet.
     *
     * PKTCTRL0 = 0x45:
     *   bit [6]   = 1  → data whitening ON: XORs payload with a PN9 sequence.
     *                     Prevents long runs of 0s or 1s which would create DC bias
     *                     in the FSK signal. Improves receiver lock. Both ends must
     *                     have this set the same way — they do, since same code.
     *   bits [5:4]= 00 → normal FIFO mode (not direct/serial mode)
     *   bit [2]   = 1  → CRC-16 enabled in hardware. On TX, appended automatically.
     *                     On RX, checked automatically — result in the status byte.
     *   bits [1:0]= 01 → variable length mode: the first byte in the packet is the
     *                     length. Allows us to send different-sized payloads.
     *
     * PKTLEN = 61: maximum allowed packet length in variable mode.
     *              61 + 1 (length byte) + 2 (CRC) = 64 = exactly the FIFO size.
     */
    CC1101_WriteReg(CC1101_PKTCTRL1, 0x04);
    CC1101_WriteReg(CC1101_PKTCTRL0, 0x45);
    CC1101_WriteReg(CC1101_PKTLEN,   61);

    /* --- Step 5: State machine behaviour ---
     *
     * MCSM1 = 0x30:
     *   bits [3:2] = 00 → after TX, go to IDLE (not back to RX — we control this)
     *   bits [1:0] = 00 → after RX, go to IDLE (we flush + restart RX explicitly)
     *
     * MCSM0 = 0x18:
     *   bits [5:4] = 01 → auto-calibrate when going from IDLE to RX or TX
     *                     This re-calibrates the synthesiser on each state change,
     *                     keeping the carrier frequency accurate over temperature.
     *   bits [3:2] = 10 → ~150 µs power-on stabilisation timeout
     */
    CC1101_WriteReg(CC1101_MCSM1, 0x30);
    CC1101_WriteReg(CC1101_MCSM0, 0x18);

    /* --- Step 6: IF, AGC and frequency offset compensation ---
     * These are TI SmartRF Studio recommended values for 868 MHz GFSK.
     * They configure the receiver's analogue front-end: intermediate frequency,
     * automatic gain control behaviour, and carrier frequency tracking.
     * Deviating from these without understanding the AGC internals will reduce
     * receiver sensitivity. Treat them as a calibrated starting point.
     */
    CC1101_WriteReg(CC1101_FSCTRL1,  0x08);  // IF = 26e6 × 8 / 2^10 = 203 kHz
    CC1101_WriteReg(CC1101_FSCTRL0,  0x00);  // No additional frequency offset
    CC1101_WriteReg(CC1101_FOCCFG,   0x16);  // Frequency offset compensation: ±BWchan/4 gate
    CC1101_WriteReg(CC1101_BSCFG,    0x6C);  // Bit synchronisation: 1/8 symbol loop bandwidth
    CC1101_WriteReg(CC1101_AGCCTRL2, 0x43);  // AGC: max LNA gain, target amplitude = 33 dB
    CC1101_WriteReg(CC1101_AGCCTRL1, 0x40);  // AGC: relative carrier sense threshold
    CC1101_WriteReg(CC1101_AGCCTRL0, 0x91);  // AGC: 16-sample filter, medium hysteresis

    /* --- Step 7: Front-end configuration ---
     * FREND1 = 0xB6: RX front-end LNA / mixer configuration for 868 MHz GFSK (TI-recommended)
     * FREND0 = 0x10: TX uses PATABLE[0] for power level (FSK/GFSK non-ramp mode)
     */
    CC1101_WriteReg(CC1101_FREND1,   0xB6);
    CC1101_WriteReg(CC1101_FREND0,   0x10);

    /* --- Step 8: Frequency synthesiser calibration constants ---
     * These are analogue calibration values derived by TI for the 868 MHz band.
     * They correct for manufacturing variation in the LC tank circuit that sets
     * the VCO frequency. Without them, your TX frequency may drift by tens of kHz.
     * Source: TI CC1101 datasheet Table 43 + Application Note DN004.
     */
    CC1101_WriteReg(CC1101_FSCAL3,   0xEA);
    CC1101_WriteReg(CC1101_FSCAL2,   0x2A);
    CC1101_WriteReg(CC1101_FSCAL1,   0x00);
    CC1101_WriteReg(CC1101_FSCAL0,   0x1F);

    /* --- Step 9: Test registers ---
     * These must be set to TI's recommended values. Changing them may cause
     * unpredictable behaviour. They configure internal analogue biasing that
     * is not fully documented publicly.
     */
    CC1101_WriteReg(CC1101_TEST2,    0x81);
    CC1101_WriteReg(CC1101_TEST1,    0x35);
    CC1101_WriteReg(CC1101_TEST0,    0x09);

    /* --- Step 10: TX power ---
     * PATABLE[0] = 0xC0 → approximately +12 dBm output power at 868 MHz.
     * FREND0 = 0x10 (set above) points the TX engine at PATABLE[0].
     * EU ISM 868.0–868.6 MHz allows max 25 mW ERP with ≤1% duty cycle.
     * Duty cycle enforcement must be done in application code (see main.c).
     */
    CC1101_WriteReg(CC1101_PATABLE,  0xC0);
}

/* =========================================================================
 * Verify — boot self-check
 * ========================================================================= */

/*
 * CC1101_Verify()
 * Reads the VERSION status register and confirms it matches the expected
 * value for a genuine CC1101 (0x14).
 *
 * This is your SPI wiring validation. If this returns ERR_NO_CHIP:
 *   1. Check your jumper cable connections (see wiring table in main.c)
 *   2. Check that CS is PB12 and MISO is PC2
 *   3. Check that the CC1101 module has 3.3V power (not 5V)
 *   4. Check SPI clock polarity: CPOL=0, CPHA=1EDGE
 */
CC1101_Status_t CC1101_Verify(void) {
    uint8_t version = CC1101_ReadStatus(CC1101_VERSION);
    if (version != 0x14) {
        return CC1101_ERR_NO_CHIP;
    }
    return CC1101_OK;
}

/* =========================================================================
 * Packet TX
 * ========================================================================= */

/*
 * CC1101_SendPacket()
 *
 * What happens on the wire when you call this:
 *
 *  1. We go to IDLE and flush any leftover bytes in the TX FIFO.
 *     Transmitting with stale FIFO data would corrupt the packet.
 *
 *  2. We build a raw byte buffer in this layout:
 *       [length_byte][packet_id][command][payload bytes...]
 *     The length_byte value = 2 + payload_len. The CC1101 uses this byte
 *     to know how many more bytes follow (it doesn't count itself).
 *
 *  3. We burst-write this buffer to the TX FIFO.
 *
 *  4. STX command: the CC1101 takes over. It transmits:
 *       4× preamble bytes (0xAA) → sync word (0xD391) → length byte →
 *       our data → CRC-16 (hardware calculated and appended)
 *     Total over-air bytes for a 10-byte payload: 4+2+1+12+2 = 21 bytes
 *     At 1.2 kbps, this takes about 17.5 ms.
 *
 *  5. We poll MARCSTATE until the radio returns to IDLE (TX complete).
 *     Polling is safe here — at 1.2 kbps, we know the max wait time.
 */
CC1101_Status_t CC1101_SendPacket(CC1101_Packet_t *pkt) {

    /* Sanity check: payload must fit in our frame */
    if (pkt->payload_len > CC1101_MAX_PAYLOAD) {
        return CC1101_ERR_OVERFLOW;
    }

    /* Go to IDLE and flush the TX FIFO.
     * Never transmit without flushing first — previous TX may have left
     * bytes behind if it was interrupted. */
    CC1101_SendCmd(CC1101_SIDLE);
    CC1101_SendCmd(CC1101_SFTX);

    /* Build the TX buffer.
     * Layout: [length_byte | packet_id | command | payload...]
     * We write length_byte = 2 + payload_len (the CC1101 length byte
     * counts everything that follows it, not including itself). */
    uint8_t total_data_len = 2 + pkt->payload_len;   // packet_id + command + payload
    uint8_t tx_buf[CC1101_MAX_PAYLOAD + 3];           // length byte + id + cmd + payload

    tx_buf[0] = total_data_len;                        // CC1101 variable-length byte
    tx_buf[1] = pkt->packet_id;                        // Our sequence number
    tx_buf[2] = (uint8_t)pkt->command;                 // Our message type
    memcpy(&tx_buf[3], pkt->payload, pkt->payload_len);// Our data

    /* Burst-write everything to TX FIFO in one SPI transaction */
    CC1101_WriteFIFO(tx_buf, total_data_len + 1);       // +1 to include the length byte itself

    /* Kick off transmission */
    CC1101_SendCmd(CC1101_STX);

    /* Wait for TX to complete.
     * The radio is in state TX (0x13) while transmitting.
     * It returns to IDLE (0x01) when done.
     * Timeout: 500 ms is generous even for maximum-length packets at 1.2 kbps. */
    uint32_t timeout = HAL_GetTick() + 500;
    uint8_t state;
    do {
        state = CC1101_ReadStatus(CC1101_MARCSTATE) & 0x1F;
        if (HAL_GetTick() > timeout) {
            CC1101_SendCmd(CC1101_SIDLE);   // Force radio back to IDLE
            CC1101_SendCmd(CC1101_SFTX);    // Clean up the FIFO
            return CC1101_ERR_TIMEOUT;
        }
    } while (state == CC1101_STATE_TX);

    /* Check for FIFO underflow (shouldn't happen, but catch it) */
    if (state == CC1101_STATE_TXFIFO_UNDERFLOW) {
        CC1101_SendCmd(CC1101_SIDLE);
        CC1101_SendCmd(CC1101_SFTX);
        return CC1101_ERR_OVERFLOW;
    }

    return CC1101_OK;
}

/* =========================================================================
 * Packet RX
 * ========================================================================= */

/*
 * CC1101_ReceivePacket()
 *
 * What happens on the wire when you call this:
 *
 *  1. We go to IDLE, flush the RX FIFO (clear any partial previous packet),
 *     then send SRX to start the receiver.
 *
 *  2. The CC1101 is now listening on 868.35 MHz. Its AGC automatically
 *     adjusts gain. It scans incoming bits for the preamble + sync word.
 *     When it finds a valid sync word, it starts capturing the packet into
 *     the RX FIFO.
 *
 *  3. We poll RXBYTES until it's non-zero (bytes have arrived).
 *     A small extra delay lets the full packet land before we read.
 *
 *  4. We burst-read the entire FIFO contents:
 *       [length_byte][packet_id][command][payload...][RSSI_raw][LQI|CRC_OK]
 *     The last 2 bytes are status bytes appended by hardware (PKTCTRL1 bit 2).
 *
 *  5. We check the CRC_OK bit (MSB of the last status byte).
 *     If CRC fails: packet arrived but was corrupted. We increment PER statistics.
 *
 *  6. We convert raw RSSI to dBm using the CC1101 datasheet formula:
 *       if RSSI_raw >= 128: RSSI_dBm = (RSSI_raw - 256) / 2 - 74
 *       else:               RSSI_dBm = RSSI_raw / 2 - 74
 *     This gives you the actual received signal strength at the antenna port.
 *     Compare this to your link budget predictions!
 */
CC1101_Status_t CC1101_ReceivePacket(CC1101_Packet_t *pkt, int8_t *rssi_out, uint32_t timeout_ms) {

    /* Flush and start RX */
    CC1101_SendCmd(CC1101_SIDLE);
    CC1101_SendCmd(CC1101_SFRX);
    CC1101_SendCmd(CC1101_SRX);

    /* Wait for bytes to appear in the RX FIFO */
    uint32_t deadline = HAL_GetTick() + timeout_ms;
    while (CC1101_ReadStatus(CC1101_RXBYTES) == 0) {
        if (HAL_GetTick() > deadline) {
            CC1101_SendCmd(CC1101_SIDLE);
            return CC1101_ERR_TIMEOUT;
        }
    }

    /* Small delay to let the full packet settle into the FIFO.
     * At 1.2 kbps, a maximum-length 61-byte packet takes ~50 ms to receive.
     * RXBYTES becoming non-zero just means the first byte arrived.
     * We wait for the radio to leave RX state (back to IDLE after full packet). */
    deadline = HAL_GetTick() + 100;
    uint8_t state;
    do {
        state = CC1101_ReadStatus(CC1101_MARCSTATE) & 0x1F;
        if (HAL_GetTick() > deadline) break;  // Proceed anyway, read what we have
    } while (state == CC1101_STATE_RX);

    /* Check for FIFO overflow (RX FIFO got too full before we read it) */
    if (state == CC1101_STATE_RXFIFO_OVERFLOW) {
        CC1101_SendCmd(CC1101_SIDLE);
        CC1101_SendCmd(CC1101_SFRX);
        return CC1101_ERR_OVERFLOW;
    }

    /* Read how many bytes are waiting. Safety check. */
    uint8_t rxbytes = CC1101_ReadStatus(CC1101_RXBYTES);
    if (rxbytes < 3) {
        /* Minimum valid frame: length byte + packet_id + command = 3 bytes
         * (not counting the 2 appended status bytes yet) */
        CC1101_SendCmd(CC1101_SFRX);
        return CC1101_ERR_TIMEOUT;
    }

    /* Burst-read everything from the RX FIFO in one SPI transaction.
     * Buffer layout after this call:
     *   rx_raw[0]              = length byte (= 2 + payload_len)
     *   rx_raw[1]              = packet_id
     *   rx_raw[2]              = command
     *   rx_raw[3 .. length]    = payload bytes
     *   rx_raw[length+1]       = RSSI_raw (appended by hardware)
     *   rx_raw[length+2]       = LQI[6:0] | CRC_OK[7] (appended by hardware)
     */
    uint8_t rx_raw[64];
    CC1101_ReadFIFO(rx_raw, rxbytes);

    uint8_t frame_len = rx_raw[0];  // How many data bytes follow (not counting length byte itself)

    /* Bounds check */
    if (frame_len < 2 || frame_len > 61) {
        CC1101_SendCmd(CC1101_SFRX);
        return CC1101_ERR_OVERFLOW;
    }

    /* The last two bytes in rx_raw are the hardware-appended status bytes.
     * rxbytes = 1 (length byte) + frame_len (data) + 2 (status bytes)
     * So status bytes sit at indices: rxbytes-2 and rxbytes-1 */
    uint8_t rssi_raw  = rx_raw[rxbytes - 2];
    uint8_t lqi_byte  = rx_raw[rxbytes - 1];
    uint8_t crc_ok    = (lqi_byte >> 7) & 0x01;   // Bit 7 = CRC_OK flag

    /* Decode our application frame */
    pkt->packet_id   = rx_raw[1];
    pkt->command     = (CC1101_Cmd_t)rx_raw[2];
    pkt->payload_len = frame_len - 2;              // Subtract packet_id and command bytes
    if (pkt->payload_len > 0) {
        memcpy(pkt->payload, &rx_raw[3], pkt->payload_len);
    }

    /* Convert RSSI to dBm.
     * CC1101 datasheet section 17.3. The raw value is a 2's complement number
     * where the LSB represents 0.5 dB. The -74 offset is the RSSI_offset for
     * the CC1101 (accounts for internal gain, verified at 868 MHz by TI).
     */
    if (rssi_raw >= 128) {
        *rssi_out = (int8_t)(((int16_t)rssi_raw - 256) / 2) - 74;
    } else {
        *rssi_out = (int8_t)(rssi_raw / 2) - 74;
    }

    /* CRC check — our hardware-based error detection */
    if (!crc_ok) {
        return CC1101_ERR_CRC;  // Packet received but corrupted — count this in PER statistics
    }

    return CC1101_OK;
}
