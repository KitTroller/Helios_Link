/*
 * cc1101.h
 *
 *  Created on: Mar 25, 2026
 *      Author: andreasodontopoulos
 */

/*
 * cc1101.h
 * HeliosLink Project — CC1101 Driver Interface
 *
 * This header defines the public API of the CC1101 driver.
 * Your main.c includes this and calls these functions — it never
 * touches SPI or GPIO directly.
 *
 * Architecture overview:
 *
 *   main.c
 *     └── CC1101_SendPacket() / CC1101_ReceivePacket()   ← you call these
 *           └── CC1101_WriteFIFO() / CC1101_ReadFIFO()   ← internal helpers
 *                 └── HAL_SPI_Transmit() / Receive()     ← HAL touches hardware
 *                       └── SPI2 peripheral → CC1101 chip
 *
 * Packet mode vs Direct mode (the key architectural choice):
 *
 *   Direct mode: the CC1101 is a dumb FSK modulator. You clock raw bits
 *   in/out of GDO pins. No FIFO, no CRC, no sync detection. You own everything.
 *
 *   Packet mode (what we use): the CC1101 is an intelligent modem.
 *   You write bytes to a FIFO. The hardware adds preamble, sync word, optional
 *   CRC-16, then transmits. On receive, it finds sync, checks CRC, and
 *   signals you when a valid packet is waiting. RSSI and link quality are
 *   appended automatically to each received packet.
 *
 *   For our thesis (CCSDS framing, PER measurement, RSSI logging, GNU Radio
 *   cross-validation) packet mode is the only sensible choice.
 */

#ifndef INC_CC1101_H_
#define INC_CC1101_H_

#include "main.h"            // HAL types (SPI_HandleTypeDef, GPIO, etc.)
#include "cc1101_registers.h"
#include <stdint.h>
#include <string.h>          // memcpy

/* =========================================================================
 * Return codes
 * Every function that can fail returns one of these.
 * Check the return value — never assume success.
 * ========================================================================= */
typedef enum {
    CC1101_OK             = 0,  // Everything worked
    CC1101_ERR_NO_CHIP    = 1,  // VERSION register didn't return 0x14 — wiring problem
    CC1101_ERR_TIMEOUT    = 2,  // Waited too long for TX to finish or RX packet to arrive
    CC1101_ERR_CRC        = 3,  // Packet received but CRC check failed — corrupted in air
    CC1101_ERR_OVERFLOW   = 4,  // FIFO overflowed — packet too large or FIFO not flushed
} CC1101_Status_t;

/* =========================================================================
 * Packet command types
 * The "command" byte in every packet header tells the receiver what kind
 * of data follows. Think of this like an Ethernet EtherType field.
 * Extend this list as the project grows.
 * ========================================================================= */
typedef enum {
    CMD_PING        = 0x01,  // "Are you alive?" — no payload expected
    CMD_BEACON      = 0x02,  // Periodic "I am alive" from the satellite node
    CMD_TELEMETRY   = 0x03,  // Housekeeping data: temperature, voltage, etc.
    CMD_ACK         = 0x04,  // Acknowledge receipt of a packet
} CC1101_Cmd_t;

/* =========================================================================
 * Packet structure
 *
 * This is our application-layer frame. It sits INSIDE the CC1101 packet
 * (which already has preamble + sync word + length + CRC handled in hardware).
 * Think of it as:
 *
 *   [RF layer, handled by CC1101 hardware]
 *     Preamble | Sync word | Length byte | <-- our frame --> | CRC-16
 *
 *   [Our frame — what this struct represents]
 *     packet_id | command | payload[0..payload_len-1]
 *
 * Inspired by CCSDS transfer frames used in real satellite telemetry.
 * packet_id is a rolling counter — the receiver uses it to detect drops
 * and calculate Packet Error Rate (PER), our key thesis metric.
 *
 * Max payload: 58 bytes keeps total frame at 60 bytes, safely inside
 * the CC1101's 64-byte FIFO (with length byte + 2 status bytes).
 * ========================================================================= */
#define CC1101_MAX_PAYLOAD  58

typedef struct {
    uint8_t packet_id;               // Rolling sequence number (0–255, wraps)
    CC1101_Cmd_t command;            // What type of data this packet carries
    uint8_t payload[CC1101_MAX_PAYLOAD]; // The actual data bytes
    uint8_t payload_len;             // How many bytes of payload[] are valid (0–58)
} CC1101_Packet_t;

/* =========================================================================
 * Public API
 * ========================================================================= */

/*
 * CC1101_Init()
 * Full chip initialisation: reset sequence, 868 MHz GFSK 1.2 kbps config,
 * packet mode with CRC, TI-recommended calibration values.
 * Call once on boot, before anything else.
 */
void CC1101_Init(void);

/*
 * CC1101_Verify()
 * Reads the VERSION status register and checks for the expected value 0x14.
 * Call immediately after CC1101_Init() to confirm SPI wiring is correct.
 * Returns CC1101_ERR_NO_CHIP if the chip isn't responding — check your jumpers.
 */
CC1101_Status_t CC1101_Verify(void);

/*
 * CC1101_SendPacket()
 * Serialises a CC1101_Packet_t into the TX FIFO and triggers transmission.
 * Blocks until the radio returns to IDLE (packet fully sent) or timeout.
 *
 * Flow: IDLE → flush TX FIFO → write frame → STX command → wait → IDLE
 *
 * Returns CC1101_OK on success, CC1101_ERR_TIMEOUT if something stalls.
 */
CC1101_Status_t CC1101_SendPacket(CC1101_Packet_t *pkt);

/*
 * CC1101_ReceivePacket()
 * Puts the radio into RX mode and blocks until a valid packet arrives
 * or the timeout_ms expires.
 *
 * On success: populates *pkt with decoded frame data, writes RSSI dBm to *rssi_out.
 * On CRC failure: returns CC1101_ERR_CRC — the packet arrived but was corrupted.
 * On timeout: returns CC1101_ERR_TIMEOUT — nothing was received.
 *
 * RSSI conversion: raw byte → dBm using the CC1101 datasheet formula.
 * This is the actual received signal strength — key data for our link budget.
 */
CC1101_Status_t CC1101_ReceivePacket(CC1101_Packet_t *pkt, int8_t *rssi_out, uint32_t timeout_ms);

/*
 * Low-level register access — available for debugging but not normally
 * called from main.c directly.
 */
void    CC1101_WriteReg(uint8_t regAddr, uint8_t data);
uint8_t CC1101_ReadReg(uint8_t regAddr);    // For config registers 0x00–0x2E
uint8_t CC1101_ReadStatus(uint8_t regAddr); // For status registers 0x30–0x3D (uses READ_BURST flag)
void    CC1101_SendCmd(uint8_t cmd);

void CC1101_WritePATable(uint8_t value);

#endif /* INC_CC1101_H_ */
/* INC_CC1101_H_ */
