/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

/* USER CODE BEGIN Header */
/**
 * main.c — HeliosLink Phase 2: Verified CC1101 Packet Link
 *
 * =========================================================================
 * HOW TO FLASH TWO BOARDS FROM ONE CODEBASE
 * =========================================================================
 * The driver (cc1101.c/h) is IDENTICAL on both boards.
 * Only main.c behaviour differs, controlled by the #define below.
 *
 *   To flash the TRANSMITTER (Satellite node):
 *     → Make sure  #define NODE_TRANSMITTER  is uncommented
 *     → Comment out #define NODE_RECEIVER
 *     → Build and flash board 1
 *
 *   To flash the RECEIVER (Ground station node):
 *     → Comment out #define NODE_TRANSMITTER
 *     → Make sure  #define NODE_RECEIVER  is uncommented
 *     → Build and flash board 2
 *
 * =========================================================================
 * WIRING — CC1101 MODULE TO NUCLEO F446RE (confirmed from your .ioc file)
 * =========================================================================
 * CC1101 Pin  | Nucleo Pin | Signal
 * ------------|------------|-----------------------------------------------
 * VCC         | 3.3V       | Power — MUST be 3.3V. 5V will destroy the chip.
 * GND         | GND        | Ground — connect both boards' GNDs together too
 * SCK         | PB10       | SPI2 clock
 * MOSI (SI)   | PC1        | SPI2 data out from STM32
 * MISO (SO)   | PC2        | SPI2 data in to STM32 (MISO)
 * CSn         | PB12       | Chip select — we control this manually via GPIO
 * GDO0        | (not connected for now — used in future interrupt-driven mode)
 * GDO2        | (not connected for now)
 *
 * Use female-to-male jumper cables from your Arduino kit.
 * The CC1101 module already has header pins soldered — no soldering needed.
 *
 * =========================================================================
 * WHAT THIS CODE DOES
 * =========================================================================
 * TRANSMITTER: Sends a packet every 1 second containing a mock telemetry
 *   payload (temperature, voltage, sequence number). Blinks the green LED
 *   fast on success, slow on failure. Prints status over UART at 115200.
 *
 * RECEIVER: Listens for packets with a 2-second window. On receipt, decodes
 *   the packet, prints the data + RSSI over UART, tracks Packet Error Rate.
 *   Blinks the green LED once per received packet.
 *
 * UART output appears in any serial terminal on your Mac:
 *   → screen /dev/tty.usbmodem* 115200
 *   → or use CubeIDE's built-in Serial Monitor
 *
 * =========================================================================
 * EU LEGAL NOTE — DUTY CYCLE
 * =========================================================================
 * 868.0–868.6 MHz: max 25 mW ERP, duty cycle ≤ 1%
 * At 1.2 kbps, a 10-byte payload takes ~17 ms to transmit.
 * 1% duty cycle at 1s period = 10 ms max TX time.
 * Our 1-second beacon interval gives ~1.7% — fine for testing in a lab.
 * For outdoor/range tests, increase interval to 2 seconds or more.
 */
/* USER CODE END Header */

#include "main.h"
#include "cc1101.h"
#include <stdio.h>
#include <string.h>

/* =========================================================================
 * ROLE SELECTION — change this line before flashing each board
 * ========================================================================= */
//#define NODE_TRANSMITTER
#define NODE_RECEIVER

/* =========================================================================
 * DIAGNOSTIC MODE — comment out to return to normal packet TX/RX.
 * When enabled, the transmitter runs back-to-back TX with instrumentation
 * so we can confirm the chip actually reaches TX state and produces RF.
 * ========================================================================= */
//#define DIAGNOSTIC_MODE

extern volatile uint8_t g_cc1101_last_peak_marcstate;

/* RX diagnostic globals (defined in cc1101.c) — printed on CRC error to
 * show what the receiver actually heard. */
extern volatile uint8_t g_cc1101_rx_raw[16];
extern volatile uint8_t g_cc1101_rx_raw_len;
extern volatile uint8_t g_cc1101_rx_lqi;

/* =========================================================================
 * Telemetry payload structure
 * Both boards share this definition so they agree on byte layout.
 * __attribute__((packed)) prevents the compiler from inserting padding bytes
 * between fields — critical when sending structs over a radio link.
 * ========================================================================= */
typedef struct __attribute__((packed)) {
    uint16_t voltage_mv;   // Battery voltage in millivolts (e.g. 3700 = 3.700V)
    int16_t  temp_c10;     // Temperature in 0.1°C units (e.g. 235 = 23.5°C)
    uint32_t uptime_s;     // Seconds since boot
} Telemetry_t;

/* =========================================================================
 * Private variables
 * ========================================================================= */
SPI_HandleTypeDef hspi2;
UART_HandleTypeDef huart2;

/* =========================================================================
 * Private function prototypes
 * ========================================================================= */
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI2_Init(void);
static void LED_Blink(uint8_t count, uint32_t on_ms, uint32_t off_ms);

/* =========================================================================
 * main()
 * ========================================================================= */
int main(void) {

    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_USART2_UART_Init();
    MX_SPI2_Init();

    printf("\r\n========================================\r\n");
    printf("  HeliosLink — CC1101 Packet Link Test\r\n");

#ifdef NODE_TRANSMITTER
    printf("  Role: TRANSMITTER (Satellite Node)\r\n");
#else
    printf("  Role: RECEIVER (Ground Station Node)\r\n");
#endif

    printf("========================================\r\n\r\n");

    /* Initialise the CC1101 radio */
    CC1101_Init();

    /* Verify the chip is responding correctly over SPI.
     * If this fails, nothing else will work. Fix wiring before proceeding. */
    CC1101_Status_t verify_result = CC1101_Verify();
    // Read back PATABLE to confirm it wrote correctly
    uint8_t pa_check = CC1101_ReadReg(CC1101_PATABLE);
    printf("PATABLE[0]: 0x%02X (expected 0xC0)\r\n", pa_check);
    if (verify_result != CC1101_OK) {
        printf("ERROR: CC1101 not detected! VERSION register returned unexpected value.\r\n");
        printf("Check: 3.3V power, SPI wiring (SCK=PB10, MOSI=PC1, MISO=PC2, CS=PB12)\r\n");
        /* Blink rapidly forever to signal hardware fault */
        while (1) {
            LED_Blink(1, 100, 100);
        }
    }

    printf("CC1101 detected OK. Hardware version: 0x14\r\n\r\n");
    LED_Blink(3, 200, 200);  // 3 quick blinks = radio is alive

#ifdef DIAGNOSTIC_MODE
    /* =====================================================================
     * DIAGNOSTIC: register dump + back-to-back TX with MARCSTATE tracking
     * ===================================================================== */
    printf("=== DIAGNOSTIC MODE ===\r\n");
    printf("Register readback (after Init):\r\n");
    printf("  FREQ2   = 0x%02X  (expect 0x21)\r\n", CC1101_ReadReg(CC1101_FREQ2));
    printf("  FREQ1   = 0x%02X  (expect 0x62)\r\n", CC1101_ReadReg(CC1101_FREQ1));
    printf("  FREQ0   = 0x%02X  (expect 0x76)\r\n", CC1101_ReadReg(CC1101_FREQ0));
    printf("  MDMCFG4 = 0x%02X  (expect 0xF5)\r\n", CC1101_ReadReg(CC1101_MDMCFG4));
    printf("  MDMCFG2 = 0x%02X  (expect 0x13)\r\n", CC1101_ReadReg(CC1101_MDMCFG2));
    printf("  MCSM0   = 0x%02X  (expect 0x18)\r\n", CC1101_ReadReg(CC1101_MCSM0));
    printf("  FREND0  = 0x%02X  (expect 0x10)\r\n", CC1101_ReadReg(CC1101_FREND0));
    printf("  PATABLE = 0x%02X  (expect 0xC0)\r\n", CC1101_ReadReg(CC1101_PATABLE));
    printf("  FSCAL3  = 0x%02X  (pre-cal)\r\n",    CC1101_ReadReg(CC1101_FSCAL3));
    printf("  FSCAL1  = 0x%02X  (pre-cal, should change after 1st STX)\r\n",
           CC1101_ReadReg(CC1101_FSCAL1));
    printf("  VERSION = 0x%02X  (expect 0x14)\r\n", CC1101_ReadStatus(CC1101_VERSION));

    printf("\r\nTuning target: 868.35 MHz. Set SDR++ to 2 MHz span, bandwidth wide open.\r\n");
    printf("Running max-length TX every 2 seconds. Watch the SDR waterfall after each marker.\r\n");
    printf("Peak MARCSTATE should be 0x13 (TX). FSCAL1 should be nonzero and STABLE across runs.\r\n\r\n");

    CC1101_Packet_t diag_pkt;
    diag_pkt.packet_id   = 0;
    diag_pkt.command     = CMD_TELEMETRY;
    diag_pkt.payload_len = CC1101_MAX_PAYLOAD;     // 58 bytes — max allowed
    memset(diag_pkt.payload, 0x55, CC1101_MAX_PAYLOAD);  // 01010101 — distinct FSK tone pair

    uint32_t diag_iter  = 0;
    uint32_t ok_count   = 0;
    uint32_t fail_count = 0;

    while (1) {
        /* Loud marker right before TX so you can align with SDR waterfall */
        printf("\r\n--- [iter %lu] TX NOW (watch SDR at 868.35 MHz) ---\r\n", diag_iter);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);  // LED ON during TX

        CC1101_Status_t r = CC1101_SendPacket(&diag_pkt);

        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
        uint8_t peak   = g_cc1101_last_peak_marcstate;
        uint8_t fscal1 = CC1101_ReadReg(CC1101_FSCAL1);
        uint8_t fscal2 = CC1101_ReadReg(CC1101_FSCAL2);
        uint8_t fscal3 = CC1101_ReadReg(CC1101_FSCAL3);
        uint8_t marc_now = CC1101_ReadStatus(CC1101_MARCSTATE) & 0x1F;

        const char *rs = (r == CC1101_OK)          ? "OK"
                       : (r == CC1101_ERR_TIMEOUT) ? "TMOUT"
                       : (r == CC1101_ERR_OVERFLOW)? "UNDERFLOW"
                       :                             "ERR";
        if (r == CC1101_OK) ok_count++; else fail_count++;

        printf("  result=%-9s peak_marc=0x%02X  post-TX_marc=0x%02X\r\n",
               rs, peak, marc_now);
        printf("  FSCAL3=0x%02X  FSCAL2=0x%02X  FSCAL1=0x%02X   (stable nonzero = good)\r\n",
               fscal3, fscal2, fscal1);
        printf("  totals: OK=%lu  FAIL=%lu\r\n", ok_count, fail_count);

        diag_iter++;
        diag_pkt.packet_id++;
        HAL_Delay(2000);   // 2 s between attempts — plenty of time to inspect SDR waterfall
    }
#endif /* DIAGNOSTIC_MODE */

    /* =====================================================================
     * TRANSMITTER MAIN LOOP
     * ===================================================================== */
#ifdef NODE_TRANSMITTER

    CC1101_Packet_t tx_pkt;
    Telemetry_t telemetry;
    uint8_t sequence_number = 0;
    uint32_t boot_time = HAL_GetTick();

    printf("Starting transmit loop. Sending 1 packet/second.\r\n");
    printf("SEQ | Status    | Uptime\r\n");
    printf("----|-----------|--------\r\n");

    while (1) {

        /* Build mock telemetry payload
         * In Phase 4 we'll replace this with real sensor readings + SGP4 position.
         * For now, simulated values let us validate the link without extra hardware. */
        telemetry.voltage_mv = 3700 + (HAL_GetTick() % 200);  // Simulate 3.70–3.90V
        telemetry.temp_c10   = 235 + (sequence_number % 20);  // Simulate 23.5–25.5°C
        telemetry.uptime_s   = (HAL_GetTick() - boot_time) / 1000;

        /* Fill the packet */
        tx_pkt.packet_id   = sequence_number;
        tx_pkt.command     = CMD_TELEMETRY;
        tx_pkt.payload_len = sizeof(Telemetry_t);              // 8 bytes
        memcpy(tx_pkt.payload, &telemetry, sizeof(Telemetry_t));

        /* Transmit */
        CC1101_Status_t result = CC1101_SendPacket(&tx_pkt);

        if (result == CC1101_OK) {
            printf("%3d | OK        | %lu s\r\n", sequence_number, telemetry.uptime_s);
            LED_Blink(1, 50, 0);   // Quick flash = TX success
        } else {
            printf("%3d | TX FAILED | %lu s\r\n", sequence_number, telemetry.uptime_s);
            LED_Blink(3, 500, 200); // Slow triple-blink = TX failure
        }

        sequence_number++;         // Rolls over at 255 → 0, that's fine
        HAL_Delay(1000);           // 1 second between packets (~1.7% duty cycle)
    }

#endif /* NODE_TRANSMITTER */

    /* =====================================================================
     * RECEIVER MAIN LOOP
     * ===================================================================== */
#ifdef NODE_RECEIVER

    CC1101_Packet_t rx_pkt;
    int8_t rssi_dbm;
    uint32_t total_packets    = 0;  // Packets we successfully received
    uint32_t crc_errors       = 0;  // Packets received with bad CRC
    uint32_t timeouts         = 0;  // Listen windows with no packet
    uint8_t last_seq          = 0;
    uint32_t missed_packets   = 0;

    printf("Starting receive loop. Listening with 2s window each cycle.\r\n");
    printf("SEQ | RSSI      | Voltage | Temp  | Uptime | PER\r\n");
    printf("----|-----------|---------|-------|--------|---------\r\n");

    while (1) {

        CC1101_Status_t result = CC1101_ReceivePacket(&rx_pkt, &rssi_dbm, 2000);

        if (result == CC1101_OK) {

            /* Count missed sequence numbers (dropped packets in the air)
             * This is our Packet Error Rate measurement — the core thesis metric. */
            uint8_t expected_seq = last_seq + 1;
            if (rx_pkt.packet_id != expected_seq && total_packets > 0) {
                /* Gap in sequence — calculate how many were missed */
                uint8_t gap = (uint8_t)(rx_pkt.packet_id - expected_seq);
                missed_packets += gap;
            }
            last_seq = rx_pkt.packet_id;
            total_packets++;

            /* Decode telemetry payload */
            Telemetry_t received_data;
            if (rx_pkt.payload_len == sizeof(Telemetry_t)) {
                memcpy(&received_data, rx_pkt.payload, sizeof(Telemetry_t));
            } else {
                memset(&received_data, 0, sizeof(Telemetry_t));
            }

            /* Calculate PER = missed / (received + missed) as a percentage */
            uint32_t total_sent_estimate = total_packets + missed_packets;
            uint32_t per_tenths = (total_sent_estimate > 0)
                                  ? (missed_packets * 1000 / total_sent_estimate)
                                  : 0;

            printf("%3d | %4d dBm  | %4d mV | %3d.%1d°C | %5lu s | %lu.%lu%%\r\n",
                rx_pkt.packet_id,
                rssi_dbm,
                received_data.voltage_mv,
                received_data.temp_c10 / 10,
                received_data.temp_c10 % 10,
                received_data.uptime_s,
                per_tenths / 10,
                per_tenths % 10
            );

            LED_Blink(1, 50, 0);   // Quick flash = packet received

        } else if (result == CC1101_ERR_CRC) {
            /* Packet arrived but failed CRC — dump the raw frame so we can
             * tell what kind of corruption we have:
             *   — RSSI very strong (-30 dBm) + LQI bad → AGC overload, add
             *     attenuation or move antennas apart.
             *   — RSSI moderate + LQI 20-40 + only 1–2 bytes look wrong →
             *     frequency offset between the two xtals, widen RX channel BW.
             *   — Bytes look like pure noise → wrong sync word / modulation
             *     mismatch or receiver tuned far off.
             *   — length byte absurd (>61) → length byte got corrupted,
             *     which alone is enough to fail CRC on any frame. */
            crc_errors++;
            printf("--- | CRC ERR | RSSI %4d dBm | LQI %3d | len=%3u | raw:",
                   rssi_dbm, g_cc1101_rx_lqi, g_cc1101_rx_raw_len);
            uint8_t n = (g_cc1101_rx_raw_len < 8) ? g_cc1101_rx_raw_len : 8;
            for (uint8_t i = 0; i < n; i++) {
                printf(" %02X", g_cc1101_rx_raw[i]);
            }
            printf("  Errs=%lu\r\n", crc_errors);

        } else if (result == CC1101_ERR_TIMEOUT) {
            /* No packet during the 2-second window */
            timeouts++;
            /* Only print every 5 timeouts to avoid flooding the terminal */
            if (timeouts % 5 == 0) {
                printf("--- | TIMEOUT   | (no packet) Timeouts: %lu\r\n", timeouts);
            }
        }
    }

#endif /* NODE_RECEIVER */

    /* Should never reach here */
    while (1) {}
}

/* =========================================================================
 * LED blink helper
 * Uses the green LD2 LED on the Nucleo board (PA5).
 * count  = number of blinks
 * on_ms  = LED on duration in milliseconds
 * off_ms = LED off duration in milliseconds (0 = no trailing off delay)
 * ========================================================================= */
static void LED_Blink(uint8_t count, uint32_t on_ms, uint32_t off_ms) {
    for (uint8_t i = 0; i < count; i++) {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
        HAL_Delay(on_ms);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
        if (off_ms > 0) HAL_Delay(off_ms);
    }
}

/* =========================================================================
 * printf redirect — sends printf output over UART2 to your Mac
 * ========================================================================= */
int _write(int file, char *ptr, int len) {
    HAL_UART_Transmit(&huart2, (uint8_t*)ptr, len, HAL_MAX_DELAY);
    return len;
}

/* =========================================================================
 * CubeMX-generated peripheral init — do not modify
 * ========================================================================= */
void SystemClock_Config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
    RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState            = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource       = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM            = 16;
    RCC_OscInitStruct.PLL.PLLN            = 336;
    RCC_OscInitStruct.PLL.PLLP            = RCC_PLLP_DIV4;
    RCC_OscInitStruct.PLL.PLLQ            = 2;
    RCC_OscInitStruct.PLL.PLLR            = 2;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) { Error_Handler(); }
    RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                       RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) { Error_Handler(); }
}

static void MX_SPI2_Init(void) {
    hspi2.Instance               = SPI2;
    hspi2.Init.Mode              = SPI_MODE_MASTER;
    hspi2.Init.Direction         = SPI_DIRECTION_2LINES;
    hspi2.Init.DataSize          = SPI_DATASIZE_8BIT;
    hspi2.Init.CLKPolarity       = SPI_POLARITY_LOW;   // CPOL=0: clock idles LOW
    hspi2.Init.CLKPhase          = SPI_PHASE_1EDGE;    // CPHA=0: sample on rising edge
    hspi2.Init.NSS               = SPI_NSS_SOFT;       // We control CS manually via GPIO
    hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;  // ~2.6 MHz (well under CC1101's 10 MHz max)
    hspi2.Init.FirstBit          = SPI_FIRSTBIT_MSB;
    hspi2.Init.TIMode            = SPI_TIMODE_DISABLE;
    hspi2.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
    hspi2.Init.CRCPolynomial     = 10;
    if (HAL_SPI_Init(&hspi2) != HAL_OK) { Error_Handler(); }
}

static void MX_USART2_UART_Init(void) {
    huart2.Instance          = USART2;
    huart2.Init.BaudRate     = 115200;
    huart2.Init.WordLength   = UART_WORDLENGTH_8B;
    huart2.Init.StopBits     = UART_STOPBITS_1;
    huart2.Init.Parity       = UART_PARITY_NONE;
    huart2.Init.Mode         = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart2) != HAL_OK) { Error_Handler(); }
}

static void MX_GPIO_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /* CS pin: PB12 — output, starts HIGH (chip deselected) */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
    GPIO_InitStruct.Pin   = GPIO_PIN_12;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* Green LED: PA5 */
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
    GPIO_InitStruct.Pin   = GPIO_PIN_5;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* Blue button: PC13 */
    GPIO_InitStruct.Pin  = GPIO_PIN_13;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
}

void Error_Handler(void) {
    __disable_irq();
    while (1) {}
}
