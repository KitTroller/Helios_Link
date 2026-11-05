# Helios Link

Helios Link is an educational project attempting to achieve a GFSK RF link over the 868 MHz ISM band. To achieve this, 2 STM32F446RE developer boards, alongside 2 CC1101 transceiver modules, were used. As an alternative for the receiver, and due to bad SPI connection soldering, a GNU Radio receiver was built instead, using the RTL-SDR Blog V4 as the receiver.

## Radio link

The actual link is over **867.905 MHz** due to a -95khz, **-108 ppm**, offset from programmed 868Mhz was measured on the transmitter with the RTL SDR V4. That offset translates directly onto the CC1101's synthesizer frequency used for modulating the signal. The CC1101 receiver's offset is assumed to be similar since the signal was received without any further frequency tuning, thus its offset being between -82ppm and -135ppm so the signal lands within the receivers filter. 

A bit rate of **1.2 kbps** was used to maximise receiver sensitivity. Modulation index was set to **h = 8.6**, sending packets every **1.2 s**.

## Why a software receiver

Another motive for the creation of the GNU Radio software receiver was the ability to use a smaller filter in the receiver, as our signal spans approximately **12 kHz** wide, whereas the minimum filter bandwidth on the CC1101 was **58 kHz**. Thus we theorize the software receiver will have even greater sensitivity.

## Packet format

The on-air packet format is the following:

| Field | Size |
|---|---|
| Preamble (`0xAA`) | 4 bytes |
| Sync word (`0xD391`, sent twice) | 4 bytes |
| Length | 1 byte |
| Packet ID | 1 byte |
| Command ID | 1 byte |
| Payload | 8 bytes |
| CRC | 2 bytes |
| **Total on air** | **21 bytes** |

## Testing so far

So far, tests were done on the bench, with the antennas a few decimeters away from one another and the antennas being simple Arduino jumper cables, with an estimated **PER of 0.38%**, in other words only 2 out of 522 packets were lost over a 10-minute window.
