"""
Embedded Python Blocks:

Each time this file is saved, GRC will instantiate the first class it finds
to get ports and parameters of your block. The arguments to __init__  will
be the parameters. All of them are required to have default values!
"""
import numpy as np
import pmt
from gnuradio import gr
import struct

def pn9(n):
    state = 0b111111111
    out = []
    for _ in range(n):
        out.append(state & 0xFF)
        for _ in range(8):
            feed_back_bit = ((state >> 5) ^ (state)) & 1
            state = (state >> 1) | (feed_back_bit << 8)
    return out


def crc_16(data):
    crc = 0xFFFF
    crc_polynomial = 0x8005
    for byte in data:
        for _ in range(8):
            if(((crc & 0x8000) >> 8)  ^ (byte & 0x80)):
                crc = (crc << 1) ^ crc_polynomial
            else:
                crc = crc << 1
            
            crc = crc & 0xFFFF
            byte = (byte << 1) & 0xFF
    return crc



class blk(gr.sync_block):
    def __init__(self, tag_name="sync"):           # shows up as a parameter in GRC
        gr.sync_block.__init__(self, name="HeliosLink deframer", in_sig=[np.uint8], out_sig=None)
        self.tag_name = tag_name
        self.white = pn9(64)                # PN9 table, made once
        self.collecting = False
        self.bits = []                   # bits of the packet being collected
        self.need = 0                    # total bits this packet has
        self.last_id = None
        self.good = 0
        self.lost = 0

    def work(self, input_items, output_items):
        bits = input_items[0]
        start = self.nitems_read(0)
        tag_pos = set()
        for tag in self.get_tags_in_window(0, 0, len(bits)):
            if pmt.symbol_to_string(tag.key) == self.tag_name:
                tag_pos.add(tag.offset - start)

        for i in range(len(bits)):
            if not self.collecting:
                if i not in tag_pos:
                    continue  # keep waiting (IDLE)
                self.collecting = True
                self.bits = []
                self.need = 0
            self.bits.append(int(bits[i]))
            if len(self.bits) == 8:
                L = int(np.packbits(self.bits)[0]) ^ self.white[0]  # packs into bytes
                if ((L < 2) or (L > 61)):  # impossible length it was a false sync buffer is 64 bytes in cc1101 61 + 1 itself + RSSI and LQI/CRC_OK = 64
                    self.collecting = False
                    self.bits = []
                    self.need = 0
                    continue
                self.need = (1 + L + 2) * 8  # length byte + L + 2 CRC, in bits
            if len(self.bits) == self.need:
                self.handle_packet()
                self.collecting = False
                self.bits = []
                self.need = 0
        return len(bits)

    def handle_packet(self):
        raw = np.packbits(self.bits).tolist()
        white = self.white
        dewhitened_bits = [r ^ w for r, w in zip(raw, white)]
        Length = dewhitened_bits[0]
        crc_rx = dewhitened_bits[Length + 1] << 8 | dewhitened_bits[Length + 2]  # the 2 received CRC bytes as one 16-bit number, length does not count itself so indexing is correct
        if (crc_16(dewhitened_bits[:Length + 1]) ^ crc_rx):
            print("CRC FAIL")
            return
        self.good += 1
        pid, cmd = dewhitened_bits[1], dewhitened_bits[2]
        if self.last_id is not None:
            self.lost += (pid - self.last_id - 1) & 0xFF
        self.last_id = pid
        payload = dewhitened_bits[3:Length + 1] # voltage_mv, temp_c10, uptime_s from d[3 : L+1]
        v, t, up = struct.unpack("<HhI", bytes(payload))
        print(f"id={pid} cmd={cmd} V={v} mV T={t/10:.1f} C up={up} s lost={self.lost}", flush=True)