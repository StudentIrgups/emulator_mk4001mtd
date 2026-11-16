import sigrokdecode as srd

CCCR_REGS = {
    0x00: 'CCCR_REV',
    0x01: 'SD_SPEC_REV',
    0x02: 'IO_ENABLE',
    0x03: 'IO_READY',
    0x04: 'INT_ENABLE',
    0x05: 'INT_PENDING',
    0x06: 'IO_ABORT',
    0x07: 'BUS_IF_CTRL',
    0x08: 'CARD_CAP',
    0x09: 'CIS_PTR_0',
    0x0A: 'CIS_PTR_1',
    0x0B: 'CIS_PTR_2',
    0x0C: 'BUS_SUSPEND',
    0x0D: 'FUNC_SELECT',
    0x0E: 'EXEC_FLAGS',
    0x0F: 'READY_FLAGS',
    0x10: 'FN0_BLK_SIZE_0',
    0x11: 'FN0_BLK_SIZE_1',
    0x12: 'POWER_CTRL',
    0x13: 'HIGH_SPEED',
}

FBR_REGS = {
    0x00: 'FBR_IF_CODE',
    0x01: 'FBR_EXT_IF_CODE',
    0x02: 'FBR_PWR_SEL',
    0x10: 'FBR_BLK_SIZE_0',
    0x11: 'FBR_BLK_SIZE_1',
}

ATA_REGS = {
    0x00: 'ATA_DATA',
    0x01: 'ATA_ERR/FEAT',
    0x02: 'ATA_SECCOUNT',
    0x03: 'ATA_LBA_LO',
    0x04: 'ATA_LBA_MID',
    0x05: 'ATA_LBA_HI',
    0x06: 'ATA_DEV/HEAD',
    0x07: 'ATA_CMD/STATUS',
}

ATA_CMDS = {
    0x00: 'NOP',
    0x20: 'READ_SECTORS',
    0x30: 'WRITE_SECTORS',
    0xC2: 'VENDOR_TOSHIBA_0xC2',
    0xC4: 'READ_MULTIPLE',
    0xC5: 'WRITE_MULTIPLE',
    0xC6: 'SET_MULTIPLE_MODE',
    0xC8: 'READ_DMA',
    0xCA: 'WRITE_DMA',
    0xE0: 'STANDBY_IMMEDIATE',
    0xE1: 'IDLE_IMMEDIATE',
    0xE7: 'FLUSH_CACHE',
    0xEC: 'IDENTIFY_DEVICE',
    0xEF: 'SET_FEATURES',
}

def crc7(bits):
    crc = 0
    for b in bits:
        c = (crc >> 6) & 1
        crc = ((crc << 1) | b) & 0x7F
        if c:
            crc ^= 0x09
    return crc

def reg_name(fn, addr):
    if fn == 0:
        if addr in CCCR_REGS:
            return CCCR_REGS[addr]
        fbr_fn = addr >> 8
        fbr_off = addr & 0xFF
        if 1 <= fbr_fn <= 7 and fbr_off in FBR_REGS:
            return f'FN{fbr_fn}_{FBR_REGS[fbr_off]}'
    elif fn == 1:
        if addr in ATA_REGS:
            return ATA_REGS[addr]
    return f'0x{addr:04X}'

class Decoder(srd.Decoder):
    api_version = 3
    id = 'sdio_ata'
    name = 'SDIO / ATA'
    longname = 'SDIO & ATA-over-SDIO (MK4001MTD)'
    desc = 'SDIO bus protocol with CCCR and ATA Task File decoding.'
    license = 'gplv2+'
    inputs = ['logic']
    outputs = []
    tags = ['Memory', 'PC']

    channels = (
        {'id': 'clk', 'name': 'CLK', 'desc': 'Clock'},
        {'id': 'cmd', 'name': 'CMD', 'desc': 'Command / Response'},
    )
    optional_channels = (
        {'id': 'dat0', 'name': 'DAT0', 'desc': 'Data 0'},
        {'id': 'dat1', 'name': 'DAT1', 'desc': 'Data 1 / IRQ'},
        {'id': 'dat2', 'name': 'DAT2', 'desc': 'Data 2'},
        {'id': 'dat3', 'name': 'DAT3', 'desc': 'Data 3 / CD'},
    )

    annotations = (
        ('cmd', 'Command'),
        ('resp', 'Response'),
        ('ata', 'ATA / CCCR Detail'),
        ('data', 'CMD53 Data'),
        ('err', 'Error / CRC'),
    )
    annotation_rows = (
        ('cmds', 'CMD / RESP', (0, 1, 4)),
        ('details', 'Register / ATA', (2,)),
        ('data_row', 'DAT0-3 Payload', (3,)),
    )

    def __init__(self):
        self.reset()

    def reset(self):
        self.cmd_bits = []
        self.cmd_samples = []
        self.expect_resp = False
        self.resp_len = 48
        self.last_cmd_idx = None
        self.last_cmd52 = None
        self.last_cmd53 = None
        self.bus_width = 1
        self.block_size = 512

        self.data_active = False
        self.data_bits = []
        self.data_start_sample = 0
        self.data_blocks_left = 0
        self.bytes_per_block = 512

    def start(self):
        self.out_ann = self.register(srd.OUTPUT_ANN)

    def put_ann(self, ss, es, ann_idx, text_list):
        self.put(ss, es, self.out_ann, [ann_idx, text_list])

    def decode_cmd_packet(self, bits, samples):
        ss = samples[0]
        es = samples[-1]

        dir_bit = bits[1]  # 1 = Host -> Card, 0 = Card -> Host
        cmd_idx = 0
        for b in bits[2:8]:
            cmd_idx = (cmd_idx << 1) | b

        arg = 0
        for b in bits[8:40]:
            arg = (arg << 1) | b

        rx_crc = 0
        for b in bits[40:47]:
            rx_crc = (rx_crc << 1) | b

        calc_crc = crc7(bits[0:40])
        crc_ok = (rx_crc == calc_crc) or (rx_crc == 0x7F)

        # HOST COMMAND
        if dir_bit == 1:
            self.last_cmd_idx = cmd_idx
            self.expect_resp = True
            self.resp_len = 136 if cmd_idx == 2 else 48

            if cmd_idx == 5:
                self.put_ann(ss, es, 0, [
                    f'CMD5 OCR=0x{arg:06X}',
                    f'CMD5 (0x{arg:06X})',
                    'CMD5',
                ])
                self.put_ann(ss, es, 2, [
                    f'IO_SEND_OP_COND: Voltage=0x{arg & 0xFFFFFF:06X}',
                    'IO_SEND_OP_COND',
                ])

            elif cmd_idx == 3:
                self.put_ann(ss, es, 0, ['CMD3 SEND_RELATIVE_ADDR', 'CMD3'])

            elif cmd_idx == 7:
                rca = (arg >> 16) & 0xFFFF
                self.put_ann(ss, es, 0, [
                    f'CMD7 SELECT RCA=0x{rca:04X}',
                    f'CMD7 (0x{rca:04X})',
                    'CMD7',
                ])

            elif cmd_idx == 52:
                rw = (arg >> 31) & 1
                fn = (arg >> 28) & 0x7
                addr = (arg >> 9) & 0x1FFFF
                wdata = arg & 0xFF
                self.last_cmd52 = (rw, fn, addr, wdata)

                rname = reg_name(fn, addr)
                if rw:
                    self.put_ann(ss, es, 0, [
                        f'CMD52 WR Fn{fn} {rname} <- 0x{wdata:02X}',
                        f'CMD52 WR {rname}=0x{wdata:02X}',
                        'CMD52 WR',
                    ])
                else:
                    self.put_ann(ss, es, 0, [
                        f'CMD52 RD Fn{fn} {rname}',
                        f'CMD52 RD {rname}',
                        'CMD52 RD',
                    ])

                if rw == 1 and fn == 0:
                    if addr == 0x0007:
                        self.bus_width = 4 if (wdata & 0x03) == 0x02 else 1
                    elif addr == 0x0110:
                        self.block_size = (self.block_size & 0xFF00) | wdata
                    elif addr == 0x0111:
                        self.block_size = (self.block_size & 0x00FF) | (wdata << 8)

                if rw == 1 and fn == 1 and addr == 0x07:
                    ata_name = ATA_CMDS.get(wdata, f'UNKNOWN_0x{wdata:02X}')
                    self.put_ann(ss, es, 2, [
                        f'ATA CMD: {ata_name} (0x{wdata:02X})',
                        ata_name,
                    ])
                elif rw == 1:
                    self.put_ann(ss, es, 2, [
                        f'{rname} <- 0x{wdata:02X}',
                        f'0x{wdata:02X}',
                    ])

            elif cmd_idx == 53:
                rw = (arg >> 31) & 1
                fn = (arg >> 28) & 0x7
                blk = (arg >> 27) & 1
                op = (arg >> 26) & 1
                addr = (arg >> 9) & 0x1FFFF
                cnt = arg & 0x1FF
                if cnt == 0 and blk == 0:
                    cnt = 512

                self.last_cmd53 = (rw, fn, blk, op, addr, cnt)
                rw_str = 'WR' if rw else 'RD'
                mode_str = f'{cnt} blks' if blk else f'{cnt} B'
                rname = reg_name(fn, addr)

                self.put_ann(ss, es, 0, [
                    f'CMD53 {rw_str} Fn{fn} {rname} ({mode_str})',
                    f'CMD53 {rw_str} {mode_str}',
                    'CMD53',
                ])
                self.put_ann(ss, es, 2, [
                    f'CMD53 {rw_str}: {"Block" if blk else "Byte"} mode, count={cnt}, bus={self.bus_width}b',
                    f'{rw_str} {mode_str}',
                ])

                self.data_blocks_left = cnt if blk else 1
                self.bytes_per_block = self.block_size if blk else cnt
                self.data_active = False
                self.data_bits = []

            else:
                self.put_ann(ss, es, 0, [
                    f'CMD{cmd_idx} arg=0x{arg:08X}',
                    f'CMD{cmd_idx}',
                ])

            if not crc_ok and cmd_idx != 5:
                self.put_ann(samples[40], es, 4, [
                    f'CRC7 ERR (got 0x{rx_crc:02X}, exp 0x{calc_crc:02X})',
                    'CRC ERR',
                ])

        # CARD RESPONSE
        else:
            self.expect_resp = False
            if self.last_cmd_idx == 5:
                ready = (arg >> 31) & 1
                nfn = (arg >> 28) & 0x7
                ocr = arg & 0xFFFFFF
                self.put_ann(ss, es, 1, [
                    f'R4 Ready={ready} Fn={nfn} OCR=0x{ocr:06X}',
                    f'R4 Rdy={ready}',
                    'R4',
                ])
            elif self.last_cmd_idx == 3:
                rca = (arg >> 16) & 0xFFFF
                self.put_ann(ss, es, 1, [f'R6 RCA=0x{rca:04X}', 'R6'])
            elif self.last_cmd_idx == 7:
                self.put_ann(ss, es, 1, [f'R1b Status=0x{arg:08X}', 'R1b'])
            elif self.last_cmd_idx == 52:
                flags = (arg >> 8) & 0xFF
                rdata = arg & 0xFF
                self.put_ann(ss, es, 1, [
                    f'R5 Data=0x{rdata:02X} Flags=0x{flags:02X}',
                    f'R5 0x{rdata:02X}',
                    'R5',
                ])
                if self.last_cmd52:
                    rw, fn, addr, _ = self.last_cmd52
                    rname = reg_name(fn, addr)
                    if rw == 0:
                        self.put_ann(ss, es, 2, [
                            f'{rname} -> 0x{rdata:02X}',
                            f'0x{rdata:02X}',
                        ])
            elif self.last_cmd_idx == 53:
                flags = (arg >> 8) & 0xFF
                self.put_ann(ss, es, 1, [f'R5 Flags=0x{flags:02X}', 'R5'])
            else:
                self.put_ann(ss, es, 1, [
                    f'RESP CMD{cmd_idx} 0x{arg:08X}',
                    'RESP',
                ])

    def decode(self):
        has_dat0 = self.has_channel(2)
        has_dat1 = self.has_channel(3)
        has_dat2 = self.has_channel(4)
        has_dat3 = self.has_channel(5)
        while True:
            pins = self.wait({0: 'r'})
            clk = pins[0]
            cmd = pins[1]
            dat0 = pins[2] if has_dat0 else 1
            dat1 = pins[3] if has_dat1 else 1
            dat2 = pins[4] if has_dat2 else 1
            dat3 = pins[5] if has_dat3 else 1
            sample = self.samplenum
            
            # 1. CMD LINE STATE MACHINE
            if not self.cmd_bits:
                if cmd == 0:
                    self.cmd_bits.append(0)
                    self.cmd_samples.append(sample)
            else:
                self.cmd_bits.append(cmd)
                self.cmd_samples.append(sample)
                target_len = self.resp_len if self.expect_resp else 48
                if len(self.cmd_bits) == target_len:
                    self.decode_cmd_packet(self.cmd_bits, self.cmd_samples)
                    self.cmd_bits = []
                    self.cmd_samples = []
            
            # 2. DAT0-DAT3 LINE STATE MACHINE (for CMD53)
            if self.data_blocks_left > 0 and has_dat0:
                if not self.data_active:
                    if dat0 == 0:
                        self.data_active = True
                        self.data_start_sample = sample
                        self.data_bits = []
                else:
                    if self.bus_width == 4 and has_dat1 and has_dat2 and has_dat3:
                        nibble = (dat3 << 3) | (dat2 << 2) | (dat1 << 1) | dat0
                        self.data_bits.append(nibble)
                        needed_clocks = (self.bytes_per_block * 2) + 16 + 1
                        if len(self.data_bits) >= needed_clocks:
                            raw_bytes = bytearray()
                            for i in range(0, self.bytes_per_block * 2, 2):
                                b = (self.data_bits[i] << 4) | self.data_bits[i + 1]
                                raw_bytes.append(b)
                            preview = ' '.join(f'{x:02X}' for x in raw_bytes[:16])
                            if len(raw_bytes) > 16:
                                preview += ' ...'
                            self.put_ann(self.data_start_sample, sample, 3, [
                                f'Block ({len(raw_bytes)}B): {preview}',
                                f'{len(raw_bytes)}B Data',
                                'DATA',
                            ])
                            self.data_active = False
                            self.data_blocks_left -= 1
                    else:
                        self.data_bits.append(dat0)
                        needed_clocks = (self.bytes_per_block * 8) + 16 + 1
                        if len(self.data_bits) >= needed_clocks:
                            raw_bytes = bytearray()
                            for i in range(0, self.bytes_per_block * 8, 8):
                                b = 0
                                for bit_i in range(8):
                                    b = (b << 1) | self.data_bits[i + bit_i]
                                raw_bytes.append(b)
                            preview = ' '.join(f'{x:02X}' for x in raw_bytes[:16])
                            self.put_ann(self.data_start_sample, sample, 3, [
                                f'Block ({len(raw_bytes)}B): {preview}',
                                'DATA',
                            ])
                            self.data_active = False
                            self.data_blocks_left -= 1