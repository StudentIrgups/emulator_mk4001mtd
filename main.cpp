#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "sdio_cmd.pio.h"
#include "sdio_dat.pio.h"
#include "hardware/clocks.h"

// Назначение пинов (строго подряд: CMD = GP2, CLK = GP3)
#define PIN_CMD 2
#define PIN_CLK 3
// Пины данных SDIO (GP4..GP7)
#define PIN_DAT0 4
#define PIN_DAT1 5  // Линия аппаратного прерывания IRQ (активный 0, в покое должна быть 1!)
#define PIN_DAT2 6
#define PIN_DAT3 7

void init_sdio_dat_pins() {
    return;
    gpio_init(PIN_DAT1);
    gpio_set_dir(PIN_DAT1, GPIO_IN);
    gpio_pull_up(PIN_DAT1);

    return;
    for (uint pin = PIN_DAT0; pin <= PIN_DAT3; pin++) {
        gpio_init(pin);
        gpio_set_dir(pin, GPIO_IN);
        gpio_pull_up(pin); // Подтягиваем все DAT0-DAT3 к 3.3V (в том числе DAT1/IRQ!)
    }
}

// Состояния SDIO
enum SdioState {
    STATE_IDLE,
    STATE_READY,
    STATE_IDENT,
    STATE_STBY,
    STATE_CMD,
    STATE_TRN
};

// Регистры CCCR (Function 0)
uint8_t cccr[0x200] = {0};

// Регистры ATA Task File (Function 1)
struct AtaRegisters {
    uint8_t error = 0x00;
    uint8_t features = 0x00;
    uint8_t seccount = 0x00;
    uint8_t lba_lo = 0x00;
    uint8_t lba_mid = 0x00;
    uint8_t lba_hi = 0x00;
    uint8_t dev_head = 0xE0;
    uint8_t status = 0x50;   // DRDY | DSC
    uint8_t command = 0x00;
} ata;

SdioState current_state = STATE_IDLE;
uint8_t cmd5_poll_count = 0;
uint8_t sector_buffer[512 * 16]; // Буфер на 16 секторов (8 КБ)
uint16_t pending_cmd53_blocks = 0;

// 512-байтный паспорт диска Toshiba MK4001MTD (4 ГБ)
uint8_t identify_data[512] = {0};

PIO pio = pio0;
uint sm_dat =1;
uint offset_dat = 0;

// Byte-wise CRC16-CCITT lookup table (polynomial 0x1021)
static const uint16_t crc16_lut[256] = {
    0x0000,0x1021,0x2042,0x3063,0x4084,0x50A5,0x60C6,0x70E7,
    0x8108,0x9129,0xA14A,0xB16B,0xC18C,0xD1AD,0xE1CE,0xF1EF,
    0x1231,0x0210,0x3273,0x2252,0x52B5,0x4294,0x72F7,0x62D6,
    0x9339,0x8318,0xB37B,0xA35A,0xD3BD,0xC39C,0xF3FF,0xE3DE,
    0x2462,0x3443,0x0420,0x1401,0x64E6,0x74C7,0x44A4,0x5485,
    0xA56A,0xB54B,0x8528,0x9509,0xE5EE,0xF5CF,0xC5AC,0xD58D,
    0x3653,0x2672,0x1611,0x0630,0x76D7,0x66F6,0x5695,0x46B4,
    0xB75B,0xA77A,0x9719,0x8738,0xF7DF,0xE7FE,0xD79D,0xC7BC,
    0x48C4,0x58E5,0x6886,0x78A7,0x0840,0x1861,0x2802,0x3823,
    0xC9CC,0xD9ED,0xE98E,0xF9AF,0x8948,0x9969,0xA90A,0xB92B,
    0x5AF5,0x4AD4,0x7AB7,0x6A96,0x1A71,0x0A50,0x3A33,0x2A12,
    0xDBFD,0xCBDC,0xFBBF,0xEB9E,0x9B79,0x8B58,0xBB3B,0xAB1A,
    0x6CA6,0x7C87,0x4CE4,0x5CC5,0x2C22,0x3C03,0x0C60,0x1C41,
    0xEDAE,0xFD8F,0xCDEC,0xDDCD,0xAD2A,0xBD0B,0x8D68,0x9D49,
    0x7E97,0x6EB6,0x5ED5,0x4EF4,0x3E13,0x2E32,0x1E51,0x0E70,
    0xFF9F,0xEFBE,0xDFDD,0xCFFC,0xBF1B,0xAF3A,0x9F59,0x8F78,
    0x9188,0x81A9,0xB1CA,0xA1EB,0xD10C,0xC12D,0xF14E,0xE16F,
    0x1080,0x00A1,0x30C2,0x20E3,0x5004,0x4025,0x7046,0x6067,
    0x83B9,0x9398,0xA3FB,0xB3DA,0xC33D,0xD31C,0xE37F,0xF35E,
    0x02B1,0x1290,0x22F3,0x32D2,0x4235,0x5214,0x6277,0x7256,
    0xB5EA,0xA5CB,0x95A8,0x8589,0xF56E,0xE54F,0xD52C,0xC50D,
    0x34E2,0x24C3,0x14A0,0x0481,0x7466,0x6447,0x5424,0x4405,
    0xA7DB,0xB7FA,0x8799,0x97B8,0xE75F,0xF77E,0xC71D,0xD73C,
    0x26D3,0x36F2,0x0691,0x16B0,0x6657,0x7676,0x4615,0x5634,
    0xD94C,0xC96D,0xF90E,0xE92F,0x99C8,0x89E9,0xB98A,0xA9AB,
    0x5844,0x4865,0x7806,0x6827,0x18C0,0x08E1,0x3882,0x28A3,
    0xCB7D,0xDB5C,0xEB3F,0xFB1E,0x8BF9,0x9BD8,0xABBB,0xBB9A,
    0x4A75,0x5A54,0x6A37,0x7A16,0x0AF1,0x1AD0,0x2AB3,0x3A92,
    0xFD2E,0xED0F,0xDD6C,0xCD4D,0xBDAA,0xAD8B,0x9DE8,0x8DC9,
    0x7C26,0x6C07,0x5C64,0x4C45,0x3CA2,0x2C83,0x1CE0,0x0CC1,
    0xEF1F,0xFF3E,0xCF5D,0xDF7C,0xAF9B,0xBFBA,0x8FD9,0x9FF8,
    0x6E17,0x7E36,0x4E55,0x5E74,0x2E93,0x3EB2,0x0ED1,0x1EF0,
};

static inline uint16_t crc16_byte(uint16_t crc, uint8_t byte) {
    return (crc << 8) ^ crc16_lut[((crc >> 8) ^ byte) & 0xFF];
}

// Compute CRC16 for all 4 DAT lines using byte-wise LUT
// Extracts bits for each line, packs into bytes, then LUT-driven CRC
void calc_sdio_4bit_crc16(const uint8_t* data, uint8_t* crc_out_8bytes) {
    // Each DAT line gets 2 bits per data byte (1 from hi nibble, 1 from lo nibble)
    // 512 data bytes → 1024 bits per line → 128 packed bytes per line
    // Process 4 data bytes at a time → produces 1 packed byte per line (8 bits)
    uint16_t c0 = 0, c1 = 0, c2 = 0, c3 = 0;

    for (int i = 0; i < 512; i += 4) {
        // Extract 8 bits for each DAT line from 4 consecutive data bytes
        // Each data byte contributes 2 bits: hi_nibble[line] then lo_nibble[line]
        uint8_t b0 = data[i], b1 = data[i+1], b2 = data[i+2], b3 = data[i+3];

        uint8_t p0 = ((b0 >> 4) & 1) << 7 | ((b0 >> 0) & 1) << 6 |
                     ((b1 >> 4) & 1) << 5 | ((b1 >> 0) & 1) << 4 |
                     ((b2 >> 4) & 1) << 3 | ((b2 >> 0) & 1) << 2 |
                     ((b3 >> 4) & 1) << 1 | ((b3 >> 0) & 1);
        uint8_t p1 = ((b0 >> 5) & 1) << 7 | ((b0 >> 1) & 1) << 6 |
                     ((b1 >> 5) & 1) << 5 | ((b1 >> 1) & 1) << 4 |
                     ((b2 >> 5) & 1) << 3 | ((b2 >> 1) & 1) << 2 |
                     ((b3 >> 5) & 1) << 1 | ((b3 >> 1) & 1);
        uint8_t p2 = ((b0 >> 6) & 1) << 7 | ((b0 >> 2) & 1) << 6 |
                     ((b1 >> 6) & 1) << 5 | ((b1 >> 2) & 1) << 4 |
                     ((b2 >> 6) & 1) << 3 | ((b2 >> 2) & 1) << 2 |
                     ((b3 >> 6) & 1) << 1 | ((b3 >> 2) & 1);
        uint8_t p3 = ((b0 >> 7) & 1) << 7 | ((b0 >> 3) & 1) << 6 |
                     ((b1 >> 7) & 1) << 5 | ((b1 >> 3) & 1) << 4 |
                     ((b2 >> 7) & 1) << 3 | ((b2 >> 3) & 1) << 2 |
                     ((b3 >> 7) & 1) << 1 | ((b3 >> 3) & 1);

        c0 = crc16_byte(c0, p0);
        c1 = crc16_byte(c1, p1);
        c2 = crc16_byte(c2, p2);
        c3 = crc16_byte(c3, p3);
    }
    uint16_t crc[4] = {c0, c1, c2, c3};
    memset(crc_out_8bytes, 0, 8);
    for (int bit_idx = 0; bit_idx < 16; bit_idx++) {
        uint8_t nibble = 0;
        for (int line = 0; line < 4; line++) {
            if ((crc[line] >> (15 - bit_idx)) & 1) {
                nibble |= (1 << line);
            }
        }
        if (bit_idx % 2 == 0) {
            crc_out_8bytes[bit_idx / 2] |= (nibble << 4);
        } else {
            crc_out_8bytes[bit_idx / 2] |= nibble;
        }
    }
}

// Передача одного 512-байтного блока по 4-битной шине DAT0..DAT3 (GP4..GP7) через PIO
void send_512b_block_pio(const uint8_t* block_512) {
    uint8_t crc_bytes[8];
    calc_sdio_4bit_crc16(block_512, crc_bytes);

    // 1. Передаем управление пинами DAT0..DAT3 (GP4..GP7) в PIO0
    for (uint pin = PIN_DAT0; pin <= PIN_DAT3; pin++) {
        gpio_disable_pulls(PIN_DAT1);
        pio_gpio_init(pio1, pin);
    }

    // 2. Перезапускаем State Machine 1 (sm_dat) на метку start_tx
    pio_sm_set_enabled(pio1, sm_dat, false);
    pio_sm_clear_fifos(pio1, sm_dat);
    pio_interrupt_clear(pio1, 0);
    pio_sm_exec(pio1, sm_dat, pio_encode_jmp(offset_dat + sdio_dat_tx_offset_start_tx));
    pio_sm_set_enabled(pio1, sm_dat, true);

    // 3. Отправляем счетчик нибблов - 1: 1 (start) + 1024 (data) + 16 (crc) - 1 = 1040
    pio_sm_put_blocking(pio1, sm_dat, 1040);

    // 4. Формируем первое 32-битное слово (8 нибблов):
    // [Start nibble 0x0 (4 бита)] + [первые 7 нибблов данных (block_512[0..2] и старший ниббл block_512[3])]
    uint32_t first_word = 0; // Старший ниббл = 0x0 (Start bit на всех 4 линиях)
    first_word |= ((uint32_t)block_512[0] << 20);
    first_word |= ((uint32_t)block_512[1] << 12);
    first_word |= ((uint32_t)block_512[2] << 4);
    first_word |= ((uint32_t)(block_512[3] >> 4) & 0x0F);
    pio_sm_put_blocking(pio1, sm_dat, first_word);

    // 5. Упаковываем оставшиеся 1017 нибблов данных + 16 нибблов CRC + 3 ниббла заполнения (0xF) = 1036 нибблов (518 байт)
    uint8_t tail_buf[518];
    for (int i = 0; i < 508; i++) {
        tail_buf[i] = (block_512[3 + i] << 4) | (block_512[4 + i] >> 4);
    }
    tail_buf[508] = (block_512[511] << 4) | (crc_bytes[0] >> 4);
    for (int i = 0; i < 7; i++) {
        tail_buf[509 + i] = (crc_bytes[i] << 4) | (crc_bytes[i + 1] >> 4);
    }
    tail_buf[516] = (crc_bytes[7] << 4) | 0x0F;
    tail_buf[517] = 0xFF;

    // Отправляем 516 байт (129 полных 32-битных слов) в TX FIFO
    for (int i = 0; i < 516; i += 4) {
        uint32_t w = ((uint32_t)tail_buf[i] << 24) |
                     ((uint32_t)tail_buf[i + 1] << 16) |
                     ((uint32_t)tail_buf[i + 2] << 8) |
                     ((uint32_t)tail_buf[i + 3]);
        pio_sm_put_blocking(pio1, sm_dat, w);
    }
    // Последний ниббл CRC + дополнение 0xF (130-е слово)
    uint32_t last_word = ((uint32_t)tail_buf[516] << 24) | ((uint32_t)tail_buf[517] << 16) | 0xFFFF;
    pio_sm_put_blocking(pio1, sm_dat, last_word);

    // 6. Ждем аппаратного прерывания IRQ 0 от PIO об окончании передачи блока (включая Stop nibble 0xF)
    while (!pio_interrupt_get(pio1, 0)) {
        tight_loop_contents();
    }

    // 7. Возвращаем пины DAT0..DAT3 в управление процессора (SIO)
    for (uint pin = PIN_DAT0; pin <= PIN_DAT3; pin++) {
        gpio_init(pin);
        gpio_set_dir(pin, GPIO_IN);
        gpio_disable_pulls(pin);
    }
    // Удерживаем DAT1 в '1' (подтяжка вверх — состояние отсутствия прерывания IRQ)
    gpio_pull_up(PIN_DAT1);
}


void init_identify_table() {
    memset(identify_data, 0, 512);

    // Word 0: 0x0040 (Non-removable, non-ATAPI) — строго проверяется Nokia N91
    identify_data[0] = 0x40;
    identify_data[1] = 0x00;

    // Word 1: Cylinders = 7769 (0x1E59)
    identify_data[2] = 0x59;
    identify_data[3] = 0x1E;

    // Word 3: Heads = 16 (0x0010)
    identify_data[6] = 0x10;
    identify_data[7] = 0x00;

    // Word 6: Sectors per track = 63 (0x003F)
    identify_data[12] = 0x3F;
    identify_data[13] = 0x00;

    // Word 27-46: Model number (byte-swapped ASCII: "TOSHIBA MK4001MTD")
    const char* model = "OTHSBI AM4K00M1DT                       ";
    memcpy(&identify_data[54], model, 40);

    // Word 49: Capabilities (LBA supported = bit 9 -> 0x0200)
    identify_data[98] = 0x00;
    identify_data[99] = 0x02;

    // Word 60-61: Total LBA sectors для 4 ГБ (7,831,152 = 0x00777E70)
    uint32_t total_sectors = 7831152;
    identify_data[120] = (total_sectors >> 0) & 0xFF;
    identify_data[121] = (total_sectors >> 8) & 0xFF;
    identify_data[122] = (total_sectors >> 16) & 0xFF;
    identify_data[123] = (total_sectors >> 24) & 0xFF;
}

void reset_sdio_controller() {
    memset(cccr, 0, sizeof(cccr));
    cccr[0x00] = 0x11; // CCCR/SDIO rev 1.10
    cccr[0x01] = 0x01; // SD Spec 1.10
    cccr[0x08] = 0x03; // Card Capability: SDC + SMB (Multi-block)
    current_state = STATE_IDLE;
    cmd5_poll_count = 0;
    ata.status = 0x50;
}

uint8_t calc_crc7(uint64_t data_40bit) {
    uint8_t crc = 0;
    for (int i = 39; i >= 0; i--) {
        uint8_t bit = (data_40bit >> i) & 1;
        uint8_t c = ((crc >> 6) & 1) ^ bit;
        crc = (crc << 1) & 0x7F;
        if (c) crc ^= 0x09;
    }
    return crc;
}

uint64_t make_response(uint8_t cmd_idx, uint32_t arg, bool use_crc = true) {
    uint64_t frame = ((uint64_t)(cmd_idx & 0x3F) << 32) | arg;
    uint8_t crc = use_crc ? calc_crc7(frame) : 0x7F;
    return (frame << 8) | (crc << 1) | 1;
}


// Выполнение ATA-команды, записанной в регистр 0x07 (Function 1)
void execute_ata_command(uint8_t cmd) {
    ata.command = cmd;

    switch (cmd) {
        case 0xEC: // IDENTIFY DEVICE
            memcpy(sector_buffer, identify_data, 512);
            ata.status = 0xD8; // BSY + DRQ (как в реальном MK4001MTD)
            cccr[0x05] |= 0x02; // INT_PENDING (Function 1 IRQ)
            gpio_set_dir(PIN_DAT1, GPIO_OUT);
            gpio_put(PIN_DAT1, 0); // Аппаратный IRQ = 0
            break;

        case 0x20: // READ SECTORS
            ata.status = 0xD8;
            cccr[0x05] |= 0x02;
            gpio_set_dir(PIN_DAT1, GPIO_OUT);
            gpio_put(PIN_DAT1, 0); // Аппаратный IRQ = 0
            break;

        case 0x30: // WRITE SECTORS
            ata.status = 0xD8;
            cccr[0x05] |= 0x02;
            break;

        case 0xC2: // Vendor Toshiba Multi-sector config
            if (ata.features == 0x20) {
                ata.seccount = 0xFF;
                ata.lba_mid = 0xFF;
                ata.lba_lo = 0x00;
                ata.lba_hi = 0x00;
            } else if (ata.features == 0x21) {
                ata.seccount = 0x20; // Текущий лимит: 32 сектора
            }
            ata.status = 0x50;
            cccr[0x05] |= 0x02;
            gpio_set_dir(PIN_DAT1, GPIO_OUT);
            gpio_put(PIN_DAT1, 0); // Аппаратный IRQ = 0
            break;

        case 0xE0: // STANDBY IMMEDIATE
            ata.status = 0x50;
            cccr[0x05] = 0x00;
            //reset_sdio_controller();
            break;

        default:
            ata.status = 0x50;
            break;
    }
}

// Обработчик входящей 48-битной команды от PIO
uint64_t handle_sdio_command(uint64_t raw_cmd, bool* has_response) {
    uint8_t cmd_idx = (raw_cmd >> 40) & 0x3F;
    uint32_t arg = (raw_cmd >> 8) & 0xFFFFFFFF;
    uint64_t resp = 0;
    
    *has_response = true;

    switch (cmd_idx) {
        case 5: { // IO_SEND_OP_COND
            if (arg == 0x00000000) {
                // Шаг 1: Пробный запрос -> Ready=0, Fn=1, OCR=0x1F8000
                cmd5_poll_count = 0;
                resp = make_response(0x3F, 0x101F8000, false);
            } else {
                cmd5_poll_count++;
                if (cmd5_poll_count <= 6) {
                    // Шаги 2..7 (6 раз): Ready=0, Fn=1, OCR=0x1F8000
                    resp = make_response(0x3F, 0x101F8000, false);
                } else {
                    // Шаги 8..9 (с 7-го раза и далее): Ready=1, Fn=1, OCR=0x1C0000
                    current_state = STATE_READY;
                    resp = make_response(0x3F, 0x901C0000, false);
                }
            }
            break;
        }

        case 3: {
            current_state = STATE_STBY;
            resp = make_response(3, 0x00011E00);
            break;        }

        case 7: {
            current_state = STATE_CMD;
            resp = make_response(7, 0x00001E00);
            break;
        }

        case 52: { // IO_RW_DIRECT (Шаги 12..18 и далее)
            uint8_t rw = (arg >> 31) & 1;
            uint8_t fn = (arg >> 28) & 0x7;
            uint32_t addr = (arg >> 9) & 0x1FFFF;
            uint8_t data = arg & 0xFF;
            uint8_t out_data = 0;

            if (fn == 0) { // CCCR / FBR (Шаги 13..18)
                if (rw) {
                    cccr[addr & 0x1FF] = data;
                    if (addr == 0x0002 && (data & 0x02)) {
                        cccr[0x0003] |= 0x02; // IO_ENABLE -> сразу ставим IO_READY = 0x02
                        gpio_init(PIN_DAT1);
                        gpio_set_dir(PIN_DAT1, GPIO_IN);
                        gpio_pull_up(PIN_DAT1);
                    }
                    out_data = data;
                } else {
                    out_data = cccr[addr & 0x1FF];
                }
            } else if (fn == 1) { // ATA Task File (Шаг 12 и ATA-команды)
                if (rw) {
                    switch (addr) {
                        case 0x01: ata.features = data; break;
                        case 0x02: ata.seccount = data; break;
                        case 0x03: ata.lba_lo = data; break;
                        case 0x04: ata.lba_mid = data; break;
                        case 0x05: ata.lba_hi = data; break;
                        case 0x06: ata.dev_head = data; break;
                        case 0x07: execute_ata_command(data); break;
                    }
                    out_data = data;
                } else {
                    switch (addr) {
                        case 0x01: out_data = ata.error; break;
                        case 0x02: out_data = ata.seccount; break;
                        case 0x03: out_data = ata.lba_lo; break;
                        case 0x04: out_data = ata.lba_mid; break;
                        case 0x05: out_data = ata.lba_hi; break;
                        case 0x06: out_data = ata.dev_head; break;
                        case 0x07:
                            out_data = ata.status; // На шаге 12 вернет 0x50
                            cccr[0x05] &= ~0x02;   // Сброс флага прерывания при чтении STATUS
                            gpio_set_dir(PIN_DAT1, GPIO_IN);
                            gpio_pull_up(PIN_DAT1); // Снимаем IRQ (1)
                            break;
                    }
                }
            }
            // R5 response: Flags = 0x10 (STATE=CMD), Data = out_data
            uint32_t r5_arg = (0x10 << 8) | out_data;
            resp = make_response(52, r5_arg);
            break;
        }

        case 53: {
            uint8_t rw = (arg >> 31) & 1;
            uint16_t blocks = arg & 0x1FF;
            if (rw == 0) {
                pending_cmd53_blocks = (blocks == 0) ? 512 : blocks;
            }
            resp = make_response(53, (0x20 << 8) | 0x00);
            break;
        }
        default:
            *has_response = false;
            break;     
    }

    return resp;
}

void init_sdio_cmd_pio(PIO pio, uint sm, uint offset) {
    pio_sm_config c = sdio_cmd_program_get_default_config(offset);

    // Пины: in_base = PIN_CMD (pin 0 -> CMD, pin 1 -> CLK)
    sm_config_set_in_pins(&c, PIN_CMD);
    sm_config_set_out_pins(&c, PIN_CMD, 1);
    sm_config_set_set_pins(&c, PIN_CMD, 1);

    // Включаем autopush и autopull ровно на 24 битах (MSB-first / сдвиг влево)
    sm_config_set_in_shift(&c, false, true, 24);
    sm_config_set_out_shift(&c, false, true, 24);

    pio_gpio_init(pio, PIN_CMD);
    pio_gpio_init(pio, PIN_CLK);
    gpio_pull_up(PIN_CMD);

    pio_sm_set_consecutive_pindirs(pio, sm, PIN_CMD, 1, false);
    pio_sm_set_consecutive_pindirs(pio, sm, PIN_CLK, 1, false);
    sm_config_set_clkdiv(&c, 1.0f);

    pio_sm_init(pio, sm, offset, &c);
    pio_sm_set_enabled(pio, sm, true);
}

int main() {
    set_sys_clock_khz(200000, true);    
    stdio_init_all();
    init_identify_table();
    reset_sdio_controller();

    PIO pio = pio0;
    uint sm = 0;
    uint offset = pio_add_program(pio, &sdio_cmd_program);
    init_sdio_cmd_pio(pio, sm, offset);

    offset_dat = pio_add_program(pio1, &sdio_dat_tx_program);
    pio_sm_config c_dat = sdio_dat_tx_program_get_default_config(offset_dat);
    sm_config_set_in_pins(&c_dat, PIN_CLK);           // pin 0 для wait = CLK (GP3)
    sm_config_set_out_pins(&c_dat, PIN_DAT0, 4);      // DAT0..DAT3 (GP4..GP7)
    sm_config_set_set_pins(&c_dat, PIN_DAT0, 4);
    sm_config_set_out_shift(&c_dat, false, true, 32); // MSB-first, autopull = 32
    sm_config_set_clkdiv(&c_dat, 1.0f);
    pio_sm_init(pio1, sm_dat, offset_dat, &c_dat);

    init_sdio_dat_pins();

    while (true) {
        
        // 1. Читаем 48 бит команды из PIO RX FIFO (два слова по 24 бита)
        uint32_t cmd_hi = pio_sm_get_blocking(pio, sm) & 0xFFFFFF;
        uint32_t cmd_lo = pio_sm_get_blocking(pio, sm) & 0xFFFFFF;
        uint64_t raw_cmd = ((uint64_t)cmd_hi << 24) | cmd_lo;

        // 2. Обрабатываем команду
        bool has_resp = false;
        uint64_t resp = handle_sdio_command(raw_cmd, &has_resp);

        // 3. Отправляем ответ в PIO TX FIFO:
        // Слово 1 (24 бита): [флаг '1' (1 бит)] + [биты 47..25 ответа (23 бита)]
        // Слово 2 (24 бита): [биты 24..1 ответа (24 бита)]
        // Слово 3 (24 бита): [бит 0 ответа в старшем бите]
        if (has_resp) {
            uint8_t cmd_idx = (raw_cmd >> 40) & 0x3F;
            // Для CMD5 задержка = 3 (16 тактов паузы), для CMD3/CMD7/CMD52 = 1 (14 тактов паузы)
            uint32_t delay_flag = (cmd_idx == 5) ? 3u : 1u;


            // Упаковываем 8 бит флага + 48 бит ответа = 56 бит в 3 слова по 24 бита (выровненные << 8):
            // Слово 1 (24 бита): [delay_flag (8 бит)] + [биты 47..32 ответа (16 бит)]
            uint32_t word1_24 = (delay_flag << 16) | (uint32_t)((resp >> 32) & 0xFFFF);
            // Слово 2 (24 бита): [биты 31..8 ответа (24 бита)]
            uint32_t word2_24 = (uint32_t)((resp >> 8) & 0xFFFFFF);
            // Слово 3 (24 бита): [биты 7..0 ответа (8 бит) в старшем байте]
            uint32_t word3_24 = (uint32_t)(resp & 0xFF) << 16;

            pio_sm_put_blocking(pio, sm, word1_24 << 8);
            pio_sm_put_blocking(pio, sm, word2_24 << 8);
            pio_sm_put_blocking(pio, sm, word3_24 << 8);

            if (pending_cmd53_blocks > 0) {
                // 1. Ждем, пока все 3 слова ответа R5 уйдут из TX FIFO в сдвиговый регистр
                while (!pio_sm_is_tx_fifo_empty(pio, sm)) {
                    tight_loop_contents();
                }
                // 2. Ждем, пока sm_cmd закончит выталкивать последний байт R5 и вернется в wait_start
                while (pio_sm_get_pc(pio, sm) > offset + 2) {
                    tight_loop_contents();
                }

                // 3. Мгновенно (через ~20 нс, строго в окне N_AC) отдаем блок по DAT0..DAT3
                for (uint16_t b = 0; b < pending_cmd53_blocks; b++) {
                    send_512b_block_pio(&sector_buffer[b * 512]);
                }
                pending_cmd53_blocks = 0;
                ata.status = 0x50;
            }
        } else {
            pio_sm_put_blocking(pio, sm, 0);
        }
    }
}