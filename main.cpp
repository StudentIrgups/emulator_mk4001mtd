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

// Расчет 4-канального CRC16-CCITT (независимый CRC16 для каждой линии DAT0..DAT3)
void calc_sdio_4bit_crc16(const uint8_t* data, uint8_t* crc_out_8bytes) {
    uint16_t crc[4] = {0, 0, 0, 0};

    for (int i = 0; i < 512; i++) {
        uint8_t byte = data[i];
        // Сначала старший ниббл (биты 7..4), затем младший ниббл (биты 3..0)
        for (int nib = 1; nib >= 0; nib--) {
            uint8_t val = (byte >> (nib * 4)) & 0x0F;
            for (int line = 0; line < 4; line++) {
                uint8_t bit = (val >> line) & 1;
                uint8_t inv = ((crc[line] >> 15) & 1) ^ bit;
                crc[line] <<= 1;
                if (inv) {
                    crc[line] ^= 0x1021;
                }
            }
        }
    }

    memset(crc_out_8bytes, 0, 8);
    // Упаковываем 16 бит CRC по 4 линиям в 16 нибблов (8 байт)
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
        pio_gpio_init(pio0, pin);
    }

    // 2. Перезапускаем State Machine 1 (sm_dat) на метку start_tx
    pio_sm_set_enabled(pio0, sm_dat, false);
    pio_sm_clear_fifos(pio0, sm_dat);
    pio_interrupt_clear(pio0, 0);
    pio_sm_exec(pio0, sm_dat, pio_encode_jmp(offset_dat + sdio_dat_tx_offset_start_tx));
    pio_sm_set_enabled(pio0, sm_dat, true);

    // 3. Отправляем счетчик нибблов - 1: 1 (start) + 1024 (data) + 16 (crc) - 1 = 1040
    pio_sm_put_blocking(pio0, sm_dat, 1040);

    // 4. Формируем первое 32-битное слово (8 нибблов):
    // [Start nibble 0x0 (4 бита)] + [первые 7 нибблов данных (block_512[0..2] и старший ниббл block_512[3])]
    uint32_t first_word = 0; // Старший ниббл = 0x0 (Start bit на всех 4 линиях)
    first_word |= ((uint32_t)block_512[0] << 20);
    first_word |= ((uint32_t)block_512[1] << 12);
    first_word |= ((uint32_t)block_512[2] << 4);
    first_word |= ((uint32_t)(block_512[3] >> 4) & 0x0F);
    pio_sm_put_blocking(pio0, sm_dat, first_word);

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
        pio_sm_put_blocking(pio0, sm_dat, w);
    }
    // Последний ниббл CRC + дополнение 0xF (130-е слово)
    uint32_t last_word = ((uint32_t)tail_buf[516] << 24) | ((uint32_t)tail_buf[517] << 16) | 0xFFFF;
    pio_sm_put_blocking(pio0, sm_dat, last_word);

    // 6. Ждем аппаратного прерывания IRQ 0 от PIO об окончании передачи блока (включая Stop nibble 0xF)
    while (!pio_interrupt_get(pio0, 0)) {
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
            break;

        case 0x20: // READ SECTORS
            ata.status = 0xD8;
            cccr[0x05] |= 0x02;
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

    offset_dat = pio_add_program(pio0, &sdio_dat_tx_program);
    pio_sm_config c_dat = sdio_dat_tx_program_get_default_config(offset_dat);
    sm_config_set_in_pins(&c_dat, PIN_CLK);           // pin 0 для wait = CLK (GP3)
    sm_config_set_out_pins(&c_dat, PIN_DAT0, 4);      // DAT0..DAT3 (GP4..GP7)
    sm_config_set_set_pins(&c_dat, PIN_DAT0, 4);
    sm_config_set_out_shift(&c_dat, false, true, 32); // MSB-first, autopull = 32
    sm_config_set_clkdiv(&c_dat, 1.0f);
    pio_sm_init(pio0, sm_dat, offset_dat, &c_dat);

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
                cccr[0x05] |= 0x02;
            }
        } else {
            pio_sm_put_blocking(pio, sm, 0);
        }
    }
}