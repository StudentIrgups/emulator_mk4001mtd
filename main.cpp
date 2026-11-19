#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "sdio_cmd.pio.h"

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

// 512-байтный паспорт диска Toshiba MK4001MTD (4 ГБ)
uint8_t identify_data[512] = {0};

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

        case 53: { // IO_RW_EXTENDED
            uint32_t r5_arg = (0x20 << 8) | 0x00; // STATE=TRN
            resp = make_response(53, r5_arg);
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
    init_sdio_dat_pins();
    stdio_init_all();
    init_identify_table();
    reset_sdio_controller();

    PIO pio = pio0;
    uint sm = 0;
    uint offset = pio_add_program(pio, &sdio_cmd_program);
    init_sdio_cmd_pio(pio, sm, offset);

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
        } else {
            pio_sm_put_blocking(pio, sm, 0);
        }
    }
}