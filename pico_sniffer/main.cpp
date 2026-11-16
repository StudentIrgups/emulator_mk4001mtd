#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/pio.h"
#include "sdio_sniffer.pio.h"

#define PIN_CMD 2
#define PIN_CLK 3

// Кольцевой буфер на 4096 пакетов (32 КБ ОЗУ) — защищает от потери данных при печати в USB
#define RING_SIZE 4096
volatile uint64_t ring_buf[RING_SIZE];
volatile uint32_t ring_head = 0;
volatile uint32_t ring_tail = 0;

// Таблицы имен регистров для красивого вывода
const char* get_cccr_name(uint32_t addr) {
    switch (addr) {
        case 0x0002: return "IO_ENABLE";
        case 0x0003: return "IO_READY";
        case 0x0004: return "INT_ENABLE";
        case 0x0005: return "INT_PENDING";
        case 0x0007: return "BUS_IF_CTRL";
        case 0x0110: return "FN1_BLK_SIZE_0";
        case 0x0111: return "FN1_BLK_SIZE_1";
        default: return "";
    }
}

const char* get_ata_reg_name(uint32_t addr) {
    switch (addr) {
        case 0x00: return "ATA_DATA";
        case 0x01: return "ATA_ERR/FEAT";
        case 0x02: return "ATA_SECCOUNT";
        case 0x03: return "ATA_LBA_LO";
        case 0x04: return "ATA_LBA_MID";
        case 0x05: return "ATA_LBA_HI";
        case 0x06: return "ATA_DEV/HEAD";
        case 0x07: return "ATA_CMD/STATUS";
        default: return "";
    }
}

const char* get_ata_cmd_name(uint8_t cmd) {
    switch (cmd) {
        case 0x20: return "READ_SECTORS";
        case 0x30: return "WRITE_SECTORS";
        case 0xC2: return "VENDOR_0xC2";
        case 0xE0: return "STANDBY_IMMEDIATE";
        case 0xEC: return "IDENTIFY_DEVICE";
        default: return "";
    }
}

// Расшифровка одного 48-битного пакета и вывод в USB Serial (работает на Core 1)
void print_packet(uint64_t raw) {
    uint8_t dir = (raw >> 46) & 1; // 1 = Host -> HDD, 0 = HDD -> Host
    uint8_t cmd = (raw >> 40) & 0x3F;
    uint32_t arg = (raw >> 8) & 0xFFFFFFFF;

    if (dir == 1) {
        // КОМАНДА ОТ NOKIA N91 К HDD
        if (cmd == 52) {
            uint8_t rw = (arg >> 31) & 1;
            uint8_t fn = (arg >> 28) & 0x7;
            uint32_t addr = (arg >> 9) & 0x1FFFF;
            uint8_t data = arg & 0xFF;

            // Раскомментируйте строку ниже, если хотите скрыть пустые опросы INT_PENDING (Fn0 0x05):
            // if (rw == 0 && fn == 0 && addr == 0x05) return;

            const char* rname = (fn == 0) ? get_cccr_name(addr) : get_ata_reg_name(addr);
            if (rw) {
                const char* cname = (fn == 1 && addr == 0x07) ? get_ata_cmd_name(data) : "";
                printf("HOST -> CMD52 WR Fn%d 0x%04lX (%s) <- 0x%02X %s\n", fn, addr, rname, data, cname);
            } else {
                printf("HOST -> CMD52 RD Fn%d 0x%04lX (%s)\n", fn, addr, rname);
            }
        } else if (cmd == 53) {
            uint8_t rw = (arg >> 31) & 1;
            uint8_t fn = (arg >> 28) & 0x7;
            uint8_t blk = (arg >> 27) & 1;
            uint16_t cnt = arg & 0x1FF;
            printf("HOST -> CMD53 %s Fn%d %s count=%d\n", rw ? "WR" : "RD", fn, blk ? "BLK" : "BYTE", cnt);
        } else {
            printf("HOST -> CMD%-2d arg=0x%08lX\n", cmd, arg);
        }
    } else {
        // ОТВЕТ ОТ ЖИВОГО HDD К NOKIA N91
        if (cmd == 0x3F) { // R4 (ответ на CMD5)
            uint8_t rdy = (arg >> 31) & 1;
            uint8_t nfn = (arg >> 28) & 0x7;
            printf("  HDD -> R4   Ready=%d Fn=%d OCR=0x%06lX\n", rdy, nfn, arg & 0xFFFFFF);
        } else if (cmd == 52 || cmd == 53) { // R5
            uint8_t flags = (arg >> 8) & 0xFF;
            uint8_t data = arg & 0xFF;
            printf("  HDD -> R5   Data=0x%02X Flags=0x%02X\n", data, flags);
        } else {
            printf("  HDD -> RESP (cmd=%d) arg=0x%08lX\n", cmd, arg);
        }
    }
}

// Core 1: бесконечный цикл разбора буфера и печати в терминал
void core1_entry() {
    while (true) {
        if (ring_tail != ring_head) {
            uint64_t pkt = ring_buf[ring_tail];
            ring_tail = (ring_tail + 1) % RING_SIZE;
            print_packet(pkt);
        } else {
            tight_loop_contents();
        }
    }
}

int main() {
    stdio_init_all();
    sleep_ms(2000); // Даем время открыть /dev/ttyACM0
    printf("=== SDIO Sniffer Ready (GP2=CMD, GP3=CLK) ===\n");

    // Запускаем поток печати на втором ядре
    multicore_launch_core1(core1_entry);

    // Настройка PIO0 SM0 в режиме чистого входа (без подтяжек, чтобы не влиять на шину)
    PIO pio = pio0;
    uint sm = 0;
    uint offset = pio_add_program(pio, &sdio_sniffer_program);
    pio_sm_config c = sdio_sniffer_program_get_default_config(offset);

    sm_config_set_in_pins(&c, PIN_CMD);
    sm_config_set_in_shift(&c, false, true, 24); // MSB-first, autopush = 24
    sm_config_set_clkdiv(&c, 1.0f);

    pio_gpio_init(pio, PIN_CMD);
    pio_gpio_init(pio, PIN_CLK);
    gpio_set_pulls(PIN_CMD, false, false); // Строго High-Z
    gpio_set_pulls(PIN_CLK, false, false);

    pio_sm_set_consecutive_pindirs(pio, sm, PIN_CMD, 2, false);
    pio_sm_init(pio, sm, offset, &c);
    pio_sm_set_enabled(pio, sm, true);

    // Core 0: только мгновенно перекладывает 48-битные пакеты из PIO в кольцевой буфер
    while (true) {
        uint32_t hi = pio_sm_get_blocking(pio, sm) & 0xFFFFFF;
        uint32_t lo = pio_sm_get_blocking(pio, sm) & 0xFFFFFF;
        uint64_t raw = ((uint64_t)hi << 24) | lo;

        uint32_t next_head = (ring_head + 1) % RING_SIZE;
        if (next_head != ring_tail) {
            ring_buf[ring_head] = raw;
            ring_head = next_head;
        }
    }
}
