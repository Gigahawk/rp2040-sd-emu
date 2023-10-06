// Copyright (c) 2021 Michael Stoops. All rights reserved.
// Portions copyright (c) 2021 Raspberry Pi (Trading) Ltd.
// 
// Redistribution and use in source and binary forms, with or without modification, are permitted provided that the 
// following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following
//    disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
//    following disclaimer in the documentation and/or other materials provided with the distribution.
// 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote
//    products derived from this software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, 
// INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE 
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR 
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, 
// WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE 
// USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// SPDX-License-Identifier: BSD-3-Clause
//
// Example of an SPI bus slave using the PL022 SPI interface

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/spi.h"

#define CMD_LEN 0x6

#define spi_default spi0
#define PICO_DEFAULT_SPI_SCK_PIN 18
#define PICO_DEFAULT_SPI_TX_PIN 19
#define PICO_DEFAULT_SPI_RX_PIN 20
#define PICO_DEFAULT_SPI_CSN_PIN 1


#define SECTOR_SIZE 512

typedef struct Sector {
    uint32_t block;
    uint8_t data[SECTOR_SIZE];
    uint16_t crc;
    struct Sector* next;
} Sector;

// FAT16 sector pointer or something, do we need this?
Sector sec1 = {
    0x01,
    {0xF8, 0xFF, 0xFF, 0xFF},
    0,
    NULL
};

// FAT16 header
Sector header = {
    0x00,
    {
        0xEB, 0x3C, 0x90, 0x6D, 0x6B, 0x66, 0x73, 0x2E, 0x66, 0x61, 0x74, 0x00, 0x02, 0x01, 0x01, 0x00,
        0x01, 0x00, 0x02, 0x00, 0x18, 0xF8, 0x18, 0x00, 0x20, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x80, 0x00, 0x29, 0xED, 0xBD, 0xCC, 0xF0, 0x4E, 0x4F, 0x20, 0x4E, 0x41,
        0x4D, 0x45, 0x20, 0x20, 0x20, 0x20, 0x46, 0x41, 0x54, 0x31, 0x36, 0x20, 0x20, 0x20, 0x0E, 0x1F,
        0xBE, 0x5B, 0x7C, 0xAC, 0x22, 0xC0, 0x74, 0x0B, 0x56, 0xB4, 0x0E, 0xBB, 0x07, 0x00, 0xCD, 0x10,
        0x5E, 0xEB, 0xF0, 0x32, 0xE4, 0xCD, 0x16, 0xCD, 0x19, 0xEB, 0xFE, 0x54, 0x68, 0x69, 0x73, 0x20,
        0x69, 0x73, 0x20, 0x6E, 0x6F, 0x74, 0x20, 0x61, 0x20, 0x62, 0x6F, 0x6F, 0x74, 0x61, 0x62, 0x6C,
        0x65, 0x20, 0x64, 0x69, 0x73, 0x6B, 0x2E, 0x20, 0x20, 0x50, 0x6C, 0x65, 0x61, 0x73, 0x65, 0x20,
        0x69, 0x6E, 0x73, 0x65, 0x72, 0x74, 0x20, 0x61, 0x20, 0x62, 0x6F, 0x6F, 0x74, 0x61, 0x62, 0x6C,
        0x65, 0x20, 0x66, 0x6C, 0x6F, 0x70, 0x70, 0x79, 0x20, 0x61, 0x6E, 0x64, 0x0D, 0x0A, 0x70, 0x72,
        0x65, 0x73, 0x73, 0x20, 0x61, 0x6E, 0x79, 0x20, 0x6B, 0x65, 0x79, 0x20, 0x74, 0x6F, 0x20, 0x74,
        0x72, 0x79, 0x20, 0x61, 0x67, 0x61, 0x69, 0x6E, 0x20, 0x2E, 0x2E, 0x2E, 0x20, 0x0D, 0x0A, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x55, 0xAA
    },
    0,
    &sec1,
};

void setCrc(Sector* sector) {
    // TODO: Set CRC
    // Currently the arduino SD lib ignores this
    return;
}

Sector* getBlock(uint32_t block) {
    Sector* ptr = &header;
    while(1) {
        if(ptr->block == block) {
            setCrc(ptr);
            return ptr;
        }
        if(ptr->next == NULL)
            return NULL;
        ptr = ptr->next;
    }
}

Sector* allocateBlock(uint32_t block) {
    Sector* ptr = &header;
    while(1) {
        if(ptr->block == block) {
            setCrc(ptr);
            return ptr;
        }
        if(ptr->next == NULL)
            break;
        ptr = ptr->next;
    }
    printf("Allocating block %d\n", block);
    ptr->next = (Sector*)malloc(sizeof(Sector));
    ptr = ptr->next;
    if (ptr == NULL) {
        printf("Allocation failed\n");
        return NULL;
    }
    ptr->block = block;
    ptr->next = NULL;
    return ptr;
}

void printbuf(uint8_t buf[], size_t len) {
    int i;
    for (i = 0; i < len; ++i) {
        if (i % 16 == 15)
            printf("%02x\n", buf[i]);
        else
            printf("%02x ", buf[i]);
    }

    // append trailing newline if there isn't one
    if (i % 16) {
        putchar('\n');
    }
}

// In most cases we want to send 0xFF if we are idling
uint8_t spi_tx_idle_value = 0xFF;
void spi_irq_handler() {
    while(spi_get_hw(spi_default)->ris & 0b1000) {
        spi_get_hw(spi_default)->dr = spi_tx_idle_value;
    }
}

void setup_spi() {
    // Enable SPI 0 at 1 MHz and connect to GPIOs
    spi_init(spi_default, 1000 * 1000);
    // From the SPI section of the datasheet, the SPI controller defaults to using the 
    // Motorla SPI format, which only allows continuous transfers with chip select held
    // low across multiple frames if the clock phase is set to 1.
    // We must therefore also set the clock polarity to 1 to properly support SD transfers
    // (where both are 0)
    spi_set_format(spi_default, 8, SPI_CPOL_1, SPI_CPHA_1, SPI_MSB_FIRST);
    spi_set_slave(spi_default, true);
    gpio_set_function(PICO_DEFAULT_SPI_RX_PIN, GPIO_FUNC_SPI);
    gpio_set_function(PICO_DEFAULT_SPI_SCK_PIN, GPIO_FUNC_SPI);
    gpio_set_function(PICO_DEFAULT_SPI_TX_PIN, GPIO_FUNC_SPI);
    gpio_set_function(PICO_DEFAULT_SPI_CSN_PIN, GPIO_FUNC_SPI);
    //// Make the SPI pins available to picotool
    //bi_decl(bi_4pins_with_func(PICO_DEFAULT_SPI_RX_PIN, PICO_DEFAULT_SPI_TX_PIN, PICO_DEFAULT_SPI_SCK_PIN, PICO_DEFAULT_SPI_CSN_PIN, GPIO_FUNC_SPI));


    // TODO: idk make this parametric
    irq_set_enabled(SPI0_IRQ, true);
    irq_set_exclusive_handler(SPI0_IRQ, &spi_irq_handler);
    // Enable interrupt to fire when TX buffer is almost empty
    spi_get_hw(spi_default)->imsc = SPI_SSPIMSC_TXIM_BITS;
}

// TODO: this should probably be an enum/struct
#define STATE_CMD_WAIT 0
#define STATE_CMD 1


#define CMD0 0
#define CMD8 8
#define CMD13 13
#define CMD17 17
#define CMD24 24
#define CMD55 55
#define CMD58 58

#define ACMD41 41

#define DATA_START_BLOCK 0XFE
#define DATA_RES_ACCEPTED 0X05

void spi_clear_read_buf(spi_inst_t *spi) {
    uint8_t i;
    while(spi_is_readable(spi)) {
        i = (uint8_t)spi_get_hw(spi)->dr;
    }
}

void spi_read(spi_inst_t *spi, uint8_t* dst, size_t len) {
    for(size_t i = 0; i < len; ++i) {
        while(!spi_is_readable(spi))
            tight_loop_contents();
        *dst++ = (uint8_t)(spi_get_hw(spi)->dr);
    }
}

void spi_read_large(spi_inst_t *spi, uint8_t* dst, size_t len) {
    for(size_t i = 0; i < len; ++i) {
        while(!spi_is_readable(spi))
            tight_loop_contents();
        *dst++ = (uint8_t)(spi_get_hw(spi)->dr);
        // Hack: ensure TX buffer is always filled
        spi_irq_handler();
    }
}

void spi_write(spi_inst_t *spi, uint8_t* src, size_t len) {
    for(size_t i = 0; i < len; ++i) {
        while(!spi_is_writable(spi))
            tight_loop_contents();
        spi_get_hw(spi)->dr = (uint32_t)src[i];
    }
}

void spi_write_const(spi_inst_t *spi, uint8_t val, size_t len) {
    for(size_t i = 0; i < len; ++i) {
        while(!spi_is_writable(spi))
            tight_loop_contents();
        spi_get_hw(spi)->dr = (uint32_t)val;
    }
}

uint32_t get_block_num(uint8_t* arg) {
    return (
        (uint32_t)arg[0] << 24 |
        (uint32_t)arg[1] << 16 |
        (uint32_t)arg[2] << 8 |
        (uint32_t)arg[3]
    );
}

void handle_cmd(uint8_t* cmd_buf) {
    static bool app_mode = false;
    uint8_t* arg = cmd_buf + 1;
    uint8_t cmd = cmd_buf[0] & 0b00111111;
    uint32_t block;
    Sector* sector;
    uint8_t chksum = cmd_buf[5];
    uint8_t resp[5];
    if(!app_mode) {
        switch(cmd) {
            case CMD0:
                printf("Handling reset CMD0\n");
                // TODO: integrity check?
                // reset always responds with 0x1 (in idle mode?)
                spi_write_const(spi_default, 0x01, 1);
                printf("Response sent\n");
                break;
            case CMD8:
                printf("Handling init CMD8\n");
                // Return an R7 response
                // idle mode
                resp[0] = 0x01;
                // voltage level?
                resp[3] = cmd_buf[3];
                // read back
                resp[4] = cmd_buf[4];
                spi_write(spi_default, resp, 5);
                printf("Response sent\n");
                break;
            case CMD13:
                printf("Handling CSD CMD13\n");
                // Return an R2 response
                spi_write_const(spi_default, 0x00, 2);
                printf("Response sent\n");
                break;
            case CMD17:
                printf("Handling read block CMD17\n");
                // Return an R1 response
                spi_write_const(spi_default, 0x00, 1);
                printf("Response sent\n");
                // Because we are pretending to be an SDHC
                // the arg is always block number, for SD this value is
                // an address
                block = get_block_num(arg);
                printf("Sending block %d\n", block);
                sector = getBlock(block);
                // Disable IRQ, we don't want the buffer to accidentally be filled with idle data
                // TODO: idk make this parametric
                irq_set_enabled(SPI0_IRQ, false);
                // Indicate data is ready to read
                spi_write_const(spi_default, DATA_START_BLOCK, 1);
                if (sector == NULL) {
                    printf("block doesn't exist, returning zeros\n");
                    // Write data plus CRC
                    spi_write_const(spi_default, 0x00, SECTOR_SIZE + 2);
                } else {
                    // Write data plus CRC
                    spi_write(spi_default, sector->data, SECTOR_SIZE + 2);
                }
                // turn interrupt back on, ensure buffer is full again
                // TODO: idk make this parametric
                irq_set_enabled(SPI0_IRQ, true);
                spi_irq_handler();
                printf("Finished sending block\n");
                break;
            case CMD24:
                printf("Handling write block CMD24\n");
                // Because we are pretending to be an SDHC
                // the arg is always block number, for SD this value is
                // an address
                block = get_block_num(arg);
                printf("Writing to block %d\n", block);
                // Make sure the block is actually available for writing first
                sector = allocateBlock(block);
                // Ensure read buffer is cleared so that we can recieve the start block right away
                spi_clear_read_buf(spi_default);
                // Return an R1 response
                spi_write_const(spi_default, 0x00, 1);
                // Arduino lib expects the response to be sent on the transfer immediately after
                // the data/crc, it's too slow to manually send that so instead we start filling
                // the buffer with the response
                spi_tx_idle_value = DATA_RES_ACCEPTED;
                // Wait for data start block, use resp array
                //printf("Waiting for data start\n");
                do {
                    spi_read(spi_default, resp, 1);
                } while(resp[0] != DATA_START_BLOCK);
                //printf("Writing data\n");
                // SPI interrupt doesn't seem to fire here if do a large read
                // Seems like if we use spi_read 0xFF still gets filled to the bus
                // until the transfer ends, and then we get a bunch of 0x00 instead,
                // DATA_RES_ACCEPTED never shows up on the bus
                spi_read_large(spi_default, sector->data, SECTOR_SIZE + 2);
                // return idle value back to normal
                spi_tx_idle_value = 0xFF;
                //// Respond with accepted
                //spi_write_const(spi_default, DATA_RES_ACCEPTED, 1);
                //printf("Data written:\n");
                //printbuf(sector->data, DATA_START_BLOCK);
                //printf("CRC written: %x\n", sector->crc);
                break;
            case CMD55:
                printf("Handling app CMD55\n");
                app_mode = true;
                spi_write_const(spi_default, 0x01, SECTOR_SIZE + 2);
                printf("Response sent\n");
                break;
            case CMD58:
                printf("Handling ocr CMD58\n");
                // Return an R3 response
                resp[0] = 0x00;
                // SDHC mode
                resp[1] = 0xC0;
                spi_write(spi_default, resp, 5);
                printf("Response sent\n");
                break;
            default:
                printf("Unknown command %d\n", cmd);
        }
    } else {
        switch(cmd) {
            case ACMD41:
                printf("Handling app command CMD41\n");
                resp[0] = 0x00;
                spi_write(spi_default, resp, 1);
                printf("Response sent\n");
                break;
            default:
                printf("Unknown app command %d\n", cmd);
        }
        app_mode = false;
    }
}



int main() {
    // Enable UART so we can print
    stdio_init_all();
    while(!stdio_usb_connected());
#if !defined(spi_default) || !defined(PICO_DEFAULT_SPI_SCK_PIN) || !defined(PICO_DEFAULT_SPI_TX_PIN) || !defined(PICO_DEFAULT_SPI_RX_PIN) || !defined(PICO_DEFAULT_SPI_CSN_PIN)
#warning spi/spi_slave example requires a board with SPI pins
    puts("Default SPI pins were not defined");
#else
    int state = STATE_CMD_WAIT;

    printf("SD emulation example\n");
    setup_spi();

    uint8_t cmd_buf[CMD_LEN];

    printf("Waiting for data from SPI master\n");
    while(1) {
        switch(state) {
            case STATE_CMD_WAIT:
                spi_read(spi_default, cmd_buf, 1);
                // First two bits of transfer must be 0b01 to be a valid command
                if((cmd_buf[0] & 0xC0) == 0x40){
                    state = STATE_CMD;
                }
                break;
            case STATE_CMD:
                // Read rest of command if valid
                spi_read(spi_default, cmd_buf + 1, CMD_LEN - 1);
                printbuf(cmd_buf, CMD_LEN);
                handle_cmd(cmd_buf);
                state = STATE_CMD_WAIT;
                break;
        }
    }
#endif
}