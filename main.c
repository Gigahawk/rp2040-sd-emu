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

void spi_irq_handler() {
    while(spi_get_hw(spi_default)->ris & 0b1000) {
        spi_get_hw(spi_default)->dr = 0xFF;
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
#define CMD55 55
#define CMD58 58

#define ACMD41 41


void spi_read(spi_inst_t *spi, uint8_t* dst, size_t len) {
    for(size_t i = 0; i < len; ++i) {
        while(!spi_is_readable(spi))
            tight_loop_contents();
        *dst++ = (uint8_t)(spi_get_hw(spi)->dr);
    }
}

void spi_write(spi_inst_t *spi, uint8_t* src, size_t len) {
    for(size_t i = 0; i < len; ++i) {
        while(!spi_is_writable(spi)) 
            tight_loop_contents();
        spi_get_hw(spi)->dr = (uint32_t)src[i];
    }
}

void handle_cmd(uint8_t* cmd_buf) {
    static bool app_mode = false;
    uint8_t cmd = cmd_buf[0] & 0b00111111;
    uint8_t chksum = cmd_buf[5];
    uint8_t resp[5];
    if(!app_mode) {
        switch(cmd) {
            case CMD0:
                printf("Handling reset CMD0\n");
                // TODO: integrity check?
                // reset always responds with 0x1 (in idle mode?)
                resp[0] = 0x01;
                spi_write(spi_default, resp, 1);
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
            case CMD55:
                printf("Handling app CMD55\n");
                app_mode = true;
                resp[0] = 0x01;
                spi_write(spi_default, resp, 1);
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