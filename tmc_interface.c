/*
 * tmc_interface.c - SPI/UART interface dummy functions for Trinamic stepper drivers
 *
 * v0.0.1 / 2020-02-04 / (c) Io Engineering / Terje
 */

/*

Copyright (c) 2021, Terje Io
All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its contributors may
be used to endorse or promote products derived from this software without
specific prior written permission..

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include <string.h>
#include <stdio.h>

#include "pico/stdlib.h"
#include "hardware/spi.h"

#include "common.h"

// RP2040 SPI wiring for the TMC2130 (matches spi_master demo)
#define TMC_SPI_PORT    spi1
#define TMC_SPI_BAUD    (1000 * 1000)  // 1 MHz
#define TMC_PIN_MISO    12
#define TMC_PIN_CS      13
#define TMC_PIN_SCK     14
#define TMC_PIN_MOSI    15

// A TMC SPI datagram is 40 bits: 1 address byte + 4 data bytes, MSB first.
#define TMC_DATAGRAM_LEN 5

static bool spi_initialized = false;

static void tmc_spi_init (void)
{
    if(spi_initialized)
        return;

    spi_init(TMC_SPI_PORT, TMC_SPI_BAUD);

    gpio_set_function(TMC_PIN_MISO, GPIO_FUNC_SPI);
    gpio_set_function(TMC_PIN_SCK, GPIO_FUNC_SPI);
    gpio_set_function(TMC_PIN_MOSI, GPIO_FUNC_SPI);

    // TMC2130 requires SPI mode 3 (CPOL = 1, CPHA = 1), MSB first
    spi_set_format(TMC_SPI_PORT, 8, SPI_CPOL_1, SPI_CPHA_1, SPI_MSB_FIRST);

    // CS is driven manually
    gpio_init(TMC_PIN_CS);
    gpio_set_dir(TMC_PIN_CS, GPIO_OUT);
    gpio_put(TMC_PIN_CS, 1);

    spi_initialized = true;
}

// Transfer one 40-bit datagram, returning the SPI status byte clocked out first.
static TMC_spi_status_t tmc_spi_transfer (TMC_spi_datagram_t *datagram)
{
    uint8_t tx[TMC_DATAGRAM_LEN], rx[TMC_DATAGRAM_LEN];

    // Address byte followed by the 32-bit payload, MSB first.
    tx[0] = datagram->addr.value;
    tx[1] = datagram->payload.data[3];
    tx[2] = datagram->payload.data[2];
    tx[3] = datagram->payload.data[1];
    tx[4] = datagram->payload.data[0];

    gpio_put(TMC_PIN_CS, 0);
    sleep_us(1);
    spi_write_read_blocking(TMC_SPI_PORT, tx, rx, TMC_DATAGRAM_LEN);
    sleep_us(1);
    gpio_put(TMC_PIN_CS, 1);

    // Store the read-back payload (MSB first on the wire).
    datagram->payload.data[3] = rx[1];
    datagram->payload.data[2] = rx[2];
    datagram->payload.data[1] = rx[3];
    datagram->payload.data[0] = rx[4];

    return (TMC_spi_status_t)rx[0];
}

TMC_spi_status_t tmc_spi_write (trinamic_motor_t driver, TMC_spi_datagram_t *datagram)
{
    tmc_spi_init();

    datagram->addr.write = 1;

    return tmc_spi_transfer(datagram);
}

TMC_spi_status_t tmc_spi_read (trinamic_motor_t driver, TMC_spi_datagram_t *datagram)
{
    TMC_spi_status_t status;

    tmc_spi_init();

    datagram->addr.write = 0;

    // The TMC2130 returns the requested register on the transfer following
    // the read request, so the datagram is clocked twice.
    tmc_spi_transfer(datagram);
    status = tmc_spi_transfer(datagram);

    return status;
}
