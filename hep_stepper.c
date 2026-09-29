#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "core1_main.h"

#include "common.h"
#include "tmc2130.h"
#include "stepper_controller.h"
#include "servo_control.h"

#define DRV_EN 11   // driver enable pin (active low)

// Timers
struct repeating_timer servo_timer;

stepper_t * volatile stepper;
bool direction = true;

// Helper: write a 32-bit value to a TMC2130 register.
static void write_reg (trinamic_motor_t motor, tmc2130_regaddr_t reg, uint32_t value)
{
    TMC_spi_datagram_t datagram = {0};
    datagram.addr.idx = reg;
    datagram.payload.value = value;
    tmc_spi_write(motor, &datagram);
}

// Helper: read a 32-bit value from a TMC2130 register.
static uint32_t read_reg (trinamic_motor_t motor, tmc2130_regaddr_t reg)
{
    TMC_spi_datagram_t datagram = {0};
    datagram.addr.idx = reg;
    tmc_spi_read(motor, &datagram);

    return datagram.payload.value;
}

bool servo_timer_callback(struct repeating_timer *t) {
    stepper_compute(stepper);
    return true;
}

int main() {
    stdio_init_all();
    stepper = stepper_init(9, 0, 200.0, 256.0, 50.0, 100.0);
    // Timer for servo control
    add_repeating_timer_ms(-1, servo_timer_callback, NULL, &servo_timer);

    // Pass stepper pointer to Core 1
    multicore_launch_core1(core1_main);

    sleep_ms(2000);

    printf("SPI master pre TMC2130 startuje...\n");

    gpio_init(DRV_EN);
    gpio_set_dir(DRV_EN, GPIO_OUT);
    gpio_put(DRV_EN, 0);

    trinamic_motor_t motor = {0};

    // Reset GSTAT
    write_reg(motor, TMC2130Reg_GSTAT, 0x07);
    sleep_ms(10);

    // Set current (IHOLD=6, IRUN=31, IHOLDDELAY=15)
    write_reg(motor, TMC2130Reg_IHOLD_IRUN, 0x000F1006);
    sleep_ms(10);

    // Chopperer enable (TOFF=3)
    write_reg(motor, TMC2130Reg_CHOPCONF, 0x000100C3);
    sleep_ms(10);

    while (true) {
        // printf("--- New cycle ---\n");

        // TMC2130_ioin_reg_t ioin = { .value = read_reg(motor, TMC2130Reg_IOIN) };

        // printf("IOIN = 0x%08lx\n", ioin.value);
        // printf("  STEP      = %lu\n", ioin.step);
        // printf("  DIR       = %lu\n", ioin.dir);
        // printf("  DCEN_CFG4 = %lu\n", ioin.dcen_cfg4);
        // printf("  DCEN_CFG5 = %lu\n", ioin.dcen_cfg5);
        // printf("  DRV_ENN   = %lu\n", ioin.drv_enn_cfg6);
        // printf("  DCO       = %lu\n", ioin.dco);
        // printf("  version   = 0x%02lx\n", ioin.version);

        // if(ioin.version != 0x11)
        //     printf("  Warning: Unexpected version (COMM error?)\n");

        // TMC2130_drv_status_reg_t drv = { .value = read_reg(motor, TMC2130Reg_DRV_STATUS) };

        // printf("DRV_STATUS = 0x%08lx\n", drv.value);
        // printf("  SG_RESULT (load) = %lu\n", drv.sg_result);
        // printf("  CS_ACTUAL (prud) = %lu/31\n", drv.cs_actual);
        // printf("  standstill = %lu\n", drv.stst);
        // if(drv.ot)   printf("  ERROR: Overtemperature (OT)\n");
        // if(drv.otpw) printf("  WARNING: Temperature (OTPW)\n");
        // if(drv.s2ga || drv.s2gb) printf("  ERROR: Short circuit\n");
        // if(drv.ola || drv.olb)   printf("  WARNING: Open load\n");

        // uint32_t lost = read_reg(motor, TMC2130Reg_LOST_STEPS);

        // if (lost != 0) {
        //     printf("ERROR: DcStep lost steps = %lu\n", lost);
        // }

        if (stepper_is_standstill(stepper)) {
            if (direction) {
                stepper_goto(stepper, 20.0, 5.0);
                direction = false;
            } else {
                stepper_goto(stepper, 0.0, 20.0);
                direction = true;
            }
        }
        sleep_ms(500);
    }
}