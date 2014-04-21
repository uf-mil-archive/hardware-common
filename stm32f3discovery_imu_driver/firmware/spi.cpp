#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/spi.h>
#include <libopencm3/stm32/gpio.h>

#include "spi.h"

#define GYR_RNW            (1 << 7) /* Write when zero */
#define GYR_MNS            (1 << 6) /* Multiple reads when 1 */
#define GYR_WHO_AM_I        0x0F
#define GYR_OUT_TEMP        0x26
#define GYR_STATUS_REG        0x27
#define GYR_CTRL_REG1        0x20
#define GYR_CTRL_REG1_PD    (1 << 3)
#define GYR_CTRL_REG1_XEN    (1 << 1)
#define GYR_CTRL_REG1_YEN    (1 << 0)
#define GYR_CTRL_REG1_ZEN    (1 << 2)
#define GYR_CTRL_REG1_BW_SHIFT    4
#define GYR_CTRL_REG4        0x23
#define GYR_CTRL_REG4_FS_SHIFT    4

#define GYR_OUT_X_L        0x28
#define GYR_OUT_X_H        0x29

static void write(uint8_t addr, uint8_t data) {
    gpio_clear(GPIOE, GPIO3);
    spi_send8(SPI1, GYR_MNS | addr);
    spi_read8(SPI1);
    spi_send8(SPI1, data);
    spi_read8(SPI1);
    gpio_set(GPIOE, GPIO3);
}

static void read(uint8_t addr, uint8_t * data, uint16_t length=1) {
    gpio_clear(GPIOE, GPIO3);
    spi_send8(SPI1, GYR_RNW | GYR_MNS | addr);
    spi_read8(SPI1);
    for(uint16_t i = 0; i < length; i++) {
        spi_send8(SPI1, 0);
        *data++ = spi_read8(SPI1);
    }
    gpio_set(GPIOE, GPIO3);
}

void spi_setup(void) {
    rcc_periph_clock_enable(RCC_SPI1);
    /* For spi signal pins */
    rcc_periph_clock_enable(RCC_GPIOA);
    /* For spi mode select on the l3gd20 */
    rcc_periph_clock_enable(RCC_GPIOE);

    /* Setup GPIOE3 pin for spi mode l3gd20 select. */
    gpio_mode_setup(GPIOE, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO3);
    /* Start with spi communication disabled */
    gpio_set(GPIOE, GPIO3);

    /* Setup GPIO pins for AF5 for SPI1 signals. */
    gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE,
            GPIO5 | GPIO6 | GPIO7);
    gpio_set_af(GPIOA, GPIO_AF5, GPIO5 | GPIO6 | GPIO7);

    //spi initialization;
    spi_set_master_mode(SPI1);
    spi_set_baudrate_prescaler(SPI1, SPI_CR1_BR_FPCLK_DIV_64);
    spi_set_clock_polarity_0(SPI1);
    spi_set_clock_phase_0(SPI1);
    spi_set_full_duplex_mode(SPI1);
    spi_set_unidirectional_mode(SPI1); /* bidirectional but in 3-wire */
    spi_set_data_size(SPI1, SPI_CR2_DS_8BIT);
    spi_enable_software_slave_management(SPI1);
    spi_send_msb_first(SPI1);
    spi_set_nss_high(SPI1);
    //spi_enable_ss_output(SPI1);
    spi_fifo_reception_threshold_8bit(SPI1);
    SPI_I2SCFGR(SPI1) &= ~SPI_I2SCFGR_I2SMOD;
    spi_enable(SPI1);

    write(GYR_CTRL_REG1, GYR_CTRL_REG1_PD | GYR_CTRL_REG1_XEN |
        GYR_CTRL_REG1_YEN | GYR_CTRL_REG1_ZEN |
        (3 << GYR_CTRL_REG1_BW_SHIFT));
    write(GYR_CTRL_REG4, (1 << GYR_CTRL_REG4_FS_SHIFT));
}

static double const pi = 3.14159265358979323846;

void spi_read_gyro(double * result) {
    uint8_t temp;

    read(GYR_WHO_AM_I, &temp);
    read(GYR_STATUS_REG, &temp);
    read(GYR_OUT_TEMP, &temp);
    
    uint8_t data[6];
    read(GYR_OUT_X_L, data, 6);
    
    for(int i = 0; i < 3; i++) {
        result[i] = static_cast<int16_t>((data[2*i+1] << 8) | data[2*i]) * (17.50e-3 * pi/180);
    }
}
