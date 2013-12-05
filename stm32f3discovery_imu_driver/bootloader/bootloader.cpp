#include <libopencm3/stm32/f3/rcc.h>
#include <libopencm3/stm32/f3/usart.h>
#include <libopencm3/stm32/gpio.h>

#include <arm_bootloader/handler.cpp>
#include <arm_bootloader/uniqueid.h>


class UARTSink : public uf_subbus_protocol::ISink {
public:
  void handleStart() {
  }
  void handleByte(uint8_t byte) {
    usart_send_blocking(USART2, byte);
  }
  void handleEnd() {
    // make sure write finishes
    usart_send_blocking(USART2, 0);
    usart_wait_send_ready(USART2);
  }
};

void usart_setup(void) {
  /* Enable clocks for GPIO port A (for GPIO_USART2_TX) and USART2. */
  rcc_peripheral_enable_clock(&RCC_APB1ENR, RCC_APB1ENR_USART2EN);
  rcc_peripheral_enable_clock(&RCC_AHBENR, RCC_AHBENR_IOPAEN);

  /* Setup GPIO pin GPIO_USART2_TX/GPIO9 on GPIO port A for transmit. */
  gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO2 | GPIO3);
  gpio_set_af(GPIOA, GPIO_AF7, GPIO2| GPIO3);

  /* Setup UART parameters. */
  usart_set_baudrate(USART2, 115200);
  usart_set_databits(USART2, 8);
  usart_set_stopbits(USART2, USART_STOPBITS_1);
  usart_set_mode(USART2, USART_MODE_TX_RX);
  usart_set_parity(USART2, USART_PARITY_NONE);
  usart_set_flow_control(USART2, USART_FLOWCONTROL_NONE);

  /* Finally enable the USART. */
  usart_enable(USART2);
}

uint8_t read_byte() {
  usart_wait_recv_ready(USART2);
  return usart_recv(USART2);
}

int main() {
  rcc_clock_setup_hsi(&hsi_8mhz[CLOCK_64MHZ]);
  usart_setup();

  UARTSink uartsink;

  arm_bootloader::Handler handler(
    uartsink,
    arm_bootloader::get_unique_dest());
  while(true) {
    handler.handleByte(read_byte());
  }
}
