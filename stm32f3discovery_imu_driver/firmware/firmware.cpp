#include <stdlib.h>
#include <libopencm3/cm3/scb.h>
#include <libopencm3/stm32/f3/rcc.h>
#include <libopencm3/stm32/f3/gpio.h>
#include <libopencm3/usb/usbd.h>
#include <libopencm3/usb/cdc.h>

#include <uf_subbus_protocol/protocol.h>
#include <arm_bootloader/uniqueid.h>
#include <uf_subbus_protocol/usb.h>

#include <stm32f3discovery_imu_driver/protocol.h>

#include "i2c.h"
#include "spi.h"

using namespace stm32f3discovery_imu_driver;


class Handler {
  class GotMessageFunctor {
    Handler &handler;
  public:
    GotMessageFunctor(Handler &handler) :
      handler(handler) {
    }
    void operator()(const Command &command) {
      handler.handle_message(command);
    }
  };
  
  GotMessageFunctor gmf;
  uf_subbus_protocol::SimpleReceiver<Command, GotMessageFunctor> receiver;
  uf_subbus_protocol::SimpleSender<Response, uf_subbus_protocol::ISink> sender;
  arm_bootloader::Dest dest;

public:
  Handler(uf_subbus_protocol::ISink &sink, arm_bootloader::Dest dest) :
    gmf(*this), receiver(gmf), sender(sink), dest(dest) {
  }
  
  void handleByte(uint8_t byte) {
    receiver.handleRawByte(byte);
  }
  
  void handle_message(const Command &msg) {
    if(msg.dest != dest) return;
    
    Response resp; memset(&resp, 0, sizeof(resp));
    resp.id = msg.id;
    
    switch(msg.command) {
    
      case CommandID::Reset: {
        // action happens later, after response is sent
      } break;
    
      case CommandID::GetStatus: {
        resp.resp.GetStatus.magic = GetStatusResponse::MAGIC_VALUE;
      } break;
      
      case CommandID::GetIMUData: {
        i2c_read_imu(resp.resp.GetIMUData);
        spi_read_gyro(resp.resp.GetIMUData.angular_velocity);
      } break;

      default: {
        return; // send nothing back if command is invalid
      } break;

    }
    
    if(resp.id) {
      sender.write_object(resp);
    }

    switch(msg.command) {
      case CommandID::Reset: {
        scb_reset_system();
      } break;
      
      default: {
      } break;
    }
  }
};

static Handler * handlerp = NULL;

static void cdcacm_data_rx_cb(usbd_device *usbd_dev, uint8_t ep) {
  (void)ep;
  (void)usbd_dev;

  char buf[64];
  int len = usbd_ep_read_packet(usbd_dev, 0x01, buf, 64);
  
  if(!handlerp) return;
  
  for(int i = 0; i < len; i++) {
    handlerp->handleByte(buf[i]);
  }
}

static void cdcacm_set_config(usbd_device *usbd_dev, uint16_t wValue) {
  (void)wValue;
  (void)usbd_dev;

  usbd_ep_setup(usbd_dev, 0x01, USB_ENDPOINT_ATTR_BULK, 64, cdcacm_data_rx_cb);
  usbd_ep_setup(usbd_dev, 0x82, USB_ENDPOINT_ATTR_BULK, 64, NULL);
  usbd_ep_setup(usbd_dev, 0x83, USB_ENDPOINT_ATTR_INTERRUPT, 16, NULL);

  usbd_register_control_callback(
        usbd_dev,
        USB_REQ_TYPE_CLASS | USB_REQ_TYPE_INTERFACE,
        USB_REQ_TYPE_TYPE | USB_REQ_TYPE_RECIPIENT,
        uf_subbus_protocol::usb::cdcacm_control_request);
}


int main() {
  rcc_clock_setup_hsi(&hsi_8mhz[CLOCK_48MHZ]);
  rcc_usb_prescale_1();
  rcc_peripheral_reset(&RCC_APB1RSTR, RCC_APB1RSTR_USBRST);
  rcc_peripheral_reset(&RCC_AHBRSTR, RCC_AHBRSTR_IOPARST);
  rcc_peripheral_clear_reset(&RCC_APB1RSTR, RCC_APB1RSTR_USBRST);
  rcc_peripheral_clear_reset(&RCC_AHBRSTR, RCC_AHBRSTR_IOPARST);
  rcc_peripheral_enable_clock(&RCC_APB1ENR, RCC_APB1ENR_USBEN);
  rcc_peripheral_enable_clock(&RCC_AHBENR, RCC_AHBENR_IOPAEN);
  

  gpio_mode_setup(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO11 | GPIO12);
  gpio_clear(GPIOA, GPIO11 | GPIO12);
  for (int i = 0; i < 0x800000; i++)
    __asm__ volatile("nop");
  
  
  gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO11 | GPIO12);
  gpio_set_af(GPIOA, GPIO_AF14, GPIO11| GPIO12);
  
  i2c_setup();
  spi_setup();

  arm_bootloader::Dest dest = arm_bootloader::get_unique_dest();

  char serial[10] = {0};
  {
    uint32_t x = dest;
    char hex[] = "0123456789abcdef";
    for(int i = 0; i < 8; i++) {
      serial[7-i] = hex[x & 0xF];
      x = x >> 4;
    }
  }
  
  /* Buffer to be used for control requests. */
  uint8_t usbd_control_buffer[128];
  char const *usb_strings[] = {
    "uf-mil",
    "subbus",
    serial,
  };
  usbd_device * usbd_dev = usbd_init(&stm32f103_usb_driver, &uf_subbus_protocol::usb::dev, &uf_subbus_protocol::usb::config, usb_strings, 3, usbd_control_buffer, sizeof(usbd_control_buffer));
  usbd_register_set_config_callback(usbd_dev, cdcacm_set_config);

  uf_subbus_protocol::usb::Sink sink(usbd_dev);

  Handler handler(
    sink,
    dest);
  handlerp = &handler;

  while (1)
    usbd_poll(usbd_dev);
}
