// For testing DMXUART using two UARTs with loopback cable
// Monitor using oscilloscope

#include <DMXUART.h>

// remove standard Serial and Serial1
#define NO_GLOBAL_INSTANCES

#define DMXA_TXPIN  1
#define DMXA_DIRPIN 5
#define DMXA_RXPIN  3
#define DMXB_TXPIN  2
#define DMXB_DIRPIN 16
#define DMXB_RXPIN  -1

DMXUART *DMXA = 0;
uint8_t RXbuf[dmx_channels] = {0};
DMXUART *DMXB = 0;
uint8_t TXbuf[dmx_channels] = {0};

void setup_DMX() {
  // DMXUART(int uart_nr, uint8_t* buf, int8_t tx_pin, int8_t dir_pin, int8_t rx_pin, bool invert, bool tx_mode);
  DMXA = new DMXUART(0, RXbuf, DMXA_TXPIN, DMXA_DIRPIN, DMXA_RXPIN, true, false);
  DMXB = new DMXUART(1, TXbuf, DMXB_TXPIN, DMXB_DIRPIN, DMXB_RXPIN, true, true);
}

void setup_data() {
  for (uint32_t i = 0; i < dmx_channels; i++) {
    TXbuf[i] = (uint8_t) i & 0xFF;
  }
}

void setup() {
  setup_data();
  setup_DMX();
  delay(1000);
  DMXB->write(dmx_channels, 0x55);
}

void loop() {
  int start_byte = -1;
  int bytes = 0;
  bytes = DMXA->read(&start_byte);
  if (bytes > 0 && start_byte >= 0) {
    memcpy(TXbuf, RXbuf, bytes);
    DMXB->write(bytes, start_byte);
  }
  delay(1);
}
