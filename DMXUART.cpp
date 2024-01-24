/*
MIT License

Copyright (c) 2024 Richard Case, Case Solved

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#include <mem.h>
#include "Arduino.h"    // for min, max, digitalWrite, etc
#include "DMXUART.h"

#ifdef ESP8266
#include "esp8266_peri.h"
#define NUM_UARTS           2
#define INTR_LOCK           ETS_UART_INTR_DISABLE()
#define INTR_UNLOCK         ETS_UART_INTR_ENABLE()
#else
//ESP32
#include "soc/uart_struct.h"

// ESP32 1.0.6 compatible
struct uart_struct_t {
    uart_dev_t * dev;
#if !CONFIG_DISABLE_HAL_LOCKS
    xSemaphoreHandle lock;
#endif
    uint8_t num;
    xQueueHandle queue;
    intr_handle_t intr_handle;
};

#define NUM_UARTS           3
#define INTR_LOCK           portENTER_CRITICAL(&_tx_spinlock)
#define INTR_UNLOCK         portEXIT_CRITICAL(&_tx_spinlock)
#define UART_TX_FIFO_SIZE   0x7f    // This is defined as 0x80 in ESP8266!!
#endif

//#define DMX_DEBUG

#ifdef DMX_DEBUG
HardwareSerial* Debug = 0;
#define DEBUG_PRINTF(x...)   if(Debug) Debug->printf(x)
#else
#define DEBUG_PRINTF(x...)
#endif

// 250kbps
#define DMXBAUD             250000UL
#define DMXFORMAT           SERIAL_8N2
#define BAUDDETECTTIMEOUT   1000UL  // default 20,000
#define TXFIFO_INTTHRES     10      // chunks of 112 bytes, 4 interrupts
#define RXFIFO_INTTHRES     112     // default 112, max 127, figure from espressif: 4 - 120
#define RX_TIMEOUT          127     // 64 -> ~2ms, 10 -> ~324us, 127 -> ~4ms
#define RXBUFFER_SIZE       dmx_channels + 4
#define MAGIC               0xfeed5ea1UL

#define dmx_state_invalid   0
#define dmx_state_ready     1
#define dmx_state_tx        2
#define dmx_state_txpending 3
#define dmx_state_rx        4


void start_isr(uint8_t uart_idx, DMXUART* ptr);

// both read and write are enabled but only half-duplex is possible
DMXUART::DMXUART(int uart_nr, uint8_t* buf, int8_t tx_pin, int8_t dir_pin, int8_t rx_pin, bool invert, bool tx_mode)
: HardwareSerial(uart_nr)
, _state(dmx_state_invalid)
, _rxbuf(nullptr)
, _new_rx(false)
, _rx_overflow_count(0)
, _magic(MAGIC)
, _extbuf(buf)
#ifndef ESP8266
, _tx_spinlock(portMUX_INITIALIZER_UNLOCKED)
#endif
{
#ifdef DMX_DEBUG
    if (uart_nr==0) {
        HardwareSerial::begin(115200, SERIAL_8N1, SerialMode::SERIAL_TX_ONLY);
        Debug = this;
        return;
    }
#endif
    HardwareSerial::end();
    if (!buf) return;
    if (tx_pin < 0 && rx_pin < 0) return;
    if (tx_mode && tx_pin < 0) return;
    if (!tx_mode && rx_pin < 0) return;

    if (dir_pin > 0) pinMode((uint8_t) dir_pin, OUTPUT);

    _state = dmx_state_ready;
    _tx_pin = tx_pin;
    _dir_pin = dir_pin;
    _rx_pin = rx_pin;
    _uart_num = uart_nr;
    DEBUG_PRINTF("\nStarted\n");

#ifdef ESP8266
    SerialMode mode = (tx_pin > 0 && rx_pin > 0) ? SerialMode::SERIAL_FULL : (tx_pin > 0) ? SerialMode::SERIAL_TX_ONLY : SerialMode::SERIAL_RX_ONLY;
    HardwareSerial::begin(DMXBAUD, DMXFORMAT, mode, tx_pin, invert);
    if (_uart_num == 0 && tx_pin == 15) swap();
    if (rx_pin > 0) _rxbuf = (uint8_t*)os_zalloc(RXBUFFER_SIZE); // space for the start byte and a few others
    start_isr(_uart_num, this);
#else
    HardwareSerial::begin(DMXBAUD, DMXFORMAT, rx_pin, tx_pin, invert, BAUDDETECTTIMEOUT, RXFIFO_INTTHRES);
    // ESP32 GPIO pin attachment is done at a lower level
    if (rx_pin > 0) setRxBufferSize(RXBUFFER_SIZE);
    start_isr(_uart_num, this);
#endif
    if (!set_mode(tx_mode)) return;
    DEBUG_PRINTF("%d: USS: 0x%08x, USIS: 0x%08x, USC0: 0x%08x, USC1: 0x%08x\n",
        _uart_num, USS(_uart_num), USIS(_uart_num), USC0(_uart_num), USC1(_uart_num));
}

// may block until TX FIFO is empty if mode is changed
bool DMXUART::set_mode(bool tx_mode) {
    // although TX & RX are requested the UART may not be capable
    if (tx_mode && !isTxEnabled()) return false;
    if (!tx_mode && !isRxEnabled()) return false;

    if (_state == dmx_state_invalid) return false;
    if (!tx_mode && _state == dmx_state_ready && _new_rx) return true; // RX ready
    if (_state == dmx_state_rx) return false; // busy
    if (_state == dmx_state_txpending) return false; // busy
    if (tx_mode && _state == dmx_state_tx) return true;

    // we know we are changing direction now
    if (_state == dmx_state_tx) { flush(); _state = dmx_state_ready; } // TX only flush
    else _state = dmx_state_ready;

    if (_dir_pin > 0) digitalWrite((uint8_t) _dir_pin, tx_mode ? HIGH : LOW);
#ifdef ESP8266
    //uart_flush(_uart); // resets the RX FIFO too
#else
    // ESP32 driver has absolutely no error checking
    //flush(false); // empties RX too
#endif
    return true;
}

#ifdef ESP8266
#define INTR(uart_nr, bit)   USIS(uart_nr) & (1 << bit) ? USIC(uart_nr) = (1 << bit), true : false

void IRAM_ATTR esp8266_isr(void* arg, void* frame) {
    (void) frame;
    DMXUART* *dmxs = (DMXUART**)arg;
    int loops = 1;

    if (!arg) {
        ETS_UART_INTR_DISABLE();
        return;
    }

    // service all uarts
    while (loops--) {
        for (int uart_fired = 0; uart_fired < NUM_UARTS; uart_fired++) {
            DMXUART* dmx = nullptr;

            for (int idx = 0; idx < NUM_UARTS; idx++) {
                DMXUART* tmp = dmxs[idx];
                if (tmp && tmp->_magic == MAGIC && tmp->_uart_num == uart_fired) {
                    dmx = tmp;
                    break;
                }
            }
            if (!dmx) {
                USIC(uart_fired) = 0xFFFF;
                continue;
            }

            // RX
            if (dmx->_state != dmx_state_txpending) {
                // a break or timeout signifies an RX frame start/end depending on the state
                bool _break = INTR(uart_fired, UIBD);
                bool _timeout = INTR(uart_fired, UITO);
                if ((_break || _timeout) && !dmx->_new_rx) { 
                    // When we receive a break, reset the FIFO and go to RX mode
                    if (dmx->_state == dmx_state_ready && _break) {
                        uint32_t mask = (1 << UCRXRST);
                        USC0(uart_fired) |= mask;
                        USC0(uart_fired) &= ~mask;
                        dmx->_chan = dmx->_rxbuf; // mem clear is done in userspace
                        dmx->_remaining = RXBUFFER_SIZE;
                        dmx->_state = dmx_state_rx; // ready to receive
                        //USIC(uart_fired) = (1 << UITO);
                        //USC1(uart_fired) |= (1 << UCTOE);
                        USIE(uart_fired) |= (1 << UITO); // enable timeout when a new frame is started
                    // If we receive a timeout or break whilst receiving, end the frame
                    } else if (dmx->_state == dmx_state_rx && (_break || _timeout)) {
                        USIE(uart_fired) &= ~(1 << UITO); // disable timeout when a new frame is ended
                        dmx->_new_rx = true; // the only place this happens
                        dmx->_state = dmx_state_ready; // transfer to idle
                        //USC1(uart_fired) &= ~(1 << UCTOE);
                        //USIE(uart_fired) &= ~(1 << UIBD);
                    }
                    //continue;
                }

                //if (INTR(uart_fired, UIFF)) {
                    while ((USS(uart_fired) >> USRXC) & 0xFF) { // RX FIFO count; equivalent to FIFO reset
                        uint8_t data = USF(uart_fired); // empty the RX buffer no matter what
                        if (dmx->_remaining && dmx->_state == dmx_state_rx) { // userspace must read the new RX frame before overwriting
                            *dmx->_chan++ = data;
                            dmx->_remaining--;
                        }
                    }
                    INTR(uart_fired, UIFF);
                    //}

                if (INTR(uart_fired, UIOF) || INTR(uart_fired, UIFR) || INTR(uart_fired, UIPE)) { // reset buffer on error
                    uint32_t mask = (1 << UCRXRST);
                    USC0(uart_fired) |= mask;
                    USC0(uart_fired) &= ~mask;
                    //INTR(uart_fired, UIOF);
                    //INTR(uart_fired, UIFR);
                    //INTR(uart_fired, UIPE);
                    if (!dmx->_new_rx) { // keep the oldest complete frame
                        dmx->_chan = dmx->_rxbuf;
                        dmx->_remaining = RXBUFFER_SIZE;
                        //memset(dmx->_chan, 0, RXBUFFER_SIZE);
                        dmx->_rx_overflow_count++;
                        dmx->_state = dmx_state_ready; // transfer to idle
                    }
                }
            //TX
            } else { // fill TX FIFO
                if (INTR(uart_fired, UIFE)) {
                    USIE(uart_fired) &= ~(1 << UIFE); // disable
                    while (dmx->_remaining && (UART_TX_FIFO_SIZE - ((USS(uart_fired) >> USTXC) & 0xFF))) {
                        USF(uart_fired) = *dmx->_chan++;
                        dmx->_remaining--;
                    }
                    DEBUG_PRINTF("TX rem %d\n", dmx->_remaining);
                    if (dmx->_remaining == 0) dmx->_state = dmx_state_tx; // exit if buffer is empty
                    // re-enable TX FIFO empty interrupt only if necessary
                    else {
                        USIC(uart_fired) = (1 << UIFE);
                        USIE(uart_fired) |= (1 << UIFE);
                    }
                }
            }
        } // for all uarts
    } // while
}
#endif

void start_isr(uint8_t uart_idx, DMXUART* uart) {
    static DMXUART* uarts[NUM_UARTS] = {0}; // TODO: clear these pointers in end

    if (uart_idx < NUM_UARTS && uart) uarts[uart_idx] = uart;
    else return;

#ifdef ESP8266
    ETS_UART_INTR_DISABLE();
    // UCTOT: 7bit, number of byte duration?
    USC1(uart->_uart_num) = (RXFIFO_INTTHRES << UCFFT) | (RX_TIMEOUT << UCTOT) | (TXFIFO_INTTHRES << UCFET) | (1 << UCTOE);
    USIC(uart->_uart_num) = 0xFFFF; // clear interrupts
    // TX FIFO empty interrupts (UIFE) are always generated and must be switched on/off as required
    // RX Timeout interrupts are enabled as required (UITO)
    USIE(uart->_uart_num) = ( (1 << UIFF) | (1 << UIOF) | (1 << UIFR) | (1 << UIPE) | (1 << UIBD) | (1 << UITO) );
    ETS_UART_INTR_ATTACH(esp8266_isr, uarts); // we have to deal with all uarts in 1 ISR
    ETS_UART_INTR_ENABLE();
#endif
}

// returns the number of bytes read or a negative error
// param returns a start_byte only when it is detected, -1 otherwise
int DMXUART::read(int* start_byte) {
    int result = 0;
    if (start_byte) *start_byte = -1;
    if (!set_mode(false)) return -1;

    if (_new_rx) {
        result = _chan - _rxbuf - 2;
        if (result > 0) {
            if (start_byte) *start_byte = _rxbuf[1]; // ignore the first zero caused by the break
            memcpy(_extbuf, &_rxbuf[2], result);
        } else result = 0;
        _chan = _rxbuf;
        _remaining = RXBUFFER_SIZE;
        memset(_rxbuf, 0, RXBUFFER_SIZE);
        _new_rx = false; // the only place this happens
        USIC(_uart_num) = 0xFFFF; // clear the interrupt status
        USIE(_uart_num) |= (1 << UIBD); // enable RX break detection
    }

    return result;
}

// We don't double buffer the data assuming nothing can write to the external buffer
// ISR empties the TX buffer
size_t DMXUART::write(size_t chans, uint8_t start_byte) {
    // short circuit tx polling
    if (chans == 0) return 0;
    if (!set_mode(true)) return 0;
    if (chans > dmx_channels) chans = dmx_channels;
    size_t fifo_free = availableForWrite();

    switch(_state) {
        case dmx_state_tx:
            // If FIFO not empty ? discard & exit : ready
            if (UART_TX_FIFO_SIZE != fifo_free) break;
            else _state = dmx_state_ready;
        case dmx_state_ready: // fall through to start another frame
            _remaining = max(chans, UART_MINCHANS_DMX);
            _chan = _extbuf;
            if (_remaining == 0) break;
            DEBUG_PRINTF("TX rem %d\n", _remaining);
            start_frame(start_byte);
            DEBUG_PRINTF("TX rem %d\n", _remaining);
    }
    return _chan - _extbuf;
}

#ifdef ESP8266
void DMXUART::start_frame(uint8_t start_byte) {
    uint32_t _break = (1 << UCBRK);

    // Allow break to be interrupted for loopback RX break detect
    // send break of 100us
    USC0(_uart_num) |= _break;
    delayMicroseconds(100);
    USC0(_uart_num) &= ~_break;
    // wait for >12us mark after break
    delayMicroseconds(12);

    // keep fifo fill together with no interrupts
    ETS_UART_INTR_DISABLE();

    // send start byte & data
    USF(_uart_num) = start_byte;
    // don't account for the start byte
    size_t fifo_free = UART_TX_FIFO_SIZE - ((USS(_uart_num) >> USTXC) & 0xFF);
    while (_remaining && fifo_free) {
        USF(_uart_num) = *_chan++;
        _remaining--;
        fifo_free--;
    }
    _state = _remaining ? dmx_state_txpending : dmx_state_tx;
    if (_state == dmx_state_txpending) {
        USIC(_uart_num) = 0xFFFF; // clear the interrupt status
        USIE(_uart_num) |= (1 << UIFE); // enable TX FIFO empty interrupts
    }
    ETS_UART_INTR_ENABLE();
}
#else
void DMXUART::start_frame(uint8_t start_byte) {
    // ESP32 has special rs485 features which are worth looking into: uart_struct.h
    // reset after previous send
    _uart->dev->conf0.txd_brk = 0; // reset break
    _uart->dev->idle_conf.tx_idle_num = 10; // 10bit 12us between transfers

    // send break of 100us
    _uart->dev->idle_conf.tx_brk_num = 128; // 8bit break length
    _uart->dev->conf0.txd_brk = 1; // enable break
    //delayMicroseconds(100);
    //_uart->dev->conf0.txd_brk = 0;
    // wait for >12us mark after break
    //delayMicroseconds(12);

    // send start byte
    HardwareSerial::write(start_byte);
}
#endif

void DMXUART::end() {
    HardwareSerial::end();
    ETS_UART_INTR_DISABLE();
    ETS_UART_INTR_ATTACH(NULL, NULL);
#ifdef ESP8266
    uart_flush(_uart); // resets the RX FIFO too
#else
    // ESP32 driver has absolutely no error checking
    flush(false); // empties RX too
#endif
    _state = dmx_state_invalid;
    _extbuf = nullptr;
    if(_rxbuf) free(_rxbuf);
    _rxbuf = nullptr;
}