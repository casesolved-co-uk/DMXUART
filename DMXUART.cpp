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

#include "Arduino.h"    // for min, max, digitalWrite, etc
#include "DMXUART.h"

#ifdef ESP8266
#include "esp8266_peri.h"
#define INTR_LOCK           ETS_UART_INTR_DISABLE()
#define INTR_UNLOCK         ETS_UART_INTR_ENABLE()
#else
//ESP32
#include "esp32-hal-uart.h"
#include "soc/uart_struct.h"

struct uart_struct_t {
    uart_dev_t * dev;
#if !CONFIG_DISABLE_HAL_LOCKS
    xSemaphoreHandle lock;
#endif
    uint8_t num;
    xQueueHandle queue;
    intr_handle_t intr_handle;
};

#define INTR_LOCK           portENTER_CRITICAL(&_tx_spinlock)
#define INTR_UNLOCK         portEXIT_CRITICAL(&_tx_spinlock)
#define UART_TX_FIFO_SIZE   0x7f    // This is defined as 0x80 in ESP8266!!
#endif

// 250kbps
#define DMXBAUD             250000UL
#define DMXFORMAT           SERIAL_8N2
#define BAUDDETECTTIMEOUT   1000UL  // default 20,000
#define RXFIFO_INTTHRES     120     // default 112, max 127

#define dmx_state_invalid   0
#define dmx_state_ready     1
#define dmx_state_tx        2
#define dmx_state_txpending 3
#define dmx_state_rx        4


// both read and write are enabled but only half-duplex is possible
DMXUART::DMXUART(int uart_nr, uint8_t* buf, int8_t tx_pin, int8_t dir_pin, int8_t rx_pin, bool invert, bool tx_mode)
: HardwareSerial(uart_nr)
, _state(dmx_state_invalid)
, _extbuf(buf)
#ifndef ESP8266
, _tx_spinlock(portMUX_INITIALIZER_UNLOCKED)
#endif
{
    HardwareSerial::end();
    if (!buf) return;
    if (tx_pin < 0 && rx_pin < 0) return;
    if (tx_mode && tx_pin < 0) return;
    if (!tx_mode && rx_pin < 0) return;

#ifdef ESP8266
    SerialMode mode = (tx_pin > 0 && rx_pin > 0) ? SerialMode::SERIAL_FULL : (tx_pin > 0) ? SerialMode::SERIAL_TX_ONLY : SerialMode::SERIAL_RX_ONLY;
    HardwareSerial::begin(DMXBAUD, DMXFORMAT, mode, tx_pin, invert);
    if (uart_nr == 0 && tx_pin == 15) swap();
#else
    HardwareSerial::begin(DMXBAUD, DMXFORMAT, rx_pin, tx_pin, invert, BAUDDETECTTIMEOUT, RXFIFO_INTTHRES);
    // ESP32 GPIO pin attachment is done at a lower level
#endif

    _state = dmx_state_ready;
    _tx_pin = tx_pin;
    _dir_pin = dir_pin;
    _rx_pin = rx_pin;

    if (rx_pin > 0) setRxBufferSize(dmx_channels + 4); // space for the start byte and a few others; 256 by default
    if (dir_pin > 0) pinMode((uint8_t) dir_pin, OUTPUT);
    set_mode(tx_mode);
}

// may block until FIFO is empty if mode is changed
bool DMXUART::set_mode(bool tx_mode) {
    // although TX & RX are requested the UART may not be capable
    if (tx_mode && !isTxEnabled()) return false;
    if (!tx_mode && !isRxEnabled()) return false;

    if (_state == dmx_state_invalid) return false;
    if (!tx_mode && _state == dmx_state_rx) return true;
    if (tx_mode && (_state == dmx_state_tx || _state == dmx_state_txpending)) return true;
    if (_state == dmx_state_txpending) return false;

    // we know we are changing direction now
    if (_state == dmx_state_tx) { flush(); _state = dmx_state_ready; } // TX only flush
    if (_state == dmx_state_rx && available()) return false; // RX available
    else _state = dmx_state_ready;

    if (_dir_pin > 0) digitalWrite((uint8_t) _dir_pin, tx_mode ? HIGH : LOW);
#ifdef ESP8266
    // clear any errors
    hasOverrun();
    hasRxError();
    uart_flush(_uart); // resets the RX FIFO too
#else
    // ESP32 driver has absolutely no error checking
    flush(false); // empties RX too
#endif
    return true;
}

// returns the number of bytes read or a negative error
// param returns a start_byte only when it is detected, -1 otherwise
int DMXUART::read(int* start_byte) {
    if (!set_mode(false)) return -1;
    size_t result = 0;
    uint8_t* chan = _extbuf;
    size_t remaining = dmx_channels;
    int attempts = 6;
    int temp;

    _state = dmx_state_rx;
    if (start_byte) *start_byte = -1;

    // if the buffer has overrun we discard the frame and clear any errors
#ifdef ESP8266
    if (hasOverrun()) { hasRxError(); uart_flush(_uart); return -2; }
    hasRxError(); // RxError is never set
    // we just have to hope for the best on ESP32(!)
#endif

    // We cannot use break detect because ESP8266 HardwareSerial has an ISR that clears it without saving
    // ESP32 driver support is even more sparse, but we do have access to the full set of registers
    // Monitor data for 60us and if no change assume start/end of frame
    // Assumes a frame is contiguous on the wire; according to ChatGPT(!) this is correct

    temp = HardwareSerial::read(); // We get an extra zero for the break, discard
    if (temp == 0) {
        temp = -1;
        do {
            if (temp == -1) { // wait for start byte
                temp = HardwareSerial::read(); // returns -1 if no data
                attempts--;
            } else {
                if (start_byte) *start_byte = temp;
                result = HardwareSerial::read((char*)chan, remaining);
                if (result == 0) {
                    attempts--;
                } else {
                    attempts = 6;
                    chan += result;
                    remaining -= result;
                }
            }
            delayMicroseconds(10);
        } while (attempts && remaining > 0);
    }

    return chan - _extbuf;
}

// Will only block until all data has been written to the 128 byte FIFO
// We don't double buffer the data assuming nothing can write to the external buffer
// exits in dmx_state_tx
bool DMXUART::write(size_t chans, uint8_t start_byte) {
    if (chans > dmx_channels) return false;
    if (!set_mode(true)) return false;
    size_t tx_size;
    size_t fifo_free;
    uint8_t* chan = _extbuf;
    size_t remaining = dmx_channels;
    do {
        fifo_free = availableForWrite();
        switch(_state) {
            case dmx_state_txpending:
                tx_size = min(fifo_free, remaining);
                write_buf(chan, tx_size);
                chan += tx_size;
                remaining -= tx_size;
                if (remaining == 0) _state = dmx_state_tx; // exit if buffer is empty
                // a full buffer will take >5ms to empty at 250kbaud
                else delay(5); // ms; will yield; has a gap at 6ms
                break;
            case dmx_state_tx:
                // If FIFO not empty ? discard & exit : ready
                if (UART_TX_FIFO_SIZE != fifo_free) break;
                else _state = dmx_state_ready;
                // fall through to start another frame
            case dmx_state_ready:
                tx_size = max(chans, UART_MINCHANS_DMX);
                if (tx_size == 0) break;
                remaining = tx_size;
                chan = _extbuf;

                tx_size = min(fifo_free, tx_size);
                INTR_LOCK;
                start_frame(start_byte); // keep initial break and fifo fill together with no interrupts
                write_buf(chan, tx_size);
                INTR_UNLOCK;

                chan += tx_size;
                remaining -= tx_size;
                _state = remaining ? dmx_state_txpending : dmx_state_tx;
        }
    } while (_state == dmx_state_txpending);
    return true;
}

void DMXUART::write_buf(uint8_t* buf, size_t size) {
    while (size--) {
        // avoid optimistic_yield(10000UL); on ESP8266 by writing each byte
        HardwareSerial::write(*buf++);
    }
}

#ifdef ESP8266
void DMXUART::start_frame(uint8_t start_byte) {
    uint32_t _break = (1 << UCBRK);

    // send break of 100us
    USC0(_uart_nr) |= _break;
    delayMicroseconds(100);
    USC0(_uart_nr) &= ~_break;
    // wait for >12us mark after break
    delayMicroseconds(12);

    // send start byte
    HardwareSerial::write(start_byte);
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
    _state = dmx_state_invalid;
    _extbuf = nullptr;
}