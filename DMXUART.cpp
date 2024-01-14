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

// ESP DMX wrapper for HardwareSerial
// TX is lossy meaning if a frame is still in progress a new write will be ignored

#include "Arduino.h"
#include "DMXUART.h"
#ifdef ESP8266
#include "esp8266_peri.h"
#else
//ESP32
#endif

// 250kbps
#define DMXBAUD             250000UL
// CONF0/USC0: 0B00001100 | 0B00000000 | 0B00110000 = 0B00111100 = 60 = 0x3C
#define DMXFORMAT           SerialConfig::SERIAL_8N2

#define dmx_state_invalid   0
#define dmx_state_ready     1
#define dmx_state_tx        2
#define dmx_state_txpending 3
#define dmx_state_rx        4


// both read and write are enabled but only half-duplex is possible
DMXUART::DMXUART(int uart_nr, uint8_t* buf, int8_t tx_pin, int8_t dir_pin, bool invert, bool tx_mode)
: HardwareSerial(uart_nr)
, _state(dmx_state_invalid)
, _extbuf(buf)
{
    end();
    if (!buf || dir_pin < 0) return;
    _tx_pin = tx_pin;
    _dir_pin = dir_pin;
    HardwareSerial::begin(DMXBAUD, DMXFORMAT, SerialMode::SERIAL_FULL, tx_pin, invert);
    setRxBufferSize(dmx_channels + 4); // space for the start byte and a few others
    #ifdef ESP8266
    if (tx_pin==15) swap();
    #else
    #endif
    _state = dmx_state_ready;
    pinMode(_dir_pin, OUTPUT);
    set_mode(tx_mode);
}

// may block until TX FIFO is empty if mode is changed
bool DMXUART::set_mode(bool tx_mode) {
    // although TX & RX are requested the UART may not be capable
    if (tx_mode && !isTxEnabled()) return false;
    if (!tx_mode && !isRxEnabled()) return false;

    if (_state == dmx_state_invalid) return false;
    if (!tx_mode && _state == dmx_state_rx) return true;
    if (tx_mode && (_state == dmx_state_tx || _state == dmx_state_txpending)) return true;
    if (_state == dmx_state_txpending) return false;

    if (_state == dmx_state_tx) { flush(); _state = dmx_state_ready; }
    if (_state == dmx_state_rx && available()) return false;
    else _state = dmx_state_ready;

    digitalWrite(_dir_pin, tx_mode ? HIGH : LOW);
    // clear any errors
    hasOverrun();
    hasRxError();
    uart_flush(_uart);

    _tx_mode = tx_mode;
    return true;
}

// returns the number of bytes read or a negative error
// param returns a start_byte only when it is detected, -1 otherwise
int DMXUART::read(int* start_byte) {
    if (!set_mode(false)) return -1;
    size_t result;
    uint8_t* chan = _extbuf;
    size_t remaining = dmx_channels;
    int attempts = 6;

    _state = dmx_state_rx;

    // if the buffer has overrun we discard the frame and clear any errors
    if (hasOverrun()) { hasRxError(); uart_flush(_uart); return -2; }
    hasRxError(); // RxError is never set

    // We cannot use break detect because HardwareSerial has an ISR that clears it without saving
    // Monitor data for 60us and if no change assume start/end of frame

    HardwareSerial::read(); // We get an extra zero, discard
    if (start_byte) *start_byte = (int) HardwareSerial::read(); // returns -1 if no data
    do {
      result = HardwareSerial::read((char*)chan, remaining);
      if (result == 0) {
        attempts--;
      } else {
        attempts = 6;
        chan += result;
        remaining -= result;
      }
      delayMicroseconds(10);
    } while (attempts && remaining > 0);

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
                tx_size = max(chans, WLED_MINCHANS_DMX);
                if (tx_size == 0) break;
                if (fifo_free >= tx_size) {
                    _state = dmx_state_tx;
                    ETS_UART_INTR_DISABLE();
                    start_frame(start_byte); // keep initial break and fifo fill together with no interrupts
                    write_buf(_extbuf, tx_size);
                    ETS_UART_INTR_ENABLE();
                } else {
                    chan = _extbuf + fifo_free;
                    remaining = tx_size - fifo_free;
                    _state = dmx_state_txpending;
                    ETS_UART_INTR_DISABLE();
                    start_frame(start_byte);
                    write_buf(_extbuf, fifo_free);
                    ETS_UART_INTR_ENABLE();
                }
        }
    } while (_state == dmx_state_txpending);
    return true;
}

void DMXUART::write_buf(uint8_t* buf, size_t size) {
    while (size--) {
        uart_write_char(_uart, pgm_read_byte(buf++));
    }
}

void DMXUART::start_frame(uint8_t start_byte) {
    uint32_t _break = (1 << UCBRK);

    // send break of 100us
    USC0(_uart_nr) |= _break;
    delayMicroseconds(100);
    USC0(_uart_nr) &= ~_break;
    // wait for >12us mark after break
    delayMicroseconds(12);

    // send start byte
    write_buf(&start_byte, 1);
}

void DMXUART::end() {
    _state = dmx_state_invalid;
}