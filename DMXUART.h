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

/*
 * ESP DMX wrapper for HardwareSerial
 * TX is lossy meaning if a frame is still in progress a new write will be ignored
 * direction pin (if provided) is HIGH for transmit and LOW for receive
 * Use ESP8266 board/platform package version 3ish onwards for UART invert to work
 * If using RX, must call at least as fast as the frames coming in to ensure no loss
 * This may not be only every 22ms because DMX frames don't need to have all 512 channels sent
 * Now copes with receive of non-contiguous frames with up to 4ms gap.
 * This does mean there is a 4ms delay on all received frames before they are made available.
 * Set a pin to -1 to not use it
 * We're going to assume you don't want to use GPIO pin zero since it is involved with booting
 */

#ifndef DMXUART_h
#define DMXUART_h

// must set this for ESP32 otherwise RX FIFO bytes when read() is called will not be returned
#define UART_READ_RX_FIFO

// includes inttypes and uart headers
#include <HardwareSerial.h>

#ifndef UART_MINCHANS_DMX
#define UART_MINCHANS_DMX   24U  // minimum required for DMX inter-break spacing
#endif
#define dmx_channels        512U

class DMXUART: public HardwareSerial {
public:
    DMXUART(int uart_nr, uint8_t* buf, int8_t tx_pin, int8_t dir_pin, int8_t rx_pin, bool invert, bool tx_mode);
    ~DMXUART() { end(); }
    int read(int* start_byte);
    size_t write(size_t chans, uint8_t start_byte = 0); // must poll with chans=0 to empty the tx buffer
    bool set_mode(bool tx_mode);
    bool isTxEnabled() { return _tx_pin > 0 ? true : false; }
    bool isRxEnabled() { return _rx_pin > 0 ? true : false; }
    // these are made public for ISR access
    int _state;
    uint8_t* _rxbuf;
    uint8_t* _chan;
    size_t _remaining;
    int _uart_num;
    // isr variables
    bool _new_rx;
    uint32_t _rx_overflow_count;
    uint32_t _magic;
private:
    void end();
    void start_frame(uint8_t start_byte);
    int8_t _tx_pin;
    int8_t _dir_pin;
    int8_t _rx_pin;
    uint8_t* _extbuf;
#ifndef ESP8266
    portMUX_TYPE _tx_spinlock;
#endif
};

#endif