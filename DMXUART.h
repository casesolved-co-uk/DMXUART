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
// Use ESP8266 board/platform package version 3ish onwards for UART invert to work
// If using RX, must call at least as fast as the frames coming in to ensure no corruption
// This may not be only every 22us because DMX frames don't need to have all 512 channels sent
// Every 1ms should be enough
// We should really implement our own ISR to detect frame breaks

#ifndef DMXUART_h
#define DMXUART_h

// includes inttypes and uart headers
#include <HardwareSerial.h>

#ifndef WLED_MINCHANS_DMX
#define WLED_MINCHANS_DMX 24U  // minimum required for DMX inter-break spacing
#endif
#define dmx_channels        512U

class DMXUART: public HardwareSerial {
public:
    DMXUART(int uart_nr, uint8_t* buf, int8_t tx_pin, int8_t dir_pin, bool invert, bool tx_mode);
    ~DMXUART() { end(); }
    int read(int* start_byte);
    bool write(size_t chans, uint8_t start_byte = 0);
    bool set_mode(bool tx_mode);
private:
    void end();
    void start_frame(uint8_t start_byte);
    void write_buf(uint8_t* buf, size_t size);
    int _state;
    uint8_t _tx_pin;
    uint8_t _dir_pin;
    uint8_t* _extbuf;
    bool _tx_mode;
};

#endif