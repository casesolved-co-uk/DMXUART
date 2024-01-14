# DMXUART library for ESP8266 and ESP32

A library to send and receive data over DMX512 which is a half-duplex protocol.
It extends the Arduino HardwareSerial class for the ESP8266 and ESP32.

**NOTE: ESP32 currently not implemented**

There are certain hardware requirements, such as:
 - an RS-485 driver capable of 250kbps
 - a UART with 2 pins for transmit and receive
 - a third pin to specify direction and associated circuitry

## License

MIT
Copyright (c) 2024 Richard Case, Case Solved
