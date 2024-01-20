# DMXUART library for ESP8266 and ESP32

A library to send and receive data over DMX512 which is a half-duplex protocol.
It extends the Arduino HardwareSerial class for the ESP8266 and ESP32.

NOTE: ESP32 is implemented but untested, especially start_frame()

There are certain hardware requirements, such as:
 - an RS-485 driver capable of 250kbps for DMX
 - a UART. Transmit and receive are completely separated (but share the same UART hardware)
 - an optional third pin to specify direction and associated circuitry

## License

MIT
Copyright (c) 2024 Richard Case, Case Solved
