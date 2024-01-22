# DMXUART library for ESP8266 and ESP32

A library to send and receive data over DMX512 which is a half-duplex protocol.
It extends the Arduino HardwareSerial class for the ESP8266 and ESP32.

NOTE: ESP32 ISR not written & untested, especially start_frame()

There are certain hardware requirements, such as:
 - an RS-485 driver capable of 250kbps for DMX
 - a UART. Transmit and receive are completely separated (but share the same UART hardware)
 - an optional third pin to specify direction and associated circuitry

## Compatibility

ESP8266: arduino_core 3.1.2, platformio 4.2.0

ESP32: https://github.com/espressif/arduino-esp32/ v1.0.6

## License

MIT
Copyright (c) 2024 Richard Case, Case Solved
