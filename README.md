# T-TWR 2.1 Modem (4-FSK/500 Baud, or 2-FSK/600 Baud)

The [Lilygo T-TWR device (revision
2.1)](https://github.com/Xinyuan-LilyGO/T-TWR) is a programmable
walkie talkie for the UHF band (400-480MHz, 70cm). The frequency range
covers the license-free PMR bands in Europe as well as the IARU-R1
70cm radio amateur band. Note that the device can only be operated by
radio amateurs due to the device characteristics (detachable antenna,
2W HF power).

![T-TWR r2.1](t-twr-r2.1.jpg)

The C code in this repo implements a FSK modem for the T-TWR with up
to 1000 bits-per-second. Thanks to the use of LDPC forward error
correction and CRCs, error-free transmission can easily be
demonstrated for distances beyond one mile, out-of-the box.

## Features

- demonstrates 4-FSK as well as 2-FSK modulation
- uses the ESP32-S3 Sigma-Delta modulation for generating the FSK signal
- uses the ESP32-S3 hardware-based ADC sampling at 6400 Hz
- shows the use of the Fast Hartley Transform for analyzing the received audio
- demonstrates a real-time waterfall display on the serial line using ASCII ART
- variable-length data packets (up to 128 bytes)
- offers configurable Low Density Partity Check (LDPC) forward error correction (256, 512 or 1024 partity bits), resulting in code rates of 4/5, 2/3 or 1/2
- LDPC decoding is executed on the ESP32
- demonstrates the synthesis of a bit-level synchronization clock signal
- uses the CCSDS (Council of the Consultative Committee for Space Data Systems) randomizer/scrambler
- uses the 64-bit CCSDS CSMs (Codeblock Sync Marker, 2023) for framing
- uses a 12-bit CRC checksum for aditional protection
- implements a simple echo-request service for testing connectivity in the field


## Desirable Features

Future extensions may include

- BLE interface for sending and receiving packets


---
