# T-TWR-Modem - howto

This directory contains the source code for a T-TWR modem
(ESP32-S3). No external dependencies have to be installed as all
required Arduino libraries are included in the ```lib``` directory.

## Howto:

The Makefile works for a UNIX environment (MacOS, in my case) and uses
the Arduino CLI (tested with v1.0.4).

```
% make compile  # generates the firmware image, to be found in the build directory
% make flash    # automatically detects the serial ports of the TWRs, on Mac
```
