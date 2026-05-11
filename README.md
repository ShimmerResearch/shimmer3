# FW_Shimmer3

Shimmer3 firmware repository. Contains firmware projects for the [Shimmer3](https://shimmersensing.com) platform, targeting the MSP430F5437A microcontroller.

## Projects

### [LogAndStream](LogAndStream/README.md)
General-purpose, configurable application for Shimmer3 and supported daughter-cards. Supports:
- Real-time data streaming over Bluetooth
- On-board SD card logging
- **LogAndStream** variant: simultaneous logging and streaming
- **SDLog** variant: logging with optional real-time clock syncing between Shimmer units over Bluetooth

### [S3_Sleep](S3_Sleep/README.md)
Minimal firmware that puts the Shimmer3 into its lowest power-consuming state. All pins are configured correctly via `Board_init()` based on the unit's SR number. Useful as a starting point for custom firmware development.

## Repository Structure

```
shimmer3-firmware/
├── LogAndStream/               # LogAndStream & SDLog firmware project
│   ├── log-and-stream-common/  # Shared source (git submodule)
│   ├── Shimmer_Driver/         # On-board sensor and peripheral drivers
│   ├── main.c
│   ├── README.md
│   └── CHANGELOG.txt
├── S3_Sleep/                   # Low-power sleep firmware project
│   ├── shimmer3_common_source/
│   ├── main.c
│   └── README.md
├── Extras/                     # Miscellaneous tools
├── FirmwareIdentifierList.txt  # Firmware identifier registry
└── README.md
```

## Build Environment

| Setting | Value |
| ------- | ----- |
| IDE | Code Composer Studio (CCS) v12.8.1.00005 |
| Compiler | TI MSP430 Compiler v21.6.1.LTS |
| Target device | MSP430F5437A |
| Output format | EABI (ELF) |

## Getting Started

### Prerequisites
1. Install [Code Composer Studio](https://www.ti.com/tool/CCSTUDIO) v12.x or later.
2. Ensure the **TI MSP430 Compiler v21.6.1.LTS** is installed (available via *Help → Install New Software* or the CCS App Center).
3. Install the **MSP430** device support package if not already present.

### Cloning

This repository uses a git submodule. Clone with:

```bash
git clone --recurse-submodules https://github.com/ShimmerResearch/shimmer3-firmware.git
```

If you have already cloned without submodules, initialise them with:

```bash
git submodule update --init --recursive
```

### Importing into CCS
1. Open CCS and select *File → Import → Code Composer Studio → CCS Projects*.
2. Browse to the cloned repository root and select the project(s) you want to import (`LogAndStream` and/or `S3_Sleep`).
3. Build the project (*Project → Build Project* or `Ctrl+B`).

### Compiling LogAndStream vs SDLog

The same source tree produces multiple firmware images depending on preprocessor definitions. See the [LogAndStream README](LogAndStream/README.md#compiling-firmware) for the full definition table.

## Firmware Identifiers

| Firmware | Identifier |
| -------- | ---------- |
| BtStream | 1 |
| SdLog | 2 |
| LogAndStream | 3 |

See `FirmwareIdentifierList.txt` for the authoritative list.

## Changelog

See [LogAndStream/CHANGELOG.txt](LogAndStream/CHANGELOG.txt) for a history of LogAndStream and SDLog releases.

## License

Copyright © Shimmer Research, Ltd. Redistribution and use are subject to the BSD-style license found in the individual source files.
