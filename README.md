# Shimmer3 Firmware — LogAndStream

Shimmer3 firmware repository. Contains firmware projects for the [Shimmer3](https://shimmersensing.com) platform, targeting the MSP430F5437A microcontroller.

## Projects

### [LogAndStream](LogAndStream_Shimmer3/README.md)
The primary, actively maintained firmware for Shimmer3 and all supported daughter-cards. LogAndStream is the recommended firmware for all new projects and supersedes the older BtStream and SDLog standalone firmwares.

> **⚠️ Deprecation notice:** **BtStream** and **SDLog** as standalone firmwares are deprecated and no longer actively developed. SDLog can still be compiled as a standalone image from this source tree, but all of its functionality has been fully integrated into LogAndStream and is configurable through the firmware's configuration settings. New projects should use LogAndStream.

Features:
- Real-time data streaming over Bluetooth (classic and BLE via RN4678)
- On-board SD card logging
- **LogAndStream** variant: simultaneous logging and streaming
- **SDLog** variant (legacy): logging with optional real-time clock syncing between Shimmer units over Bluetooth — functionality fully available in the LogAndStream variant

### [S3_Sleep](S3_Sleep/README.md)
Minimal firmware that puts the Shimmer3 into its lowest power-consuming state. All pins are configured correctly via `Board_init()` based on the unit's SR number. Useful as a starting point for custom firmware development.

## Repository Structure

```
shimmer3-firmware/
├── LogAndStream_Shimmer3/          # LogAndStream & SDLog firmware project
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
2. Browse to the cloned repository root and select the project(s) you want to import (`LogAndStream_Shimmer3` and/or `S3_Sleep`).
3. Build the project (*Project → Build Project* or `Ctrl+B`).

### Compiling LogAndStream vs SDLog (legacy)

The same source tree produces multiple firmware images depending on preprocessor definitions. See the [LogAndStream README](LogAndStream_Shimmer3/README.md#compiling-firmware) for the full definition table. For new projects, the LogAndStream variant is recommended over the standalone SDLog build.

## Firmware Identifiers

| Firmware | Identifier | Status |
| -------- | ---------- | ------ |
| BtStream | 1 | ⚠️ Deprecated |
| SdLog | 2 | ⚠️ Deprecated (standalone) — functionality integrated into LogAndStream |
| LogAndStream | 3 | ✅ Active |

See `FirmwareIdentifierList.txt` for the authoritative list.

## Changelog

See [LogAndStream_Shimmer3/CHANGELOG.txt](LogAndStream_Shimmer3/CHANGELOG.txt) for a history of LogAndStream and SDLog releases.

## License

Copyright © Shimmer Research, Ltd. Redistribution and use are subject to the BSD-style license found in the individual source files.
