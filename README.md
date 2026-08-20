# DIY-OBD2-Scanner

A DIY OBD2 scanner built from scratch on an STM32F103C8T6 ("Blue Pill") — reads and clears DTCs, displays live vehicle data, and checks emissions monitor readiness by talking directly to a vehicle's PCM over CAN bus.

No commercial OBD2 chipset (ELM327 or similar) is used — this implements the CAN transaction, OBD2 request/response handling, and ISO-TP multi-frame reassembly (for DTC reads with more than 3 stored codes) directly against the STM32's onboard CAN peripheral.

## Features

- **DTC Scan** — reads stored diagnostic trouble codes (Mode 03), including full ISO-TP multi-frame reassembly with flow control, so it's not limited to the 3 DTCs that fit in a single CAN frame
- **Clear DTCs** — clears stored codes (Mode 04)
- **Vehicle Data** — auto-discovers which PIDs the connected ECU actually supports (Mode 01, PIDs 00/20/40 bitmask queries), then live-polls and displays them (RPM, coolant temp, speed, fuel trims, O2 sensor voltages, and more — see the PID table in the source)
- **Emissions Monitors** — reads continuous and non-continuous monitor readiness status (spark-ignition vehicles)

## Hardware

| Component | Role |
|---|---|
| STM32F103C8T6 ("Blue Pill") | Main microcontroller |
| SN65HVD230 | CAN transceiver — converts logic-level TX/RX to differential CAN_H/CAN_L |
| SSD1306 128x64 OLED (I2C) | Display |
| KY-040 rotary encoder | Menu navigation |
| Buck converter (e.g. MP1584) | Steps vehicle 12V down to logic level |
| OBD2 connector / pigtail | Vehicle interface |

## Wiring

### OBD2 port pinout

| Pin | Signal |
|---|---|
| Pin 4 | Chassis ground |
| Pin 5 | Signal ground |
| Pin 6 | CAN-H (high speed CAN) |
| Pin 14 | CAN-L (high speed CAN) |
| Pin 16 | Battery voltage (12V constant) |

### Blue Pill pinout

| Function | Pin |
|---|---|
| Encoder CLK | PA8 |
| Encoder DT | PB15 |
| Encoder SW | PB14 |
| Display SCL | PB6 |
| Display SDA | PB7 |
| CAN RX | PA11 |
| CAN TX | PA12 |

CAN RX/TX use the STM32F103's default CAN1 pin mapping (PA11/PA12). PB8/PB9 is an alternate remap option on this chip but is **not** what this build uses.

Pin 16 (12V) feeds the buck converter, which steps down to logic level before reaching the Blue Pill and SN65HVD230. CAN-H/CAN-L from the OBD2 port go to the SN65HVD230's bus side; the transceiver's logic-side D/R pins connect to the Blue Pill's CAN TX/RX (PA12/PA11).

## Software / build requirements

- [Arduino IDE](https://www.arduino.cc/en/software)
- [STM32duino core](https://github.com/stm32duino/Arduino_Core_STM32) (STMicroelectronics' official Arduino board package)
- [STM32_CAN library](https://github.com/pazi88/STM32_CAN) by pazi88
- [U8g2lib](https://github.com/olikraus/u8g2) for the SSD1306 display
- A `hal_conf_extra.h` file in the sketch folder containing `#define HAL_CAN_MODULE_ENABLED` — required to enable the CAN peripheral in the STM32 HAL layer, since it isn't on by default in the STM32duino core

Board setting in Arduino IDE: Generic STM32F1 series, Blue Pill F103C8 variant.

### Programming

This build uses a CP2102 USB-to-UART adapter to flash the Blue Pill via its built-in serial bootloader (no ST-Link required).

- **BOOT0 jumper in position 1** — puts the STM32 into system bootloader mode for flashing. Set the board to this position, upload from Arduino IDE, then reset the board.
- **BOOT0 jumper in position 0** — normal boot from flash, runs the program. Move it back here after flashing to actually run the code.

## A note on AI use

This project was built with heavy use of AI assistance (Claude) for code generation, debugging, and working through the CAN bus / ISO-TP protocol details. All hardware design, wiring, testing, and integration were done by hand on the bench. Wanted to be upfront about that rather than have it look like a from-scratch solo effort in the traditional sense — the understanding is real, but the code didn't come from a blank editor.

## Status

Functional v1, tested against a real vehicle and against a second Blue Pill/SN65HVD230 rig built specifically to simulate multi-frame ISO-TP responses on the bench.

Code is a personal project, built iteratively — functional and reasonably commented, not polished to a library-grade standard. Contributions, forks, and questions are welcome.

## Known limitations

- Spark-ignition (gasoline) monitor decoding only — compression-ignition (diesel) uses a different monitor set not implemented here
- PID table covers ~35 commonly-supported PIDs with linear scaling formulas; bitmask/status-type PIDs (fuel system status, O2 sensor presence maps, etc.) are intentionally not decoded
- No ABS/manufacturer-specific module access — standard OBD2 (SAE J1979) only

## License

MIT — use it however you want.
