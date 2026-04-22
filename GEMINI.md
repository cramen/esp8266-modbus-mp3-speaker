# Gemini Context: ESP8266 Modbus MP3 Speaker

This project implements a Modbus RTU Slave on an ESP8266 to control a DFPlayer Mini MP3 player. It is designed for industrial or home automation systems where audio feedback is required via a standard RS-485 bus.

## Project Overview

- **Core Hardware:** ESP8266 (NodeMCU), DFPlayer Mini, MAX485 (RS-485 Transceiver).
- **Communication:** Modbus RTU over Hardware Serial (UART0).
- **Audio Control:** Managed via `DFPlayerMini_Fast` library over SoftwareSerial.
- **OTA Updates:** A custom web-based OTA mode can be triggered via Modbus, allowing firmware updates over Wi-Fi (which is otherwise disabled).
- **Persistence:** Configuration and volume settings are stored in EEPROM.

## Hardware Mapping

| Component | Pin (ESP8266) | Function |
| :--- | :--- | :--- |
| MAX485 DE+RE | D5 (GPIO14) | RS-485 Data Enable |
| DFPlayer RX | D7 (GPIO13) | SoftwareSerial TX (via 1kΩ resistor) |
| DFPlayer TX | D6 (GPIO12) | SoftwareSerial RX |
| RS-485 A/B | - | External Bus |

## Modbus Register Map

### Holding Registers (RW)

| Address | Name | Range / Description |
| :--- | :--- | :--- |
| 1 | `PLAY_CONTROL` | Write 1 to start playback of `TRACK_NUMBER` |
| 2 | `LOOP_CONTROL` | Write 1 to loop `TRACK_NUMBER` |
| 3 | `STOP_CONTROL` | Write 1 to stop playback |
| 4 | `TRACK_NUMBER` | 1 - 9999 |
| 5 | `VOLUME` | 0 - 30 |
| 6 | `EQ_MODE` | 0 (Normal) to 5 (Base) |
| 7 | `CFG_BAUD_INDEX`| 0-7 (1200, 2400, 4800, 9600, 19200, 38400, 57600, 115200) |
| 8 | `CFG_PARITY` | 0: None, 1: Even, 2: Odd |
| 9 | `CFG_STOPBITS` | 1 or 2 |
| 10| `CFG_SLAVE_ID` | 1 - 247 |
| 11| `REBOOT` | Write 1 to restart the device |
| 12| `OTA_MODE` | Write 1 to enable Wi-Fi AP and OTA Web Server |

### Coils (Write Only)

- **Addresses 1 - 200:** Writing `0xFF00` (ON) to a coil triggers the immediate playback of the track number corresponding to that address.

## Building and Running

### Requirements
- **Framework:** Arduino ESP8266 Core.
- **Libraries:**
  - `SoftwareSerial`
  - `DFPlayerMini_Fast`
  - `EEPROM`, `ESP8266WiFi`, `ESP8266WebServer` (Standard ESP8266 libraries).

### Compilation
1. Open `firmware.ino` in the Arduino IDE.
2. Select **NodeMCU 1.0 (ESP-12E Module)** or equivalent.
3. Install missing libraries via the Library Manager.
4. Upload to the device.

### Operation
- **Normal Mode:** Wi-Fi is OFF. The device listens for Modbus commands on UART0.
- **OTA Mode:** Triggered by writing `1` to Holding Register `12`.
  - SSID: `cr_dr_bl`
  - Password: `maq3n7v5`
  - Web Interface: `http://192.168.4.1`
  - The device reboots back to Normal Mode after 5 minutes or after a successful flash.

## Development Conventions

- **Modbus Implementation:** The Modbus stack is a lightweight custom implementation found in `firmware.ino`. Avoid adding heavy Modbus libraries to keep the binary small.
- **Serial Conflict:** UART0 is used for Modbus. Do NOT use `Serial.print()` for debugging in Normal Mode as it will corrupt the RS-485 bus.
- **Reliability:** Audio commands are sent to the DFPlayer via SoftwareSerial. Ensure the 1kΩ resistor is present on the RX line of the DFPlayer to reduce noise.
