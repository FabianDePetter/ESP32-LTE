# ESP32-WAN Project

This project targets the LilyGO TTGO T-SIM7070G (ESP32-WROVER + SIM7070G) and focuses on WAN communication (LTE/NB-IoT). It reads DSMR P1 telegrams from a smart meter and sends binary data over LTE-M UDP to a remote server or PC.

## Features
- ESP-IDF framework
- SIM7070G LTE-M initialization and AT command integration
- Binary data upload via UDP
- Modular code structure (main, p1_reader, p1_crc, p1_data, esp_lte_cli)
- Battery management (optional)

## Project Structure
- main/ : main application code
- All code, comments, and documentation are in English

## Steps
1. Initialize SIM7070G modem
2. Connect to mobile network (APN setup)
3. Read and parse P1 telegrams
4. Send binary data to server via LTE-M UDP
5. Monitor battery status (optional)

## Documentation
- LilyGO GitHub: https://github.com/Xinyuan-LilyGO/T-SIM7070G
- SIM7070G AT commands: see datasheet
- Example UDP server: see PROJECT.md

## Getting Started
- Place your ESP-IDF code in main/
- Configure APN, server IP, and port in esp_lte_cli.h
- Build with `idf.py build`
- Flash to ESP32 and connect SIM7070G

## License
MIT License
