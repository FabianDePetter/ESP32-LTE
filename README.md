# ESP32-WAN Project

This project targets the LilyGO TTGO T-SIM7070G (ESP32-WROVER + SIM7070G) and focuses on WAN communication (LTE/NB-IoT).

## Features
- ESP-IDF framework
- SIM7070G LTE-M initialization and AT command integration

## Project Structure
- main/ : main application code
- All code, comments, and documentation are in English

## Steps
1. Initialize SIM7070G modem
2. Connect to mobile network (APN setup)
3. Read and parse P1 telegrams
4. Send binary data to server via LTE-M/NB-Iot

## Documentation
- LilyGO GitHub: https://github.com/Xinyuan-LilyGO/T-SIM7070G
- SIM7070G AT commands: see datasheet

## Getting Started
- Place your ESP-IDF code in main/
- Configure APN, server IP, and port in esp_wan_cli.h
- Build with `idf.py build`
- Flash to ESP32

## License
MIT License
