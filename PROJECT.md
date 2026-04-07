# ESP32-WAN Project

## Overview
This project implements a WAN-enabled IoT endpoint using the ESP32 and a SIM7070G LTE-M module. It reads DSMR P1 telegrams from a smart meter, parses and validates the data, and sends it as binary messages over LTE-M UDP to a remote server or PC.

## Features
- Modular code structure (main.c, p1_reader.cpp/h, p1_crc.c/h, p1_data.h, esp_lte_cli.c/h)
- English code and comments throughout
- P1 telegram parsing and CRC validation
- Binary message packing for efficient IoT communication
- LTE-M UDP send logic (SIM7070G AT commands)
- Configurable server IP and port

## Usage
1. Configure your APN, server IP, and port in esp_lte_cli.h
2. Build the project with ESP-IDF (`idf.py build`)
3. Flash to ESP32 and connect SIM7070G
4. Start a UDP server on your PC or remote endpoint
5. The ESP32 will send binary P1 data to your server via LTE-M

## Structure
- All code, comments, and documentation are in English
- No Dutch language present

## Example UDP Server (Python)
```python
import socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(('0.0.0.0', 5683))
while True:
    data, addr = sock.recvfrom(1024)
    print(f"Received from {addr}: {data}")
```

## Notes
- Make sure your PC/server is reachable via public IP and UDP port
- Update LTE_M_SERVER_IP in esp_lte_cli.h accordingly
- For SIM7070G AT command integration, see esp_lte_cli.c/h

## License
MIT License
