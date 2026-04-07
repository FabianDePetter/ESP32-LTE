/*
 File: p1_data.h

Example P1 telegram with explanation per line:

/FLU5\253769484_A         // Manufacturer and serial number
0-0:96.1.4(50221)         // DSMR/eMUCS-P1 version
1-0:94.32.1(400)          // Firmware version
0-0:96.1.1(3153414733313031303231363035) // Meter ID (hex)
0-0:96.1.2(353431343430303132333435363738393030) // EAN-13 code
0-0:1.0.0(200512135409S)  // Date and time (YYMMDDhhmmssS/W)
1-0:1.8.1(000000.034*kWh) // T1 import (kWh)
1-0:1.8.2(000015.758*kWh) // T2 import (kWh)
1-0:2.8.1(000000.000*kWh) // T1 export (kWh)
1-0:2.8.2(000000.011*kWh) // T2 export (kWh)
0-0:96.14.0(0001)         // Active tariff (1=normal, 2=low)
1-0:1.4.0(02.351*kW)      // Current quarter-hour average power (kW)
1-0:1.6.0(200509134558S)(02.589*kW) // Monthly peak power + timestamp
0-0:98.1.0(3)(1-0:1.6.0)... // Historical peaks
1-0:1.7.0(00.000*kW)      // Instantaneous total import power
1-0:2.7.0(00.000*kW)      // Instantaneous total export power
1-0:21.7.0(00.000*kW)     // L1 import power
1-0:22.7.0(00.000*kW)     // L1 export power
1-0:41.7.0(00.000*kW)     // L2 import power
1-0:42.7.0(00.000*kW)     // L2 export power
1-0:61.7.0(00.000*kW)     // L3 import power
1-0:62.7.0(00.000*kW)     // L3 export power
1-0:32.7.0(234.7*V)       // L1 voltage
1-0:52.7.0(234.7*V)       // L2 voltage
1-0:72.7.0(234.7*V)       // L3 voltage
1-0:31.7.0(000.00*A)      // L1 current (F5)
1-0:51.7.0(000.00*A)      // L2 current (F5)
1-0:71.7.0(000.00*A)      // L3 current (F5)
0-0:96.3.10(1)            // Main breaker status
0-0:17.0.0(99.999*kW)     // Power limiter
1-0:31.4.0(999.99*A)      // L1 fuse supervision
0-1:96.3.10(0)            // M-Bus CH1 relay
0-2:96.3.10(0)            // M-Bus CH2 relay
0-3:96.3.10(0)            // M-Bus CH3 relay
0-4:96.3.10(0)            // M-Bus CH4 relay
0-0:96.13.0()             // Operator text field
0-1:24.1.0(003)           // Gas meter type
0-1:96.1.1(...)           // Gas meter ID
0-1:96.1.2(...)           // Gas meter EAN
0-1:24.4.0(1)             // Gas open valve status
0-1:24.2.3(200512134558S)(00112.384*m3) // Gas consumption + timestamp
0-2:24.1.0(007)           // Water meter type
0-2:96.1.1(...)           // Water meter ID
0-2:96.1.2(...)           // Water meter EAN
0-2:24.2.1(200512134558S)(00872.234*m3) // Water consumption + timestamp
!XXX                       // CRC16 checksum
*/

#pragma once
#include <stdint.h>

typedef struct {
    // ===== Identification =====
    char manufacturer[16];      // e.g. "/FLU5"
    char serial[32];            // e.g. "253769484_A"
    char dsmr_version[16];      // 0-0:96.1.4
    char firmware_version[16];  // 1-0:94.32.1
    char meter_id[32];          // 0-0:96.1.1 (hex)
    char ean_code[32];          // 0-0:96.1.2

    // ===== Timestamp and message ID =====
    char timestamp[20];         // 0-0:1.0.0 (YYYYMMDDhhmmssS)
    char msg_id[32];            // Custom message ID (incremented counter)

    // ===== Energy (Wh) =====
    uint32_t energy_import_t1;  // 1-0:1.8.1
    uint32_t energy_import_t2;  // 1-0:1.8.2
    uint32_t energy_export_t1;  // 1-0:2.8.1
    uint32_t energy_export_t2;  // 1-0:2.8.2

    // ===== Tariff =====
    uint8_t active_tariff;      // 0-0:96.14.0 (1=normal, 2=low)

    // ===== Demand / peaks (W) =====
    int16_t avg_demand_quarter;   // 1-0:1.4.0
    int16_t max_demand_month;     // 1-0:1.6.0
    int16_t max_demand_history[13]; // 0-0:98.1.0

    // ===== Instantaneous power (W) =====
    int16_t power_import_total;   // 1-0:1.7.0
    int16_t power_export_total;   // 1-0:2.7.0
    int16_t power_import_l1;      // 1-0:21.7.0
    int16_t power_export_l1;      // 1-0:22.7.0
    int16_t power_import_l2;      // 1-0:41.7.0
    int16_t power_export_l2;      // 1-0:42.7.0
    int16_t power_import_l3;      // 1-0:61.7.0
    int16_t power_export_l3;      // 1-0:62.7.0

    // ===== Voltage (deciVolt) =====
    int16_t voltage_l1;           // 1-0:32.7.0
    int16_t voltage_l2;           // 1-0:52.7.0
    int16_t voltage_l3;           // 1-0:72.7.0

    // ===== Current (centiAmpere) =====
    int16_t current_l1;           // 1-0:31.7.0
    int16_t current_l2;           // 1-0:51.7.0
    int16_t current_l3;           // 1-0:71.7.0

    // ===== Status / limits =====
    uint8_t main_breaker;         // 0-0:96.3.10 (0=off, 1=on)
    uint32_t power_limiter;        // 0-0:17.0.0 (W)
    int16_t fuse_supervision_l1;  // 1-0:31.4.0 (centiAmpere)

    // ===== M-Bus relays (channels 1‑4) =====
    uint8_t mbus_relay[4];        // 0-1:96.3.10 .. 0-4:96.3.10

    // ===== Operator text =====
    char operator_text[64];       // 0-0:96.13.0

    // ===== Gas meter =====
    char gas_type[16];            // 0-1:24.1.0
    char gas_id[32];              // 0-1:96.1.1
    char gas_ean[32];             // 0-1:96.1.2
    uint8_t gas_valve_status;     // 0-1:24.4.0 (0=closed, 1=open)
    char gas_timestamp[20];       // 0-1:24.2.3 timestamp
    uint32_t gas_consumption;     // 0-1:24.2.3 consumption in liters (m³ * 1000)

    // ===== Water meter =====
    char water_type[16];          // 0-2:24.1.0
    char water_id[32];            // 0-2:96.1.1
    char water_ean[32];           // 0-2:96.1.2
    char water_timestamp[20];     // 0-2:24.2.1 timestamp
    uint32_t water_consumption;   // 0-2:24.2.1 consumption in liters (m³ * 1000)

    // ===== CRC =====
    uint16_t crc;                 // Last 4 hex digits after '!'
    bool crc_valid;               // true = last CRC check passed, false = failed
} p1_data_t;
