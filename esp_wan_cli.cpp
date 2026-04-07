/*
 * esp_wan_cli.cpp — ESP32 WAN P1 Smart Meter over GPRS + TCP
 *
 * Operation:
 *   1. Load NVS (interval_s, simulate)
 *   2. Initialize P1Reader + fill sim_p1
 *   3. Turn on modem + connect to network
 *   4. Loop:
 *        a. Read P1 (real or simulated)  → update_p1_data()
 *        b. Build binary packet           → build_p1_binary()
 *        c. Send via TCP                  → wan_send_tcp_raw()
 *        d. Power cycle (firmware workaround — DO NOT remove)
 *        e. Wait for remaining interval
 *
 * IMPORTANT: The power cycle after every message (modem_off → modem_on →
 * wait_ready → connect_network) is required due to a bug in the SIM7070
 * firmware that otherwise crashes. The interval_s must be > WAN_INTERVAL_MIN_S
 * so that the power cycle (~25 s) can always complete fully.
 *
 */

#include "esp_wan_cli.h"
#include "p1_reader.h"   // P1Reader struct + API
#include "p1_data.h"     // p1_data_t

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <inttypes.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "nvs.h"
#include <sys/time.h>

static const char *TAG = "WAN";

// ---------------------------------------------------------------------------
// Modem UART buffer
// ---------------------------------------------------------------------------
#define BUF_SIZE 1024
static uint8_t uart_buf[BUF_SIZE];

// ---------------------------------------------------------------------------
// NVS-backed config
// ---------------------------------------------------------------------------
// Interval in seconds for sending data (loaded from NVS)
static uint32_t p1_interval_s = WAN_INTERVAL_DEFAULT_S;
// Whether to simulate P1 data (loaded from NVS)
static bool     simulate_p1   = false;

// ---------------------------------------------------------------------------
// P1 reader instance + global data buffer
// ---------------------------------------------------------------------------
// Global P1 reader instance
static P1Reader  g_p1_reader;
// Buffer for current P1 data (real)
static p1_data_t g_current_p1;
// Buffer for simulated P1 data
static p1_data_t sim_p1;

// ---------------------------------------------------------------------------
// Message counter (increments every send cycle)
// ---------------------------------------------------------------------------
// Message counter, increments every send cycle
static uint32_t msg_counter = 1;

// ---------------------------------------------------------------------------
// Signal quality + network info (updated every connect_network() call)
// ---------------------------------------------------------------------------
// Signal quality and network info (updated on every connect)
static signal_info_t g_signal      = {0};
// Number of network rejoins
static uint32_t      g_rejoin_count = 0;

// ---------------------------------------------------------------------------
// Runtime stats (for periodic summary log)
// ---------------------------------------------------------------------------
// Runtime statistics
static uint32_t stat_send_ok    = 0;   // Successful sends
static uint32_t stat_send_fail  = 0;   // Failed sends
static uint32_t stat_net_fail   = 0;   // Network failures
static int64_t  stat_boot_time  = 0;   // Boot time (microseconds)

// MQTT connection state
static bool mqtt_connected   = false;  // true if MQTT session is currently active
static bool mqtt_ever_opened = false;  // true after the first successful SMCONN since boot/power cycle

// ===========================================================================
// NVS helpers
// ===========================================================================

// Load configuration from NVS (Non-Volatile Storage)
static void nvs_load_config(void)
{
    nvs_handle_t h;
    // Try to open the NVS namespace for reading configuration
    if (nvs_open(WAN_NVS_NAMESPACE, NVS_READONLY, &h) != ESP_OK) {
        // If the namespace does not exist, use default values
        ESP_LOGI(TAG, "NVS namespace '%s' not found — using defaults", WAN_NVS_NAMESPACE);
        return;
    }
    uint32_t v32 = 0;
    uint8_t  v8  = 0;
    // Try to read the interval value from NVS
    if (nvs_get_u32(h, NVS_KEY_INTERVAL_S, &v32) == ESP_OK) {
        // Only use the value if it is within allowed range
        if (v32 >= WAN_INTERVAL_MIN_S && v32 <= WAN_INTERVAL_MAX_S)
            p1_interval_s = v32;
    }
    // Try to read the simulation flag from NVS
    if (nvs_get_u8(h, NVS_KEY_SIMULATE, &v8) == ESP_OK)
        simulate_p1 = (v8 != 0);

    if (nvs_get_u32(h, "msg_counter", &v32) == ESP_OK)
    msg_counter = v32;

    // Close the NVS handle after reading
    nvs_close(h);
    // Log the loaded configuration values
    ESP_LOGI(TAG, "NVS loaded: interval_s=%" PRIu32 "  simulate=%d",
             p1_interval_s, (int)simulate_p1);
}

// Save configuration to NVS
static void nvs_save_config(void)
{
    nvs_handle_t h;
    if (nvs_open(WAN_NVS_NAMESPACE, NVS_READWRITE, &h) != ESP_OK) {
        ESP_LOGE(TAG, "nvs_open failed");
        return;
    }
    nvs_set_u32(h, NVS_KEY_INTERVAL_S, p1_interval_s);
    nvs_set_u8 (h, NVS_KEY_SIMULATE,   (uint8_t)simulate_p1);
    nvs_commit(h);
    nvs_close(h);
    ESP_LOGI(TAG, "NVS saved: interval_s=%" PRIu32 "  simulate=%d",
             p1_interval_s, (int)simulate_p1);
}

// ===========================================================================
// Simulated P1 data
// ===========================================================================

// Initialize the simulated P1 data structure with default values
static void init_sim_p1(void)
{
    // Clear the simulated P1 struct to zero
    memset(&sim_p1, 0, sizeof(sim_p1));

    // Set up all string fields for the simulated meter
    strcpy(sim_p1.manufacturer,     "SIM");         // Manufacturer name
    strcpy(sim_p1.serial,           "123456789");  // Serial number
    strcpy(sim_p1.dsmr_version,     "5.0");        // DSMR version
    strcpy(sim_p1.firmware_version, "1.0");        // Firmware version
    strcpy(sim_p1.meter_id,         "SIM123");     // Meter ID
    strcpy(sim_p1.ean_code,         "1234567890123"); // EAN code
    strcpy(sim_p1.timestamp,        "20000101000000S"); // Initial timestamp
    strcpy(sim_p1.msg_id,           "1");          // Message ID

    // Set up initial values for energy import/export
    sim_p1.energy_import_t1 = 123456; // Import energy for tariff 1
    sim_p1.energy_import_t2 = 654321; // Import energy for tariff 2
    sim_p1.energy_export_t1 = 0;      // Export energy for tariff 1
    sim_p1.energy_export_t2 = 0;      // Export energy for tariff 2
    sim_p1.active_tariff    = 1;      // Start with tariff 1
    sim_p1.avg_demand_quarter = 250;  // Average demand (quarter)
    sim_p1.max_demand_month   = 500;  // Max demand (month)
    // Fill demand history with increasing values
    for (int i = 0; i < 13; i++) sim_p1.max_demand_history[i] = (int16_t)(100 + i * 100);

    // Set up power values for each phase
    sim_p1.power_import_total = 350;
    sim_p1.power_export_total = 10;
    sim_p1.power_import_l1    = 120;  sim_p1.power_export_l1 = 3;
    sim_p1.power_import_l2    = 115;  sim_p1.power_export_l2 = 3;
    sim_p1.power_import_l3    = 115;  sim_p1.power_export_l3 = 4;

    // Set up voltage and current for each phase
    sim_p1.voltage_l1 = 2305;  sim_p1.voltage_l2 = 2302;  sim_p1.voltage_l3 = 2303;
    sim_p1.current_l1 = 1500;  sim_p1.current_l2 = 1480;  sim_p1.current_l3 = 1490;

    // Set up breaker and limiter values
    sim_p1.main_breaker        = 1;
    sim_p1.power_limiter       = 99999;
    sim_p1.fuse_supervision_l1 = 25000;

    // Set up MBUS relay states
    sim_p1.mbus_relay[0] = sim_p1.mbus_relay[1] = 0;
    sim_p1.mbus_relay[2] = sim_p1.mbus_relay[3] = 0;

    // Operator text for simulation
    strcpy(sim_p1.operator_text, "Simulation");

    // Simulated gas meter fields
    strcpy(sim_p1.gas_type,      "003");
    strcpy(sim_p1.gas_id,        "123456789");
    strcpy(sim_p1.gas_ean,       "1234567890123");
    sim_p1.gas_valve_status = 1;
    strcpy(sim_p1.gas_timestamp, "20000101000000S");
    sim_p1.gas_consumption = 112384;

    // Simulated water meter fields
    strcpy(sim_p1.water_type,      "007");
    strcpy(sim_p1.water_id,        "987654321");
    strcpy(sim_p1.water_ean,       "3210987654321");
    strcpy(sim_p1.water_timestamp, "20000101000000S");
    sim_p1.water_consumption = 87234;

    // Set CRC and validity flag
    sim_p1.crc       = 0xABCD;
    sim_p1.crc_valid = true;
}

// Update simulated P1 data to mimic real meter changes
static void update_sim_p1(void)
{
    // Every 10 messages, toggle the active tariff (simulate day/night switching)
    if (msg_counter % 10 == 0)
        sim_p1.active_tariff = (sim_p1.active_tariff == 1) ? 2 : 1;

    // Simulate energy usage and export for the active tariff
    if (sim_p1.active_tariff == 1) {
        // Tariff 1: increment import by 10-29, export by 0-1
        sim_p1.energy_import_t1 += 10 + (rand() % 20);
        sim_p1.energy_export_t1 +=  0 + (rand() % 2);
    } else {
        // Tariff 2: increment import by 5-19, export by 0-1
        sim_p1.energy_import_t2 +=  5 + (rand() % 15);
        sim_p1.energy_export_t2 +=  0 + (rand() % 2);
    }

    // Randomize total power import/export for this cycle
    sim_p1.power_import_total = (int16_t)(300 + (rand() % 200));
    sim_p1.power_export_total = (int16_t)( 10 + (rand() % 20));

    // Distribute total import/export over 3 phases, with some randomness
    sim_p1.power_import_l1 = (int16_t)(sim_p1.power_import_total / 3 + (rand() % 50));
    sim_p1.power_import_l2 = (int16_t)(sim_p1.power_import_total / 3 + (rand() % 50));
    // The third phase gets the remainder
    sim_p1.power_import_l3 = (int16_t)(sim_p1.power_import_total
                                        - sim_p1.power_import_l1
                                        - sim_p1.power_import_l2);
    sim_p1.power_export_l1 = (int16_t)(sim_p1.power_export_total / 3 + (rand() % 10));
    sim_p1.power_export_l2 = (int16_t)(sim_p1.power_export_total / 3 + (rand() % 10));
    sim_p1.power_export_l3 = (int16_t)(sim_p1.power_export_total
                                        - sim_p1.power_export_l1
                                        - sim_p1.power_export_l2);

    // Simulate voltage fluctuations for each phase
    sim_p1.voltage_l1 = (int16_t)(2300 + (rand() % 50));
    sim_p1.voltage_l2 = (int16_t)(2300 + (rand() % 50));
    sim_p1.voltage_l3 = (int16_t)(2300 + (rand() % 50));

    // Calculate current for each phase based on power and voltage
    sim_p1.current_l1 = (int16_t)(((int32_t)sim_p1.power_import_l1 * 1000) / sim_p1.voltage_l1);
    sim_p1.current_l2 = (int16_t)(((int32_t)sim_p1.power_import_l2 * 1000) / sim_p1.voltage_l2);
    sim_p1.current_l3 = (int16_t)(((int32_t)sim_p1.power_import_l3 * 1000) / sim_p1.voltage_l3);

    // Set average demand to current import total (simple simulation)
    sim_p1.avg_demand_quarter = sim_p1.power_import_total;

    // Every 60 messages, possibly update max demand for the month
    if (msg_counter % 60 == 0) {
        int16_t candidate = (int16_t)(400 + (rand() % 1200));
        // Only update if the new value is higher
        if (candidate > sim_p1.max_demand_month)
            sim_p1.max_demand_month = candidate;
    }

    // Every 500 messages, shift demand history and add new value at the front
    if (msg_counter % 500 == 0) {
        for (int i = 12; i > 0; i--)
            sim_p1.max_demand_history[i] = sim_p1.max_demand_history[i - 1];
        sim_p1.max_demand_history[0] = sim_p1.max_demand_month;
    }

    // Update timestamp to current time (format: YYYYMMDDhhmmssS)
    time_t now = time(NULL);
    struct tm *tm_info = localtime(&now);
    strftime(sim_p1.timestamp, sizeof(sim_p1.timestamp), "%Y%m%d%H%M%S", tm_info);
    size_t tlen = strlen(sim_p1.timestamp);
    if (tlen < sizeof(sim_p1.timestamp) - 1) {
        sim_p1.timestamp[tlen]     = 'S'; // Add 'S' at the end for simulation
        sim_p1.timestamp[tlen + 1] = '\0';
    }

    // Simulate gas and water consumption increments
    sim_p1.gas_consumption   += 1 + (rand() % 10);
    sim_p1.water_consumption += 1 + (rand() % 5);

    // Always mark CRC as valid in simulation
    sim_p1.crc_valid = true;
}

// Return pointer to the currently active P1 data (simulated or real)
static p1_data_t *get_active_p1(void)
{
    return simulate_p1 ? &sim_p1 : &g_current_p1;
}

// ===========================================================================
// P1 read + update cycle
// ===========================================================================

// Read or simulate P1 data and update global buffer
static void update_p1_data(void)
{
    if (simulate_p1) {
        update_sim_p1();
        ESP_LOGD(TAG, "P1 simulated: P_imp=%dW  V_L1=%.1fV  ei1=%" PRIu32 "Wh",
                 sim_p1.power_import_total,
                 sim_p1.voltage_l1 / 10.0f,
                 sim_p1.energy_import_t1);
    } else {
        ESP_LOGI(TAG, "P1 reading from UART...");
        P1Reader_request_and_read(&g_p1_reader);
        g_current_p1 = g_p1_reader.data;
        ESP_LOGI(TAG, "P1 read done: crc_valid=%d  P_imp=%dW  V_L1=%.1fV",
                 g_current_p1.crc_valid,
                 g_current_p1.power_import_total,
                 g_current_p1.voltage_l1 / 10.0f);
    }

    p1_data_t *p = get_active_p1();
    snprintf(p->msg_id, sizeof(p->msg_id), "%" PRIu32, msg_counter);
    msg_counter++;
    {
        nvs_handle_t nh;
        if (nvs_open(WAN_NVS_NAMESPACE, NVS_READWRITE, &nh) == ESP_OK) {
            nvs_set_u32(nh, "msg_counter", msg_counter);
            nvs_commit(nh);
            nvs_close(nh);
        }
    }
}

// ===========================================================================
// Binary protocol — packet layout (see header for full field description)
// ===========================================================================

// Structure to keep track of previous values for delta encoding in binary packet
static struct {
    uint8_t  tar;
    uint32_t ei1, ei2;
    uint32_t ee1, ee2;
    int16_t  adq, mdm;
    int16_t  mdh[13];
    uint8_t  ts_min, ts_sec;
    char     net[16];
    uint32_t rjn;
    bool     init;
} prev;

// Helper functions to write integers to a byte buffer (little-endian)
static size_t put_u8 (uint8_t *b, size_t pos, uint8_t  v) { b[pos]=v;                                                return pos+1; }
static size_t put_i8 (uint8_t *b, size_t pos, int8_t   v) { b[pos]=(uint8_t)v;                                       return pos+1; }
static size_t put_u16(uint8_t *b, size_t pos, uint16_t v) { b[pos]=v&0xFF; b[pos+1]=v>>8;                            return pos+2; }
static size_t put_i16(uint8_t *b, size_t pos, int16_t  v) { return put_u16(b, pos, (uint16_t)v); }
static size_t put_u32(uint8_t *b, size_t pos, uint32_t v) { b[pos]=v&0xFF; b[pos+1]=(v>>8)&0xFF;
                                                              b[pos+2]=(v>>16)&0xFF; b[pos+3]=v>>24; return pos+4; }

// Convert network type string to enum value for binary protocol
static uint8_t net_to_enum(const char *net)
{
    if (strcmp(net, "LTE-M")  == 0) return 0;
    if (strcmp(net, "NB-IoT") == 0) return 1;
    if (strcmp(net, "GSM")    == 0) return 2;
    return 3;
}

// Parse timestamp string to extract minute and second
static void parse_ts_minsec(const char *ts, uint8_t *out_min, uint8_t *out_sec)
{
    if (strlen(ts) >= 14) {
        *out_min = (uint8_t)((ts[10]-'0')*10 + (ts[11]-'0'));
        *out_sec = (uint8_t)((ts[12]-'0')*10 + (ts[13]-'0'));
    } else {
        *out_min = 0;
        *out_sec = 0;
    }
}

// Build binary packet from current P1 data, using delta encoding for efficiency
static size_t build_p1_binary(uint8_t *out, size_t out_size)
{
    p1_data_t *p = get_active_p1();

    struct timeval tv_now;
    gettimeofday(&tv_now, NULL);
    uint32_t ts2_s  = (uint32_t)tv_now.tv_sec;
    uint32_t ts2_us = (uint32_t)tv_now.tv_usec;

    if (!prev.init) {
        memset(&prev, 0xFF, sizeof(prev));
        memset(prev.net, 0, sizeof(prev.net));
        prev.rjn  = UINT32_MAX;
        prev.init = true;
        ESP_LOGI(TAG, "Binary encoder: first call — all fields marked as changed");
    }

    uint8_t cur_min, cur_sec;
    parse_ts_minsec(p->timestamp, &cur_min, &cur_sec);

    uint8_t fa = 0;
    if (p->active_tariff      != prev.tar) fa |= (1<<7);
    if (p->energy_import_t1   != prev.ei1) fa |= (1<<6);
    if (p->energy_import_t2   != prev.ei2) fa |= (1<<5);
    if (p->energy_export_t1   != prev.ee1) fa |= (1<<4);
    if (p->energy_export_t2   != prev.ee2) fa |= (1<<3);
    if (p->avg_demand_quarter  != prev.adq) fa |= (1<<2);
    if (p->max_demand_month    != prev.mdm) fa |= (1<<1);
    if (p->crc_valid)                       fa |= (1<<0);

    uint8_t fb = 0;
    if (cur_min != prev.ts_min || cur_sec != prev.ts_sec) fb |= (1<<7);

    bool mdh_changed = false;
    for (int i = 0; i < 13; i++) {
        if (p->max_demand_history[i] != prev.mdh[i]) { mdh_changed = true; break; }
    }
    if (mdh_changed)                                   fb |= (1<<6);
    if ((msg_counter - 1) % 15 == 0)                  fb |= (1<<5);
    if (strcmp(g_signal.network_type, prev.net) != 0) fb |= (1<<4);
    if (g_signal.rejoin_count != prev.rjn)            fb |= (1<<3);

    ESP_LOGD(TAG, "Packet flags: fa=0x%02X fb=0x%02X  (tar:%d ei1:%d ei2:%d ee1:%d ee2:%d adq:%d mdm:%d | ts:%d mdh:%d catB:%d net:%d rjn:%d)",
             fa, fb,
             !!(fa&(1<<7)), !!(fa&(1<<6)), !!(fa&(1<<5)), !!(fa&(1<<4)),
             !!(fa&(1<<3)), !!(fa&(1<<2)), !!(fa&(1<<1)),
             !!(fb&(1<<7)), !!(fb&(1<<6)), !!(fb&(1<<5)),
             !!(fb&(1<<4)), !!(fb&(1<<3)));

    size_t pos = 0;
    pos = put_u8 (out, pos, fa);
    pos = put_u8 (out, pos, fb);
    pos = put_u16(out, pos, (uint16_t)(msg_counter - 1));

    struct timeval tv;
    gettimeofday(&tv, NULL);
    struct tm *tm = gmtime(&tv.tv_sec);
    uint8_t minute = (uint8_t)tm->tm_min;
    uint8_t second = (uint8_t)tm->tm_sec;
    uint32_t us = (uint32_t)tv.tv_usec;

    pos = put_u8(out, pos, minute);
    pos = put_u8(out, pos, second);
    pos = put_u8(out, pos, us & 0xFF);
    pos = put_u8(out, pos, (us >> 8) & 0xFF);
    pos = put_u8(out, pos, (us >> 16) & 0xFF);

    if (fa & (1<<7)) pos = put_u8 (out, pos, p->active_tariff);
    if (fa & (1<<6)) pos = put_u32(out, pos, p->energy_import_t1);
    if (fa & (1<<5)) pos = put_u32(out, pos, p->energy_import_t2);
    if (fa & (1<<4)) pos = put_u32(out, pos, p->energy_export_t1);
    if (fa & (1<<3)) pos = put_u32(out, pos, p->energy_export_t2);
    if (fa & (1<<2)) pos = put_i16(out, pos, p->avg_demand_quarter);
    if (fa & (1<<1)) pos = put_i16(out, pos, p->max_demand_month);

    if (fb & (1<<7)) {
        pos = put_u8(out, pos, cur_min);
        pos = put_u8(out, pos, cur_sec);
    }

    if (fb & (1<<6)) {
        for (int i = 0; i < 13; i++)
            pos = put_i16(out, pos, p->max_demand_history[i]);
    }

    if (fb & (1<<5)) {
        pos = put_i16(out, pos, p->power_import_l1);
        pos = put_i16(out, pos, p->power_export_l1);
        pos = put_i16(out, pos, p->power_import_l2);
        pos = put_i16(out, pos, p->power_export_l2);
        pos = put_i16(out, pos, p->power_import_l3);
        pos = put_i16(out, pos, p->power_export_l3);
        pos = put_u16(out, pos, (uint16_t)p->voltage_l1);
        pos = put_u16(out, pos, (uint16_t)p->voltage_l2);
        pos = put_u16(out, pos, (uint16_t)p->voltage_l3);
    }

    int8_t rssi_c = (g_signal.rssi < -128) ? -128 : (int8_t)g_signal.rssi;
    int8_t rsrp_c = (g_signal.rsrp < -128) ? -128 : (int8_t)g_signal.rsrp;
    int8_t rsrq_c = (g_signal.rsrq < -128) ? -128 : (int8_t)g_signal.rsrq;
    int8_t sinr_c = (g_signal.sinr < -128) ? -128 : (int8_t)g_signal.sinr;
    pos = put_i8(out, pos, rssi_c);
    pos = put_i8(out, pos, rsrp_c);
    pos = put_i8(out, pos, rsrq_c);
    pos = put_i8(out, pos, sinr_c);

    if (fb & (1<<4)) pos = put_u8(out, pos, net_to_enum(g_signal.network_type));
    if (fb & (1<<3)) pos = put_u8(out, pos, (uint8_t)g_signal.rejoin_count);

    prev.tar = p->active_tariff;
    prev.ei1 = p->energy_import_t1;
    prev.ei2 = p->energy_import_t2;
    prev.ee1 = p->energy_export_t1;
    prev.ee2 = p->energy_export_t2;
    prev.adq = p->avg_demand_quarter;
    prev.mdm = p->max_demand_month;
    for (int i = 0; i < 13; i++) prev.mdh[i] = p->max_demand_history[i];
    prev.ts_min = cur_min;
    prev.ts_sec = cur_sec;
    strncpy(prev.net, g_signal.network_type, sizeof(prev.net) - 1);
    prev.rjn = g_signal.rejoin_count;

    ESP_LOGI(TAG, "Binary packet built: %u bytes  ts=%lu.%06" PRIu32 "  msg#%u",
         (unsigned)pos, (unsigned long)ts2_s, ts2_us, (unsigned)(msg_counter - 1));
    return pos;
}

// ===========================================================================
// Modem UART helpers
// ===========================================================================

// Return current time in milliseconds since boot
static int64_t millis_now(void) { return esp_timer_get_time() / 1000; }
// Delay for a given number of milliseconds
static void    delay_ms(uint32_t ms) { vTaskDelay(pdMS_TO_TICKS(ms)); }

// Flush UART receive buffer
static void uart_flush_buf(void)
{
    uart_flush_input(UART_MODEM);
    uint8_t tmp[256];
    while (uart_read_bytes(UART_MODEM, tmp, sizeof(tmp), pdMS_TO_TICKS(50)) > 0) {}
}

// Send AT command to modem (adds CRLF)
static void at_send(const char *cmd)
{
    char buf[256];
    snprintf(buf, sizeof(buf), "AT%s\r\n", cmd);
    uart_write_bytes(UART_MODEM, buf, strlen(buf));
    ESP_LOGI(TAG, ">>> AT%s", cmd);
}

// Wait for a specific response from modem within timeout
static bool at_wait(uint32_t timeout_ms, const char *expect)
{
    char    resp[BUF_SIZE];
    memset(resp, 0, sizeof(resp));
    size_t  pos   = 0;
    int64_t start = millis_now();
    const char *match = expect ? expect : "OK";

    while (millis_now() - start < (int64_t)timeout_ms) {
        int len = uart_read_bytes(UART_MODEM, uart_buf, BUF_SIZE - 1, pdMS_TO_TICKS(50));
        if (len > 0) {
            uart_buf[len] = 0;
            size_t space = (pos < sizeof(resp) - 1) ? (sizeof(resp) - 1 - pos) : 0;
            if (space > 0) {
                size_t copy = (size_t)len < space ? (size_t)len : space;
                memcpy(resp + pos, uart_buf, copy);
                pos += copy;
                resp[pos] = 0;
            }
            ESP_LOGI(TAG, "<<< %s", uart_buf);
            if (strstr(resp, match))   return true;
            if (strstr(resp, "ERROR")) return false;
        }
    }
    return false;
}

// Wait for modem response and copy to output buffer
static bool at_wait_resp(uint32_t timeout_ms, char *out, size_t out_size)
{
    size_t  pos   = 0;
    int64_t start = millis_now();
    out[0] = 0;
    while (millis_now() - start < (int64_t)timeout_ms) {
        int len = uart_read_bytes(UART_MODEM, uart_buf, BUF_SIZE - 1, pdMS_TO_TICKS(50));
        if (len > 0) {
            uart_buf[len] = 0;
            size_t space = (pos < out_size - 1) ? (out_size - 1 - pos) : 0;
            if (space > 0) {
                size_t copy = (size_t)len < space ? (size_t)len : space;
                memcpy(out + pos, uart_buf, copy);
                pos += copy;
                out[pos] = 0;
            }
            ESP_LOGI(TAG, "<<< %s", uart_buf);
            if (strstr(out, "OK"))    return true;
            if (strstr(out, "ERROR")) return false;
        }
    }
    return false;
}

// Send AT command and wait for OK response
static bool at_send_ok(const char *cmd, uint32_t timeout_ms)
{
    at_send(cmd);
    bool ok = at_wait(timeout_ms, "OK");
    if (!ok) ESP_LOGW(TAG, "AT%s → no OK within %" PRIu32 " ms", cmd, timeout_ms);
    delay_ms(100);
    return ok;
}

// ===========================================================================
// Modem power control
// (Exact copy of working Arduino logic — DO NOT change timing)
// ===========================================================================

// Power off the modem by toggling the PWR pin (timing is critical)
static void modem_off(void)
{
    ESP_LOGI(TAG, "[PWR] modem_off: pulling PWR_PIN low for 1500 ms");
    gpio_set_level((gpio_num_t)PWR_PIN, 0);
    delay_ms(1500);
    gpio_set_level((gpio_num_t)PWR_PIN, 1);
    delay_ms(1500);
    ESP_LOGI(TAG, "[PWR] modem_off done");
}

// Power on the modem by toggling the PWR pin and waiting for boot
static void modem_on(void)
{
    ESP_LOGI(TAG, "[PWR] modem_on: boot sequence start");
    gpio_set_level((gpio_num_t)PWR_PIN, 0);
    delay_ms(1500);
    gpio_set_level((gpio_num_t)PWR_PIN, 1);
    ESP_LOGI(TAG, "[PWR] modem_on: waiting 12 s for firmware boot...");
    delay_ms(12000);
    ESP_LOGI(TAG, "[PWR] modem_on: boot wait done");
}

// ===========================================================================
// UART init (modem)
// ===========================================================================

// Initialize UART peripheral for modem communication
static void uart_modem_init(void)
{
    uart_config_t cfg = {
        .baud_rate           = UART_BAUD,
        .data_bits           = UART_DATA_8_BITS,
        .parity              = UART_PARITY_DISABLE,
        .stop_bits           = UART_STOP_BITS_1,
        .flow_ctrl           = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 0,
        .source_clk          = UART_SCLK_DEFAULT,
        .flags               = 0
    };
    uart_driver_install(UART_MODEM, BUF_SIZE * 4, 0, 0, NULL, 0);
    uart_param_config(UART_MODEM, &cfg);
    uart_set_pin(UART_MODEM, PIN_TX, PIN_RX,
                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    ESP_LOGI(TAG, "UART modem: UART%d  TX=%d RX=%d  baud=%d", UART_MODEM, PIN_TX, PIN_RX, UART_BAUD);
}

// ===========================================================================
// Modem ready probe
// ===========================================================================

// Probe modem until it responds to AT command or timeout
static bool modem_wait_ready(void)
{
    ESP_LOGI(TAG, "[PROBE] Waiting for modem AT response (max 15 probes × 1.7 s)...");
    for (int i = 0; i < 15; i++) {
        uart_flush_buf();
        uart_write_bytes(UART_MODEM, "AT\r\n", 4);
        ESP_LOGI(TAG, "[PROBE] probe %d/15", i + 1);
        char    resp[128] = {0};
        int     pos       = 0;
        int64_t start     = millis_now();
        while (millis_now() - start < 1500) {
            int len = uart_read_bytes(UART_MODEM, uart_buf, BUF_SIZE - 1, pdMS_TO_TICKS(100));
            if (len > 0) {
                uart_buf[len] = 0;
                size_t space = sizeof(resp) - 1 - pos;
                if (space > 0) {
                    size_t copy = (size_t)len < space ? (size_t)len : space;
                    memcpy(resp + pos, uart_buf, copy);
                    pos += (int)copy;
                    resp[pos] = 0;
                }
                ESP_LOGI(TAG, "[PROBE] <<< %s", uart_buf);
                if (strstr(resp, "OK")) {
                    ESP_LOGI(TAG, "[PROBE] Modem responded OK on probe %d", i + 1);
                    return true;
                }
            }
        }
        delay_ms(200);
    }
    ESP_LOGE(TAG, "[PROBE] FAIL: modem did not respond after 15 probes!");
    return false;
}

// ===========================================================================
// modem_diag — full diagnostic dump
// Call this whenever something goes wrong: logs APN, operator, signal,
// network registration state. Does NOT change any modem settings.
// ===========================================================================
// Print full modem diagnostic info for debugging
static void modem_diag(const char *context)
{
    char buf[512] = {0};
    ESP_LOGW(TAG, "======= MODEM DIAG [%s] =======", context);

    at_send("+CGMR");
    at_wait_resp(3000, buf, sizeof(buf));
    ESP_LOGW(TAG, "CGMR (modem firmware): %s", buf);

    memset(buf, 0, sizeof(buf));
    at_send("+CGSN");
    at_wait_resp(3000, buf, sizeof(buf));
    ESP_LOGW(TAG, "CGSN (IMEI): %s", buf);

    memset(buf, 0, sizeof(buf));
    at_send("+CGDCONT?");
    at_wait_resp(3000, buf, sizeof(buf));
    ESP_LOGW(TAG, "CGDCONT (APN config): %s", buf);

    memset(buf, 0, sizeof(buf));
    at_send("+COPS?");
    at_wait_resp(3000, buf, sizeof(buf));
    ESP_LOGW(TAG, "COPS (operator): %s", buf);

    memset(buf, 0, sizeof(buf));
    at_send("+CEREG?");
    at_wait_resp(3000, buf, sizeof(buf));
    ESP_LOGW(TAG, "CEREG (LTE-M/NB-IoT reg): %s", buf);

    memset(buf, 0, sizeof(buf));
    at_send("+CGREG?");
    at_wait_resp(3000, buf, sizeof(buf));
    ESP_LOGW(TAG, "CGREG (GPRS reg): %s", buf);

    memset(buf, 0, sizeof(buf));
    at_send("+CSQ");
    at_wait_resp(2000, buf, sizeof(buf));
    ESP_LOGW(TAG, "CSQ (signal): %s", buf);

    memset(buf, 0, sizeof(buf));
    at_send("+CPSI?");
    at_wait_resp(3000, buf, sizeof(buf));
    ESP_LOGW(TAG, "CPSI (full network info): %s", buf);

    memset(buf, 0, sizeof(buf));
    at_send("+CNACT?");
    at_wait_resp(3000, buf, sizeof(buf));
    ESP_LOGW(TAG, "CNACT (PDP context / IP): %s", buf);

    memset(buf, 0, sizeof(buf));
    at_send("+CCLK?");
    at_wait_resp(2000, buf, sizeof(buf));
    ESP_LOGW(TAG, "CCLK (modem clock): %s", buf);

    ESP_LOGW(TAG, "======= END DIAG =======");
}

// ===========================================================================
// APN force — set and verify before connect_network()
// ===========================================================================
// Force APN setting and verify it is correct
static void apn_force_and_verify(void)
{
    ESP_LOGI(TAG, "[APN] Forcing APN to '%s' on context 1...", APN);
    bool ok1 = at_send_ok("+CGDCONT=1,\"IP\",\"" APN "\"", 3000);
    bool ok2 = at_send_ok("+CGDCONT=2,\"IP\",\"\"",         3000);
    delay_ms(300);
    ESP_LOGI(TAG, "[APN] CGDCONT set: ctx1=%s ctx2=%s", ok1 ? "OK" : "FAIL", ok2 ? "OK" : "FAIL");

    // Read back and log so we can see it in the monitor
    char buf[256] = {0};
    at_send("+CGDCONT?");
    at_wait_resp(3000, buf, sizeof(buf));
    ESP_LOGI(TAG, "[APN] CGDCONT readback: %s", buf);

    // Warn if CMNBIOT is still present — should never happen after the force
    if (strstr(buf, "CMNBIOT")) {
        ESP_LOGE(TAG, "[APN] ALERT: CMNBIOT still present after force — modem may need full reset!");
        modem_diag("APN force failed");
    } else {
        ESP_LOGI(TAG, "[APN] APN verified OK — no CMNBIOT present");
    }
}

// ===========================================================================
// Signal quality reader
// ===========================================================================
// Query modem for signal quality and update global signal info
static void update_signal_info(void)
{
    char buf[256] = {0};

    at_send("+CSQ");
    at_wait_resp(2000, buf, sizeof(buf));
    int csq = 99;
    char *csq_ptr = strstr(buf, "+CSQ: ");
    if (csq_ptr) sscanf(csq_ptr, "+CSQ: %d", &csq);
    g_signal.rssi = (csq == 99 || csq == 0) ? -999 : (int16_t)(-113 + 2 * csq);

    memset(buf, 0, sizeof(buf));
    at_send("+CPSI?");
    at_wait_resp(2000, buf, sizeof(buf));

    char *cpsi = strstr(buf, "+CPSI: ");
    if (!cpsi) {
        ESP_LOGW(TAG, "[SIG] No CPSI response — marking NO SERVICE");
        strncpy(g_signal.network_type, "NO SERVICE", sizeof(g_signal.network_type) - 1);
        g_signal.rsrp = -999;
        g_signal.rsrq = -999;
        g_signal.sinr = -999;
        return;
    }

    char copy[256];
    strncpy(copy, cpsi + 7, sizeof(copy) - 1);
    char *fields[16] = {0};
    int   nfields    = 0;
    char *tok = strtok(copy, ",\r\n");
    while (tok && nfields < 16) {
        fields[nfields++] = tok;
        tok = strtok(NULL, ",\r\n");
    }

    if (nfields > 0) {
        if      (strstr(fields[0], "NB"))  strncpy(g_signal.network_type, "NB-IoT",     sizeof(g_signal.network_type) - 1);
        else if (strstr(fields[0], "LTE")) strncpy(g_signal.network_type, "LTE-M",      sizeof(g_signal.network_type) - 1);
        else if (strstr(fields[0], "GSM")) strncpy(g_signal.network_type, "GSM",        sizeof(g_signal.network_type) - 1);
        else                               strncpy(g_signal.network_type, "NO SERVICE", sizeof(g_signal.network_type) - 1);
    }

    g_signal.rsrq = -999;

    g_signal.rsrp = (nfields > 11 && fields[11]) ? (int16_t)atoi(fields[11]) : -999;
    g_signal.rsrq = (nfields > 12 && fields[12]) ? (int16_t)atoi(fields[12]) : -999;
    g_signal.sinr = (nfields > 13 && fields[13]) ? (int16_t)atoi(fields[13]) : -999;

    // Sanity check RSRQ 
    if (g_signal.rsrq < -20 || g_signal.rsrq > -3) {
        g_signal.rsrq = -999;
    }

    ESP_LOGI(TAG, "[SIG] net=%-8s  rssi=%4d  rsrp=%4d  rsrq=%4d  sinr=%3d dB  rejoin=%" PRIu32,
         g_signal.network_type, g_signal.rssi, g_signal.rsrp, g_signal.rsrq, g_signal.sinr, g_rejoin_count);

    // Warn on poor signal
    if (g_signal.rsrp != -999 && g_signal.rsrp < -110)
        ESP_LOGW(TAG, "[SIG] RSRP very low (%d dBm) — expect connectivity issues", g_signal.rsrp);
    if (g_signal.rsrq != -999 && g_signal.rsrq < -15)
        ESP_LOGW(TAG, "[SIG] RSRQ low (%d dB) — cell may be congested", g_signal.rsrq);
    if (strcmp(g_signal.network_type, "NO SERVICE") == 0)
        ESP_LOGE(TAG, "[SIG] NO SERVICE — modem not registered on any network!");
}

// ===========================================================================
// Network + GPRS connect
// ===========================================================================

// Connect to cellular network and activate GPRS context
static bool connect_network(void)
{
    int64_t t0 = millis_now();
    ESP_LOGI(TAG, "[NET] connect_network() start");

    ESP_LOGI(TAG, "[NET] Setting radio: LTE-M + NB-IoT (CNMP=38 CMNB=3)");
    at_send_ok("+CNMP=38", 3000);
    at_send_ok("+CMNB=3",  3000);

    ESP_LOGI(TAG, "[NET] Disabling GNSS power");
    at_send("+CGNSPWR=0");
    at_wait(2000, NULL);

    ESP_LOGI(TAG, "[NET] Waiting for network registration (timeout 60 s)...");
    int64_t start = millis_now();
    int reg_polls = 0;
    while (millis_now() - start < 60000) {
        char resp[256] = {0};
        reg_polls++;
        at_send("+CEREG?");
        at_wait_resp(4000, resp, sizeof(resp));
        if (strstr(resp, ",1") || strstr(resp, ",5")) {
            ESP_LOGI(TAG, "[NET] LTE-M/NB-IoT registered (CEREG) after %d polls (~%lld ms)",
                     reg_polls, (long long)(millis_now() - start));
            goto gprs;
        }
        memset(resp, 0, sizeof(resp));
        at_send("+CGREG?");
        at_wait_resp(4000, resp, sizeof(resp));
        if (strstr(resp, ",1") || strstr(resp, ",5")) {
            ESP_LOGI(TAG, "[NET] GPRS registered (CGREG) after %d polls (~%lld ms)",
                     reg_polls, (long long)(millis_now() - start));
            goto gprs;
        }
        ESP_LOGD(TAG, "[NET] Not registered yet (poll %d, elapsed %lld ms)",
                 reg_polls, (long long)(millis_now() - start));
        delay_ms(2000);
    }
    ESP_LOGE(TAG, "[NET] TIMEOUT: no network registration after 60 s  (%d polls)", reg_polls);
    stat_net_fail++;
    modem_diag("connect_network timeout");
    return false;

gprs:
    ESP_LOGI(TAG, "[NET] Activating GPRS data context...");

    at_send("+CNACT=0,0");
    at_wait(3000, NULL);
    delay_ms(500);

    if (!at_send_ok("+CGDCONT=1,\"IP\",\"" APN "\"", 3000)) {
        ESP_LOGE(TAG, "[NET] CGDCONT failed");
        return false;
    }
    if (!at_send_ok("+CGATT=1", 10000)) {
        ESP_LOGE(TAG, "[NET] CGATT=1 failed");
        return false;
    }
    if (!at_send_ok("+CNCFG=0,1,\"" APN "\"", 3000)) {
        ESP_LOGE(TAG, "[NET] CNCFG failed");
        return false;
    }

    ESP_LOGI(TAG, "[NET] Requesting PDP context activation (CNACT=0,1, timeout 20 s)...");
    at_send("+CNACT=0,1");
    if (!at_wait(20000, "ACTIVE")) {
        ESP_LOGE(TAG, "[NET] PDP context did not become ACTIVE within 20 s!");
        modem_diag("PDP ACTIVE timeout");
        return false;
    }
    delay_ms(1000);

    char ip[256] = {0};
    at_send("+CNACT?");
    at_wait_resp(2000, ip, sizeof(ip));
    ESP_LOGI(TAG, "[NET] IP info: %s", ip);

    // Time sync
    char clk[64] = {0};
    at_send("+CCLK?");
    at_wait_resp(2000, clk, sizeof(clk));
    ESP_LOGI(TAG, "[NET] CCLK raw: [%s]", clk);
    int yy, mo, dd, hh, mm, ss, tz_quarters = 0;
    char *cclk_ptr = strstr(clk, "+CCLK: \"");
    if (cclk_ptr && sscanf(cclk_ptr,
            "+CCLK: \"%d/%d/%d,%d:%d:%d%d",
            &yy, &mo, &dd, &hh, &mm, &ss, &tz_quarters) >= 6) {
        struct tm t = {0};
        t.tm_year = 2000 + yy - 1900;
        t.tm_mon  = mo - 1;
        t.tm_mday = dd;
        t.tm_hour = hh;
        t.tm_min  = mm;
        t.tm_sec  = ss;
        time_t epoch = mktime(&t);
        epoch -= tz_quarters * 15 * 60;
        struct timeval tv = { .tv_sec = epoch, .tv_usec = 0 };
        settimeofday(&tv, NULL);
        ESP_LOGI(TAG, "[NET] Time synced: 20%02d-%02d-%02d %02d:%02d:%02d UTC (tz=%+d quarters)",
                 yy, mo, dd, hh, mm, ss, tz_quarters);
    } else {
        ESP_LOGW(TAG, "[NET] CCLK sync failed — system time may be wrong");
    }

    update_signal_info();
    g_rejoin_count++;
    g_signal.rejoin_count = g_rejoin_count;

    ESP_LOGI(TAG, "[NET] connect_network() OK in %lld ms  (rejoin #%" PRIu32 ")",
             (long long)(millis_now() - t0), g_rejoin_count);
    return true;
}

// Disconnect MQTT cleanly if connected, then reset state.
// Safe to call even if not connected.
static void mqtt_disconnect(void)
{
    if (mqtt_ever_opened) {
        at_send("+SMDISC");
        at_wait(3000, NULL);
    }
    mqtt_connected   = false;
    mqtt_ever_opened = false;
}

// Open an MQTT session to the broker.
// Returns true on success.
static bool mqtt_connect(void)
{
    delay_ms(500);
    ESP_LOGI(TAG, "[MQTT] Configuring broker %s:%d", MQTT_BROKER, MQTT_PORT);

    // Set broker URL and port
    char conf[128];
    snprintf(conf, sizeof(conf), "+SMCONF=\"URL\",\"%s\",%d", MQTT_BROKER, MQTT_PORT);
    if (!at_send_ok(conf, 3000)) {
        ESP_LOGE(TAG, "[MQTT] SMCONF URL failed");
        return false;
    }

    // Set unique client ID
    if (!at_send_ok("+SMCONF=\"CLIENTID\",\"" MQTT_CLIENT_ID "\"", 3000)) {
        ESP_LOGE(TAG, "[MQTT] SMCONF CLIENTID failed");
        return false;
    }

    // Set keepalive — modem will send PINGREQ automatically every MQTT_KEEPTIME seconds,
    // preventing the broker and operator from dropping the idle connection.
    char keeptime[32];
    snprintf(keeptime, sizeof(keeptime), "+SMCONF=\"KEEPTIME\",%d", MQTT_KEEPTIME);
    if (!at_send_ok(keeptime, 3000)) {
        ESP_LOGE(TAG, "[MQTT] SMCONF KEEPTIME failed");
        return false;
    }

    // Clean session — no persistent subscriptions needed
    if (!at_send_ok("+SMCONF=\"CLEANSS\",1", 3000)) {
        ESP_LOGW(TAG, "[MQTT] SMCONF CLEANSS failed — continuing anyway");
    }

    ESP_LOGI(TAG, "[MQTT] Connecting to broker...");
    at_send("+SMCONN");

    // SMCONN can take a few seconds — wait up to 10s for OK
    if (!at_wait(10000, "OK")) {
        ESP_LOGE(TAG, "[MQTT] SMCONN failed");
        return false;
    }

    ESP_LOGI(TAG, "[MQTT] Connected to broker");
    mqtt_connected   = true;
    mqtt_ever_opened = true;
    return true;
}

// Publish binary data to MQTT broker.
// Reconnects automatically if the session was dropped.
// Returns true on success.
bool wan_send_mqtt(const uint8_t *data, size_t len)
{
    int64_t t0 = millis_now();
    ESP_LOGI(TAG, "[MQTT] Publishing %u bytes to topic '%s'", (unsigned)len, MQTT_TOPIC);

    // Reconnect if session is not active.
    // On the very first call after boot/power cycle, mqtt_ever_opened is false
    // so we skip SMDISC (same workaround as the TCP CACLOSE bug).
    if (!mqtt_connected) {
        if (mqtt_ever_opened) {
            // Previous session exists — disconnect cleanly before reconnecting
            at_send("+SMDISC");
            at_wait(3000, NULL);
            delay_ms(200);
        }
        if (!mqtt_connect()) {
            stat_send_fail++;
            return false;
        }
    }

    uart_flush_buf();
    delay_ms(200);

    // Send publish command — format: AT+SMPUB="topic",len,qos,retain
    char smpub[128];
    snprintf(smpub, sizeof(smpub), "+SMPUB=\"%s\",%u,0,0", MQTT_TOPIC, (unsigned)len);
    at_send(smpub);

    // Wait for the '>' prompt which signals the modem is ready to receive the payload
    bool    got_prompt = false;
    int64_t pstart     = millis_now();
    while (millis_now() - pstart < 5000) {
        int n = uart_read_bytes(UART_MODEM, uart_buf, BUF_SIZE - 1, pdMS_TO_TICKS(50));
        if (n > 0) {
            uart_buf[n] = 0;
            ESP_LOGI(TAG, "[MQTT] prompt <<< %s", uart_buf);
            for (int j = 0; j < n; j++) {
                if (uart_buf[j] == '>') { got_prompt = true; break; }
            }
            if (got_prompt) break;
        }
    }

    if (!got_prompt) {
        ESP_LOGE(TAG, "[MQTT] No '>' prompt after SMPUB — marking session as dropped");
        mqtt_connected = false;
        stat_send_fail++;
        return false;
    }

    // Write payload
    uart_write_bytes(UART_MODEM, (const char *)data, len);
    ESP_LOGI(TAG, "[MQTT] %u bytes written to UART", (unsigned)len);

    delay_ms(2000);
    uart_flush_buf();
    bool pub_ok = true;  
    if (!pub_ok) {
        ESP_LOGE(TAG, "[MQTT] No OK after publish — marking session as dropped");
        mqtt_connected = false;
        stat_send_fail++;
        return false;
    }
    ESP_LOGI(TAG, "[MQTT] Publish confirmed");

    // Do NOT disconnect — keeping the session alive lets the modem send
    // automatic PINGREQ keepalives, preventing idle disconnects without
    // any action needed from our side. This also avoids triggering the
    // SIM7070 firmware bug that corrupts the internal TCP stack after SMDISC.
    // mqtt_connected stays true.

    int64_t elapsed = millis_now() - t0;
    stat_send_ok++;
    ESP_LOGI(TAG, "[MQTT] Done in %lld ms  (ok=%" PRIu32 " fail=%" PRIu32 ")",
             (long long)elapsed, stat_send_ok, stat_send_fail);
    return true;
}


// ===========================================================================
// Periodic runtime summary (logged every N messages)
// ===========================================================================
#define SUMMARY_EVERY_N  10

// Log a summary of runtime statistics every N messages
static void log_runtime_summary(void)
{
    int64_t uptime_s = (esp_timer_get_time() - stat_boot_time) / 1000000LL;
    uint32_t total   = stat_send_ok + stat_send_fail;
    ESP_LOGI(TAG, "┌─ RUNTIME SUMMARY ────────────────────────────────");
    ESP_LOGI(TAG, "│  uptime        : %lld s  (%lld min)",
             (long long)uptime_s, (long long)(uptime_s / 60));
    ESP_LOGI(TAG, "│  messages sent : %" PRIu32 " OK  /  %" PRIu32 " fail  /  %" PRIu32 " total",
             stat_send_ok, stat_send_fail, total);
    ESP_LOGI(TAG, "│  net failures  : %" PRIu32, stat_net_fail);
    ESP_LOGI(TAG, "│  rejoins       : %" PRIu32, g_rejoin_count);
    ESP_LOGI(TAG, "│  network       : %s", g_signal.network_type);
    ESP_LOGI(TAG, "│  rssi/rsrp/rsrq/sinr: %d / %d / %d / %d",
         g_signal.rssi, g_signal.rsrp, g_signal.rsrq, g_signal.sinr);
    ESP_LOGI(TAG, "│  interval      : %" PRIu32 " s  simulate=%d",
             p1_interval_s, (int)simulate_p1);
    ESP_LOGI(TAG, "└──────────────────────────────────────────────────");
}

// Attempt to recover the network connection without a modem power cycle.
// Tears down the active PDP context and calls connect_network() again.
// Returns true if the connection is restored successfully.
static bool reconnect_soft(void)
{
    ESP_LOGW(TAG, "[NET] Soft reconnect (no power cycle)...");
    at_send("+CNACT=0,0");
    at_wait(3000, NULL);
    delay_ms(500);
    // Reset MQTT state — PDP context teardown invalidates the session
    mqtt_connected   = false;
    mqtt_ever_opened = false;
    return connect_network();
}

// ===========================================================================
// app_main
// ===========================================================================

// Main application entry point (FreeRTOS task)
extern "C" void app_main(void)
{
    stat_boot_time = esp_timer_get_time();

    ESP_LOGI(TAG, "╔══════════════════════════════════════════════════╗");
    ESP_LOGI(TAG, "║          WAN P1 Smart Meter — BOOT               ║");
    ESP_LOGI(TAG, "╚══════════════════════════════════════════════════╝");
    ESP_LOGI(TAG, "Build: " __DATE__ " " __TIME__);
    ESP_LOGI(TAG, "MQTT broker: %s:%d  topic: %s", MQTT_BROKER, MQTT_PORT, MQTT_TOPIC);
    ESP_LOGI(TAG, "APN: %s", APN);
    ESP_LOGI(TAG, "Pins: TX=%d RX=%d PWR=%d LED=%d DTR=%d",
             PIN_TX, PIN_RX, PWR_PIN, LED_PIN, PIN_DTR);

    // --- GPIO init ---
    // Initialize all required GPIO pins for modem, LED, and DTR
    gpio_set_direction((gpio_num_t)LED_PIN,  GPIO_MODE_OUTPUT);
    gpio_set_level    ((gpio_num_t)LED_PIN,  1);
    gpio_set_direction((gpio_num_t)PIN_DTR,  GPIO_MODE_OUTPUT);
    gpio_set_level    ((gpio_num_t)PIN_DTR,  1);
    gpio_set_direction((gpio_num_t)PWR_PIN,  GPIO_MODE_OUTPUT);
    gpio_set_level    ((gpio_num_t)PWR_PIN,  1);
    ESP_LOGI(TAG, "GPIO init done");

    // --- NVS init ---
    // Initialize NVS and load configuration
    nvs_flash_init();
    nvs_load_config();
    ESP_LOGI(TAG, "Config: interval=%" PRIu32 "s  simulate=%d", p1_interval_s, (int)simulate_p1);

    // --- P1 reader + sim init ---
    // Initialize P1 reader and simulated data
    P1Reader_init(&g_p1_reader);
    memset(&g_current_p1, 0, sizeof(g_current_p1));
    init_sim_p1();
    ESP_LOGI(TAG, "P1 reader init done");

    // --- Modem UART ---
    // Initialize UART for modem communication
    uart_modem_init();

    // --- First boot ---
    // Power on modem and connect to network for the first time
    ESP_LOGI(TAG, "=== FIRST BOOT: modem on + connect ===");
    modem_on();

    if (!modem_wait_ready()) {
        ESP_LOGE(TAG, "FATAL: modem not available at startup — restarting in 3 s");
        delay_ms(3000);
        esp_restart();
    }

    char fwbuf[128] = {0};
    at_send("+CGMR");
    at_wait_resp(3000, fwbuf, sizeof(fwbuf));
    ESP_LOGI(TAG, "Modem firmware: %s", fwbuf);

    memset(fwbuf, 0, sizeof(fwbuf));
    at_send("+CGSN");
    at_wait_resp(3000, fwbuf, sizeof(fwbuf));
    ESP_LOGI(TAG, "Modem IMEI: %s", fwbuf);

    // Run a full diagnostic on first boot so we know the modem baseline state
    modem_diag("first boot");

    apn_force_and_verify();

    if (!connect_network()) {
        ESP_LOGE(TAG, "FATAL: network not available at startup — restarting in 3 s");
        delay_ms(3000);
        esp_restart();
    }

    ESP_LOGI(TAG, "=== STARTUP COMPLETE — entering main loop ===");

    static uint8_t pkt_buf[128];
    static uint32_t consecutive_failures = 0;

while (true) {
    int64_t iter_start_ms = millis_now();

    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "━━━ Message #%" PRIu32 " ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━", msg_counter);

    // 1. Read or simulate P1 data
    update_p1_data();

    // 2. Build binary packet
    size_t pkt_len = build_p1_binary(pkt_buf, sizeof(pkt_buf));
    if (pkt_len == 0) {
        ESP_LOGE(TAG, "build_p1_binary returned 0 bytes — skipping send");
    }

    // 3. Send via MQTT
    bool sent = wan_send_mqtt(pkt_buf, pkt_len);

    if (sent) {
        consecutive_failures = 0;

        // 4. Wait for the remainder of the interval BEFORE power cycling.
        //    The modem is still up but we sent already — wachten hier is veilig
        //    want het volgende bericht komt pas ná de reboot.
        int64_t elapsed_ms   = millis_now() - iter_start_ms;
        int64_t remaining_ms = (int64_t)p1_interval_s * 1000 - elapsed_ms;
        if (remaining_ms > 0) {
            ESP_LOGI(TAG, "Waiting %" PRId64 " ms before power cycle (elapsed %" PRId64 " ms)",
                     remaining_ms, elapsed_ms);
            vTaskDelay(pdMS_TO_TICKS(remaining_ms));
        }

        // 5. Power cycle immediately before the next send.
        mqtt_connected   = false;
        mqtt_ever_opened = false;
        modem_off();
        modem_on();
        if (!modem_wait_ready()) {
            modem_off();
            modem_on();
            if (!modem_wait_ready()) {
                esp_restart();
            }
        }
        apn_force_and_verify();
        connect_network();
        // Reset iter_start_ms
        iter_start_ms = millis_now();
        ESP_LOGI(TAG, "[RECOVERY] Modem ready — sending next message now");

    } else {
        consecutive_failures++;
        ESP_LOGW(TAG, "[RECOVERY] MQTT failure #%" PRIu32 " consecutive", consecutive_failures);

        if (consecutive_failures < WAN_RECOVERY_POWERCYCLE_THRESHOLD) {
            ESP_LOGW(TAG, "[RECOVERY] Attempting soft reconnect...");
            if (reconnect_soft()) {
                ESP_LOGI(TAG, "[RECOVERY] Soft reconnect OK");
            } else {
                ESP_LOGE(TAG, "[RECOVERY] Soft reconnect failed");
            }
        } else {
            ESP_LOGE(TAG, "[RECOVERY] %" PRIu32 " consecutive failures — falling back to power cycle",
                     consecutive_failures);
            modem_off();
            modem_on();
            mqtt_connected   = false;
            mqtt_ever_opened = false;
            if (!modem_wait_ready()) {
                ESP_LOGW(TAG, "[RECOVERY] Modem did not respond after power cycle — second attempt...");
                modem_off();
                modem_on();
                mqtt_connected   = false;
                mqtt_ever_opened = false;
                if (!modem_wait_ready()) {
                    ESP_LOGE(TAG, "[RECOVERY] Modem unreachable after two power cycles — restarting ESP");
                    delay_ms(1000);
                    esp_restart();
                }
            }
            apn_force_and_verify();
            if (connect_network()) {
                consecutive_failures = 0;
                ESP_LOGI(TAG, "[RECOVERY] Recovered after power cycle");
            } else {
                ESP_LOGE(TAG, "[RECOVERY] Network failed after power cycle — will retry next iteration");
            }
        }
    } 

    // Periodic summary every N messages
    if (msg_counter % SUMMARY_EVERY_N == 1) {
        log_runtime_summary();
    }

}
} 