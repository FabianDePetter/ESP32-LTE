/*
 * File: p1_reader.cpp
 *
 * P1 Reader Implementation for ESP32-Thread-Endpoint
 *
 * This file provides a complete implementation for reading EMUCS/DSMR P1 telegrams
 * from a digital smart meter using an ESP32 microcontroller. It is designed for
 * use in a Thread/Matter endpoint.
 *
 * Features:
 * - Controls the P1 request line using a MOSFET-based level shifter (active HIGH logic)
 * - Configures and manages UART for reliable reception of serial telegram data
 * - Reads, validates (CRC), and parses telegrams into a structured C data type
 * - Parses all OBIS codes (electricity, gas, water, relays, etc.)
 * - Handles all timing, retries, and error conditions robustly
 *
 * Hardware Context:
 * - The request line is connected to the ESP32 via a MOSFET-based level shifter.
 *   Setting the GPIO HIGH triggers the smart meter to send a telegram.
 * - The data line from the smart meter is connected to a UART RX pin on the ESP32.
 * - The code assumes 3.3V logic on the ESP32 side and 5V tolerant level shifting.
 *
 * Architecture Overview:
 * - The main interface is the P1Reader struct, which holds buffers and parsed data.
 * - set_request_line_floating() disables the request line output (high-impedance)
 * - P1Reader_init() configures all hardware and prepares the reader
 * - P1Reader_request_and_read() triggers a telegram request and reads the response
 * - P1Reader_read_telegram() handles UART reception and timeout logic
 * - P1Reader_parse_telegram() extracts all relevant fields from the received telegram
 * - validate_crc() performs CRC validation and stores the received CRC
 *
 * Author: [Fabian De Petter]
 * Date: 2026-03-26 (extended parsing)
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_rom_crc.h"
#include "hal/uart_types.h"
#include <string.h>
#include <time.h>
#include <sys/time.h>
#include <inttypes.h>
#include <stdlib.h>
#include <ctype.h>
#include "p1_reader.h"
#include <inttypes.h>   // for SCNx16

static const char *TAG = "P1_READER";
static char telegram[MAX_TELEGRAM_LENGTH];

// -------------------------------------------------------------------------
// Helper: extract a value between parentheses (e.g., "123.456*kWh")
// The returned string is null-terminated and points to the value part (without parentheses).
// If no parentheses found, returns NULL.
static char* extract_parentheses_value(const char* start, char* buffer, size_t buffer_size) {
    char* open = strchr(start, '(');
    if (!open) return NULL;
    char* close = strchr(open, ')');
    if (!close) return NULL;
    size_t len = close - open - 1;
    if (len >= buffer_size) len = buffer_size - 1;
    strncpy(buffer, open + 1, len);
    buffer[len] = '\0';
    return buffer;
}

// Helper: parse a float value and multiply by a scaling factor, returning integer.
static int32_t parse_float_to_int(const char *str, int multiplier) {
    float value = 0.0f;
    sscanf(str, "%f", &value);
    return (int32_t)(value * multiplier);
}

// Helper: parse a string field (no conversion)
static void parse_string_field(const char* start, char* dest, size_t dest_size) {
    char buffer[64];
    if (extract_parentheses_value(start, buffer, sizeof(buffer))) {
        strncpy(dest, buffer, dest_size - 1);
        dest[dest_size - 1] = '\0';
    } else {
        dest[0] = '\0';
    }
}

// Helper: parse a numeric field (int) with scaling
static int32_t parse_numeric_field(const char* start, int multiplier) {
    char buffer[32];
    if (extract_parentheses_value(start, buffer, sizeof(buffer))) {
        return parse_float_to_int(buffer, multiplier);
    }
    return 0;
}

// Helper: parse a boolean/uint8_t field (0 or 1)
static uint8_t parse_boolean_field(const char* start) {
    char buffer[8];
    if (extract_parentheses_value(start, buffer, sizeof(buffer))) {
        return (uint8_t)atoi(buffer);
    }
    return 0;
}

// -------------------------------------------------------------------------
// Request line control
void set_request_line_floating() {
    gpio_set_direction(REQUEST_LINE_PIN, GPIO_MODE_DISABLE);
    ESP_LOGI(TAG, "Request line set to floating (high-impedance) state");
}

// -------------------------------------------------------------------------
// Initialization
void P1Reader_init(P1Reader *reader) {
    ESP_LOGI(TAG, "DATA_LINE_PIN = %d, REQUEST_LINE_PIN = %d", DATA_LINE_PIN, REQUEST_LINE_PIN);
    ESP_LOGI(TAG, "Initializing P1Reader with UART configuration.");

    memset(reader->buffer, 0, P1_BUF_SIZE);
    memset(&reader->data, 0, sizeof(p1_data_t));

    // GPIO for request line (floating)
    gpio_config_t request_line_conf = {
        .pin_bit_mask = (1ULL << REQUEST_LINE_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
#if SOC_GPIO_SUPPORT_PIN_HYS_FILTER
        .hys_ctrl_mode = GPIO_HYS_SOFT_DISABLE
#endif
    };
    gpio_config(&request_line_conf);
    ESP_LOGI(TAG, "Request line configured on GPIO %d as floating/high-impedance", REQUEST_LINE_PIN);

    // UART configuration
    uart_config_t uart_config = {
        .baud_rate = P1_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 0,
        .source_clk = (uart_sclk_t)0,
        .flags = { .allow_pd = 0, .backup_before_sleep = 0 }
    };
    if (uart_param_config(UART_NUM_1, &uart_config) != ESP_OK) {
        ESP_LOGE(TAG, "uart_param_config failed!");
        return;
    }
    if (uart_set_pin(UART_NUM_1, UART_PIN_NO_CHANGE, DATA_LINE_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE) != ESP_OK) {
        ESP_LOGE(TAG, "uart_set_pin failed!");
        return;
    }
    if (uart_driver_install(UART_NUM_1, P1_BUF_SIZE * 2, 0, 0, NULL, 0) != ESP_OK) {
        ESP_LOGE(TAG, "uart_driver_install failed!");
        return;
    }
    uart_flush(UART_NUM_1);
    uart_flush_input(UART_NUM_1);
    //uart_set_line_inverse(UART_NUM_1, UART_SIGNAL_RXD_INV);

    ESP_LOGI(TAG, "UART configured: RX on GPIO %d, baud rate %d", DATA_LINE_PIN, P1_BAUD_RATE);
    ESP_LOGI(TAG, "P1Reader initialized successfully.");
}


// -------------------------------------------------------------------------
// Read telegram from UART
// -------------------------------------------------------------------------
bool P1Reader_read_telegram(P1Reader *reader) {
    uint8_t chunk[64];
    int timeout_count = 0;
    const int max_timeout = 300; // 3 seconds
    size_t len = 0;
    bool end_detected = false;

    ESP_LOGI(TAG, "Reading telegram from UART (chunk mode).");
    uart_flush_input(UART_NUM_1);

    while (timeout_count < max_timeout && !end_detected) {
        int bytes_read = uart_read_bytes(UART_NUM_1, chunk, sizeof(chunk),
                                         10 / portTICK_PERIOD_MS);
        if (bytes_read > 0) {
            timeout_count = 0;
            for (int i = 0; i < bytes_read; i++) {
                if (len == 0 && chunk[i] != '/') continue;
                if (len == 0) {
                    ESP_LOGI(TAG, "Telegram start detected: 0x%02X ('%c')", chunk[i], chunk[i]);
                }
                if (len < MAX_TELEGRAM_LENGTH - 1) {
                    telegram[len++] = chunk[i];
                } else {
                    ESP_LOGW(TAG, "Telegram too long, discarding.");
                    return false;
                }
                if (chunk[i] == P1_TELEGRAM_DATA_END) {
                    ESP_LOGI(TAG, "End of telegram detected at position %d.", len);
                    end_detected = true;
                    // Do NOT break; continue processing the rest of this chunk
                }
            }
            // After processing the whole chunk, if end was detected, read remaining CRC line.
            // Only extract exactly 4 hex digits after '!' and ignore any surrounding noise bytes.
            if (end_detected) {
                vTaskDelay(pdMS_TO_TICKS(50));
                int extra = uart_read_bytes(UART_NUM_1, chunk, sizeof(chunk),
                                            200 / portTICK_PERIOD_MS);
                int hex_count = 0;
                for (int j = 0; j < extra && len < MAX_TELEGRAM_LENGTH - 1; j++) {
                    if (isxdigit((unsigned char)chunk[j])) {
                        telegram[len++] = chunk[j];
                        hex_count++;
                        if (hex_count == 4) break; // Exactly 4 hex digits read, stop here
                    }
                    // Skip non-hex characters (noise, \r, \n, garbage bytes)
                }
                telegram[len] = '\0';
                strncpy(reader->buffer, telegram, P1_BUF_SIZE - 1);
                reader->buffer[P1_BUF_SIZE - 1] = '\0';
                ESP_LOGI(TAG, "Full telegram received (%d bytes).", len);
                return true;
            }
        } else {
            if (timeout_count % 20 == 0 && timeout_count > 0) {
                ESP_LOGI(TAG, "Waiting for data... (%d/%d ms)",
                         timeout_count * 10, max_timeout * 10);
            }
            timeout_count++;
        }
    }

    ESP_LOGW(TAG, "Telegram read timeout after %d ms.", max_timeout * 10);
    if (len > 0) {
        ESP_LOGW(TAG, "Partial data received (%d bytes):", len);
        ESP_LOG_BUFFER_HEXDUMP(TAG, telegram, len, ESP_LOG_WARN);
    }
    return false;
}
// -------------------------------------------------------------------------
// CRC-16 for DSMR P1 telegrams (MSB-first, polynomial 0x8005, init 0x0000)
static uint16_t crc16_p1(const uint8_t *data, size_t len) {
    uint16_t crc = 0x0000;
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i];
        for (int bit = 0; bit < 8; bit++) {
            if (crc & 1) {
                crc = (crc >> 1) ^ 0x8005;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

bool validate_crc(P1Reader *reader) {
    ESP_LOGI(TAG, "Validating CRC for the telegram.");

    size_t buflen = strlen(reader->buffer);
    if (buflen > 30) {
        ESP_LOGI(TAG, "Buffer tail: ...%s", reader->buffer + buflen - 30);
    } else {
        ESP_LOGI(TAG, "Full buffer: %s", reader->buffer);
    }

    const char *crc_marker = strchr(reader->buffer, P1_TELEGRAM_DATA_END);
    if (!crc_marker || strlen(crc_marker) < 5) {
        ESP_LOGW(TAG, "CRC marker not found or too short.");
        reader->data.crc_valid = false;
        return false;
    }

    // Extract exactly 4 hex digits after '!', skipping any non-hex noise
    char crc_str[5] = {0};
    int hex_count = 0;
    for (int i = 1; hex_count < 4 && crc_marker[i] != '\0'; i++) {
        if (isxdigit((unsigned char)crc_marker[i])) {
            crc_str[hex_count++] = crc_marker[i];
        }
    }
    if (hex_count < 4) {
        ESP_LOGW(TAG, "Failed to extract 4 CRC hex digits, only got %d.", hex_count);
        reader->data.crc_valid = false;
        return false;
    }

    uint16_t received_crc;
    if (sscanf(crc_str, "%04" SCNx16, &received_crc) != 1) {
        ESP_LOGW(TAG, "Failed to parse CRC hex digits.");
        reader->data.crc_valid = false;
        return false;
    }
    reader->data.crc = received_crc;

    // Include the '!' in the CRC calculation
    size_t telegram_length = crc_marker - reader->buffer + 1;
    uint16_t calculated_crc = crc16_p1((const uint8_t *)reader->buffer, telegram_length);

    if (received_crc == calculated_crc) {
        ESP_LOGI(TAG, "CRC validation passed (CRC=0x%04X).", received_crc);
        reader->data.crc_valid = true;
        return true;
    } else {
        ESP_LOGW(TAG, "CRC validation failed. Received: 0x%04X, Calculated: 0x%04X",
                 received_crc, calculated_crc);
        reader->data.crc_valid = false;
        return false;
    }
}

// -------------------------------------------------------------------------
// Main parsing routine
bool P1Reader_parse_telegram(P1Reader *reader) {
    ESP_LOGI(TAG, "Parsing telegram.");

    bool crc_was_valid = reader->data.crc_valid;
    // Clear the data structure first
    memset(&reader->data, 0, sizeof(p1_data_t));
    reader->data.crc_valid = crc_was_valid;

    char *buffer = reader->buffer;
    char line[256]; // temporary line buffer

    // -----------------------------------------------------------------
    // 1. First line: manufacturer and serial (e.g., "/FLU5\253769484_A")
    char *first_line_end = strchr(buffer, '\n');
    if (first_line_end) {
        size_t first_line_len = first_line_end - buffer;
        if (first_line_len > 0 && first_line_len < sizeof(line)) {
            strncpy(line, buffer, first_line_len);
            line[first_line_len] = '\0';
            // Find the slash and backslash
            char *slash = strchr(line, '/');
            char *backslash = strchr(line, '\\');
            if (slash && backslash && backslash > slash) {
                size_t mfg_len = backslash - slash - 1;
                if (mfg_len < sizeof(reader->data.manufacturer))
                    strncpy(reader->data.manufacturer, slash + 1, mfg_len);
                reader->data.manufacturer[mfg_len] = '\0';
                size_t serial_len = strlen(backslash + 1);
                if (serial_len < sizeof(reader->data.serial))
                    strncpy(reader->data.serial, backslash + 1, serial_len);
                reader->data.serial[serial_len] = '\0';
            } else {
                // fallback: just copy the whole line as manufacturer? Not ideal.
                strncpy(reader->data.manufacturer, line, sizeof(reader->data.manufacturer)-1);
                reader->data.manufacturer[sizeof(reader->data.manufacturer)-1] = '\0';
            }
        }
    }

    // -----------------------------------------------------------------
    // Helper macro to find an OBIS line and extract its value
    #define PARSE_STRING(marker, field) do { \
        char* p = strstr(buffer, marker); \
        if (p) parse_string_field(p, reader->data.field, sizeof(reader->data.field)); \
    } while(0)

    #define PARSE_NUM(marker, field, mult) do { \
        char* p = strstr(buffer, marker); \
        if (p) reader->data.field = (typeof(reader->data.field))parse_numeric_field(p, mult); \
    } while(0)

    #define PARSE_BOOL(marker, field) do { \
        char* p = strstr(buffer, marker); \
        if (p) reader->data.field = parse_boolean_field(p); \
    } while(0)

    // -----------------------------------------------------------------
    // Parse all OBIS codes

    // Version and ID strings
    PARSE_STRING("0-0:96.1.4(", dsmr_version);
    PARSE_STRING("1-0:94.32.1(", firmware_version);
    PARSE_STRING("0-0:96.1.1(", meter_id);
    PARSE_STRING("0-0:96.1.2(", ean_code);

    // Timestamp
    PARSE_STRING("0-0:1.0.0(", timestamp);

    // Energy
    PARSE_NUM("1-0:1.8.1(", energy_import_t1, 1000);
    PARSE_NUM("1-0:1.8.2(", energy_import_t2, 1000);
    PARSE_NUM("1-0:2.8.1(", energy_export_t1, 1000);
    PARSE_NUM("1-0:2.8.2(", energy_export_t2, 1000);

    // Tariff
    PARSE_NUM("0-0:96.14.0(", active_tariff, 1);

    // Demand / peaks
    PARSE_NUM("1-0:1.4.0(", avg_demand_quarter, 1000);
    // Parse max_demand_month (1-0:1.6.0) – it has two parentheses: (timestamp)(value*kW)
    char *max_demand_start = strstr(buffer, "1-0:1.6.0(");
    if (max_demand_start) {
        // Find the first closing parenthesis after the marker
        char *first_close = strchr(max_demand_start + strlen("1-0:1.6.0("), ')');
        if (first_close) {
            // Find the second '(' after that
            char *second_open = strchr(first_close, '(');
            if (second_open) {
                char *second_close = strchr(second_open, ')');
                if (second_close) {
                    size_t val_len = second_close - second_open - 1;
                    char val_str[32];
                    if (val_len < sizeof(val_str)) {
                        strncpy(val_str, second_open + 1, val_len);
                        val_str[val_len] = '\0';
                        reader->data.max_demand_month = (int16_t)parse_float_to_int(val_str, 1000);
                    }
                }
            }
        }
    }

    char *history_start = strstr(buffer, "0-0:98.1.0(");
if (history_start) {
    int found = 0;
    char *ptr = history_start;
    // Skip the count field e.g. "(13)"
    ptr = strchr(ptr, ')');
    if (ptr) ptr++;
    // Skip the two unit OBIS fields e.g. "(1-0:1.6.0)(1-0:1.6.0)"
    for (int skip = 0; skip < 2 && ptr; skip++) {
        ptr = strchr(ptr, '(');
        if (ptr) ptr = strchr(ptr, ')');
        if (ptr) ptr++;
    }
    // Now parse each entry: (ts)(ts)(value*kW)
    while (found < 13 && ptr) {
        // Skip first timestamp
        ptr = strchr(ptr, '(');
        if (!ptr) break;
        ptr = strchr(ptr, ')');
        if (!ptr) break;
        ptr++;
        // Skip second timestamp
        ptr = strchr(ptr, '(');
        if (!ptr) break;
        ptr = strchr(ptr, ')');
        if (!ptr) break;
        ptr++;
        // Read value
        char *val_open = strchr(ptr, '(');
        if (!val_open) break;
        char *val_close = strchr(val_open, ')');
        if (!val_close) break;
        size_t val_len = val_close - val_open - 1;
        char val_str[16];
        if (val_len < sizeof(val_str)) {
            strncpy(val_str, val_open + 1, val_len);
            val_str[val_len] = '\0';
            reader->data.max_demand_history[found] = 
                (int16_t)parse_float_to_int(val_str, 1000);
            found++;
        }
        ptr = val_close + 1;
    }
}

    // Instantaneous power (total and per phase)
    PARSE_NUM("1-0:1.7.0(", power_import_total, 1000);
    PARSE_NUM("1-0:2.7.0(", power_export_total, 1000);
    PARSE_NUM("1-0:21.7.0(", power_import_l1, 1000);
    PARSE_NUM("1-0:22.7.0(", power_export_l1, 1000);
    PARSE_NUM("1-0:41.7.0(", power_import_l2, 1000);
    PARSE_NUM("1-0:42.7.0(", power_export_l2, 1000);
    PARSE_NUM("1-0:61.7.0(", power_import_l3, 1000);
    PARSE_NUM("1-0:62.7.0(", power_export_l3, 1000);

    // Voltage
    PARSE_NUM("1-0:32.7.0(", voltage_l1, 10);
    PARSE_NUM("1-0:52.7.0(", voltage_l2, 10);
    PARSE_NUM("1-0:72.7.0(", voltage_l3, 10);

    // Current
    PARSE_NUM("1-0:31.7.0(", current_l1, 100);
    PARSE_NUM("1-0:51.7.0(", current_l2, 100);
    PARSE_NUM("1-0:71.7.0(", current_l3, 100);

    // Status / limits
    PARSE_BOOL("0-0:96.3.10(", main_breaker);
    PARSE_NUM("0-0:17.0.0(", power_limiter, 1000);
    PARSE_NUM("1-0:31.4.0(", fuse_supervision_l1, 100);

    // M-Bus relays (channels 1-4)
    PARSE_BOOL("0-1:96.3.10(", mbus_relay[0]);
    PARSE_BOOL("0-2:96.3.10(", mbus_relay[1]);
    PARSE_BOOL("0-3:96.3.10(", mbus_relay[2]);
    PARSE_BOOL("0-4:96.3.10(", mbus_relay[3]);

    // Operator text
    PARSE_STRING("0-0:96.13.0(", operator_text);

    // -----------------------------------------------------------------
    // Gas meter
    PARSE_STRING("0-1:24.1.0(", gas_type);
    PARSE_STRING("0-1:96.1.1(", gas_id);
    PARSE_STRING("0-1:96.1.2(", gas_ean);
    PARSE_BOOL("0-1:24.4.0(", gas_valve_status);

    // Gas consumption (0-1:24.2.3) has two parentheses: (timestamp)(value*m3)
    char *gas_consumption_start = strstr(buffer, "0-1:24.2.3(");
    if (gas_consumption_start) {
        // Find the first '(' after the marker (timestamp)
        char *first_open = strchr(gas_consumption_start, '(');
        if (first_open) {
            char *first_close = strchr(first_open, ')');
            if (first_close) {
                // Extract timestamp
                size_t ts_len = first_close - first_open - 1;
                if (ts_len < sizeof(reader->data.gas_timestamp)) {
                    strncpy(reader->data.gas_timestamp, first_open + 1, ts_len);
                    reader->data.gas_timestamp[ts_len] = '\0';
                }
                // Next, find the second '(' for the consumption value
                char *second_open = strchr(first_close, '(');
                if (second_open) {
                    char *second_close = strchr(second_open, ')');
                    if (second_close) {
                        size_t val_len = second_close - second_open - 1;
                        char val_str[32];
                        if (val_len < sizeof(val_str)) {
                            strncpy(val_str, second_open + 1, val_len);
                            val_str[val_len] = '\0';
                            // Value is in m³, convert to liters (multiply by 1000)
                            reader->data.gas_consumption = (uint32_t)parse_float_to_int(val_str, 1000);
                        }
                    }
                }
            }
        }
    }

    // -----------------------------------------------------------------
    // Water meter
    PARSE_STRING("0-2:24.1.0(", water_type);
    PARSE_STRING("0-2:96.1.1(", water_id);
    PARSE_STRING("0-2:96.1.2(", water_ean);

    // Water consumption (0-2:24.2.1) similar format
    char *water_consumption_start = strstr(buffer, "0-2:24.2.1(");
    if (water_consumption_start) {
        char *first_open = strchr(water_consumption_start, '(');
        if (first_open) {
            char *first_close = strchr(first_open, ')');
            if (first_close) {
                size_t ts_len = first_close - first_open - 1;
                if (ts_len < sizeof(reader->data.water_timestamp)) {
                    strncpy(reader->data.water_timestamp, first_open + 1, ts_len);
                    reader->data.water_timestamp[ts_len] = '\0';
                }
                char *second_open = strchr(first_close, '(');
                if (second_open) {
                    char *second_close = strchr(second_open, ')');
                    if (second_close) {
                        size_t val_len = second_close - second_open - 1;
                        char val_str[32];
                        if (val_len < sizeof(val_str)) {
                            strncpy(val_str, second_open + 1, val_len);
                            val_str[val_len] = '\0';
                            reader->data.water_consumption = (uint32_t)parse_float_to_int(val_str, 1000);
                        }
                    }
                }
            }
        }
    }

    // -----------------------------------------------------------------
    // CRC is already stored in validate_crc, but we also need to store it here
    // (validate_crc is called separately, but we can also extract it if needed)
    // We'll rely on validate_crc to set reader->data.crc.

    // Clear any remaining unused fields (though we zeroed at start)
    return true;
}

// -------------------------------------------------------------------------
// Getter for parsed data
p1_data_t P1Reader_get_data(P1Reader *reader) {
    return reader->data;
}

// -------------------------------------------------------------------------
// Request and read full cycle
void P1Reader_request_and_read(P1Reader *reader) {
    ESP_LOGI(TAG, "Sending request and reading telegram (MOSFET logic, active HIGH).");
    uart_flush(P1_UART_NUM);
    uart_flush_input(P1_UART_NUM);
    bool pulse_mode = true;
    int max_retries = 3;
    int retry_count = 0;
    bool success = false;
    while (retry_count < max_retries && !success) {
        ESP_LOGI(TAG, "--- REQUEST ATTEMPT %d/%d ---", retry_count+1, max_retries);
        if (pulse_mode) {
            gpio_set_direction(REQUEST_LINE_PIN, GPIO_MODE_OUTPUT);
            gpio_set_level(REQUEST_LINE_PIN, 1);
            vTaskDelay(pdMS_TO_TICKS(REQUEST_LINE_HIGH_DELAY_MS));
            ESP_LOGI(TAG, "Request line HIGH (active, MOSFET) - start reading immediately");
            success = P1Reader_read_telegram(reader);
            gpio_set_level(REQUEST_LINE_PIN, 0);
            ESP_LOGI(TAG, "Request line set to LOW (end pulse, MOSFET)");
        } else {
            gpio_set_direction(REQUEST_LINE_PIN, GPIO_MODE_OUTPUT);
            gpio_set_level(REQUEST_LINE_PIN, 1);
            vTaskDelay(pdMS_TO_TICKS(REQUEST_LINE_HIGH_DELAY_MS));
            ESP_LOGI(TAG, "Request line HIGH (constant, MOSFET)");
            success = P1Reader_read_telegram(reader);
            gpio_set_level(REQUEST_LINE_PIN, 0);
            ESP_LOGI(TAG, "Request line set to LOW (end constant, MOSFET)");
        }
        if (!pulse_mode) {
            gpio_set_level(REQUEST_LINE_PIN, 0);
            ESP_LOGI(TAG, "Request line set to LOW (end constant, MOSFET)");
        }
        if (!success) {
            ESP_LOGW(TAG, "No telegram received, retrying (%d/%d)", retry_count+1, max_retries);
            ESP_LOGW(TAG, "Last buffer contents (ASCII): %s", reader->buffer);
            vTaskDelay(pdMS_TO_TICKS(REQUEST_LINE_HIGH_DELAY_MS));
        }
        retry_count++;
    }
    if (success) {
        // Validate CRC after successful read
        if (!validate_crc(reader)) {
            ESP_LOGE(TAG, "CRC validation failed, keeping previous data.");
            // Data structure is not cleared; crc_valid is false.
            // success remains true so that update_p1_data can still report.
        } else {
            // Parse the telegram into the data structure
            if (!P1Reader_parse_telegram(reader)) {
                ESP_LOGE(TAG, "Parsing failed.");
                memset(&reader->data, 0, sizeof(p1_data_t));
                success = false;
            }
        }
        size_t buflen = strlen(reader->buffer);
        ESP_LOGI(TAG, "Received telegram (%d bytes):", (int)buflen);
    } else {
        ESP_LOGE(TAG, "Failed to receive valid telegram after %d retries.", max_retries);
    }
}

void P1Reader_set_log_level(esp_log_level_t level) {
    esp_log_level_set(TAG, level);
}
