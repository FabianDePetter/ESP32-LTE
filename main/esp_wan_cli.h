/**
 * @file esp_wan_cli.h
 *
 * ESP32 WAN P1 Smart Meter over GPRS + TCP
 * - Integrates P1 reader (real UART or simulation)
 * - NVS-backed config: interval_s, simulate flag
 * - Power-cycle per iteration (firmware workaround — do NOT remove)
 *
 */
#pragma once
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h> 

#ifdef __cplusplus
extern "C" {
#endif

// ---------------------------------------------------------------------------
// Hardware pins (modem)
// ---------------------------------------------------------------------------
#define PIN_TX      27
#define PIN_RX      26
#define PWR_PIN      4
#define LED_PIN     12

/**
 * DTR pin for the SIM7000 modem.
 *
 * MUST NOT overlap with P1 reader pins:
 *   REQUEST_LINE_PIN = GPIO 32  (p1_reader.h)
 *   DATA_LINE_PIN    = GPIO 33  (p1_reader.h)
 *
 * Was 32 → changed to 25 to resolve that GPIO conflict.
 * The conflict caused P1Reader_init() to never receive data because
 * app_main drove GPIO 32 HIGH as DTR before the P1 reader could use it.
 *
 * If DTR is not physically wired on your board you can define
 * WAN_DTR_UNUSED and the pin will not be touched at all.
 */
#define PIN_DTR     25

// Compile-time guard: catch accidental re-introduction of the conflict.
#ifdef __cplusplus
static_assert(PIN_DTR != 32, "PIN_DTR conflicts with P1 REQUEST_LINE_PIN (GPIO 32)");
static_assert(PIN_DTR != 33, "PIN_DTR conflicts with P1 DATA_LINE_PIN (GPIO 33)");
static_assert(PIN_TX  != 32 && PIN_TX  != 33, "PIN_TX  conflicts with P1 reader pins");
static_assert(PIN_RX  != 32 && PIN_RX  != 33, "PIN_RX  conflicts with P1 reader pins");
static_assert(PWR_PIN != 32 && PWR_PIN != 33, "PWR_PIN conflicts with P1 reader pins");
static_assert(LED_PIN != 32 && LED_PIN != 33, "LED_PIN conflicts with P1 reader pins");
#endif

// ---------------------------------------------------------------------------
// UART (modem)
// ---------------------------------------------------------------------------
#define UART_MODEM  UART_NUM_2
#define UART_BAUD   115200

// ---------------------------------------------------------------------------
// MQTT broker
// ---------------------------------------------------------------------------
#define MQTT_BROKER      "94.110.48.167"  
#define MQTT_PORT        1883
#define MQTT_CLIENT_ID   "esp32_p1_meter"
#define MQTT_TOPIC       "p1/data"
#define MQTT_KEEPTIME    60   // keepalive in seconds — modem sends PINGREQ automatically

// ---------------------------------------------------------------------------
// APN
// ---------------------------------------------------------------------------
#define APN "sensor.net"

// ---------------------------------------------------------------------------
// NVS
// ---------------------------------------------------------------------------
#define WAN_NVS_NAMESPACE   "wancfg"
#define NVS_KEY_INTERVAL_S  "interval_s"
#define NVS_KEY_SIMULATE    "sim"

// ---------------------------------------------------------------------------
// Timing
// ---------------------------------------------------------------------------
/**
 * Default interval between telegram sends (seconds).
 *
 * The power-cycle after every send takes ~25 s:
 *   modem_off  ≈  3 s
 *   modem_on   ≈ 13 s
 *   probe      ≈  5 s (typically probe 6)
 *   reconnect  ≈  2 s
 *
 * The interval must therefore always be > 30 s.
 * After the power cycle the code waits for the remainder of the interval
 * before reading P1 + sending again.
 */
#define WAN_INTERVAL_DEFAULT_S  5u
#define WAN_INTERVAL_MIN_S      1u
#define WAN_INTERVAL_MAX_S    3600u

// Number of consecutive TCP failures before falling back to a modem power cycle.
#define WAN_RECOVERY_POWERCYCLE_THRESHOLD  1u

// ---------------------------------------------------------------------------
// JSON output buffer
// ---------------------------------------------------------------------------
/** Maximum size of the JSON payload sent over TCP. */
#define WAN_JSON_BUF_SIZE   2048

// ---------------------------------------------------------------------------
// Metrics
// ---------------------------------------------------------------------------
typedef struct {
    int16_t  rssi;             // dBm  (e.g. -85)
    int16_t  rsrp;             // dBm  (e.g. -105)
    int16_t  rsrq;             // dB   (e.g. -10), typically -3 to -20
    int16_t  sinr;             // dB   (e.g. 12)
    uint32_t rejoin_count;     // number of network reconnects since boot
    char     network_type[16]; // "LTE-M", "NB-IoT", "GSM", "NO SERVICE"
} signal_info_t;

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

/**
 * Send a null-terminated string over TCP (one shot, connection closed after).
 * Returns true on success.
 */
bool wan_send_mqtt(const uint8_t *data, size_t len);

#ifdef __cplusplus
}
#endif
