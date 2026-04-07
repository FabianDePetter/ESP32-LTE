// File: p1_reader.h
// P1 Reader interface for EMUCS/DSMR smart meter telegrams

#pragma once
#include <stdint.h>
#include "p1_data.h"
#include "esp_log.h"

#define P1_UART_NUM UART_NUM_1
#define P1_BAUD_RATE 115200
#define P1_BUF_SIZE 2400
#define REQUEST_LINE_PIN GPIO_NUM_32
#define DATA_LINE_PIN GPIO_NUM_33
#define REQUEST_LINE_HIGH_DELAY_MS 5
#define P1_TELEGRAM_START '/'
#define P1_TELEGRAM_DATA_END '!'
#define MAX_TELEGRAM_LENGTH 2400
#define TELEGRAM_TIMEOUT 2000

typedef struct {
    char buffer[P1_BUF_SIZE];        // Raw telegram buffer (null-terminated)
    p1_data_t data;                  // Parsed data structure (all OBIS fields)
} P1Reader;

void P1Reader_init(P1Reader *reader);
bool P1Reader_read_telegram(P1Reader *reader);
bool P1Reader_parse_telegram(P1Reader *reader);
p1_data_t P1Reader_get_data(P1Reader *reader);
void P1Reader_set_log_level(esp_log_level_t level);
void P1Reader_request_and_read(P1Reader *reader);
