#ifndef __DS3231_H__
#define __DS3231_H__
#include "esp_err.h"

#include <stdint.h>

// idf release/5.0 : semphr.h mast include with FreeRTOS.h
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#include <driver/i2c_master.h>

#include <time.h>

#define DS3231_HOUR_FMT_MSK (0x40)

#define DS3231_EOSC_MSK (0x80)
#define DS3231_BBSQW_MSK (0x40)
#define DS3231_CONV_MSK (0x20)
#define DS3231_RSX_MSK (0x18)
#define DS3231_INTCH_MSK (0x04)
#define DS3231_A2IE_MSK (0x02)
#define DS3231_A1IE_MSK (0x01)

#define DS3231_OSF_MSK (0x80)
#define DS3231_EN32KHZ_MSK (0x08) // this is config
#define DS3231_BSY_MSK (0x04)
#define DS3231_A2F_MSK (0x02)
#define DS3231_A1F_MSK (0x01)

#define DS3231_ALARM1 (0)
#define DS3231_ALARM2 (1)

typedef enum
{
    DS_FREQ_1HZ = 0,  // 1hz
    DS_FREQ_1KHZ = 1, // 1.024khz
    DS_FREQ_4KHZ = 2, // 4.096khz
    DS_FREQ_8KHZ = 3, // 8.192khz
    DS_FREQ_MAX,
} __attribute__((packed)) ds_pluse_freq_t;

typedef enum
{
    DS_INT_MODE_PLUSE = 0,
    DS_INT_MODE_INT = 1,
    DS_INT_MODE_MAX,
} __attribute__((packed)) ds_int_mode_t;

typedef enum
{
    DS_ALARM_MODE_NO_MATCH = 0,  // Alarm once per second (alarm1) , Alarm once per minute  (alarm2)
    DS_ALARM_MODE_SEC_MATCH = 1, // Alarm when seconds match (alarm1 only)
    DS_ALARM_MODE_MIN_MATCH,     // Alarm when minutes (and seconds) match
    DS_ALARM_MODE_HOR_MATCH,     // Alarm when hours, minutes, (and seconds) match
    DS_ALARM_MODE_DATE_MATCH,    // Alarm when date, hours, minutes, (and seconds) match
    DS_ALARM_MODE_WEEK_MATCH,    // Alarm when day(week), hours, minutes, (and seconds) match
    DS_ALARM_MODE_ERROR,         // illogical operation
    DS_ALARM_MODE_MAX,
} __attribute__((packed)) ds_alarm_mode_t;

typedef struct
{
    // reg 0x0e config
    uint8_t osc_disable;                  // EOSC# : When set to logic 0, the oscillator is started. When set to logic 1, the oscillator is stopped when the DS3231 switches to VBAT.
    uint8_t pluse_out_in_bat_mode_enable; // BBSQW : When set to logic 1 and the DS3231 is being powered by the VBAT pin, this bit enables the square-wave or interrupt output when VCC is absent. When BBSQW is logic 0, the INT/SQW pin goes high impedance when VCC falls below the power-fail trip point
    ds_pluse_freq_t pluse_freq;           // RSX : frequency of the square-wave output
    ds_int_mode_t int_mode;               // INTCN : controls the INT#/SQW signal
    uint8_t alarm2_int_en;                // A2IE : When set to logic 1, this bit permits the alarm 2 flag (A2F) bit in the status register to assert INT/SQW (when INTCN = 1)
    uint8_t alarm1_int_en;                // A1IE : When set to logic 1, this bit permits the alarm 1 flag (A1F) bit in the status register to assert INT/SQW (when INTCN = 1).
    // reg 0x0f Status
    uint8_t pluse_enable; // EN32kHz : This bit controls the status of the 32kHz pin. When set to logic 1, the 32kHz pin is enabled and outputs a 32.768kHz square-wave signal. When set to logic 0, the 32kHz pin goes to a high-impedance state
} ds_config_t;

typedef struct
{
    // reg 0x0f Status
    uint8_t osc_stop;    // OSF : A logic 1 in this bit indicates that the oscillator either is stopped or was stopped for some period and may be used to judge the validity of the timekeeping data
    uint8_t tcxo_active; // BSY : This bit indicates the device is busy executing TCXO functions
    uint8_t alarm2;      // A2F : A logic 1 in the alarm 2 flag bit indicates that the time matched the alarm 2 registers.
    uint8_t alarm1;      // A1F : A logic 1 in the alarm 1 flag bit indicates that the time matched the alarm 1 registers.
} ds_flags_t;

esp_err_t ds3231_init(SemaphoreHandle_t *i2c_mux, i2c_master_dev_handle_t *i2c, int xfer_time_out);

esp_err_t ds3231_set_config(ds_config_t conf);
esp_err_t ds3231_get_config(ds_config_t *conf);
esp_err_t ds3231_get_flags(ds_flags_t *flag, uint8_t clean);

esp_err_t ds3231_get_time(struct tm *t);
esp_err_t ds3231_set_time(struct tm t);

esp_err_t ds3231_set_alarm(struct tm t, ds_alarm_mode_t match, uint8_t id);
esp_err_t ds3231_get_alarm(struct tm *t, ds_alarm_mode_t *match, uint8_t id);

esp_err_t ds3231_set_aging_offset(int8_t ago);
esp_err_t ds3231_get_aging_offset(int8_t *ago);

esp_err_t ds3231_get_temperature(float *t);

esp_err_t ds3231_active_tcxo();

void ds3231_debug_log_time(struct tm tm);
void ds3231_debug_log_config(ds_config_t *conf);
void ds3231_debug_log_flags(ds_flags_t *f);

void ds3231_debug_set_time_to_compile();
#endif