#include "ds3231.h"

#include "esp_log.h"
#include "esp_system.h"
#include "esp_check.h"

const static char *TAG = "ds3231";

// for idf 5.2

#define DS_GET_ALARM_MSK(x) (((x) & 0x80) ? 1 : 0)

static i2c_master_dev_handle_t *dev_iic;
static SemaphoreHandle_t *xSemIICFree;
static int ctp_xfer_time_out;

static esp_err_t ds_iic_readReg(uint8_t reg, void *dst, uint16_t len)
{
    if (xSemIICFree != 0)
    {
        xSemaphoreTake(*xSemIICFree, portMAX_DELAY);
    }

    esp_err_t ret = i2c_master_transmit_receive(*dev_iic, &reg, 1, dst, len, ctp_xfer_time_out);

    if (xSemIICFree != 0)
    {
        xSemaphoreGive(*xSemIICFree);
    }

    ESP_RETURN_ON_ERROR(ret, TAG, "read io fail");
    return ret;
} // ds_iic_readReg

static esp_err_t ds_iic_writeReg(uint8_t reg, void *src, uint16_t len)
{
    i2c_master_transmit_multi_buffer_info_t xfer_desc[2] = {
        {
            .buffer_size = 1,
            .write_buffer = &reg,
        },
        {
            .buffer_size = len,
            .write_buffer = src,
        },
    };

    if (xSemIICFree != 0)
    {
        xSemaphoreTake(*xSemIICFree, portMAX_DELAY);
    }

    esp_err_t ret = i2c_master_multi_buffer_transmit(*dev_iic, xfer_desc, 2, ctp_xfer_time_out);

    if (xSemIICFree != 0)
    {
        xSemaphoreGive(*xSemIICFree);
    }

    ESP_RETURN_ON_ERROR(ret, TAG, "write io fail");
    return ret;
} // ds_iic_writeReg

/**
 *
 * @return 0 = success
 */
static int8_t bcd2bin(uint8_t bcd, int *bin)
{
    // uint8_t ret = 0;
    uint8_t tmp = bcd & 0x0f;
    if (tmp > 9)
    {
        return -1;
    }
    *bin = tmp;

    tmp = bcd >> 4;
    if (tmp > 9)
    {
        return -2;
    }

    *bin += (tmp * 10);

    return 0;
} // bcd2bin

/**
 *
 * @return 0 = success
 */
static int8_t bin2bcd(int bin, uint8_t *bcd)
{
    // uint8_t ret = 0;
    if (bin > 99)
    {
        ESP_LOGW(TAG, "bin range error  %d", bin);
        bin = 99;
        return -1;
    }
    uint8_t tmp = bin % 10;

    *bcd = tmp;

    tmp = (bin / 10) % 10;

    *bcd |= (tmp << 4);

    return 0;
} // bin2bcd

static void Get_Compile_DateTime_Base(int *Year, int *Month, int *Day,
                                      int *Hour, int *Minute, int *Second)
{
    const char *pMonth[] = {"Jan", "Feb", "Mar", "Apr", "May", "Jun", "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"};
    const char Date[12] = __DATE__; // 取编译时间
    char Time[9] = __TIME__;
    uint8_t i;
    for (i = 0; i < 12; i++)
        if (memcmp(Date, pMonth[i], 3) == 0)
            *Month = i + 1, i = 12;
    *Year = (uint8_t)atoi(Date + 9); // Date[9]为２位年份，Date[7]为完整年份
    *Day = (uint8_t)atoi(Date + 4);

    Time[2] = ' ';
    Time[5] = ' ';

    *Hour = (uint8_t)atoi(Time + 0);
    *Minute = (uint8_t)atoi(Time + 3);
    *Second = (uint8_t)atoi(Time + 6);
} // Get_Compile_DateTime_Base

esp_err_t ds3231_init(SemaphoreHandle_t *i2c_mux, i2c_master_dev_handle_t *i2c, int xfer_time_out)
{

    uint8_t reg;

    dev_iic = i2c;
    xSemIICFree = i2c_mux;
    ctp_xfer_time_out = xfer_time_out;

    ESP_RETURN_ON_ERROR(ds_iic_readReg(0x02, &reg, 1), TAG, "init io err");

    if ((reg & DS3231_HOUR_FMT_MSK))
    {
        ESP_LOGW(TAG, "in 12h mode , set to 24h mode");
        // 12h
        reg = reg & (~DS3231_HOUR_FMT_MSK);
        ESP_RETURN_ON_ERROR(ds_iic_writeReg(0x02, &reg, 1), TAG, "init write hout fmt err");
    } // DS3231_HOUR_FMT_MSK

    return ESP_OK;
} // ds3231_init

/**
 *
 * @note set config only bit CONV = 0
 */
esp_err_t ds3231_set_config(ds_config_t conf)
{
    uint8_t reg = 0;

    // read conf reg to look CONV bit
    ESP_RETURN_ON_ERROR(ds_iic_readReg(0x0e, &reg, 1), TAG, "set conf io err");
    // check conv
    ESP_RETURN_ON_FALSE((reg & DS3231_CONV_MSK) == 0, ESP_FAIL, TAG, "TCXO running");
    // write conf reg
    reg = (conf.osc_disable ? DS3231_EOSC_MSK : 0) |
          (conf.pluse_out_in_bat_mode_enable ? DS3231_BBSQW_MSK : 0) |
          ((conf.pluse_freq & 0x3) << 3) |
          (conf.int_mode ? DS3231_INTCH_MSK : 0) |
          (conf.alarm2_int_en ? DS3231_A2IE_MSK : 0) |
          (conf.alarm1_int_en ? DS3231_A1IE_MSK : 0);
    ESP_RETURN_ON_ERROR(ds_iic_writeReg(0x0e, &reg, 1), TAG, "set conf io err");

    // change EN32kHz
    ESP_RETURN_ON_ERROR(ds_iic_readReg(0x0f, &reg, 1), TAG, "set conf io err");
    reg = reg & (~DS3231_EN32KHZ_MSK);
    reg = reg | (conf.pluse_enable ? DS3231_EN32KHZ_MSK : 0);
    ESP_RETURN_ON_ERROR(ds_iic_writeReg(0x0f, &reg, 1), TAG, "set conf io err");

    return ESP_OK;
} // ds3231_set_config

esp_err_t ds3231_get_config(ds_config_t *conf)
{
    uint8_t reg[2];

    // read conf regs {0xe,0xf}
    ESP_RETURN_ON_ERROR(ds_iic_readReg(0x0e, reg, 2), TAG, "get conf io err");

    conf->osc_disable = (reg[0] & DS3231_EOSC_MSK) ? 1 : 0;
    conf->pluse_out_in_bat_mode_enable = (reg[0] & DS3231_BBSQW_MSK) ? 1 : 0;
    conf->pluse_freq = (reg[0] & DS3231_RSX_MSK) >> 3;
    conf->int_mode = (reg[0] & DS3231_INTCH_MSK) ? 1 : 0;
    conf->alarm2_int_en = (reg[0] & DS3231_A2IE_MSK) ? 1 : 0;
    conf->alarm1_int_en = (reg[0] & DS3231_A1IE_MSK) ? 1 : 0;

    conf->pluse_enable = (reg[1] & DS3231_EN32KHZ_MSK) ? 1 : 0;

    return ESP_OK;
} // ds3231_get_config

esp_err_t ds3231_get_flags(ds_flags_t *flag, uint8_t clean)
{
    uint8_t reg = 0;
    ESP_RETURN_ON_ERROR(ds_iic_readReg(0x0f, &reg, 1), TAG, "get flag io err");

    flag->osc_stop = (reg & DS3231_OSF_MSK) ? 1 : 0;
    flag->tcxo_active = (reg & DS3231_BSY_MSK) ? 1 : 0;
    flag->alarm2 = (reg & DS3231_A2F_MSK) ? 1 : 0;
    flag->alarm1 = (reg & DS3231_A1F_MSK) ? 1 : 0;

    if (clean)
    {
        reg = reg & (~(DS3231_OSF_MSK | DS3231_A2F_MSK | DS3231_A1F_MSK));
        ESP_RETURN_ON_ERROR(ds_iic_writeReg(0x0f, &reg, 1), TAG, "clean flag io err");
    } // clean
    return ESP_OK;
} // ds3231_get_flags

esp_err_t ds3231_get_time(struct tm *t)
{
    uint8_t ret_reg_raw[7];
    memset(t, 0x0, sizeof(struct tm));

    ESP_RETURN_ON_ERROR(ds_iic_readReg(0x00, ret_reg_raw, 7), TAG, "get time io err");

    // Seconds
    ESP_RETURN_ON_FALSE(bcd2bin((ret_reg_raw[0] & 0x7f), &t->tm_sec) == 0, ESP_ERR_INVALID_ARG, TAG, "bcd2bin fmt conv err"); // 0x00 Seconds

    // Minutes
    ESP_RETURN_ON_FALSE(bcd2bin((ret_reg_raw[1] & 0x7f), &t->tm_min) == 0, ESP_ERR_INVALID_ARG, TAG, "bcd2bin fmt conv err"); // 0x01 Minutes

    // hour
    if (ret_reg_raw[2] & 0x40)
    {
        // 12h support
        ESP_RETURN_ON_FALSE(bcd2bin((ret_reg_raw[2] & 0x1f), &t->tm_hour) == 0, ESP_ERR_INVALID_ARG, TAG, "bcd2bin fmt conv err"); // Hours(12/24 set)
        if (ret_reg_raw[2] & 0x20)
        {
            // pm
            t->tm_hour += 12;
        }
    }
    else
    {
        // 24h
        ESP_RETURN_ON_FALSE(bcd2bin((ret_reg_raw[2] & 0x3f), &t->tm_hour) == 0, ESP_ERR_INVALID_ARG, TAG, "bcd2bin fmt conv err"); // Hours(12/24 set)
    }

    // week
    ESP_RETURN_ON_FALSE(bcd2bin((ret_reg_raw[3] & 0x07), &t->tm_wday) == 0, ESP_ERR_INVALID_ARG, TAG, "bcd2bin fmt conv err"); // 0x03 Day(week) [1,7]
    t->tm_wday -= 1;                                                                                                           //[0,6]

    // Date
    ESP_RETURN_ON_FALSE(bcd2bin((ret_reg_raw[4] & 0x3f), &t->tm_mday) == 0, ESP_ERR_INVALID_ARG, TAG, "bcd2bin fmt conv err"); // 0x4 Date

    // Month
    ESP_RETURN_ON_FALSE(bcd2bin((ret_reg_raw[5] & 0x1f), &t->tm_mon) == 0, ESP_ERR_INVALID_ARG, TAG, "bcd2bin fmt conv err"); // Month/Century

    // Year&Century
    ESP_RETURN_ON_FALSE(bcd2bin(ret_reg_raw[6], &t->tm_year) == 0, ESP_ERR_INVALID_ARG, TAG, "bcd2bin fmt conv err"); // 0x6 Year  // rec data:00 = 2000

    // Century
    if (ret_reg_raw[5] & 0x80)
    {
        t->tm_year += 100;
    }
    t->tm_year += 100; // 自 1900 年起的年数

    return ESP_OK;
} // ds3231_get_time

esp_err_t ds3231_set_time(struct tm t)
{
    // check time
    if (t.tm_sec < 0 || t.tm_min < 0 || t.tm_hour < 0 ||
        t.tm_sec > 59 || t.tm_min > 59 || t.tm_hour > 23 ||
        t.tm_wday < 0 || t.tm_wday > 6 ||
        t.tm_mday < 0 || t.tm_mday > 31 ||
        t.tm_mon < 0 || t.tm_mon > 12 ||
        t.tm_year < 100 || t.tm_year > 299)
    {
        ESP_LOGE(TAG, "time fmt error");
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t ret_reg_raw[7];

    ESP_RETURN_ON_FALSE(bin2bcd(t.tm_sec, &ret_reg_raw[0]) == 0, ESP_ERR_INVALID_ARG, TAG, "bin2bcd fmt conv err");
    ESP_RETURN_ON_FALSE(bin2bcd(t.tm_min, &ret_reg_raw[1]) == 0, ESP_ERR_INVALID_ARG, TAG, "bin2bcd fmt conv err");
    ESP_RETURN_ON_FALSE(bin2bcd(t.tm_hour, &ret_reg_raw[2]) == 0, ESP_ERR_INVALID_ARG, TAG, "bin2bcd fmt conv err");
    ESP_RETURN_ON_FALSE(bin2bcd(t.tm_wday + 1, &ret_reg_raw[3]) == 0, ESP_ERR_INVALID_ARG, TAG, "bin2bcd fmt conv err");
    ESP_RETURN_ON_FALSE(bin2bcd(t.tm_mday, &ret_reg_raw[4]) == 0, ESP_ERR_INVALID_ARG, TAG, "bin2bcd fmt conv err");
    ESP_RETURN_ON_FALSE(bin2bcd(t.tm_mon, &ret_reg_raw[5]) == 0, ESP_ERR_INVALID_ARG, TAG, "bin2bcd fmt conv err");

    t.tm_year -= 100;
    if (t.tm_year >= 100)
    {
        // Century = 1
        ret_reg_raw[5] |= 0x80;
        t.tm_year -= 100;
    } // Century = 1
    ESP_RETURN_ON_FALSE(bin2bcd(t.tm_year, &ret_reg_raw[6]) == 0, ESP_ERR_INVALID_ARG, TAG, "bin2bcd fmt conv err");

    ret_reg_raw[2] = ret_reg_raw[2] & (~(DS3231_HOUR_FMT_MSK)); // set to 24h

    ESP_RETURN_ON_ERROR(ds_iic_writeReg(0x00, ret_reg_raw, 7), TAG, "set time io err");

    return ESP_OK;
} // ds3231_set_time

esp_err_t ds3231_set_alarm(struct tm t, ds_alarm_mode_t match, uint8_t id)
{

    uint8_t reg[4] = {0, 0, 0, 0};

    // check time
    if (t.tm_sec < 0 || t.tm_min < 0 || t.tm_hour < 0 ||
        t.tm_sec > 59 || t.tm_min > 59 || t.tm_hour > 23 ||
        t.tm_wday < 0 || t.tm_wday > 6 ||
        t.tm_mday < 0 || t.tm_mday > 31)
    {
        ESP_LOGE(TAG, "time fmt error");
        return ESP_ERR_INVALID_ARG;
    }

    // write
    if (id == DS3231_ALARM1)
    {
        /*
        A1
        DY/#DT | M4 | M3 | M2 | M1 | RATE
            X     1    1    1    1    Alarm once per second
            X     1    1    1    0    Alarm when seconds match
            X     1    1    0    0    Alarm when minutes and seconds match
            X     1    0    0    0    Alarm when hours, minutes, and seconds match
            0     0    0    0    0    Alarm when date, hours, minutes, and seconds match
            1     0    0    0    0    Alarm when day, hours, minutes, and seconds match
        */
        ESP_RETURN_ON_FALSE(bin2bcd(t.tm_sec, &reg[0]) == 0, ESP_ERR_INVALID_ARG, TAG, "bin2bcd fmt conv err");  // 0x07
        ESP_RETURN_ON_FALSE(bin2bcd(t.tm_min, &reg[1]) == 0, ESP_ERR_INVALID_ARG, TAG, "bin2bcd fmt conv err");  // 0x08
        ESP_RETURN_ON_FALSE(bin2bcd(t.tm_hour, &reg[2]) == 0, ESP_ERR_INVALID_ARG, TAG, "bin2bcd fmt conv err"); // 0x09

        reg[2] = reg[2] & 0xBf; // force 24h

        switch (match)
        {
        case DS_ALARM_MODE_NO_MATCH:
            reg[0] = reg[0] | 0x80; // m1
            reg[1] = reg[1] | 0x80; // m2
            reg[2] = reg[2] | 0x80; // m3
            reg[3] = reg[3] | 0x80; // m4
            break;
        case DS_ALARM_MODE_SEC_MATCH:
            reg[0] = reg[0] & 0x7f; // m1
            reg[1] = reg[1] | 0x80; // m2
            reg[2] = reg[2] | 0x80; // m3
            reg[3] = reg[3] | 0x80; // m4
            break;
        case DS_ALARM_MODE_MIN_MATCH:
            reg[0] = reg[0] & 0x7f; // m1
            reg[1] = reg[1] & 0x7f; // m2
            reg[2] = reg[2] | 0x80; // m3
            reg[3] = reg[3] | 0x80; // m4
            break;
        case DS_ALARM_MODE_HOR_MATCH:
            reg[0] = reg[0] & 0x7f; // m1
            reg[1] = reg[1] & 0x7f; // m2
            reg[2] = reg[2] & 0x7f; // m3
            reg[3] = reg[3] | 0x80; // m4
            break;
        case DS_ALARM_MODE_DATE_MATCH:
            ESP_RETURN_ON_FALSE(bin2bcd(t.tm_mday, &reg[3]) == 0, ESP_ERR_INVALID_ARG, TAG, "bin2bcd fmt conv err"); // 0x0a
            reg[0] = reg[0] & 0x7f;                                                                                  // m1
            reg[1] = reg[1] & 0x7f;                                                                                  // m2
            reg[2] = reg[2] & 0x7f;                                                                                  // m3
            reg[3] = reg[3] & 0x3f;                                                                                  // m4=0 | dy/#dt=0
            break;
        case DS_ALARM_MODE_WEEK_MATCH:
            ESP_RETURN_ON_FALSE(bin2bcd(t.tm_wday + 1, &reg[3]) == 0, ESP_ERR_INVALID_ARG, TAG, "bin2bcd fmt conv err"); // 0x0a

            reg[3] = reg[3] | 0x40; // dy/#dt=1
            reg[0] = reg[0] & 0x7f; // m1
            reg[1] = reg[1] & 0x7f; // m2
            reg[2] = reg[2] & 0x7f; // m3
            reg[3] = reg[3] & 0x7f; // m4
            break;
        default:
            ESP_LOGE(TAG, "alarm match error");
            return ESP_ERR_INVALID_ARG;
            break;
        } // match

        // write
        // ESP_LOG_BUFFER_HEX("DSA1", reg, 4);
        ESP_RETURN_ON_ERROR(ds_iic_writeReg(0x07, reg, 4), TAG, "set alarm1 io err");

    } // DS3231_ALARM1
    else if (id == DS3231_ALARM2)
    {
        /*
        A2
        DY/#DT | M4 | M3 | M2 | RATE
            X     1    1    1     Alarm once per minute (00 seconds of every minute)
            X     1    1    0     Alarm when minutes match
            X     1    0    0     Alarm when hours and minutes match
            0     0    0    0     Alarm when date, hours, and minutes match
            1     0    0    0     Alarm when day, hours, and minutes match
        */
        ESP_RETURN_ON_FALSE(bin2bcd(t.tm_min, &reg[0]) == 0, ESP_ERR_INVALID_ARG, TAG, "bin2bcd fmt conv err");  // 0x0b
        ESP_RETURN_ON_FALSE(bin2bcd(t.tm_hour, &reg[1]) == 0, ESP_ERR_INVALID_ARG, TAG, "bin2bcd fmt conv err"); // 0x0c

        reg[1] = reg[1] & 0xBf; // force 24h

        switch (match)
        {
        case DS_ALARM_MODE_NO_MATCH:
            reg[0] = reg[0] | 0x80; // m2
            reg[1] = reg[1] | 0x80; // m3
            reg[2] = reg[2] | 0x80; // m4
            break;
        case DS_ALARM_MODE_MIN_MATCH:
            reg[0] = reg[0] & 0x7f; // m2
            reg[1] = reg[1] | 0x80; // m3
            reg[2] = reg[2] | 0x80; // m4
            break;
        case DS_ALARM_MODE_HOR_MATCH:
            reg[0] = reg[0] & 0x7f; // m2
            reg[1] = reg[1] & 0x7f; // m3
            reg[2] = reg[2] | 0x80; // m4
            break;
        case DS_ALARM_MODE_DATE_MATCH:
            ESP_RETURN_ON_FALSE(bin2bcd(t.tm_mday, &reg[2]) == 0, ESP_ERR_INVALID_ARG, TAG, "bin2bcd fmt conv err"); // 0x0a
            reg[0] = reg[0] & 0x7f;                                                                                  // m2
            reg[1] = reg[1] & 0x7f;                                                                                  // m3
            reg[2] = reg[2] & 0x3f;                                                                                  // m4=0 | dy/#dt=0
            break;
        case DS_ALARM_MODE_WEEK_MATCH:
            ESP_RETURN_ON_FALSE(bin2bcd(t.tm_wday + 1, &reg[2]) == 0, ESP_ERR_INVALID_ARG, TAG, "bin2bcd fmt conv err"); // 0x0a
            reg[2] = reg[2] | 0x40;                                                                                      // dy/#dt=1
            reg[0] = reg[0] & 0x7f;                                                                                      // m1
            reg[1] = reg[1] & 0x7f;                                                                                      // m2
            reg[2] = reg[2] & 0x7f;                                                                                      // m3
            break;
        default:
            ESP_LOGE(TAG, "alarm match error");
            return ESP_ERR_INVALID_ARG;
            break;
        } // match

        // write
        // ESP_LOG_BUFFER_HEX("DSA2", reg, 3);
        ESP_RETURN_ON_ERROR(ds_iic_writeReg(0x0b, reg, 3), TAG, "set alarm1 io err");
    } // DS3231_ALARM2
    else
    {
        ESP_LOGE(TAG, "alarm id error");
        return ESP_ERR_INVALID_ARG;
    }

    return ESP_OK;
} // ds3231_set_alrm

esp_err_t ds3231_get_alarm(struct tm *t, ds_alarm_mode_t *match, uint8_t id)
{
    uint8_t reg[4] = {0, 0, 0, 0};

    if (id == DS3231_ALARM1)
    {
        /*
        A1
        DY/#DT | M4 | M3 | M2 | M1 | RATE
            X     1    1    1    1    Alarm once per second
            X     1    1    1    0    Alarm when seconds match
            X     1    1    0    0    Alarm when minutes and seconds match
            X     1    0    0    0    Alarm when hours, minutes, and seconds match
            0     0    0    0    0    Alarm when date, hours, minutes, and seconds match
            1     0    0    0    0    Alarm when day, hours, minutes, and seconds match
        */

        ESP_RETURN_ON_ERROR(ds_iic_readReg(0x07, reg, 4), TAG, "get alarm io err");
        // ESP_LOG_BUFFER_HEX("DSA1G", reg, 4);

        uint8_t masks = (DS_GET_ALARM_MSK(reg[3]) << 3) |
                        (DS_GET_ALARM_MSK(reg[2]) << 2) |
                        (DS_GET_ALARM_MSK(reg[1]) << 1) |
                        (DS_GET_ALARM_MSK(reg[0])); // 0x0000_m4 m3 m2 m1

        if (masks == 0x0f)
        {
            *match = DS_ALARM_MODE_NO_MATCH;
        }
        else if (masks == 0x0e)
        {
            *match = DS_ALARM_MODE_SEC_MATCH;
        }
        else if (masks == 0x0c)
        {
            *match = DS_ALARM_MODE_MIN_MATCH;
        }
        else if (masks == 0x08)
        {
            *match = DS_ALARM_MODE_HOR_MATCH;
        }
        else if ((masks == 0x00) && (!(reg[3] & 0x40)))
        {
            *match = DS_ALARM_MODE_DATE_MATCH;
        }
        else if ((masks == 0x00) && (reg[3] & 0x40))
        {
            *match = DS_ALARM_MODE_WEEK_MATCH;
        }
        else
        {
            *match = DS_ALARM_MODE_ERROR;
        }

        // Seconds
        ESP_RETURN_ON_FALSE(bcd2bin((reg[0] & 0x7f), &t->tm_sec) == 0, ESP_ERR_INVALID_ARG, TAG, "bcd2bin fmt conv err");

        // Minutes
        ESP_RETURN_ON_FALSE(bcd2bin((reg[1] & 0x7f), &t->tm_min) == 0, ESP_ERR_INVALID_ARG, TAG, "bcd2bin fmt conv err");

        // hour
        if (reg[2] & 0x40)
        {
            // 12h support
            ESP_RETURN_ON_FALSE(bcd2bin((reg[2] & 0x1f), &t->tm_hour) == 0, ESP_ERR_INVALID_ARG, TAG, "bcd2bin fmt conv err");
            if (reg[2] & 0x20)
            {
                // pm
                t->tm_hour += 12;
            }
        }
        else
        {
            // 24h
            ESP_RETURN_ON_FALSE(bcd2bin((reg[2] & 0x3f), &t->tm_hour) == 0, ESP_ERR_INVALID_ARG, TAG, "bcd2bin fmt conv err");
        }

        // day / week
        if ((reg[3] & 0x40))
        {
            // week
            ESP_RETURN_ON_FALSE(bcd2bin((reg[3] & 0x07), &t->tm_wday) == 0, ESP_ERR_INVALID_ARG, TAG, "bcd2bin fmt conv err");
            t->tm_wday -= 1;
        }
        else
        {
            // Date
            ESP_RETURN_ON_FALSE(bcd2bin((reg[3] & 0x3f), &t->tm_mday) == 0, ESP_ERR_INVALID_ARG, TAG, "bcd2bin fmt conv err");
        }

    } // DS3231_ALARM1
    else if (id == DS3231_ALARM2)
    {
        /*
        A2
        DY/#DT | M4 | M3 | M2 | RATE
            X     1    1    1     Alarm once per minute (00 seconds of every minute)
            X     1    1    0     Alarm when minutes match
            X     1    0    0     Alarm when hours and minutes match
            0     0    0    0     Alarm when date, hours, and minutes match
            1     0    0    0     Alarm when day, hours, and minutes match
        */

        ESP_RETURN_ON_ERROR(ds_iic_readReg(0x0b, reg, 3), TAG, "get alarm io err");
        // ESP_LOG_BUFFER_HEX("DSA2G", reg, 4);

        uint8_t masks = (DS_GET_ALARM_MSK(reg[2]) << 2) |
                        (DS_GET_ALARM_MSK(reg[1]) << 1) |
                        (DS_GET_ALARM_MSK(reg[0])); // 0x0000_0 m4 m3 m2

        if (masks == 0x07)
        {
            *match = DS_ALARM_MODE_NO_MATCH;
        }
        else if (masks == 0x06)
        {
            *match = DS_ALARM_MODE_MIN_MATCH;
        }
        else if (masks == 0x04)
        {
            *match = DS_ALARM_MODE_HOR_MATCH;
        }
        else if ((masks == 0x00) && (!(reg[2] & 0x40)))
        {
            *match = DS_ALARM_MODE_DATE_MATCH;
        }
        else if ((masks == 0x00) && (reg[2] & 0x40))
        {
            *match = DS_ALARM_MODE_WEEK_MATCH;
        }
        else
        {
            *match = DS_ALARM_MODE_ERROR;
        }

        // Minutes
        ESP_RETURN_ON_FALSE(bcd2bin((reg[0] & 0x7f), &t->tm_min) == 0, ESP_ERR_INVALID_ARG, TAG, "bcd2bin fmt conv err");

        // hour
        if (reg[1] & 0x40)
        {
            // 12h support
            ESP_RETURN_ON_FALSE(bcd2bin((reg[1] & 0x1f), &t->tm_hour) == 0, ESP_ERR_INVALID_ARG, TAG, "bcd2bin fmt conv err");
            if (reg[1] & 0x20)
            {
                // pm
                t->tm_hour += 12;
            }
        }
        else
        {
            // 24h
            ESP_RETURN_ON_FALSE(bcd2bin((reg[1] & 0x3f), &t->tm_hour) == 0, ESP_ERR_INVALID_ARG, TAG, "bcd2bin fmt conv err");
        }

        // day / week
        if ((reg[2] & 0x40))
        {
            // week
            ESP_RETURN_ON_FALSE(bcd2bin((reg[2] & 0x07), &t->tm_wday) == 0, ESP_ERR_INVALID_ARG, TAG, "bcd2bin fmt conv err");
            t->tm_wday -= 1;
        }
        else
        {
            // Date
            ESP_RETURN_ON_FALSE(bcd2bin((reg[2] & 0x3f), &t->tm_mday) == 0, ESP_ERR_INVALID_ARG, TAG, "bcd2bin fmt conv err");
        }

    } // DS3231_ALARM2
    else
    {
        ESP_LOGE(TAG, "alarm id error");
        return ESP_ERR_INVALID_ARG;
    }

    return ESP_OK;
} // ds3231_get_alrm

/*
    老化偏移寄存器将用户提供的值加到电容阵列寄存器中的代码中或从中减去。
    该代码以二进制补码编码，位7表示符号位。
    一个LSB表示一个小电容器，可以在晶体引脚的电容阵列中切换。
    老化偏移寄存器电容值与设备为每次温度补偿计算的电容值相加或相减。
    如果温度与之前的转换不同，则在常温转换期间或在手动用户转换期间（设置CONV位），偏移寄存器会添加到电容阵列中。
    要立即查看老化寄存器对32kHz输出频率的影响，应在每次老化寄存器更改后开始手动转换。
    正老化值会增加阵列的电容，降低振荡器频率。
    负值会消除阵列中的电容，增加振荡器频率。
    每LSB的ppm变化在不同温度下是不同的。频率与温度曲线因该寄存器中使用的值而偏移。
    在+25°C时，一个LSB通常提供约0.1ppm的变化频率。
    不需要使用老化寄存器来实现EC表中定义的精度，但可以用来帮助补偿给定温度下的老化。
    有关寄存器对温度精度影响的图表，请参阅典型操作特性部分。
*/
esp_err_t ds3231_set_aging_offset(int8_t ago)
{
    ESP_RETURN_ON_ERROR(ds_iic_writeReg(0x10, &ago, 1), TAG, "set ag io err");
    return ESP_OK;
} // ds3231_set_aging_offset

esp_err_t ds3231_get_aging_offset(int8_t *ago)
{
    ESP_RETURN_ON_ERROR(ds_iic_readReg(0x10, ago, 1), TAG, "get ag io err");
    return ESP_OK;
} // ds3231_get_aging_offset

/**
 * @brief 温度
 */
esp_err_t ds3231_get_temperature(float *t)
{
    uint8_t reg[2];

    ESP_RETURN_ON_ERROR(ds_iic_readReg(0x11, reg, 2), TAG, "get temp io err");

    uint16_t tmp = reg[0] << 2 | ((reg[1] & 0xc0) >> 6);
    int16_t result = ((int16_t)(tmp << 4)) >> 4; // sign extension

    *t = (float)result / 4.0f;

    ESP_LOGD(TAG, "rt : %x %x {%x} -> %f", reg[0], reg[1], tmp, *t);
    return ESP_OK;
} // ds3231_get_temperature

/**
 * @brief 强制温度传感器将温度转换为数字代码，并执行TCXO算法，以更新振荡器的电容阵列。
 *
 */
esp_err_t ds3231_active_tcxo()
{
    uint8_t reg = 0;

    // read conf reg to look CONV bit
    ESP_RETURN_ON_ERROR(ds_iic_readReg(0x0e, &reg, 1), TAG, "act tcxo io err");
    // check conv
    ESP_RETURN_ON_FALSE((reg & DS3231_CONV_MSK) == 0, ESP_FAIL, TAG, "TCXO already running");

    reg = reg | DS3231_CONV_MSK;

    ESP_RETURN_ON_ERROR(ds_iic_writeReg(0x0e, &reg, 1), TAG, "act tcxo io err");

    return ESP_OK;
} // ds3231_active_tcxo

void ds3231_debug_log_time(struct tm tm)
{
    ESP_LOGI(TAG, "==================================================");
    ESP_LOGI(TAG, "%d - %d - %d |%d| %d : %d : %d",
             tm.tm_year - 100, tm.tm_mon, tm.tm_mday, tm.tm_wday,
             tm.tm_hour, tm.tm_min, tm.tm_sec);
} // ds3231_debug_log_time

void ds3231_debug_log_config(ds_config_t *conf)
{
    ESP_LOGI(TAG, "==================================================");
    ESP_LOGI(TAG, "osc_disable(EOSC#) = %d", conf->osc_disable);
    ESP_LOGI(TAG, "pluse_out_in_bat_mode_enable(BBSQW) = %d", conf->pluse_out_in_bat_mode_enable);
    ESP_LOGI(TAG, "pluse_freq(RSX) = %d", conf->pluse_freq);
    ESP_LOGI(TAG, "int_mode(INTCN) = %d", conf->int_mode);
    ESP_LOGI(TAG, "alarm2_int_en(A2IE) = %d", conf->alarm2_int_en);
    ESP_LOGI(TAG, "alarm1_int_en(A1IE) = %d", conf->alarm1_int_en);
    ESP_LOGI(TAG, "pluse_enable(EN32kHz) = %d", conf->pluse_enable);
} // ds3231_debug_log_config

void ds3231_debug_log_flags(ds_flags_t *f)
{
    ESP_LOGI(TAG, "==================================================");
    ESP_LOGI(TAG, "osc_stop(OSF) = %d", f->osc_stop);
    ESP_LOGI(TAG, "tcxo_active(BSY) = %d", f->tcxo_active);
    ESP_LOGI(TAG, "alarm2(A2F) = %d", f->alarm2);
    ESP_LOGI(TAG, "alarm1(A1F) = %d", f->alarm1);
} // ds3231_debug_log_flags

void ds3231_debug_set_time_to_compile()
{
    struct tm tm;
    memset(&tm, 0x0, sizeof(struct tm));
    Get_Compile_DateTime_Base(&tm.tm_year, &tm.tm_mon, &tm.tm_mday, &tm.tm_hour, &tm.tm_min, &tm.tm_sec);
    tm.tm_year += 100;
    ESP_LOGI(TAG, "set time to");
    ds3231_debug_log_time(tm);
    ds3231_set_time(tm);
} // ds3231_debug_set_time_to_compile

// eof
