#ifndef RTC_H
#define RTC_H

#include <time.h>
#include "driver/i2c.h"
#include "esp_log.h"
#include "esp_sleep.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_sntp.h"

// I2C configuration for DS3231 RTC
#define I2C_MASTER_SCL_IO 22       // Define your SCL pin
#define I2C_MASTER_SDA_IO 21       // Define your SDA pin
#define I2C_MASTER_NUM I2C_NUM_0   // I2C port number for master dev
#define I2C_MASTER_FREQ_HZ 100000  // I2C master clock frequency
#define DS3231_ADDRESS 0x68        // DS3231 I2C address

#define GPIO_WAKEUP_PIN GPIO_NUM_33 // GPIO connected to DS3231 SQW

// Function declarations
void i2c_master_init(void);
void ds3231_set_alarm(uint8_t hour, uint8_t minute);
void initialize_sntp(void);
void sync_time_with_ntp_and_set_ds3231(void);
void configure_wakeup_pin(void);
void enter_deep_sleep_until_alarm(void);

#endif // RTC_H
