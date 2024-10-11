#include "rtc.h"


static const char *TAG = "rtc";

// Convert decimal to BCD
static uint8_t dec_to_bcd(uint8_t val) {
    return ((val / 10 * 16) + (val % 10));
}

// Convert BCD to decimal
static uint8_t bcd_to_dec(uint8_t val) {
    return ((val >> 4) * 10) + (val & 0x0F);
}

// Read time from DS3231
bool ds3231_get_time(struct tm *timeinfo) {
    uint8_t data[7];

    // Create a new I2C command
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    // Start I2C communication
    i2c_master_start(cmd);

    // Write DS3231 address with write flag
    i2c_master_write_byte(cmd, (DS3231_ADDRESS << 1) | I2C_MASTER_WRITE, true);

    // Set DS3231 register pointer to 0x00 (Seconds register)
    i2c_master_write_byte(cmd, 0x00, true);

    // Restart I2C communication
    i2c_master_start(cmd);

    // Write DS3231 address with read flag
    i2c_master_write_byte(cmd, (DS3231_ADDRESS << 1) | I2C_MASTER_READ, true);

    // Read 7 bytes of data (Seconds, Minutes, Hours, Day, Date, Month, Year)
    i2c_master_read(cmd, data, 6, I2C_MASTER_ACK);
    i2c_master_read_byte(cmd, data + 6, I2C_MASTER_NACK); // Last byte NACK

    // Stop I2C communication
    i2c_master_stop(cmd);

    // Execute I2C command
    esp_err_t err = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);

    // Delete I2C command
    i2c_cmd_link_delete(cmd);

    if (err == ESP_OK) {
        // Convert BCD to decimal and fill timeinfo
        timeinfo->tm_sec  = bcd_to_dec(data[0]);
        timeinfo->tm_min  = bcd_to_dec(data[1]);
        timeinfo->tm_hour = bcd_to_dec(data[2] & 0x3F); // Mask for 24-hour format
        timeinfo->tm_mday = bcd_to_dec(data[4]);
        timeinfo->tm_mon  = bcd_to_dec(data[5] & 0x1F) - 1; // Months since January (0-11)
        timeinfo->tm_year = bcd_to_dec(data[6]) + 100;      // Years since 1900
        timeinfo->tm_wday = bcd_to_dec(data[3]) % 7;        // Ensure 0-6

        // Check if year is >= 122 (2022)
        if (timeinfo->tm_year >= 122) {
            return true; // Valid time
        }
    }
    ESP_LOGE("DS3231", "Failed to read time from DS3231: %s", esp_err_to_name(err));
    return false;
}

static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                               int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        // Start the Wi-Fi connection
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        // Set the failure bit to notify that connection failed
        xEventGroupSetBits(wifi_event_group, WIFI_FAIL_BIT);
        ESP_LOGI("WiFi", "Disconnected from Wi-Fi");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        // Successfully got IP address, set the connected bit
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI("WiFi", "Got IP address: " IPSTR, IP2STR(&event->ip_info.ip));
        xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

// Helper function to initialize and connect Wi-Fi in ESP-IDF
void initWiFi() {
    // Initialize NVS (used for storing Wi-Fi credentials)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Initialize TCP/IPKlenion
    ESP_ERROR_CHECK(esp_netif_init());

    // Create default event loop for Wi-Fi events
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    // Create the event group
    wifi_event_group = xEventGroupCreate();

    // Initialize Wi-Fi with default configuration
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    // Register the event handler
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL));

    // Set Wi-Fi mode to STA (Station mode)
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));

    // Set Wi-Fi credentials
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = "2G_358",        // Replace with your SSID
            .password = "mateus4570",   // Replace with your password
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));

    // Start Wi-Fi
    ESP_ERROR_CHECK(esp_wifi_start());

    // Log message
    ESP_LOGI("WiFi", "Wi-Fi initialization complete.");

    // Wait for connection
    ESP_LOGI("WiFi", "Connecting to Wi-Fi...");
    EventBits_t bits = xEventGroupWaitBits(
        wifi_event_group,
        WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
        pdFALSE,
        pdFALSE,
        portMAX_DELAY);

    // Check which bit was set
    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI("WiFi", "Connected to Wi-Fi network: %s", wifi_config.sta.ssid);
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGI("WiFi", "Failed to connect to Wi-Fi network: %s", wifi_config.sta.ssid);
    } else {
        ESP_LOGE("WiFi", "Unexpected event");
    }

    // Clean up the event handler
    ESP_ERROR_CHECK(esp_event_handler_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler));
    ESP_ERROR_CHECK(esp_event_handler_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler));
    vEventGroupDelete(wifi_event_group);
}


// Modified sync_time_with_ntp_and_set_ds3231 function
void sync_time_with_ntp_and_set_ds3231(void) {
    // Initialize I2C master (if not already initialized)
    //i2c_master_init(I2C_MASTER_SDA_IO,I2C_MASTER_SCL_IO);

    // First, read the current time from the DS3231 RTC
    struct tm rtc_timeinfo = { 0 };
    bool valid_rtc_time = ds3231_get_time(&rtc_timeinfo);

    // Format and display the current RTC time
    char rtc_time_str[64];
    strftime(rtc_time_str, sizeof(rtc_time_str), "%c", &rtc_timeinfo);
    ESP_LOGI(TAG, "Current RTC time: %s", rtc_time_str);

    // Check if the RTC time is valid (year >= 2022)
    if (valid_rtc_time) {
        ESP_LOGI(TAG, "RTC time is valid, skipping NTP sync.");
        return; // If time is valid, no need to sync with NTP
    }

    ESP_LOGI(TAG, "RTC time is invalid, syncing with NTP...");

    // Set timezone to UTC-3 for Recife/Joao Pessoa
    setenv("TZ", "<-03>3", 1);
    tzset();

    // Initialize and connect to Wi-Fi
    initWiFi();

    // Initialize SNTP for NTP synchronization
    initialize_sntp();

    // Wait for time to sync
    time_t now = 0;
    struct tm timeinfo = { 0 };
    int retry = 0;
    const int retry_count = 10;
    while (timeinfo.tm_year < (2016 - 1900) && ++retry < retry_count) {
        ESP_LOGI(TAG, "Waiting for system time to be set... (%d/%d)", retry, retry_count);
        vTaskDelay(2000 / portTICK_PERIOD_MS);
        time(&now);
        localtime_r(&now, &timeinfo);
    }

    if (retry == retry_count) {
        ESP_LOGE(TAG, "Failed to get time from NTP server.");
        // Handle error as needed
    } else {
        // Display UTC time
        struct tm utc_timeinfo;
        gmtime_r(&now, &utc_timeinfo);
        char utc_time_str[64];
        strftime(utc_time_str, sizeof(utc_time_str), "%c", &utc_timeinfo);
        ESP_LOGI(TAG, "UTC Time synchronized from NTP: %s", utc_time_str);

        // Display Local Time
        char local_time_str[64];
        strftime(local_time_str, sizeof(local_time_str), "%c", &timeinfo);
        ESP_LOGI(TAG, "Local Time synchronized from NTP: %s", local_time_str);

        // Set the DS3231 with this synchronized local time
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (DS3231_ADDRESS << 1) | I2C_MASTER_WRITE, true);
        i2c_master_write_byte(cmd, 0x00, true);  // Start at seconds register

        i2c_master_write_byte(cmd, dec_to_bcd(timeinfo.tm_sec), true);      // Seconds
        i2c_master_write_byte(cmd, dec_to_bcd(timeinfo.tm_min), true);      // Minutes
        i2c_master_write_byte(cmd, dec_to_bcd(timeinfo.tm_hour), true);     // Hours
        i2c_master_write_byte(cmd, dec_to_bcd(timeinfo.tm_wday), true);     // Day of Week
        i2c_master_write_byte(cmd, dec_to_bcd(timeinfo.tm_mday), true);     // Date
        i2c_master_write_byte(cmd, dec_to_bcd(timeinfo.tm_mon + 1), true);  // Month
        i2c_master_write_byte(cmd, dec_to_bcd(timeinfo.tm_year - 100), true); // Year

        i2c_master_stop(cmd);
        esp_err_t err = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
        i2c_cmd_link_delete(cmd);

        if (err == ESP_OK) {
            ESP_LOGI(TAG, "DS3231 RTC updated with NTP time.");
        } else {
            ESP_LOGE(TAG, "Failed to set time to DS3231: %s", esp_err_to_name(err));
        }

        // Read back the time from RTC to verify
        ds3231_get_time(&rtc_timeinfo);
        strftime(rtc_time_str, sizeof(rtc_time_str), "%c", &rtc_timeinfo);
        ESP_LOGI(TAG, "Updated RTC time: %s", rtc_time_str);
    }

    // Disconnect from Wi-Fi if not needed anymore
    ESP_ERROR_CHECK(esp_wifi_disconnect());
    ESP_LOGI(TAG, "Wi-Fi disconnected");
}

// Initialize SNTP for NTP synchronization
void initialize_sntp(void) {
    esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);
    esp_sntp_setservername(0, "pool.ntp.org");
    esp_sntp_init();
}
// I2C Master initialization

void i2c_master_init(int pin_sda, int pin_scl) {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = pin_sda,               // Use function parameter for SDA
        .scl_io_num = pin_scl,               // Use function parameter for SCL
        .sda_pullup_en = GPIO_PULLUP_ENABLE, // Enable pull-up for SDA
        .scl_pullup_en = GPIO_PULLUP_ENABLE, // Enable pull-up for SCL
        .master.clk_speed = 100000           // Set clock speed to 100kHz
    };

    esp_err_t err;

    // Configure I2C parameters
    err = i2c_param_config(I2C_MASTER_NUM, &conf);
    if (err != ESP_OK) {
        ESP_LOGE("I2C_INIT", "I2C parameter configuration failed: %s", esp_err_to_name(err));
        return;
    }

    // Install I2C driver
    err = i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
    if (err != ESP_OK) {
        ESP_LOGE("I2C_INIT", "I2C driver installation failed: %s", esp_err_to_name(err));
        return;
    }
}

void ds3231_set_alarm_at_time(uint8_t hour, uint8_t minute, uint8_t second) {
    struct tm current_time;
    ds3231_get_time(&current_time);

    // Set the specified alarm time
    struct tm alarm_time;
    alarm_time.tm_year = current_time.tm_year; // Set current year
    alarm_time.tm_mon = current_time.tm_mon;   // Set current month
    alarm_time.tm_mday = current_time.tm_mday; // Set current day
    alarm_time.tm_hour = hour;   // Set alarm hour
    alarm_time.tm_min = minute;  // Set alarm minute
    alarm_time.tm_sec = second;  // Set alarm second

    // Clear Alarm Flags
    ds3231_clear_alarm_flags();

    // Enable Alarm Interrupts (Control Register 0x0E)
    uint8_t control_reg = 0x05; // INTCN = 1, A1IE = 1

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (DS3231_ADDRESS << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, 0x0E, true); // Control register
    i2c_master_write_byte(cmd, control_reg, true);
    i2c_master_stop(cmd);
    esp_err_t err = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write control register: %s", esp_err_to_name(err));
    }

    // Set Alarm 1 registers (0x07 - 0x0A)
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (DS3231_ADDRESS << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, 0x07, true); // Start at Alarm 1 seconds register

    // A1M1 = 0 (match seconds)
    i2c_master_write_byte(cmd, dec_to_bcd(alarm_time.tm_sec) & 0x7F, true);

    // A1M2 = 0 (match minutes)
    i2c_master_write_byte(cmd, dec_to_bcd(alarm_time.tm_min) & 0x7F, true);

    // A1M3 = 0 (match hours)
    i2c_master_write_byte(cmd, dec_to_bcd(alarm_time.tm_hour) & 0x7F, true);

    // A1M4 = 1 (ignore day/date)
    i2c_master_write_byte(cmd, 0x80, true);

    i2c_master_stop(cmd);
    err = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    if (err == ESP_OK) {
        ESP_LOGI(TAG, "Alarm set for %02d:%02d:%02d", alarm_time.tm_hour, alarm_time.tm_min, alarm_time.tm_sec);
    } else {
        ESP_LOGE(TAG, "Failed to set alarm: %s", esp_err_to_name(err));
    }
}
// Configure the ESP32 to wake up from deep sleep when DS3231 alarm triggers
void configure_wakeup_pin(void) {
    esp_sleep_enable_ext0_wakeup(GPIO_WAKEUP_PIN, 0); // Wake up on low level (0)
}
// Enter deep sleep until the DS3231 alarm triggers
void enter_deep_sleep_until_alarm(void) {
    fflush(stdout);

    ESP_LOGI(TAG, "Entering deep sleep until DS3231 alarm triggers...");
    esp_deep_sleep_start();  // Enter deep sleep, wake-up when SQW triggers
}


void set_next_alarm(struct tm *current_time) {
    if (current_time->tm_hour == 6) {
        // Set alarm for 14:00 (2 PM)
        ds3231_set_alarm_at_time(14, 0, 0);
    } else if (current_time->tm_hour == 15) {
        // Set alarm for 18:00 (6 PM)
        ds3231_set_alarm_at_time(15, 10, 0);
    } else if (current_time->tm_hour == 18) {
        // Set alarm for next day's 6 AM
        ds3231_set_alarm_at_time(6, 0, 0);
    } else {
        // Default: set alarm for 6 AM
        ds3231_set_alarm_at_time(15, 10, 0);
    }
}

void ds3231_clear_alarm_flags(void) {
    // Clear Alarm Flags (Status Register 0x0F)
    i2c_cmd_handle_t cmd;
    esp_err_t err;

    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);

    i2c_master_write_byte(cmd, (DS3231_ADDRESS << 1) | I2C_MASTER_WRITE, true);

    i2c_master_write_byte(cmd, 0x0F, true); // Status register

    // Clear alarm flags A1F and A2F (bits 0 and 1)
    i2c_master_write_byte(cmd, 0x00, true);

    i2c_master_stop(cmd);

    err = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);

    i2c_cmd_link_delete(cmd);

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to clear alarm flags: %s", esp_err_to_name(err));
    }
}