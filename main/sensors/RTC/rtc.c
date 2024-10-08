#include "rtc.h"

// Helper function to initialize and connect Wi-Fi in ESP-IDF
void initWiFi() {
    // Initialize NVS (used for storing Wi-Fi credentials)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Initialize TCP/IP
    ESP_ERROR_CHECK(esp_netif_init());

    // Create default event loop for Wi-Fi events
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    // Initialize Wi-Fi with default configuration
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    // Set Wi-Fi mode to STA (Station mode)
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());

    // Set Wi-Fi credentials
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = "Your_SSID",  // Replace with your SSID
            .password = "Your_PASSWORD"  // Replace with your password
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));

    // Connect to Wi-Fi
    ESP_ERROR_CHECK(esp_wifi_connect());

    // Wait for connection
    ESP_LOGI("WiFi", "Connecting to Wi-Fi...");
    wifi_event_group_t wifi_event_group = xEventGroupCreate();
    EventBits_t bits = xEventGroupWaitBits(
        wifi_event_group,
        WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
        pdFALSE,
        pdFALSE,
        portMAX_DELAY);

    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI("WiFi", "Connected to Wi-Fi");
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGI("WiFi", "Failed to connect to Wi-Fi");
    }
}

// Modified sync_time_with_ntp_and_set_ds3231 function
void sync_time_with_ntp_and_set_ds3231(void) {
    // Initialize and connect to Wi-Fi
    initWiFi();

    // Initialize SNTP for NTP synchronization
    initialize_sntp();
    
    // Wait for time to sync
    time_t now = 0;
    struct tm timeinfo = { 0 };
    int retry = 0;
    const int retry_count = 10;
    while (timeinfo.tm_year < (1970 - 1900) && ++retry < retry_count) {
        ESP_LOGI(TAG, "Waiting for system time to be set... (%d/%d)", retry, retry_count);
        vTaskDelay(2000 / portTICK_PERIOD_MS);
        time(&now);
        localtime_r(&now, &timeinfo);
    }

    // Set the DS3231 with this synced time
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (DS3231_ADDRESS << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, 0x00, true);  // Start at seconds register
    
    i2c_master_write_byte(cmd, dec_to_bcd(timeinfo.tm_sec), true);  // Seconds
    i2c_master_write_byte(cmd, dec_to_bcd(timeinfo.tm_min), true);  // Minutes
    i2c_master_write_byte(cmd, dec_to_bcd(timeinfo.tm_hour), true); // Hours
    i2c_master_write_byte(cmd, dec_to_bcd(timeinfo.tm_mday), true); // Date
    i2c_master_write_byte(cmd, dec_to_bcd(timeinfo.tm_mon + 1), true); // Month
    i2c_master_write_byte(cmd, dec_to_bcd(timeinfo.tm_year - 100), true); // Year

    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    ESP_LOGI(TAG, "Time set from NTP: %02d:%02d:%02d", timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);

    // Disconnect from Wi-Fi if not needed anymore
    ESP_ERROR_CHECK(esp_wifi_disconnect());
    ESP_LOGI(TAG, "Wi-Fi disconnected");
}

// Initialize SNTP for NTP synchronization
void initialize_sntp(void) {
    sntp_setoperatingmode(SNTP_OPMODE_POLL);
    sntp_setservername(0, "pool.ntp.org");
    sntp_init();
}


// Convert decimal to BCD
static uint8_t dec_to_bcd(uint8_t val) {
    return ((val / 10 * 16) + (val % 10));
}

// I2C Master initialization
void i2c_master_init(void) {
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_MASTER_SDA_IO;
    conf.scl_io_num = I2C_MASTER_SCL_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    i2c_param_config(I2C_MASTER_NUM, &conf);
    i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}

// Set alarm on DS3231
void ds3231_set_alarm(uint8_t hour, uint8_t minute) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (DS3231_ADDRESS << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, 0x07, true);  // Alarm1 register
    
    // Set alarm for a specific hour and minute (in BCD format)
    i2c_master_write_byte(cmd, dec_to_bcd(0), true);  // Seconds (not used)
    i2c_master_write_byte(cmd, dec_to_bcd(minute), true);  // Minute
    i2c_master_write_byte(cmd, dec_to_bcd(hour), true);    // Hour
    i2c_master_write_byte(cmd, 0x80, true);  // Day (ignored)

    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
}

// Initialize SNTP for NTP synchronization
void initialize_sntp(void) {
    sntp_setoperatingmode(SNTP_OPMODE_POLL);
    sntp_setservername(0, "pool.ntp.org");
    sntp_init();
}



// Configure the ESP32 to wake up from deep sleep when DS3231 alarm triggers
void configure_wakeup_pin(void) {
    esp_sleep_enable_ext0_wakeup(GPIO_WAKEUP_PIN, 0); // Wake-up on falling edge (low signal)
}

// Enter deep sleep until the DS3231 alarm triggers
void enter_deep_sleep_until_alarm(void) {
    configure_wakeup_pin();
    ESP_LOGI(TAG, "Entering deep sleep until DS3231 alarm triggers...");
    esp_deep_sleep_start();  // Enter deep sleep, wake-up when SQW triggers
}
