/*
 * ESP32 Soil Moisture Sensor YL-69 or HL-69 Driver
 * Copyright 2021, Lorenzo Carnevale <lcarnevale@unime.it>
 */

#include "yl69.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"

static adc_channel_t channel;
static const char *TAG = "YL69";

void yl69_setup(adc_channel_t _channel) {
    channel = _channel;
    // ADC initialization happens elsewhere
}

uint32_t yl69_read(adc_oneshot_unit_handle_t adc_handle) {
    int adc_reading = 0;
    esp_err_t err;

    for (int i = 0; i < NO_OF_SAMPLES; i++) {
        int raw;
        err = adc_oneshot_read(adc_handle, channel, &raw);
        if (err == ESP_OK) {
            adc_reading += raw;
        } else {
            ESP_LOGE(TAG, "ADC reading failed: %s", esp_err_to_name(err));
            return 0;
        }
    }
    adc_reading /= NO_OF_SAMPLES;

    return (uint32_t)adc_reading;
}

uint32_t yl69_normalization(uint32_t value_t) {
    return (value_t * 100) / VALUE_MAX;
}

