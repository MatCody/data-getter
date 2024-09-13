#ifndef YL69_H
#define YL69_H

#include "freertos/FreeRTOS.h"
#include "esp_adc/adc_oneshot.h"

#define DEFAULT_VREF 1100
#define NO_OF_SAMPLES 64
#define VALUE_MAX 4095

void yl69_setup(adc_channel_t _channel);
uint32_t yl69_read(adc_oneshot_unit_handle_t adc_handle);
uint32_t yl69_normalization(uint32_t value_t);

#endif // YL69_H
    