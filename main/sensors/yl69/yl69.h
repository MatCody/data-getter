#ifndef YL69_H
#define YL69_H

#include "freertos/FreeRTOS.h"
#include "driver/adc.h"  // Add this to use adc1_channel_t

#define DEFAULT_VREF 1100 // Use adc2_vref_to_gpio() to obtain a better estimate
#define NO_OF_SAMPLES 64  // Multisampling

#define VALUE_MAX 4095 // Max ADC value of soil moisture

void yl69_setup(adc1_channel_t _channel);
uint32_t yl69_read();
uint32_t yl69_normalization(uint32_t value_t);

#endif // YL69_H
