#include "driver/gpio.h"
#include "buzzer.h"

// Define the GPIO pin for the buzzer
#define BUZZER_PIN GPIO_NUM_23  // Change this to the GPIO pin you're using

void activate_buzzer() {
    // Configure the GPIO pin as output
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << BUZZER_PIN);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf);

    // Turn the buzzer on
    gpio_set_level(BUZZER_PIN, 1);

    // Wait for 5 seconds
    vTaskDelay(pdMS_TO_TICKS(5000));

    // Turn the buzzer off
    gpio_set_level(BUZZER_PIN, 0);
}
