// #include "driver/gpio.h"

// #define ENC_A_PIN 4
// #define ENC_B_PIN 5

// gpio_config_t io_conf = {
//     .intr_type = GPIO_INTR_ANYEDGE,   // interrupt on both rising and falling edges
//     .mode = GPIO_MODE_INPUT,
//     .pin_bit_mask = (1ULL << ENC_A_PIN) | (1ULL << ENC_B_PIN),
//     .pull_up_en = GPIO_PULLUP_DISABLE,
//     .pull_down_en = GPIO_PULLDOWN_DISABLE
// };
// gpio_config(&io_conf);

// gpio_install_isr_service(0);  // Install the default ISR service
// gpio_isr_handler_add(ENC_A_PIN, encoder_isr, NULL);  // Attach the ISR to channel A

// volatile int position = 0;

// void IRAM_ATTR encoder_isr(void* arg) {
//     int stateA = gpio_get_level(ENC_A_PIN);
//     int stateB = gpio_get_level(ENC_B_PIN);

//     if (stateA == stateB)
//         position++;
//     else
//         position--;
// }

// while (1) {
//     printf("Position: %d\n", position);
//     vTaskDelay(pdMS_TO_TICKS(100));  // print every 100ms
// }

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include <stdio.h>

#define ENC_A_PIN 4
#define ENC_B_PIN 5

volatile int position = 0;

// ISR must be declared before using it
void IRAM_ATTR encoder_isr(void* arg) {
    int stateA = gpio_get_level(ENC_A_PIN);
    int stateB = gpio_get_level(ENC_B_PIN);

    if (stateA == stateB)
        position++;
    else
        position--;
}

void app_main(void) {
    // GPIO configuration
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_ANYEDGE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << ENC_A_PIN) | (1ULL << ENC_B_PIN),
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE
    };
    gpio_config(&io_conf);

    // Install ISR service and attach handler
    gpio_install_isr_service(0);
    gpio_isr_handler_add(ENC_A_PIN, encoder_isr, NULL);

    while (1) {
        printf("Position: %d\n", position);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
