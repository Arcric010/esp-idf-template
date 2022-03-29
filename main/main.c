#include <stdio.h>
// Jika membutuhkan serial.print, cukup printf seperti pada program C
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define GPIO_OUTPUT_A 2
#define GPIO_OUTPUT_B 4
#define GPIO_OUTPUT_C 5
#define GPIO_OUTPUT_D 18
#define GPIO_OUTPUT_E 19
#define GPIO_OUTPUT_F 21
#define GPIO_OUTPUT_G 22
#define GPIO_OUTPUT_H 23
#define GPIO_OUTPUT_PIN_SEL ((1ULL<<GPIO_OUTPUT_A) |(1ULL<<GPIO_OUTPUT_B) | (1ULL<<GPIO_OUTPUT_C) | (1ULL<<GPIO_OUTPUT_D)|(1ULL<<GPIO_OUTPUT_E) | (1ULL<<GPIO_OUTPUT_F)|(1ULL<<GPIO_OUTPUT_G)| (1ULL<<GPIO_OUTPUT_H))

#define DELAY_MS 500 // isi waktu delay
const TickType_t xDelay = DELAY_MS / portTICK_PERIOD_MS;

void app_main() {
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_DISABLE; // tidak menggunakan interrupt
    io_conf.mode = GPIO MODE OUTPUT; // mode output
    io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE; // tidak menggunakan pull down
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE; // tidak menggunakan pull up
    gpio_config(&io_conf);

    while (1) {
        gpio_set_level(GPIO_OUTPUT_A, HIGH);
        vTaskDelay(xDelay);
        gpio_set_lebel(GPIO_OUTPUT_A, LOW);
        gpio_set_level(GPIO_OUTPUT_B, HIGH);
        vTaskDelay(xDelay);
        gpio_set_lebel(GPIO_OUTPUT_B, LOW);
        gpio_set_level(GPIO_OUTPUT_C, HIGH);
        vTaskDelay(xDelay);
        gpio_set_lebel(GPIO_OUTPUT_C, LOW);
        gpio_set_level(GPIO_OUTPUT_D, HIGH);
        vTaskDelay(xDelay);
        gpio_set_lebel(GPIO_OUTPUT_D, LOW);
        gpio_set_level(GPIO_OUTPUT_E, HIGH);
        vTaskDelay(xDelay);
        gpio_set_lebel(GPIO_OUTPUT_E, LOW);
        gpio_set_level(GPIO_OUTPUT_F, HIGH);
        vTaskDelay(xDelay);
        gpio_set_lebel(GPIO_OUTPUT_F, LOW);
        gpio_set_level(GPIO_OUTPUT_G, HIGH);
        vTaskDelay(xDelay);
        gpio_set_lebel(GPIO_OUTPUT_G, LOW);
        gpio_set_level(GPIO_OUTPUT_H, HIGH);
        vTaskDelay(xDelay);
        gpio_set_lebel(GPIO_OUTPUT_H, LOW);
    }
}
