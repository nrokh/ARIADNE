#ifndef LEDS_H
#define LEDS_H

#include <Arduino.h>

#include "driver/ledc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#define LEDC_LS_TIMER LEDC_TIMER_1
#define LEDC_LS_MODE LEDC_LOW_SPEED_MODE

#define LEDC_CH_NUM (1)  // Number of LEDS using the fade service

// #define LEDC_RED_PIN(10)
//  #define LEDC_RED_CHANNEL LEDC_CHANNEL_0
uint8_t LED_ORANGE_PIN;
uint8_t LED_BLUE_PIN;
#define LEDC_BLUE_CHANNEL LEDC_CHANNEL_0

#define LEDC_BREATH_FADE_TIME (300)

typedef enum {
    LED_ORANGE_CHANNEL,
    LED_BLUE_CHANNEL
} LED_channel_select_t;

typedef enum {
    LED_STATE_OFF,
    LED_STATE_BLE_LOOKING_FOR_CONNECTION,
    LED_STATE_CONNECTED
} LEDstate_t;
/*
 * This callback function will be called when fade operation has ended
 * Use callback only if you are aware it is being called inside an ISR
 * Otherwise, you can use a semaphore to unblock tasks
 */
static bool
cb_ledc_fade_end_event(const ledc_cb_param_t *param, void *user_arg);

/**
 * @brief setup the LED configuration
 *
 * @param orange_LED Pin of the orange LED - Warning this led is not used for PWM and just referenced
 * @param blue_LED Pin of the blue LED - This led indicates BLE connection and idle status
 */
void led_setup(uint8_t orange_LED, uint8_t blue_LED);
void led_breath();
void led_pressureMode();
void led_fade_exponentially(uint16_t led_dutycycle, LED_channel_select_t led);
void led_fade_to(uint8_t dutycycle, LED_channel_select_t led);
void led_set_duty(uint8_t duty, LED_channel_select_t led);

//...
#endif