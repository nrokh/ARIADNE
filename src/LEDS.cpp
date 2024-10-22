#include <LEDS.h>

static uint8_t LED_BREATH_CYLE[] = {0, 34, 82, 154, 255, 255, 154, 82, 34, 0};
uint8_t LEDC_CYLE_index = 0;
LEDstate_t led_state = LED_STATE_CONNECTED;

ledc_channel_config_t ledc_channel[2];
static uint8_t LEDC_CYLE_size = sizeof(LED_BREATH_CYLE) / sizeof(LED_BREATH_CYLE[0]);

int const led_fadetime = 50;

const float R = (255 * log10(2)) / (log10(255));

static IRAM_ATTR bool cb_ledc_fade_end_event(const ledc_cb_param_t *param, void *user_arg) {
    portBASE_TYPE taskAwoken = pdFALSE;

    if (param->event == LEDC_FADE_END_EVT) {
        SemaphoreHandle_t counting_sem = (SemaphoreHandle_t)user_arg;
        xSemaphoreGiveFromISR(counting_sem, &taskAwoken);
        switch (led_state) {
            case LED_STATE_BLE_LOOKING_FOR_CONNECTION:

                ledc_set_fade_with_time(ledc_channel[LED_BLUE_CHANNEL].speed_mode,
                                        ledc_channel[LED_BLUE_CHANNEL].channel, LED_BREATH_CYLE[LEDC_CYLE_index++ % LEDC_CYLE_size], LEDC_BREATH_FADE_TIME);
                ledc_fade_start(ledc_channel[LED_BLUE_CHANNEL].speed_mode,
                                ledc_channel[LED_BLUE_CHANNEL].channel, LEDC_FADE_NO_WAIT);

                break;
            case LED_STATE_CONNECTED:
                break;
            default:
                break;
        }

        // ESP_LOGI("LED", "Fade Next");
    }

    return (taskAwoken == pdTRUE);
}

/**

    @brief Sets the LED state to pressure sensing.
    This function sets the LED state to LED_STATE_PRESSURE_SENSING to indicate that the device is currently in pressure sensing mode.
    */
void led_pressureMode() {
    led_state = LED_STATE_CONNECTED;
}

void led_fade_exponentially(uint16_t led_dutycycle, LED_channel_select_t led) {
    led_dutycycle = pow(2, (led_dutycycle / R)) - 1;
    led_fade_to(led_dutycycle, led);
}

void led_setup(uint8_t pin_orange_LED, uint8_t pin_blue_LED) {
    /*
     * Prepare and set configuration of timers
     * that will be used by LED Controller
     */

    ledc_timer_config_t ledc_timer;
    uint8_t ch;

    LED_ORANGE_PIN = pin_orange_LED;
    LED_BLUE_PIN = pin_blue_LED;

    // Set the duty resolution of the PWM signal
    ledc_timer.duty_resolution = LEDC_TIMER_8_BIT;

    // Set the frequency of the PWM signal
    ledc_timer.freq_hz = 5000;

    // Set the speed mode of the timer
    ledc_timer.speed_mode = LEDC_LS_MODE;

    // Set the timer index
    ledc_timer.timer_num = LEDC_LS_TIMER;

    // Set the source clock to be automatically selected
    ledc_timer.clk_cfg = LEDC_AUTO_CLK;

    // Set configuration of timer0 for high speed channels
    // Prepare and set configuration of timer1 for low speed channels
    ledc_timer_config(&ledc_timer);

    /*
     * Prepare individual configuration
     * for each channel of LED Controller
     * by selecting:
     * - controller's channel number
     * - output duty cycle, set initially to 0
     * - GPIO number where LED is connected to
     * - speed mode, either high or low
     * - timer servicing selected channel
     *   Note: if different channels use one timer,
     *         then frequency and bit_num of these channels
     *         will be the same
     */

    memset(&ledc_channel, 0, sizeof(ledc_channel_config_t));

    // Set the channel number
    ledc_channel[0].channel = LEDC_BLUE_CHANNEL;
    // Set the GPIO number where the channel is connected
    ledc_channel[0].gpio_num = LED_BLUE_PIN;

    // Set the channel number
    // ledc_channel[1].channel = LEDC_BLUE_CHANNEL;
    // Set the GPIO number where the channel is connected
    // ledc_channel[1].gpio_num = LED_BLUE_PIN;

    for (ch = 0; ch < LEDC_CH_NUM; ch++) {
        // Set the initial duty of the channel
        ledc_channel[ch].duty = 0;

        // Set the speed mode of the channel
        ledc_channel[ch].speed_mode = LEDC_LS_MODE;

        // Set the timer index to be used by the channel
        ledc_channel[ch].timer_sel = LEDC_LS_TIMER;

        // Set the output invert flag to be disabled
        ledc_channel[ch].flags.output_invert = 0;

        // Set LED Controller with previously prepared configuration
        ledc_channel_config(&ledc_channel[ch]);
    }

    // Initialize fade service.
    ledc_fade_func_install(0);
    ledc_cbs_t callbacks;
    callbacks.fade_cb = cb_ledc_fade_end_event;
    SemaphoreHandle_t counting_sem = xSemaphoreCreateCounting(2, 0);

    // SemaphoreHandle_t counting_sem = xSemaphoreCreateCounting(LEDC_TEST_CH_NUM, 0);
    for (ch = 0; ch < LEDC_CH_NUM; ch++) {
        ledc_cb_register(ledc_channel[ch].speed_mode, ledc_channel[ch].channel, &callbacks, (void *)counting_sem);
    }
}

/**

    @brief Sets the state of the blue LED to "looking for connection" and starts a breathing animation.
    The function uses the LEDC (LED Controller) API to set the state of the blue LED to "looking for connection" and
    starts a breathing animation by setting a fade with a specific cycle and fade time. The cycle is determined by the
    LEDC_CYLE_index, which is incremented and moduloed by the LEDC_CYLE_size to cycle through a sequence of values. The
    fade time is specified by the LEDC_BREATH_FADE_TIME constant.
    @note This function uses the LEDC API, which is part of the ESP-IDF (Espressif IoT Development Framework).
*/
void led_breath() {
    led_state = LED_STATE_BLE_LOOKING_FOR_CONNECTION;
    ledc_set_fade_with_time(ledc_channel[LED_BLUE_CHANNEL].speed_mode,
                            ledc_channel[LED_BLUE_CHANNEL].channel, LED_BREATH_CYLE[LEDC_CYLE_index++ % LEDC_CYLE_size], LEDC_BREATH_FADE_TIME);
    ledc_fade_start(ledc_channel[LED_BLUE_CHANNEL].speed_mode,
                    ledc_channel[LED_BLUE_CHANNEL].channel, LEDC_FADE_NO_WAIT);
}

void led_fade_to(uint8_t dutycycle, LED_channel_select_t led) {
    ledc_set_fade_with_time(ledc_channel[led].speed_mode,
                            ledc_channel[led].channel, dutycycle, led_fadetime);
    ledc_fade_start(ledc_channel[led].speed_mode,
                    ledc_channel[led].channel, LEDC_FADE_NO_WAIT);
}

void led_set_duty(uint8_t duty, LED_channel_select_t led) {
    ledc_set_duty(ledc_channel[led].speed_mode, ledc_channel[led].channel, duty);
    ledc_update_duty(ledc_channel[led].speed_mode, ledc_channel[led].channel);
}