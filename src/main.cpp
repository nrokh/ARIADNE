/** Ariadne
 *
 * DRV2605-Driver interaction
 * Currently all drivers use the same I2C bus (only one I2 peripheral available).
 * And the drivers are not individually adressable the enable pin of the drivers is used to
 * selectively activate and deactivate the devices for writing and reading purposes.

 *
 *  Created: 2023
 *      Author: JMartus
 */

/*

*/
#include <Arduino.h>
#include <Ariadne.h>
#include <BLE_HANDLER.h>
#include <DRV2605_util.h>
#include <IIS3DWB.h>
#include <LEDS.h>
#include <SPI.h>
#include <SparkFun_MAX1704x_Fuel_Gauge_Arduino_Library.h>
#include <Wire.h>

#include "Adafruit_DRV2605.h"
#include "NimBLEDevice.h"
#include "driver/spi_master.h"
#include "driver/usb_serial_jtag.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "hal/usb_serial_jtag_ll.h"

// REGULAR ARIADNE
#define LED_ORANGE D1
#define LED_BLUE D0
#define I2C_SDA D4
#define I2C_SCL D5
#define I2C_FREQ 20000UL

#define ENABLE_B D3
#define ENABLE_A D2

#define SPI_FREQUENCY (20 * 1000 * 1000)

#define SPI_CS_A D7
#define SPI_CS_B D6
#define SPI_CLK D8
#define SPI_MISO D10
#define SPI_MOSI D9

#define ACC_WORD 7
#define ACC_BUFFER_WORDS 512
#define ACC_BUFFER_HEADER_SZ 6
#define ACC_BUFFER_OVERHEAD (ACC_BUFFER_HEADER_SZ)
#define ACC_WATERMARK (ACC_BUFFER_WORDS / 2)
#define ACC_BUFFER_BYTES (ACC_BUFFER_WORDS * ACC_WORD)

#define ACC_SCALE ACCEL_16G
#define ACC_AXIS Z_AXIS_ONLY

/*
 * For compatibility reasons with inhouse scipts IDx of A is '1' and of B is '0'
 * However, I recommend to change them to IDx=(uint8_t) 'A'; and IDx=(uint8_t) 'B';
 * and adapt the respective application reading out the data.
 * When internal subject-trials have been completed this will probably be updated
 */

uint8_t *acc_A_buffer;  //[ACC_BUFFER_BYTES + ACC_BUFFER_OVERHEAD];
iis3dwb_device_t acc_A = {
    .IDx = (uint8_t)'A',  // 1
    .full_scale = ACC_SCALE,
    .xl_axis = ACC_AXIS,
    .data_buffer = acc_A_buffer};

uint8_t *acc_B_buffer;  //[ACC_BUFFER_BYTES + ACC_BUFFER_OVERHEAD];
iis3dwb_device_t acc_B{
    .IDx = (uint8_t)'B',  // 2
    .full_scale = ACC_SCALE,
    .xl_axis = ACC_AXIS,
    .data_buffer = acc_B_buffer};

#define USB_NUM_BUFFER 2
#define USB_BUFFER_SCALE 4
#define USB_BUFFER_RX 128
#define USB_BUFFER_TX ((ACC_BUFFER_BYTES + ACC_BUFFER_OVERHEAD) * USB_BUFFER_SCALE)
// uint8_t usb_buffer_tx[USB_NUM_BUFFER][USB_BUFFER_TX];
uint8_t usb_buffer_rx[USB_BUFFER_RX];

static const std::string deviceName = "Ariadne";
SFE_MAX1704X lipo(MAX1704X_MAX17048);  // Battery Monitor

auto &drv = DRV2605_UTIL::getInstance();
auto &ariadne = Ariadne::getInstance();
// ariadne_stimMode_t stimMode = ariadne.stimMode();

void ble_setup();
void accelerometer_setup(void);
void accelerometer_config(iis3dwb_device_t *device, gpio_num_t cs_pin);
void usb_init();

uint16_t cnt = 0;

static iis3dwb_OUT_WORD_t WORDS[ACC_BUFFER_WORDS];
typedef union {
    uint8_t uint8[ACC_BUFFER_WORDS * sizeof(uint16_t)];
    int16_t int16[ACC_BUFFER_WORDS];
} sendUSB_t;
sendUSB_t sendUSB_A;
sendUSB_t sendUSB_B;

// Define the start time
uint32_t startTime;

// Define the start time for each operation
uint32_t writeHeaderStartTime;
uint32_t writeAccAStartTime;
uint32_t writeNumWordsAStartTime;
uint32_t writeSendAStartTime;

void IRAM_ATTR get_acc_fifo_task(void *pvParameters) {
    char header[] = {'D', 'A', 'T'};

    int sendA = 0, sendB = 0;

    spi_transaction_t *trans_result;

    spi_transaction_t spi_read_fifo_A = {
        //.flags = SPI_DEVICE_HALFDUPLEX,
        .cmd = SPI_READ_CMD,
        .addr = IIS3DWB_FIFO_DATA_OUT_TAG,
        .rx_buffer = acc_A_buffer};

    spi_transaction_t spi_read_fifo_B = {
        //.flags = SPI_DEVICE_HALFDUPLEX,
        .cmd = SPI_READ_CMD,
        .addr = IIS3DWB_FIFO_DATA_OUT_TAG,
        .rx_buffer = acc_B_buffer};

    iis3dwb_fifo_status_t statusA, statusB;
    uint16_t num_wordsA = 0,
             num_wordsB = 0;
    ariadne.setRecordingTimestamp_us();  // save reference Timestamp offset
    iis3dwb_fifo_mode_set(&acc_A, IIS3DWB_STREAM_MODE);
    iis3dwb_fifo_mode_set(&acc_B, IIS3DWB_STREAM_MODE);
    vTaskDelay(pdMS_TO_TICKS(6));

    while (ariadne.is_recording_acc) {
        // Check if device is connected and get fifo status
        if (iis3dwb_who_am_i(&acc_A) == IIS3DWB_WHO_AM_I_EXPECTED) {
            statusA = iis3dwb_fifo_status_get(&acc_A);
            num_wordsA = (uint16_t)statusA.data & 0x03FF;

            if (statusA.fifo_status.status2.status2.fifo_ovr_ia) {
                log_e("\n\e[1;31mDevice %c - fifo overrun\e[0m\n", (char)acc_A.IDx);
                digitalWrite(LED_ORANGE_PIN, HIGH);
            }
        } else {
            num_wordsA = 0;
            log_e("\e[1;31 Device %c could not be found.", (char)acc_A.IDx);
            digitalWrite(LED_ORANGE_PIN, HIGH);
        }

        if (iis3dwb_who_am_i(&acc_B) == IIS3DWB_WHO_AM_I_EXPECTED) {
            statusB = iis3dwb_fifo_status_get(&acc_B);
            num_wordsB = (uint16_t)statusB.data & 0x03FF;

            if (statusB.fifo_status.status2.status2.fifo_ovr_ia) {
                log_e("\n\e[1;31mDevice %c - fifo overrun\e[0m\n", (char)acc_B.IDx);
                digitalWrite(LED_ORANGE_PIN, HIGH);
            }
        } else {
            num_wordsA = 0;
            log_e("\e[1;31 Device %c could not be found.", (char)acc_B.IDx);
            digitalWrite(LED_ORANGE_PIN, HIGH);
        }

        if (num_wordsA) {
            spi_read_fifo_A.length = num_wordsA * 8 * ACC_WORD;
            spi_read_fifo_A.rxlength = num_wordsA * 8 * ACC_WORD;
            ESP_ERROR_CHECK(spi_device_queue_trans(acc_A.spi_handle, &spi_read_fifo_A, pdMS_TO_TICKS(15)));
        }
        if (num_wordsB) {
            spi_read_fifo_B.length = num_wordsB * 8 * ACC_WORD;
            spi_read_fifo_B.rxlength = num_wordsB * 8 * ACC_WORD;
            ESP_ERROR_CHECK(spi_device_queue_trans(acc_B.spi_handle, &spi_read_fifo_B, pdMS_TO_TICKS(15)));
        }
        vTaskDelay(pdMS_TO_TICKS(10));

        if (num_wordsA) {
            spi_device_get_trans_result(acc_A.spi_handle, &trans_result, pdMS_TO_TICKS(2));
            memcpy(WORDS, acc_A_buffer, num_wordsA * ACC_WORD);
            for (int i = 0; i < num_wordsA; i++) {
                sendUSB_A.int16[i] = WORDS[i].OUT_WORD.OUT_A.OUT_A.OUT_A_Z;  // note: A in OUT_A does not refer to the acceleration register of the accelerometer
            }
        }
        if (num_wordsB) {
            spi_device_get_trans_result(acc_B.spi_handle, &trans_result, pdMS_TO_TICKS(2));
            memcpy(WORDS, acc_B_buffer, num_wordsB * ACC_WORD);
            for (int i = 0; i < num_wordsB; i++) {
                sendUSB_B.int16[i] = WORDS[i].OUT_WORD.OUT_A.OUT_A.OUT_A_Z;  // note: A in OUT_A does not refer to the acceleration register of the accelerometer
            }
        }
        if (num_wordsA) {
            usb_serial_jtag_write_bytes(&header, 3, pdMS_TO_TICKS(1));
            usb_serial_jtag_write_bytes(&acc_A.IDx, 1, pdMS_TO_TICKS(1));
            usb_serial_jtag_write_bytes(&num_wordsA, 2, pdMS_TO_TICKS(1));

            sendA = usb_serial_jtag_write_bytes(&sendUSB_A, num_wordsA * sizeof(uint16_t), pdMS_TO_TICKS(1));
        }
        if (num_wordsB) {
            usb_serial_jtag_write_bytes(&header, 3, pdMS_TO_TICKS(1));
            usb_serial_jtag_write_bytes(&acc_B.IDx, 1, pdMS_TO_TICKS(1));
            usb_serial_jtag_write_bytes(&num_wordsB, 2, pdMS_TO_TICKS(1));

            sendB = usb_serial_jtag_write_bytes(&sendUSB_B, num_wordsB * sizeof(uint16_t), pdMS_TO_TICKS(1));
        }
        // make data available for host
        usb_serial_jtag_ll_txfifo_flush();
        taskYIELD();
    }
    iis3dwb_fifo_mode_set(&acc_A, IIS3DWB_BYPASS_MODE);
    iis3dwb_fifo_mode_set(&acc_B, IIS3DWB_BYPASS_MODE);
    vTaskDelete(NULL);
}

int is_plugged_usb(void) {
    uint32_t *serialFrame = (uint32_t *)USB_SERIAL_JTAG_FRAM_NUM_REG;
    uint32_t firstSerFrame = *serialFrame;
    vTaskDelay(pdMS_TO_TICKS(10));
    return (int)(*serialFrame - firstSerFrame);
}
void usbTask(void *pvParameters) {
    while (1) {
        // When receiving a command via USB the "acceleration task" is startet and the recording mode is toggled
        // when recording mode is set to "false" the "acceleration task" will complete and terminate
        if (usb_serial_jtag_read_bytes(usb_buffer_rx, USB_BUFFER_RX, 1 / portTICK_PERIOD_MS)) {
            if (!ariadne.is_recording_acc) {
                ariadne.is_recording_acc = true;
                xTaskCreate(get_acc_fifo_task, "ACC TASK", 2048 * 4, NULL, tskIDLE_PRIORITY + 1, NULL);
                digitalWrite(LED_ORANGE_PIN, LOW);
            } else {
                ariadne.is_recording_acc = false;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    vTaskDelete(NULL);
}

void setup() {
    log_i("Initialising Ariadne");
    Wire.setPins(I2C_SDA, I2C_SCL);
    delay(1);

    // Init LRA-Drivers
    // drv.setTrigger(TRIGGER_A, TRIGGER_B);
    drv.setEnablePins(ENABLE_A, ENABLE_B);
    drv.begin(&Wire);
    drv.enableAll();
    drv.init();

    drv.disableAll();

    // Init lipo battery gauge
    lipo.begin();
    if (!lipo.begin(Wire))  // Connect to the MAX17048
    {
        log_w("Battery gauge not detected.");
    } else {
        lipo.quickStart();
        ariadne.setBatteryGauge(&lipo);
    }

    usb_init();
    led_setup(LED_BLUE, LED_ORANGE);
    ble_setup(deviceName);

    accelerometer_setup();

    pinMode(LED_ORANGE_PIN, OUTPUT);
    digitalWrite(LED_ORANGE_PIN, LOW);

    led_breath();  // indicate "looking for connection"
    xTaskCreate(usbTask, "USB_TASK", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, NULL);

    log_i("Initialisation Complete");
}

void loop() {
}

void accelerometer_config(iis3dwb_device_t *device, gpio_num_t cs_pin) {
    // SPI device configuration

    spi_device_interface_config_t dev_cfg = {
        .command_bits = 1,  ///< Default amount of bits in command phase (0-16), used when ``SPI_TRANS_VARIABLE_CMD`` is not used, otherwise ignored.
        .address_bits = 7,  ///< Default amount of bits in address phase (0-64), used when ``SPI_TRANS_VARIABLE_ADDR`` is not used, otherwise ignored.
        .dummy_bits = 0,    ///< Amount of dummy bits to insert between address and data phase
        .mode = 0,
        .cs_ena_pretrans = 0,  //
        .cs_ena_posttrans = 0,
        .clock_speed_hz = SPI_FREQUENCY,
        .spics_io_num = cs_pin,
        // .flags = SPI_DEVICE_HALFDUPLEX,
        .queue_size = 2,
    };
    device->spi_config = dev_cfg;
    ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &device->spi_config, &device->spi_handle));
    ESP_ERROR_CHECK(spi_device_acquire_bus(device->spi_handle, portMAX_DELAY));
    // set to sleep

    iis3dwb_sleep(device);

    delay(IIS3DWB_SHUTDOWN_TIME);
    // iis3dwb_ODR_on_int1(cs_pin); #TODO
    iis3dwb_reset_set(device);
    /* Set XL Batch Data Rate */
    iis3dwb_fifo_xl_batch_set(device, IIS3DWB_XL_BATCHED_AT_26k7Hz);

    /*  Set Temperature Batch Data Rate */
    iis3dwb_fifo_temp_batch_set(device, IIS3DWB_TEMP_NOT_BATCHED);

    /* Set  FIFO Bypass Mode */
    iis3dwb_fifo_mode_set(device, IIS3DWB_BYPASS_MODE);
    /* Set  FIFO Watermark */
    iis3dwb_fifo_watermark_set(device, ACC_WATERMARK);  // Total Bytes =  Number of words * FIFO_WORD (7 or 6) + 1 to account for occasional temperature datum

    /* Set default acceleration scale */
    iis3dwb_xl_full_scale_set(device, device->full_scale);

    /*Significantly improves noise density if only one axis is chosen*/
    iis3dwb_axis_sel_set(device, device->xl_axis);

    /*Register address automatically incremented*/
    iis3dwb_auto_increment_set(device, 1);

    /*	Configure filtering chain(No aux interface)
     *	Accelerometer - LPF1 + LPF2 path
     */
    iis3dwb_xl_hp_path_on_out_set(device, IIS3DWB_LP_6k3Hz);
    // iis3dwb_xl_filter_lp2_set(handle, PROPERTY_ENABLE);
    iis3dwb_wake(device);
    /* Wait stable output */
    if (iis3dwb_who_am_i(device) != IIS3DWB_WHO_AM_I_EXPECTED) {
        log_e("Device %c was not found", (char)device->IDx);
        // digitalWrite(D1, HIGH);
    } else {
        log_i("Device %c was found", (char)device->IDx);
    }
    spi_device_release_bus(device->spi_handle);
}

void accelerometer_setup(void) {
    // setup buffers
    acc_A_buffer = (uint8_t *)heap_caps_malloc(ACC_BUFFER_BYTES, MALLOC_CAP_DMA);

    // setup buffers
    acc_B_buffer = (uint8_t *)heap_caps_malloc(ACC_BUFFER_BYTES, MALLOC_CAP_DMA);

    // SPI
    spi_bus_config_t bus_cfg = {
        .mosi_io_num = SPI_MOSI,
        .miso_io_num = SPI_MISO,
        .sclk_io_num = SPI_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = ACC_BUFFER_BYTES,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &bus_cfg, SPI_DMA_CH_AUTO));
    log_i("init accelerometers:");
    accelerometer_config(&acc_A, (gpio_num_t)SPI_CS_A);
    accelerometer_config(&acc_B, (gpio_num_t)SPI_CS_B);
    delay(IIS3DWB_BOOT_TIME);
}

void usb_init() {
    /* Configure USB-CDC */
    usb_serial_jtag_driver_config_t usb_serial_config = {
        .tx_buffer_size = USB_BUFFER_TX,
        .rx_buffer_size = USB_BUFFER_RX};
    ESP_ERROR_CHECK(usb_serial_jtag_driver_install(&usb_serial_config));

    /* Configure a bridge buffer for the incoming data */
    memset(usb_buffer_rx, 0x00, sizeof(usb_buffer_rx) * sizeof(uint8_t));
    // memset(usb_buffer_tx, 0x00, sizeof(usb_buffer_tx) * sizeof(uint8_t));
    ESP_LOGI("USB", "USB initialised");
}