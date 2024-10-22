#ifndef iis3dwb__
#define iis3dwb__
#include <Arduino.h>
#include <m_spi_esp32c3.h>  // helper
#include <stdlib.h>

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "soc/gpio_reg.h"

#define IIS3DWB_BOOT_TIME 10             // in ms
#define IIS3DWB_SHUTDOWN_TIME 1          // in ms should be 100 us
#define IIS3DWB_WHO_AM_I_EXPECTED 0x7BU  // Device Identification (Who am I)

// IIS3DWB registers - see datasheet
#define IIS3DWB_PIN_CTRL 0x02U
typedef union {
    uint8_t data;
    struct
    {
        uint8_t not_used_01 : 6;
        uint8_t sdo_pu_en : 1;
        uint8_t not_used_02 : 1;
    } pin_ctrl;
} iis3dwb_pin_ctrl_t;

#define IIS3DWB_FIFO_CTRL1 0x07U
typedef union {
    uint8_t data;
    struct
    {
        uint8_t wtm : 8;
    } fifo_ctrl1;
} iis3dwb_fifo_ctrl1_t;

#define IIS3DWB_FIFO_CTRL2 0x08U
typedef union {
    uint8_t data;
    struct
    {
        uint8_t wtm : 1;
        uint8_t not_used_01 : 6;
        uint8_t stop_on_wtm : 1;
    } fifo_ctrl2;
} iis3dwb_fifo_ctrl2_t;

#define IIS3DWB_FIFO_CTRL3 0x09U
typedef union {
    uint8_t data;
    struct
    {
        uint8_t bdr_xl : 4;
        uint8_t not_used_01 : 4;
    } fifo_ctrl3;
} iis3dwb_fifo_ctrl3_t;

#define IIS3DWB_FIFO_CTRL4 0x0AU
typedef union {
    uint8_t data;
    struct
    {
        uint8_t fifo_mode : 3;
        uint8_t not_used_01 : 1;
        uint8_t odr_t_batch : 2;
        uint8_t odr_ts_batch : 2;
    } fifo_ctrl4;
} iis3dwb_fifo_ctrl4_t;

#define IIS3DWB_COUNTER_BDR_REG1 0x0BU
typedef union {
    uint8_t data;
    struct
    {
        uint8_t cnt_bdr_th : 3;
        uint8_t not_used_01 : 3;
        uint8_t rst_counter_bdr : 1;
        uint8_t dataready_pulsed : 1;
    } counter_bdr_reg1;
} iis3dwb_counter_bdr_reg1_t;

#define IIS3DWB_COUNTER_BDR_REG2 0x0CU
typedef union {
    uint8_t data;
    struct
    {
        uint8_t cnt_bdr_th : 8;
    } counter_bdr_reg2;
} iis3dwb_counter_bdr_reg2_t;

#define IIS3DWB_INT1_CTRL 0x0DU
typedef union {
    uint8_t data;
    struct
    {
        uint8_t int1_drdy_xl : 1;
        uint8_t not_used_01 : 1;
        uint8_t int1_boot : 1;
        uint8_t int1_fifo_th : 1;
        uint8_t int1_fifo_ovr : 1;
        uint8_t int1_fifo_full : 1;
        uint8_t int1_cnt_bdr : 1;
        uint8_t not_used_02 : 1;
    } int1_ctrl;
} iis3dwb_int1_ctrl_t;

#define IIS3DWB_INT2_CTRL 0x0EU
typedef union {
    uint8_t data;
    struct
    {
        uint8_t int2_drdy_xl : 1;
        uint8_t not_used_01 : 1;
        uint8_t int2_drdy_temp : 1;
        uint8_t int2_fifo_th : 1;
        uint8_t int2_fifo_ovr : 1;
        uint8_t int2_fifo_full : 1;
        uint8_t int2_cnt_bdr : 1;
        uint8_t not_used_02 : 1;
    } int2_ctrl;
} iis3dwb_int2_ctrl_t;

#define IIS3DWB_WHO_AM_I 0x0FU
#define IIS3DWB_CTRL1_XL 0x10U
typedef union {
    uint8_t data;
    struct
    {
        uint8_t not_used_01 : 1;
        uint8_t lpf2_xl_en : 1;
        uint8_t fs_xl : 2;
        uint8_t not_used_02 : 1;
        uint8_t xl_en : 3;
    } ctrl1_xl;
} iis3dwb_ctrl1_xl_t;

#define IIS3DWB_CTRL3_C 0x12U
typedef union {
    uint8_t data;
    struct
    {
        uint8_t sw_reset : 1;
        uint8_t not_used_01 : 1;
        uint8_t if_inc : 1;
        uint8_t sim : 1;
        uint8_t pp_od : 1;
        uint8_t h_lactive : 1;
        uint8_t bdu : 1;
        uint8_t boot : 1;
    } ctrl3_c;
} iis3dwb_ctrl3_c_t;

#define IIS3DWB_CTRL4_C 0x13U
typedef union {
    uint8_t data;
    struct
    {
        uint8_t _1ax_to_3regout : 1;
        uint8_t lpf1_sel_g : 1;
        uint8_t i2c_disable : 1;
        uint8_t drdy_mask : 1;
        uint8_t not_used_02 : 1;
        uint8_t int2_on_int1 : 1;
        uint8_t not_used_03 : 2;
    } ctrl4_c;
} iis3dwb_ctrl4_c_t;

#define IIS3DWB_CTRL5_C 0x14U
typedef union {
    uint8_t data;
    struct
    {
        uint8_t st_xl : 2;
        uint8_t not_used_01 : 3;
        uint8_t rounding : 2;
        uint8_t not_used_02 : 1;
        uint8_t i2c_disable : 1;
    } ctrl5_c;
} iis3dwb_ctrl5_c_t;

#define IIS3DWB_CTRL6_C 0x15U
typedef union {
    uint8_t data;
    struct
    {
        uint8_t xl_axis_sel : 2;
        uint8_t not_used_01 : 1;
        uint8_t usr_off_w : 1;
        uint8_t not_used_02 : 4;
    } ctrl6_c;
} iis3dwb_ctrl6_c_t;

#define IIS3DWB_CTRL8_XL 0x17U
typedef union {
    uint8_t data;
    struct
    {
        uint8_t not_used_01 : 2;
        uint8_t fds : 1;
        uint8_t fastsettl_mode_xl : 1;
        uint8_t hp_ref_mode_xl : 1;
        uint8_t hpcf_xl : 3;
    } ctrl8_xl;
} iis3dwb_ctrl8_xl_t;

#define IIS3DWB_CTRL10_C 0x19U
typedef union {
    uint8_t data;
    struct
    {
        uint8_t not_used_01 : 5;
        uint8_t timestamp_en : 1;
        uint8_t not_used_02 : 2;
    } ctrl10_c;
} iis3dwb_ctrl10_c_t;

#define IIS3DWB_ALL_INT_SRC 0x1AU
typedef union {
    uint8_t data;
    struct
    {
        uint8_t not_used_01 : 1;
        uint8_t wu_ia : 1;
        uint8_t not_used_02 : 3;
        uint8_t sleep_change : 1;
        uint8_t not_used_03 : 1;
        uint8_t timestamp_endcount : 1;
    } all_int_src;
} iis3dwb_all_int_src_t;

#define IIS3DWB_WAKE_UP_SRC 0x1BU
typedef union {
    uint8_t data;
    struct
    {
        uint8_t z_wu : 1;
        uint8_t y_wu : 1;
        uint8_t x_wu : 1;
        uint8_t wu_ia : 1;
        uint8_t sleep_state_ia : 1;
        uint8_t not_used_01 : 1;
        uint8_t sleep_change_ia : 1;
        uint8_t not_used_02 : 1;
    } wake_up_src;
} iis3dwb_wake_up_src_t;

#define IIS3DWB_STATUS_REG 0x1EU
typedef union {
    uint8_t data;
    struct
    {
        uint8_t xlda : 1;
        uint8_t not_used_01 : 1;
        uint8_t tda : 1;
        uint8_t not_used_02 : 5;
    } status_reg;
} iis3dwb_status_reg_t;

#define IIS3DWB_OUT_TEMP_L 0x20U
#define IIS3DWB_OUT_TEMP_H 0x21U
#define IIS3DWB_OUTX_L_A 0x28U
#define IIS3DWB_OUTX_H_A 0x29U
#define IIS3DWB_OUTY_L_A 0x2AU
#define IIS3DWB_OUTY_H_A 0x2BU
#define IIS3DWB_OUTZ_L_A 0x2CU
#define IIS3DWB_OUTZ_H_A 0x2DU

#define IIS3DWB_FIFO_STATUS1 0x3AU
typedef union {
    uint8_t data;
    struct
    {
        uint8_t diff_fifo : 8;
    } fifo_status1;
} iis3dwb_fifo_status1_t;

#define IIS3DWB_FIFO_STATUS2 0x3BU
typedef union {
    uint8_t data;
    struct
    {
        uint8_t diff_fifo : 2;
        uint8_t not_used_01 : 1;
        uint8_t over_run_latched : 1;
        uint8_t counter_bdr_ia : 1;
        uint8_t fifo_full_ia : 1;
        uint8_t fifo_ovr_ia : 1;
        uint8_t fifo_wtm_ia : 1;
    } status2;
} iis3dwb_fifo_status2_t;

typedef union {
    uint16_t data;
    struct
    {
        iis3dwb_fifo_status1_t status1;
        iis3dwb_fifo_status2_t status2;
    } fifo_status;
} iis3dwb_fifo_status_t;

#define IIS3DWB_TIMESTAMP0 0x40U
#define IIS3DWB_TIMESTAMP1 0x41U
#define IIS3DWB_TIMESTAMP2 0x42U
#define IIS3DWB_TIMESTAMP3 0x43U
#define IIS3DWB_SLOPE_EN 0x56U
typedef union {
    uint8_t data;
    struct
    {
        uint8_t lir : 1;
        uint8_t not_used_01 : 3;
        uint8_t slope_fds : 1;
        uint8_t sleep_status_on_int : 1;
        uint8_t not_used_02 : 2;
    } slope_en;
} iis3dwb_slope_en_t;

#define IIS3DWB_INTERRUPTS_EN 0x58U

typedef union {
    uint8_t data;
    struct
    {
        uint8_t not_used_01 : 7;
        uint8_t interrupts_enable : 1;
    } en_t;
} iis3dwb_interrupts_en_t;

#define IIS3DWB_WAKE_UP_THS 0x5BU
typedef union {
    uint8_t data;
    struct
    {
        uint8_t wk_ths : 6;
        uint8_t usr_off_on_wu : 1;
        uint8_t not_used_01 : 1;
    } wake_up_ths;
} iis3dwb_wake_up_ths_t;

#define IIS3DWB_WAKE_UP_DUR 0x5CU

typedef union {
    uint8_t data;
    struct
    {
        uint8_t sleep_dur : 4;
        uint8_t wake_ths_w : 1;
        uint8_t wake_dur : 2;
        uint8_t not_used_01 : 1;
    } wake_up_dur;
} iis3dwb_wake_up_dur_t;

#define IIS3DWB_MD1_CFG 0x5EU
typedef union {
    uint8_t data;
    struct
    {
        uint8_t not_used_01 : 5;
        uint8_t int1_wu : 1;
        uint8_t not_used_02 : 1;
        uint8_t int1_sleep_change : 1;
    } md1_cfg;
} iis3dwb_md1_cfg_t;

#define IIS3DWB_MD2_CFG 0x5FU
typedef union {
    uint8_t data;
    struct
    {
        uint8_t int2_timestamp : 1;
        uint8_t not_used_01 : 4;
        uint8_t int2_wu : 1;
        uint8_t not_used_02 : 1;
        uint8_t int2_sleep_change : 1;
    };
} iis3dwb_md2_cfg_t;

#define IIS3DWB_INTERNAL_FREQ_FINE 0x63U
typedef union {
    uint8_t data;
    struct
    {
        uint8_t freq_fine : 8;
    };
} iis3dwb_internal_freq_fine_t;

#define IIS3DWB_X_OFS_USR 0x73U
#define IIS3DWB_Y_OFS_USR 0x74U
#define IIS3DWB_Z_OFS_USR 0x75U
#define IIS3DWB_FIFO_DATA_OUT_TAG 0x78U
typedef union {
    uint8_t data;
    struct
    {
        uint8_t tag_parity : 1;
        uint8_t tag_cnt : 2;
        uint8_t tag_sensor : 5;
    };
} iis3dwb_fifo_data_out_tag_t;

#define IIS3DWB_FIFO_DATA_OUT_X_L 0x79U
#define IIS3DWB_FIFO_DATA_OUT_X_H 0x7AU
#define IIS3DWB_FIFO_DATA_OUT_Y_L 0x7BU
#define IIS3DWB_FIFO_DATA_OUT_Y_H 0x7CU
#define IIS3DWB_FIFO_DATA_OUT_Z_L 0x7DU
#define IIS3DWB_FIFO_DATA_OUT_Z_H 0x7EU

typedef union __attribute__((packed)) {
    uint16_t data[3];
    struct
    {
        int16_t OUT_A_X;
        int16_t OUT_A_Y;
        int16_t OUT_A_Z;
    } OUT_A;
} iis3dwb_OUT_A;

typedef union __attribute__((packed)) {
    uint8_t data[7];
    struct
    {
        iis3dwb_fifo_data_out_tag_t fifo_data_out_tag;
        iis3dwb_OUT_A OUT_A;
    } OUT_WORD;
} iis3dwb_OUT_WORD_t;

typedef enum {
    ACCEL_2G = 0x00u,
    ACCEL_4G = 0x02u,
    ACCEL_8G = 0x03u,
    ACCEL_16G = 0x01u
} a_range_t;

typedef enum {
    THREE_AXIS = 0,
    X_AXIS_ONLY = 1,
    Y_AXIS_ONLY = 2,
    Z_AXIS_ONLY = 3
} xl_axis_t;

typedef enum {
    BW_4 = 0x00u,  // divide ODR by 4
    BW_10 = 0x01u,
    BW_20 = 0x02u,
    BW_45 = 0x03u,
    BW_100 = 0x04u,
    BW_200 = 0x05u,
    BW_400 = 0x06u,
    BW_800 = 0x07
} lpf_accel_bw_t;

typedef enum {
    IIS3DWB_XL_ODR_OFF = 0,
    IIS3DWB_XL_ODR_26k7Hz = 5,
} iis3dwb_odr_xl_t;

typedef enum {
    IIS3DWB_XL_NOT_BATCHED = 0,
    IIS3DWB_XL_BATCHED_AT_26k7Hz = 10,
} iis3dwb_bdr_xl_t;

typedef enum {
    IIS3DWB_BYPASS_MODE = 0,
    IIS3DWB_FIFO_MODE = 1,
    IIS3DWB_STREAM_TO_FIFO_MODE = 3,
    IIS3DWB_BYPASS_TO_STREAM_MODE = 4,
    IIS3DWB_STREAM_MODE = 6,
    IIS3DWB_BYPASS_TO_FIFO_MODE = 7,
} iis3dwb_fifo_mode_t;

typedef enum {
    IIS3DWB_TEMP_NOT_BATCHED = 0,
    IIS3DWB_TEMP_BATCHED_AT_104Hz = 3,
} iis3dwb_odr_t_batch_t;

typedef enum {
    IIS3DWB_SLOPE_ODR_DIV_4 = 0x30,
    IIS3DWB_HP_ODR_DIV_10 = 0x11,
    IIS3DWB_HP_ODR_DIV_20 = 0x12,
    IIS3DWB_HP_ODR_DIV_45 = 0x13,
    IIS3DWB_HP_ODR_DIV_100 = 0x14,
    IIS3DWB_HP_ODR_DIV_200 = 0x15,
    IIS3DWB_HP_ODR_DIV_400 = 0x16,
    IIS3DWB_HP_ODR_DIV_800 = 0x17,
    IIS3DWB_LP_6k3Hz = 0x00,
    IIS3DWB_LP_ODR_DIV_4 = 0x80,
    IIS3DWB_LP_ODR_DIV_10 = 0x81,
    IIS3DWB_LP_ODR_DIV_20 = 0x82,
    IIS3DWB_LP_ODR_DIV_45 = 0x83,
    IIS3DWB_LP_ODR_DIV_100 = 0x84,
    IIS3DWB_LP_ODR_DIV_200 = 0x85,
    IIS3DWB_LP_ODR_DIV_400 = 0x86,
    IIS3DWB_LP_ODR_DIV_800 = 0x87
} iis3dwb_hp_slope_xl_en_t;

typedef enum {
    IIS3DWB_DRDY_LATCHED = 0,
    IIS3DWB_DRDY_PULSED = 1,
} iis3dwb_dataready_pulsed_t;

typedef enum {
    IIS3DWB_XL_TAG = 2,
    IIS3DWB_TEMPERATURE_TAG,
    IIS3DWB_TIMESTAMP_TAG,
} iis3dwb_fifo_tag_t;

#define PROPERTY_DISABLE (0u)
#define PROPERTY_ENABLE (1u)

// --- IIS3DWB CLASS-LIKE ---
typedef enum {
    NO_ERROR = 0,
    DEVICE_NOT_RECOGNIZED,
    FIFO_OVERRUN,
} iis3dwb_error_t;

typedef struct
{
    uint8_t IDx;
    spi_device_handle_t spi_handle;
    spi_device_interface_config_t spi_config;
    a_range_t full_scale;
    xl_axis_t xl_axis;
    uint8_t *data_buffer;     // stores acceleration data
    uint8_t *temp_buffer;     // stores temperature data
    uint16_t temp_decimator;  // determines the rate at which temperature data is stored
    uint16_t temp_dec_cnt;    // counts the number of temperature samples
    uint16_t temp_byte_cnt;   // counts the number of temperature bytes
    iis3dwb_error_t error;

} iis3dwb_device_t;

// Public functions
bool iis3dwb_init(iis3dwb_device_t *device, uint8_t watermark);
void iis3dwb_construct_device(iis3dwb_device_t *device, uint8_t index, uint8_t handle, a_range_t accel_scale, uint8_t *data_buffer, uint8_t *temp_buffer, uint16_t temp_decimator);

void iis3dwb_sleep(iis3dwb_device_t *device);
void iis3dwb_wake(iis3dwb_device_t *device);

void iis3dwb_reset_set(iis3dwb_device_t *device);
uint8_t iis3dwb_reset_get(iis3dwb_device_t *device);

uint8_t iis3dwb_who_am_i(iis3dwb_device_t *device);

void iis3dwb_xl_full_scale_set(iis3dwb_device_t *device, a_range_t accel_range);
uint8_t iis3dwb_xl_full_scale_get(iis3dwb_device_t *device);

void iis3dwb_rounding_mode_set(iis3dwb_device_t *device, uint8_t val);
uint8_t iis3dwb_rounding_mode_get(iis3dwb_device_t *device);

uint8_t iis3dwb_axis_sel_get(iis3dwb_device_t *device);
void iis3dwb_axis_sel_set(iis3dwb_device_t *device, uint8_t val);

void iis3dwb_xl_data_rate_set(iis3dwb_device_t *device, iis3dwb_odr_xl_t val);
uint8_t iis3dwb_xl_data_rate_get(iis3dwb_device_t *device);

void iis3dwb_block_data_update_set(iis3dwb_device_t *device, uint8_t val);
uint8_t iis3dwb_block_data_update_get(iis3dwb_device_t *device) __attribute__((warning("Method not implemented. Do not use!")));

void iis3dwb_auto_increment_set(iis3dwb_device_t *device, uint8_t val);
uint8_t iis3dwb_auto_increment_get(iis3dwb_device_t *device);

int32_t iis3dwb_fs_to_ug(a_range_t scale, int16_t val);

void iis3dwb_acceleration_raw(iis3dwb_device_t *device, int16_t *dest);
void iis3dwb_acceleration_raw_fast_usb(iis3dwb_device_t *device, uint8_t *dest);
// iis3dwb_OUT_A iis3dwb_accel_raw(iis3dwb_device_t *device);

int16_t iis3dwb_temp_raw(iis3dwb_device_t *device);
// int16_t iis3dwb_temp_raw(iis3dwb_device_t *device, int16_t);
uint8_t iis3dwb_xl_flag_data_ready(iis3dwb_device_t *device);

// --- FIFO ---
void iis3dwb_fifo_batch_get(iis3dwb_device_t *device, uint16_t num_words);

void iis3dwb_fifo_watermark_set(iis3dwb_device_t *device, uint16_t val);
uint16_t iis3dwb_fifo_watermark_get(iis3dwb_device_t *device);
bool iis3dwb_fifo_wtm_flag_get(iis3dwb_device_t *device);
iis3dwb_fifo_status2_t iis3dwb_fifo_status2_get(iis3dwb_device_t *device);
iis3dwb_fifo_status_t iis3dwb_fifo_status_get(iis3dwb_device_t *device);

void iis3dwb_fifo_xl_batch_set(iis3dwb_device_t *device, uint8_t val);
uint8_t iis3dwb_fifo_xl_batch_get(iis3dwb_device_t *device);

void iis3dwb_fifo_mode_set(iis3dwb_device_t *device, iis3dwb_fifo_mode_t val);
uint8_t iis3dwb_fifo_mode_get(iis3dwb_device_t *device);

void iis3dwb_fifo_temp_batch_set(iis3dwb_device_t *device, iis3dwb_odr_t_batch_t val);
uint8_t iis3dwb_fifo_temp_batch_get(iis3dwb_device_t *device);

int16_t iis3dwb_fifo_data_level_get(iis3dwb_device_t *device);

void iis3dwb_data_ready_mode_set(iis3dwb_device_t *device, iis3dwb_dataready_pulsed_t val);
uint8_t iis3dwb_data_ready_mode_get(iis3dwb_device_t *device);

void iis3dwb_odr_cal_reg_set(iis3dwb_device_t *device, uint8_t val);
uint8_t iis3dwb_odr_cal_reg_get(iis3dwb_device_t *device);

void iis3dwb_xl_filter_lp2_set(iis3dwb_device_t *device, uint8_t val);
uint8_t iis3dwb_xl_filter_lp2_get(iis3dwb_device_t *device) __attribute__((warning("Method not implemented. Do not use!")));

// --- Filters ---

void iis3dwb_xl_hp_path_on_out_set(iis3dwb_device_t *device, iis3dwb_hp_slope_xl_en_t val);
uint8_t iis3dwb_xl_hp_path_on_out_get(iis3dwb_device_t *device);

void iis3dwb_ODR_on_int1(iis3dwb_device_t *device);

#endif  // iis3dwb__
