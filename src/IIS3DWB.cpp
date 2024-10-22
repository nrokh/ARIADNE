#include "iis3dwb.h"

// private variables

void iis3dwb_construct_device(iis3dwb_device_t *device, uint8_t index, spi_device_handle_t spi_handle, a_range_t accel_scale, uint8_t *data_buffer, uint8_t *temp_buffer, uint16_t temp_decimator) {
    device->IDx = index;
    device->spi_handle = spi_handle;  // device->spi_handles_list[index];
    device->full_scale = accel_scale;
    device->data_buffer = data_buffer;  //;data_buffer[index];
    device->temp_buffer = temp_buffer;
    device->temp_decimator = temp_decimator;
    device->temp_dec_cnt = 0;
    device->temp_byte_cnt = 0;
    device->error = NO_ERROR;
}

bool iis3dwb_init(iis3dwb_device_t *device, uint8_t watermark) {
    /* Restore default configuration */
    iis3dwb_reset_set(device);
    while (iis3dwb_reset_get(device));  // wait for reset

    // iis3dwb_ODR_on_int1(device); #TODO

    /* Set XL Batch Data Rate */
    iis3dwb_fifo_xl_batch_set(device, IIS3DWB_XL_BATCHED_AT_26k7Hz);

    /*  Set Temperature Batch Data Rate */
    iis3dwb_fifo_temp_batch_set(device, IIS3DWB_TEMP_BATCHED_AT_104Hz);

    /* Set  FIFO Bypass Mode */
    iis3dwb_fifo_mode_set(device, IIS3DWB_BYPASS_MODE);

    /* Set  FIFO Watermark */
    iis3dwb_fifo_watermark_set(device, watermark + 1);  // Total Bytes =  Number of words * FIFO_WORD (7 or 6) + 1 to account for occasional temperature datum

    /* Set default acceleration scale */
    iis3dwb_xl_full_scale_set(device, ACCEL_2G);

    /*	Configure filtering chain(No aux interface)
     *	Accelerometer - LPF1 + LPF2 path
     */
    iis3dwb_xl_hp_path_on_out_set(device, IIS3DWB_LP_6k3Hz);
    // iis3dwb_xl_filter_lp2_set(device, PROPERTY_ENABLE);
    iis3dwb_wake(device);

    /* Wait stable output */
    delay(IIS3DWB_BOOT_TIME);

    return true;
}
/**
 * @brief  Accelerometer power down
 *
 * @param  device->spi_handle	spi device->spi_handle of device
 *
 */
void iis3dwb_sleep(iis3dwb_device_t *device) {
    iis3dwb_ctrl1_xl_t ctrl1_xl;
    m_spi_read_registers(device->spi_handle, IIS3DWB_CTRL1_XL, 1, &ctrl1_xl.data);  // read register
    ctrl1_xl.ctrl1_xl.xl_en = IIS3DWB_XL_ODR_OFF;                                   // power down
    m_spi_write_register(device->spi_handle, IIS3DWB_CTRL1_XL, ctrl1_xl.data);      // send modified register
    delay(1);
}
/**
 * @brief  Accelerometer wake up
 *
 * @param  device->spi_handle	spi device->spi_handle of device
 *
 */
void iis3dwb_wake(iis3dwb_device_t *device) {
    iis3dwb_ctrl1_xl_t ctrl1_xl;
    ;
    m_spi_read_registers(device->spi_handle, IIS3DWB_CTRL1_XL, 1, &ctrl1_xl.data);  // read register
    ctrl1_xl.ctrl1_xl.xl_en = IIS3DWB_XL_ODR_26k7Hz;                                // accelerometer enabled
    m_spi_write_register(device->spi_handle, IIS3DWB_CTRL1_XL, ctrl1_xl.data);      // send modified register
}

/**
 * @brief	Device Who am I.[get]
 *
 * @param	device->spi_handle	spi device->spi_handle of device
 * @return			device ID (0x7B)
 */

uint8_t iis3dwb_who_am_i(iis3dwb_device_t *device) {
    return m_spi_read_register(device->spi_handle, IIS3DWB_WHO_AM_I);
}

/**
 * @brief	Convert raw-data into engineering units.
 *
 * @param	scale	accelerometer scale of device
 * @param	val		raw-data to convert
 * @return			converted raw-data in ug
 */

int32_t iis3dwb_fs_to_ug(a_range_t scale, int16_t val) {
    switch (scale) {
        case ACCEL_2G:
            return ((int32_t)val * 61);
        case ACCEL_4G:
            return ((int32_t)val * 122);
        case ACCEL_8G:
            return ((int32_t)val * 244);
        case ACCEL_16G:
            return ((int32_t)val * 488);
        default:
            return 0;
    }
}

/**
 * @brief	Device acceleration range.[set]
 *
 * @param	device->spi_handle		spi device->spi_handle of device
 * @param	accel_range, possible configurations: 0x00=2g; 0x01=16g; 0x02=4g; 0x03=8g;
 */
void iis3dwb_xl_full_scale_set(iis3dwb_device_t *device, a_range_t accel_range) {
    iis3dwb_ctrl1_xl_t ctrl1_xl;

    ctrl1_xl.data = m_spi_read_register(device->spi_handle, IIS3DWB_CTRL1_XL);  // read register
    ctrl1_xl.ctrl1_xl.fs_xl = (uint8_t)accel_range;                             // set accel-range
    m_spi_write_register(device->spi_handle, IIS3DWB_CTRL1_XL, ctrl1_xl.data);  // send modified register
}
/**
 * @brief	Device acceleration range.[get]
 *
 * @param	device->spi_handle	spi device->spi_handle of device
 * @return	current acceleration. 0x00=2g; 0x01=16g; 0x02=4g; 0x03=8g;
 */
uint8_t iis3dwb_xl_full_scale_get(iis3dwb_device_t *device) {
    iis3dwb_ctrl1_xl_t ctrl1_xl;

    m_spi_read_registers(device->spi_handle, IIS3DWB_CTRL1_XL, 1, &ctrl1_xl.data);
    return (uint8_t)ctrl1_xl.ctrl1_xl.fs_xl;  // get FS[1:0]_XL
}

/**
 * @brief  Circular burst-mode (rounding) read of the output registers.[set]
 *
 * @param	device->spi_handle	spi device->spi_handle of device
 * @param  val    Change the values of rounding in reg CTRL5_C
 *
 */
void iis3dwb_rounding_mode_set(iis3dwb_device_t *device, uint8_t val) {
    iis3dwb_ctrl5_c_t ctrl5_c;

    m_spi_read_registers(device->spi_handle, IIS3DWB_CTRL5_C, 1, &ctrl5_c.data);  // read register
    ctrl5_c.ctrl5_c.rounding = val;
    m_spi_write_register(device->spi_handle, IIS3DWB_CTRL5_C, ctrl5_c.data);
}
/**
 * @brief  Circular burst-mode (rounding) read of the output registers.[get]
 *
 * @param	device->spi_handle spi device->spi_handle of device
 * @retval        Values of rounding in reg CTRL5_C
 *
 */
uint8_t iis3dwb_rounding_mode_get(iis3dwb_device_t *device) {
    iis3dwb_ctrl5_c_t ctrl5_c;

    m_spi_read_registers(device->spi_handle, IIS3DWB_CTRL5_C, 1, &ctrl5_c.data);
    return ctrl5_c.ctrl5_c.rounding;
}

/**
 * @brief  Circular burst-mode (rounding) read of the output registers.[get]
 *
 * @param	device->spi_handle spi device->spi_handle of device
 * @retval        Values of rounding in reg CTRL5_C
 *
 */
uint8_t iis3dwb_axis_sel_get(iis3dwb_device_t *device) {
    iis3dwb_ctrl6_c_t ctrl6_c;

    m_spi_read_registers(device->spi_handle, IIS3DWB_CTRL6_C, 1, &ctrl6_c.data);
    return ctrl6_c.ctrl6_c.xl_axis_sel;
}

/**
 * @brief  Circular burst-mode (rounding) read of the output registers.[set]
 *
 * @param	device->spi_handle	spi device->spi_handle of device
 * @param  val    Change the values of rounding in reg CTRL5_C
 *
 */
void iis3dwb_axis_sel_set(iis3dwb_device_t *device, uint8_t val) {
    iis3dwb_ctrl6_c_t ctrl6_c;

    m_spi_read_registers(device->spi_handle, IIS3DWB_CTRL6_C, 1, &ctrl6_c.data);  // read register
    ctrl6_c.ctrl6_c.xl_axis_sel = val;
    m_spi_write_register(device->spi_handle, IIS3DWB_CTRL6_C, ctrl6_c.data);
}

/**
 * @brief  Accelerometer UI data rate selection.[set]
 *
 * @param  device->spi_handle	spi device->spi_handle of device
 * @param  val		Change the values of xl_en in reg CTRL1_XL
 *
 */
void iis3dwb_xl_data_rate_set(iis3dwb_device_t *device, iis3dwb_odr_xl_t val) {
    iis3dwb_ctrl1_xl_t ctrl1_xl;

    m_spi_read_registers(device->spi_handle, IIS3DWB_CTRL1_XL, 1, &ctrl1_xl.data);  // read register
    ctrl1_xl.ctrl1_xl.xl_en = (uint8_t)val;
    m_spi_write_register(device->spi_handle, IIS3DWB_CTRL3_C, ctrl1_xl.data);
}
/**
 * @brief  Accelerometer UI data rate selection.[get]
 *
 * @param  ctx    Read / write interface definitions.(ptr)
 * @param  val    Get the values of odr_xl in reg CTRL1_XL
 * @retval        Interface status (MANDATORY: return 0 -> no Error).
 *
 */
uint8_t iis3dwb_xl_data_rate_get(iis3dwb_device_t *device) {
    iis3dwb_ctrl1_xl_t ctrl1_xl;
    m_spi_read_registers(device->spi_handle, IIS3DWB_CTRL1_XL, 1, &ctrl1_xl.data);  // read register
    return ctrl1_xl.ctrl1_xl.fs_xl;
}

/**
 * @brief  Block data update.[set]
 *
 * @param  device->spi_handle	spi device->spi_handle of device
 * @param  val		Change the values of bdu in reg CTRL3_C
 *
 */
void iis3dwb_block_data_update_set(iis3dwb_device_t *device, uint8_t val) {
    iis3dwb_ctrl3_c_t ctrl3_c;

    m_spi_read_registers(device->spi_handle, IIS3DWB_CTRL3_C, 1, &ctrl3_c.data);  // read register
    ctrl3_c.ctrl3_c.bdu = (uint8_t)val;
    m_spi_write_register(device->spi_handle, IIS3DWB_CTRL3_C, ctrl3_c.data);
}

/** TODO: iis3dwb_block_data_update_get
 * @brief  (NOT IMPLEMENTED) Block data update.[get]
 *
 * @param  device->spi_handle	spi device->spi_handle of device
 * @param  val    Change the values of bdu in reg CTRL3_C
 *
 *
 */
uint8_t iis3dwb_block_data_update_get(iis3dwb_device_t *device) {
    /*
  iis3dwb_ctrl3_c_t ctrl3_c;
  int32_t ret;

  ret = iis3dwb_read_reg(ctx, IIS3DWB_CTRL3_C, (uint8_t*)&ctrl3_c, 1);
  *val = ctrl3_c.bdu;

  return ret;
  */
    return 0;
}

/**
 * @brief  Register address automatically incremented during a multiple byte
 *         access with a serial interface.[set]
 *
 * @param  device->spi_handle	spi device->spi_handle of device
 * @param  val		Change the values of if_inc in reg CTRL3_C
 *
 */
void iis3dwb_auto_increment_set(iis3dwb_device_t *device, uint8_t val) {
    iis3dwb_ctrl3_c_t ctrl3_c;

    m_spi_read_registers(device->spi_handle, IIS3DWB_CTRL3_C, 1, &ctrl3_c.data);  // read register
    ctrl3_c.ctrl3_c.if_inc = (uint8_t)val;
    m_spi_write_register(device->spi_handle, IIS3DWB_CTRL3_C, ctrl3_c.data);
}

/**
 * @brief  Register address automatically incremented during a multiple byte
 *         access with a serial interface.[get]
 *
 * @param  device->spi_handle	spi device->spi_handle of device
 * @return			status of if_inc in reg CTRL3_C
 *
 */
uint8_t iis3dwb_auto_increment_get(iis3dwb_device_t *device) {
    iis3dwb_ctrl3_c_t ctrl3_c;

    m_spi_read_registers(device->spi_handle, IIS3DWB_CTRL3_C, 1, &ctrl3_c.data);  // read register
    return ctrl3_c.ctrl3_c.if_inc;
}

void iis3dwb_acceleration_raw(iis3dwb_device_t *device, int16_t *dest) {
    uint8_t buff[6];  // x/y/z accel register data stored here

    m_spi_read_registers(device->spi_handle, IIS3DWB_OUTX_L_A, 6, buff);
    // #TODO #DEBUG
    // dest[0] = (int16_t)0x00; // Turn the MSB and LSB into a signed 16-bit value
    // dest[1] = (int16_t)0x00;
    // dest[2] = (int16_t)0x00;

    dest[0] = (int16_t)((int16_t)buff[1] << 8) | buff[0];  // Turn the MSB and LSB into a signed 16-bit value
    dest[1] = (int16_t)((int16_t)buff[3] << 8) | buff[2];
    dest[2] = (int16_t)((int16_t)buff[5] << 8) | buff[4];

    // dest[0] = (int16_t)buff[0];
    // dest[0] = (dest[0] * 256) +  (int16_t)buff[0];
    // dest[1] = (int16_t)buff[3];
    // dest[1] = (dest[1] * 256) +  (int16_t)buff[2];
    // dest[2] = (int16_t)buff[5];
    // dest[2] = (dest[2] * 256) +  (int16_t)buff[4];
}

/*
void iis3dwb_temp_raw(iis3dwb_device_t *device, int16_t *dest)
{
     uint8_t buff[2]; // temp register data stored here
     m_spi_read_registers(device->spi_handle, IIS3DWB_OUT_TEMP_L, 2, buff); // Read the 2 raw temperature data registers into data array
     dest[0] = (int16_t)((int16_t)buff[1] << 8)  | buff[0] ;  // Turn the MSB and LSB into a signed 16-bit value
}
*/
int16_t iis3dwb_temp_raw(iis3dwb_device_t *device) {
    uint8_t buff[2];  // temp register data stored here

    m_spi_read_registers(device->spi_handle, IIS3DWB_OUT_TEMP_L, 2, buff);  // Read the 2 raw temperature data registers into data array
    return (int16_t)((int16_t)buff[1] << 8) | buff[0];                      // Turn the MSB and LSB into a signed 16-bit value
}

// --------- FIFO -----
/**
 * @brief  FIFO data output.[get]
 *
 * @param  device->spi_handle	spi device->spi_handle of device
 * @param  dest	Buffer that stores data read
 *
 */
void iis3dwb_fifo_out_raw_get(iis3dwb_device_t *device, uint8_t *dest) {
    m_spi_read_registers(device->spi_handle, IIS3DWB_FIFO_DATA_OUT_X_L, 6, dest);
}

/**
 * @brief  Identifies the sensor in FIFO_DATA_OUT.[get]
 *
 * @param  device->spi_handle	spi device->spi_handle of device
 * @retval			Change the values of tag_sensor in reg FIFO_DATA_OUT_TAG
 *
 */
uint8_t iis3dwb_fifo_sensor_tag_get(iis3dwb_device_t *device) {
    iis3dwb_fifo_data_out_tag_t fifo_data_out_tag;

    m_spi_read_registers(device->spi_handle, IIS3DWB_FIFO_DATA_OUT_TAG, 1, &fifo_data_out_tag.data);  // read register

    return (uint8_t)fifo_data_out_tag.tag_sensor;
}

/**
 * @brief  FIFO data output batch read. [get]
 *
 * @param  device->spi_handle	spi device->spi_handle of device
 * @param  num_words	number of 7-byte words read from FIFO
 * @param  dest	Buffer that stores data read
 *
 */
void iis3dwb_fifo_batch_get(iis3dwb_device_t *device, uint16_t num_words) {
    m_spi_read_registers(device->spi_handle, IIS3DWB_FIFO_DATA_OUT_TAG, num_words * 7, device->data_buffer);  // read register
}

/**
 * @brief  FIFO data output batch read. [get]
 * Data will be sorted according to type. Due to the frequency difference a fifo-batch can only contain 1 temperature datum.
 *	In this implementation reading temperature data will not count towards the number of bytes read
 *
 *
 * @param  device->spi_handle	spi device->spi_handle of device
 * @param  num_words		number of 6-byte acceleration words read from FIFO,
 * @param  acceleration	Buffer that stores acceleration data
 * @param  temperature 	Buffer that stores temperature data
 *
 * @retval				returns TRUE if temperature data was within batch
 */

/**
 * @brief  } watermark level selection.[set]
 *
 * @param  device->spi_handle	spi device->spi_handle of device
 * @param  val    Change the values of wtm in reg FIFO_CTRL1 (MAX 9bit = 0d511)
 *
 */
void iis3dwb_fifo_watermark_set(iis3dwb_device_t *device, uint16_t val) {
    iis3dwb_fifo_ctrl1_t fifo_ctrl1;
    iis3dwb_fifo_ctrl2_t fifo_ctrl2;

    m_spi_read_registers(device->spi_handle, IIS3DWB_FIFO_CTRL1, 1, &fifo_ctrl1.data);  // read register
    fifo_ctrl1.fifo_ctrl1.wtm = (uint8_t)(0x00FFU & val);
    m_spi_write_register(device->spi_handle, IIS3DWB_FIFO_CTRL1, fifo_ctrl1.data);

    m_spi_read_registers(device->spi_handle, IIS3DWB_FIFO_CTRL2, 1, &fifo_ctrl2.data);  // read register
    fifo_ctrl2.fifo_ctrl2.wtm = (uint8_t)((0x0100U & val) >> 8);
    m_spi_write_register(device->spi_handle, IIS3DWB_FIFO_CTRL2, fifo_ctrl2.data);
}

/**
 * @brief  FIFO watermark level selection.[get]
 *
 * @param  device->spi_handle	spi device->spi_handle of device
 * @retval			Change the values of wtm in reg FIFO_CTRL1
 *
 */
uint16_t iis3dwb_fifo_watermark_get(iis3dwb_device_t *device) {
    iis3dwb_fifo_ctrl1_t fifo_ctrl1;
    iis3dwb_fifo_ctrl2_t fifo_ctrl2;
    uint8_t ret;

    m_spi_read_registers(device->spi_handle, IIS3DWB_FIFO_CTRL1, 1, &fifo_ctrl1.data);  // read register
    m_spi_read_registers(device->spi_handle, IIS3DWB_FIFO_CTRL2, 1, &fifo_ctrl2.data);  // read register

    ret = fifo_ctrl2.fifo_ctrl2.wtm;
    ret = ret << 8;
    ret += fifo_ctrl1.fifo_ctrl1.wtm;
    return ret;
}

/**
 * @brief  FIFO watermark status.[get]
 *
 * @param  device->spi_handle	spi device->spi_handle of device
 * @retval			Change the values of fifo_wtm_ia in reg FIFO_STATUS2
 *
 */
bool iis3dwb_fifo_wtm_flag_get(iis3dwb_device_t *device) {
    iis3dwb_fifo_status2_t fifo_status2;

    m_spi_read_registers(device->spi_handle, IIS3DWB_FIFO_STATUS2, 1, &fifo_status2.data);  // read register
    return fifo_status2.status2.fifo_wtm_ia;
}

/**
 * @brief  Smart FIFO status.[get]
 *
 * @param  device->spi_handle	spi device->spi_handle of device
 * @retval			Registers FIFO_STATUS2
 *
 */
iis3dwb_fifo_status2_t iis3dwb_fifo_status2_get(iis3dwb_device_t *device) {
    iis3dwb_fifo_status2_t fifo_status2;

    m_spi_read_registers(device->spi_handle, IIS3DWB_FIFO_STATUS2, 1, &fifo_status2.data);  // read register
    return fifo_status2;
}
/**
 * @brief  Extendet FIFO status.[get]
 *
 * @param  device->spi_handle	spi device->spi_handle of device
 * @retval			Registers FIFO_STATUS2 and FIFO_STATUS1
 *
 */
iis3dwb_fifo_status_t iis3dwb_fifo_status_get(iis3dwb_device_t *device) {
    iis3dwb_fifo_status_t fifo_status;
    fifo_status.data = m_spi_read_16bit(device->spi_handle, IIS3DWB_FIFO_STATUS1);
    return fifo_status;
}

/**
 * @brief  Selects Batching Data Rate (writing frequency in FIFO)
 *         for accelerometer data.[set]
 *
 * @param  device->spi_handle	spi device->spi_handle of device
 * @param  val		Change the values of bdr_xl in reg FIFO_CTRL3
 *
 */
void iis3dwb_fifo_xl_batch_set(iis3dwb_device_t *device, uint8_t val) {
    iis3dwb_fifo_ctrl3_t fifo_ctrl3;

    m_spi_read_registers(device->spi_handle, IIS3DWB_FIFO_CTRL3, 1, &fifo_ctrl3.data);  // read register
    fifo_ctrl3.fifo_ctrl3.bdr_xl = (uint8_t)val;
    m_spi_write_register(device->spi_handle, IIS3DWB_FIFO_CTRL3, fifo_ctrl3.data);
}

/**
 * @brief  Selects Batching Data Rate (writing frequency in FIFO)
 *         for accelerometer data.[get]
 *
 * @param  device->spi_handle	spi device->spi_handle of device
 * @retval			Get the values of bdr_xl in reg FIFO_CTRL3
 *
 */
uint8_t iis3dwb_fifo_xl_batch_get(iis3dwb_device_t *device) {
    iis3dwb_fifo_ctrl3_t fifo_ctrl3;

    m_spi_read_registers(device->spi_handle, IIS3DWB_FIFO_CTRL3, 1, &fifo_ctrl3.data);  // read register
    return fifo_ctrl3.fifo_ctrl3.bdr_xl;
}

/**
 * @brief  FIFO mode selection.[set]
 *
 * @param  device->spi_handle	spi device->spi_handle of device
 * @param  val    Change the values of fifo_mode in reg FIFO_CTRL4
 *
 */
void iis3dwb_fifo_mode_set(iis3dwb_device_t *device, iis3dwb_fifo_mode_t val) {
    iis3dwb_fifo_ctrl4_t fifo_ctrl4;

    m_spi_read_registers(device->spi_handle, IIS3DWB_FIFO_CTRL4, 1, &fifo_ctrl4.data);  // read register
    fifo_ctrl4.fifo_ctrl4.fifo_mode = (uint8_t)val;
    m_spi_write_register(device->spi_handle, IIS3DWB_FIFO_CTRL4, fifo_ctrl4.data);
}

/**
 * @brief  FIFO mode selection.[get]
 *
 * @param  device->spi_handle	spi device->spi_handle of device
 * @retval			Get the values of fifo_mode in reg FIFO_CTRL4
 *
 */
uint8_t iis3dwb_fifo_mode_get(iis3dwb_device_t *device) {
    iis3dwb_fifo_ctrl4_t fifo_ctrl4;

    m_spi_read_registers(device->spi_handle, IIS3DWB_FIFO_CTRL4, 1, &fifo_ctrl4.data);  // read register
    return fifo_ctrl4.fifo_ctrl4.fifo_mode;
}

/**
 * @brief  Temperature Batching Data Rate (writing frequency in FIFO) [set]
 *
 * @param  device->spi_handle	spi device->spi_handle of device
 * @param  val    Change the values of fifo_mode in reg FIFO_CTRL4
 *
 */
void iis3dwb_fifo_temp_batch_set(iis3dwb_device_t *device, iis3dwb_odr_t_batch_t val) {
    iis3dwb_fifo_ctrl4_t fifo_ctrl4;

    m_spi_read_registers(device->spi_handle, IIS3DWB_FIFO_CTRL4, 1, &fifo_ctrl4.data);  // read register
    fifo_ctrl4.fifo_ctrl4.odr_t_batch = (uint8_t)val;
    m_spi_write_register(device->spi_handle, IIS3DWB_FIFO_CTRL4, fifo_ctrl4.data);
}

/**
 * @brief  Temperature Batching Data Rate (writing frequency in FIFO) [get]
 *
 * @param  device->spi_handle	spi device->spi_handle of device
 * @retval			Get the values of fifo_mode in reg FIFO_CTRL4
 *
 */
uint8_t iis3dwb_fifo_temp_batch_get(iis3dwb_device_t *device) {
    iis3dwb_fifo_ctrl4_t fifo_ctrl4;

    m_spi_read_registers(device->spi_handle, IIS3DWB_FIFO_CTRL4, 1, &fifo_ctrl4.data);  // read register
    return fifo_ctrl4.fifo_ctrl4.odr_t_batch;
}

/**
 * @brief  Number of unread sensor data (TAG + 6 bytes) stored in FIFO.[get]
 *
 * @param  ctx    Read / write interface definitions.(ptr)
 * @param  val    Get the values of diff_fifo in reg FIFO_STATUS1
 * @retval        Interface status.
 *
 */
int16_t iis3dwb_fifo_data_level_get(iis3dwb_device_t *device) {
    iis3dwb_fifo_status1_t fifo_status1;
    iis3dwb_fifo_status2_t fifo_status2;
    fifo_status1.data = m_spi_read_register(device->spi_handle, IIS3DWB_FIFO_STATUS1);  // read register
    fifo_status2.data = m_spi_read_register(device->spi_handle, IIS3DWB_FIFO_STATUS2);  // read register

    return ((uint16_t)fifo_status2.status2.diff_fifo << 8) | fifo_status1.fifo_status1.diff_fifo;
}

/**
 * @defgroup   IIS3DWB_filters
 * @brief      This section group all the functions concerning the
 *             filters configuration
 * @{
 *
 */

/**
 * @brief  Accelerometer output from LPF2 filtering stage selection.[set]
 *
 * @param  device->spi_handle	spi device->spi_handle of device
 * @param  val		Change the values of lpf2_xl_en in reg CTRL1_XL
 *
 */
void iis3dwb_xl_filter_lp2_set(iis3dwb_device_t *device, uint8_t val) {
    iis3dwb_ctrl1_xl_t ctrl1_xl;

    m_spi_read_registers(device->spi_handle, IIS3DWB_CTRL1_XL, 1, &ctrl1_xl.data);
    ctrl1_xl.ctrl1_xl.lpf2_xl_en = (uint8_t)val;
    m_spi_write_register(device->spi_handle, IIS3DWB_CTRL1_XL, ctrl1_xl.data);
}

/** #TODO:
 * @brief  (NOT YET IMPLEMENTED) Accelerometer output from LPF2 filtering stage selection.[get]
 *
 * @param  ctx    Read / write interface definitions.(ptr)
 * @param  val    Change the values of lpf2_xl_en in reg CTRL1_XL
 * @return        Interface status (MANDATORY: return 0 -> no Error).
 *
 */
uint8_t iis3dwb_xl_filter_lp2_get(iis3dwb_device_t *device) {
    /*
  iis3dwb_ctrl1_xl_t ctrl1_xl;
  int32_t ret;

  ret = iis3dwb_read_reg(ctx, IIS3DWB_CTRL1_XL, (uint8_t*)&ctrl1_xl, 1);
  *val = ctrl1_xl.lpf2_xl_en;
    */
    return 0;
}

/**
 * @brief  Accelerometer slope filter / high-pass filter selection
 *         on output.[set]
 *
 * @param  device->spi_handle	spi device->spi_handle of device
 * @param  val		Change the values of hp_slope_xl_en in reg CTRL8_XL
 *
 */
void iis3dwb_xl_hp_path_on_out_set(iis3dwb_device_t *device, iis3dwb_hp_slope_xl_en_t val) {
    iis3dwb_ctrl1_xl_t ctrl1_xl;
    iis3dwb_ctrl8_xl_t ctrl8_xl;

    m_spi_read_registers(device->spi_handle, IIS3DWB_CTRL1_XL, 1, &ctrl1_xl.data);
    ctrl1_xl.ctrl1_xl.lpf2_xl_en = ((uint8_t)val & 0x80U) >> 7;
    m_spi_write_register(device->spi_handle, IIS3DWB_CTRL1_XL, ctrl1_xl.data);

    m_spi_read_registers(device->spi_handle, IIS3DWB_CTRL8_XL, 1, &ctrl8_xl.data);

    ctrl8_xl.ctrl8_xl.fds = ((uint8_t)val & 0x10U) >> 4;
    ctrl8_xl.ctrl8_xl.hp_ref_mode_xl = ((uint8_t)val & 0x20U) >> 5;
    ctrl8_xl.ctrl8_xl.hpcf_xl = (uint8_t)val & 0x07U;
    m_spi_write_register(device->spi_handle, IIS3DWB_CTRL8_XL, ctrl8_xl.data);
}

/**
 * @brief  Accelerometer slope filter / high-pass filter selection on
 *         output.[get]
 *
 * @param		device->spi_handle	spi device->spi_handle of device
 * @retval     Get the accelerometer bandwidth selection
 *
 */
uint8_t iis3dwb_xl_hp_path_on_out_get(iis3dwb_device_t *device) {
    iis3dwb_ctrl1_xl_t ctrl1_xl;
    iis3dwb_ctrl8_xl_t ctrl8_xl;
    uint8_t ret;
    m_spi_read_registers(device->spi_handle, IIS3DWB_CTRL1_XL, 1, &ctrl1_xl.data);
    m_spi_read_registers(device->spi_handle, IIS3DWB_CTRL8_XL, 1, &ctrl8_xl.data);

    ret = (ctrl1_xl.ctrl1_xl.lpf2_xl_en << 7) + (ctrl8_xl.ctrl8_xl.hp_ref_mode_xl << 5) +
          (ctrl8_xl.ctrl8_xl.fds << 4) + ctrl8_xl.ctrl8_xl.hpcf_xl;

    return ret;
    /*
      switch ( (ctrl1_xl.lpf2_xl_en << 7) + (ctrl8_xl.hp_ref_mode_xl << 5) +
               (ctrl8_xl.fds << 4) + ctrl8_xl.hpcf_xl ){
        case IIS3DWB_SLOPE_ODR_DIV_4:
          ret = IIS3DWB_SLOPE_ODR_DIV_4;
          break;
        case IIS3DWB_HP_ODR_DIV_10:
          ret = IIS3DWB_HP_ODR_DIV_10;
          break;
        case IIS3DWB_HP_ODR_DIV_20:
          ret = IIS3DWB_HP_ODR_DIV_20;
          break;
        case IIS3DWB_HP_ODR_DIV_45:
          ret = IIS3DWB_HP_ODR_DIV_45;
          break;
        case IIS3DWB_HP_ODR_DIV_100:
          ret = IIS3DWB_HP_ODR_DIV_100;
          break;
        case IIS3DWB_HP_ODR_DIV_200:
          ret = IIS3DWB_HP_ODR_DIV_200;
          break;
        case IIS3DWB_HP_ODR_DIV_400:
          ret = IIS3DWB_HP_ODR_DIV_400;
          break;
        case IIS3DWB_HP_ODR_DIV_800:
          ret = IIS3DWB_HP_ODR_DIV_800;
          break;
        case IIS3DWB_LP_ODR_DIV_4:
          ret = IIS3DWB_LP_ODR_DIV_4;
          break;
        case IIS3DWB_LP_6k3Hz:
          ret = IIS3DWB_LP_6k3Hz;
          break;
        case IIS3DWB_LP_ODR_DIV_10:
          ret = IIS3DWB_LP_ODR_DIV_10;
          break;
        case IIS3DWB_LP_ODR_DIV_20:
          ret = IIS3DWB_LP_ODR_DIV_20;
          break;
        case IIS3DWB_LP_ODR_DIV_45:
          ret = IIS3DWB_LP_ODR_DIV_45;
          break;
        case IIS3DWB_LP_ODR_DIV_100:
          ret = IIS3DWB_LP_ODR_DIV_100;
          break;
        case IIS3DWB_LP_ODR_DIV_200:
          ret = IIS3DWB_LP_ODR_DIV_200;
          break;
        case IIS3DWB_LP_ODR_DIV_400:
          ret = IIS3DWB_LP_ODR_DIV_400;
          break;
        case IIS3DWB_LP_ODR_DIV_800:
          ret = IIS3DWB_LP_ODR_DIV_800;
          break;
        default:
          ret = IIS3DWB_SLOPE_ODR_DIV_4;
          break;
      }

      return ret;
      */
}

/**
 * @brief  Accelerometer new data available.[get]
 *
 * @param	device->spi_handle	spi device->spi_handle of device
 * @return        Status Reg xlda;
 *
 */
uint8_t iis3dwb_xl_flag_data_ready(iis3dwb_device_t *device) {
    iis3dwb_status_reg_t status_reg;
    m_spi_read_registers(device->spi_handle, IIS3DWB_STATUS_REG, 1, &status_reg.data);
    return (uint8_t)status_reg.status_reg.xlda;  // get FS[1:0]_XL
}
/**
 * @brief	Software reset. Restore the default values in user registers.[set]
 *
 * @param	device->spi_handle	spi device->spi_handle of device
 */

/**
 * @brief  Data-ready pulsed / letched mode.[set]
 *
 * @param	device->spi_handle	spi device->spi_handle of device
 * @param  val    Change the values of dataready_pulsed in
 *                reg COUNTER_BDR_REG1
 *
 */

void iis3dwb_data_ready_mode_set(iis3dwb_device_t *device, iis3dwb_dataready_pulsed_t val) {
    iis3dwb_counter_bdr_reg1_t counter_bdr_reg1;
    m_spi_read_registers(device->spi_handle, IIS3DWB_COUNTER_BDR_REG1, 1, &counter_bdr_reg1.data);  // read register
    counter_bdr_reg1.counter_bdr_reg1.dataready_pulsed = PROPERTY_ENABLE;
    m_spi_write_register(device->spi_handle, IIS3DWB_COUNTER_BDR_REG1, counter_bdr_reg1.data);  // send modified register
    delay(10);
}

/**
 * @brief (#TODO Not Yet Implemented) Data-ready pulsed / letched mode.[get]
 *
 * @param  ctx    Read / write interface definitions.(ptr)
 * @param  val    Get the values of dataready_pulsed in
 *                reg COUNTER_BDR_REG1
 * @retval        Interface status (MANDATORY: return 0 -> no Error).
 *
 */

/**
 * @brief  Difference in percentage of the effective ODR (and timestamp rate)
 *         with respect to the typical.[set]
 *         Step:  0.15%. 8-bit format, 2's complement.
 *
 * @param	device->spi_handle	spi device->spi_handle of device
 * @param  val		Change the values of freq_fine in reg INTERNAL_FREQ_FINE
 *
 */
void iis3dwb_odr_cal_reg_set(iis3dwb_device_t *device, uint8_t val) {
    iis3dwb_internal_freq_fine_t internal_freq_fine;

    m_spi_read_registers(device->spi_handle, IIS3DWB_INTERNAL_FREQ_FINE, 1, &internal_freq_fine.data);  // read register
    internal_freq_fine.freq_fine = (uint8_t)val;
    m_spi_write_register(device->spi_handle, IIS3DWB_INTERNAL_FREQ_FINE, internal_freq_fine.data);
}

/**
 * @brief  Difference in percentage of the effective ODR (and timestamp rate)
 *         with respect to the typical.[get]
 *         Step:  0.15%. 8-bit format, 2's complement.
 *
 * @param	device->spi_handle	spi device->spi_handle of device
 * @retval			Values of freq_fine in reg INTERNAL_FREQ_FINE
 *
 */
uint8_t iis3dwb_odr_cal_reg_get(iis3dwb_device_t *device) {
    iis3dwb_internal_freq_fine_t internal_freq_fine;

    m_spi_read_registers(device->spi_handle, IIS3DWB_INTERNAL_FREQ_FINE, 1, &internal_freq_fine.data);

    return internal_freq_fine.freq_fine;
}

/**
 * @brief			Generates interrupt on INT1 to measure ODR
 *
 * @param	device->spi_handle	spi device->spi_handle of device
 *
 */
void iis3dwb_ODR_on_int1(iis3dwb_device_t *device) {
    iis3dwb_data_ready_mode_set(device, IIS3DWB_DRDY_PULSED);
    iis3dwb_int1_ctrl_t int1_ctrl;

    m_spi_read_registers(device->spi_handle, IIS3DWB_INT1_CTRL, 1, &int1_ctrl.data);  // read register
    int1_ctrl.int1_ctrl.int1_drdy_xl = PROPERTY_ENABLE;
    m_spi_write_register(device->spi_handle, IIS3DWB_INT1_CTRL, int1_ctrl.data);  // send modified register
    delay(10);
}

void iis3dwb_reset_set(iis3dwb_device_t *device) {
    // power down
    // iis3dwb_sleep(device->spi_handle);
    // m_spi_write_register(device->spi_handle, IIS3DWB_CTRL1_XL, 0x00u); // since the register is reset we can just overwrite everything

    // set reset pin
    iis3dwb_ctrl3_c_t ctrl3_c;
    m_spi_read_registers(device->spi_handle, IIS3DWB_CTRL3_C, 1, &ctrl3_c.data);  // read register
    ctrl3_c.ctrl3_c.sw_reset = 0x1u;                                              // software reset
    m_spi_write_register(device->spi_handle, IIS3DWB_CTRL3_C, ctrl3_c.data);      // send modified register
    delay(10);
}
/**
 * @brief	Software reset. Restore the default values in user registers.[get]
 *
 * @param	device->spi_handle	spi device->spi_handle of device
 * @return	sw_reset status
 */

uint8_t iis3dwb_reset_get(iis3dwb_device_t *device) {
    iis3dwb_ctrl3_c_t ctrl3_c;
    m_spi_read_registers(device->spi_handle, IIS3DWB_CTRL3_C, 1, &ctrl3_c.data);  // read register
    return (uint8_t)ctrl3_c.ctrl3_c.sw_reset;                                     // software reset
}
