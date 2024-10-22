/**
 * @file Ariadne.h
 *
 * @brief Singleton management class for the Ariadne device.
 *
 * The Ariadne class is a 'singleton', providing one static instance
 * throughout the application for interfacing with the Ariadne device. This allows
 * crucial device operations such as accessing and altering internal state variables,
 * setting and retrieving properties, to be safely accessed across different scopes
 * in the program. The main goal is to streamline communication within the Ariadne device,
 * maintaining the state, data, and configuration integrity.
 *
 * A few methods are not fully implemented. If adapting the code make sure to double check
 * consistency in previously unused functions.
 *
 * @author Julian Martus
 * @date 31.01.23
 */
#ifndef ARIADNE_MANAGER_H
#define ARIADNE_MANAGER_H

#include <Arduino.h>
#include <SparkFun_MAX1704x_Fuel_Gauge_Arduino_Library.h>
#include "nvs_flash.h"
#include "nvs.h"
#include "hal/usb_serial_jtag_ll.h"
#include "driver/usb_serial_jtag.h"

/// @brief List of the user-modes available in the Ariadne device.
typedef enum
{
    // Ariadne User-Modes
    ARIADNE_USERMODE_AMPLITUDE, ///< Amplitude mode: control amplitude (default mode)
    ARIADNE_USERMODE_EFFECT,    ///< Effect mode: effect chain up to 8 effects
    ARIADNE_USERMODE_DEMO00,    ///< User demo 00
    ARIADNE_USERMODE_DEMO01,    ///< User demo 01
    ARIADNE_USERMODE_DEMO02,    ///< User demo 02
    ARIADNE_USERMODE_DIAGNOS,   ///< DO NOT USE Diagnostics mode. Due to overlaying the i2C bus diagnostics cann not be used
    ARIADNE_USERMODE_AUTOCAL    ///< DO NOT USE Auto calibration mode
} ariadne_stimMode_t;

/// @brief List of the Driver positions available in the Ariadne device.
typedef enum
{
    drv_A,
    drv_B
} DRV_port_t;

/**
 * @class Ariadne
 * @brief Singleton class for the Ariadne device, handling all interactions within the device.
 */
class Ariadne
{
public:
    /// @return Singleton instance of Ariadne.
    static Ariadne &getInstance();

    /// Fuel Gauge for battery level measurement
    SFE_MAX1704X *m_lipo;

    /// @brief Sets the battery gauge
    /// @param lipo Gauge to set
    void setBatteryGauge(SFE_MAX1704X *lipo);

    /// Flag for the activation of 'A' mode
    bool is_A_activated = false;
    /// Flag for the activation of 'B' mode
    bool is_B_activated = false;

    /// Flag to check if the device is currently recording accelerometer data
    bool is_recording_acc = false;

    /// @return Device id
    uint8_t deviceId() const;
    /// @brief Sets the device id
    /// @param value Id to set
    void deviceId(uint8_t value);

    /// @return Firmware version
    std::string getFirmwareVersion() const;
    /// @brief Sets the firmware version
    /// @param newVersion Firmware version to set
    void setFirmwareVersion(std::string newVersion);

    /// @return Battery Level
    uint8_t batteryLevel();
    /// @brief Sets the battery Level
    /// @param batteryLevel Battery level to set
    void batteryLevel(uint8_t batteryLevel);

    /// @return Stimulation mode
    ariadne_stimMode_t stimMode() const;
    /// @brief Sets the Stimulation Mode
    /// @param value Stimulation mode to set
    void stimMode(ariadne_stimMode_t value);

    /// @brief Sets the Amplitude
    /// @param data Amplitude data to set
    void setAmplitude(const uint8_t data);

    /// @brief Sets the duration (in ms) and trigger stimulation.
    /// Only A or B should be set at a time. If both are set it will cause
    /// a stimulation for the duration set by 'durationB';
    /// @param durationA Duration for 'A' mode
    /// @param durationB Duration for 'B' mode
    void setDuration(const uint16_t durationA, const uint16_t durationB);

    /// @brief Gets the duration (in ms) of the stimulation.
    /// @param port target Port for the stimulation. 0->Port_A ; 1->Port_B.
    /// @return Duration in ms
    uint16_t duration(const DRV_port_t port);

    /// @brief Gets the amplitude of the stimulation
    /// @return Amplitude
    uint8_t amp();

    /// @brief Gets the effect for one of the modes
    /// @param pos Position of the effect in the sequence
    /// @return Effect
    uint8_t effect(const uint8_t port = 0);

    /// @brief Set the recording timestamp offset to the current time.
    ///
    /// This function sets the recording timestamp offset (in microseconds). The given offset
    /// will serve as the new reference point, for stimulation timestamps.
    ///
    void setRecordingTimestamp_us();

    /// @brief Sends the last timestamp over USB
    void sendLastTimestampUSB();

    /// Type for the Stimulation Callback function
    typedef std::function<void()> StimulationCallback;

    /// @brief Registers the callback
    /// @param callback The callback function to register
    void registerStimCallback(StimulationCallback callback);

private:
    /// Constructor for the Ariadne class. Private due to singleton pattern.
    Ariadne();

    /// Deleted copy constructor to prevent multiple instances due to singleton pattern.
    Ariadne(const Ariadne &) = delete;

    /// Deleted assignment operator to prevent multiple instances due to singleton pattern.
    Ariadne &operator=(const Ariadne &) = delete;

    uint8_t _deviceId;                  ///< Device ID
    std::string _firmwareVersion = "1"; ///< Firmware version

    StimulationCallback m_callback; ///< Stimulation callback function

    /// @brief Initializes Non-Volatile Storage (NVS)
    void initialiseNVS()
    {
        // Initialize NVS
        esp_err_t err = nvs_flash_init();
        if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND)
        {
            // NVS partition was truncated and needs to be erased
            // Retry nvs_flash_init
            ESP_ERROR_CHECK(nvs_flash_erase());
            err = nvs_flash_init();
        }
        ESP_ERROR_CHECK(err);
    };

    uint8_t m_amplitude = 0xFF;                                                ///< Amplitude of stimulation
    uint16_t m_duration[2] = {0x0000, 0x0000};                                 ///< Stimulation duration.  [0]-> Port_A; [1]->Port_B
    uint8_t m_effects[8] = {0x0010, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; ///< Effects array

    unsigned long m_recording_timestampOff_us; ///< Timestamp reference
    unsigned long m_stim_timestamp_us;         ///< Last timestamp

    uint8_t m_batteryLevel; ///< Battery level
    uint8_t m_subjectID;    ///< Subject ID

    uint8_t m_packet_seq; ///< Sequence number of packets

    /// @brief Current stimulation mode
    ariadne_stimMode_t m_stimMode;

    /// @brief Namespace for NVS
    const char *m_nvs_namespace;
};

#endif // ARIADNE_MANAGER_H
