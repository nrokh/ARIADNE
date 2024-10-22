#include "Ariadne.h"

Ariadne::Ariadne() {
    // Initialize NVS
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        // NVS partition was truncated and needs to be erased
        // Retry nvs_flash_init
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
}

Ariadne &Ariadne::getInstance() {
    static Ariadne instance;
    return instance;
}

uint8_t Ariadne::deviceId() const {
    return _deviceId;
}

void Ariadne::deviceId(uint8_t value) {
    _deviceId = value;
}

std::string Ariadne::getFirmwareVersion() const {
    return _firmwareVersion;
}
void Ariadne::setFirmwareVersion(std::string newVersion) {
    _firmwareVersion = newVersion;
}

void Ariadne::setBatteryGauge(SFE_MAX1704X *lipo) {
    m_lipo = lipo;
}

uint8_t Ariadne::batteryLevel() {
    float soc = m_lipo->getSOC();
    m_batteryLevel = static_cast<uint8_t>(soc);

    return m_batteryLevel;
}

void Ariadne::batteryLevel(uint8_t batteryLevel) {
    m_batteryLevel = batteryLevel;
}

// #TODO implement get backup of subjectIDs pressure

ariadne_stimMode_t Ariadne::stimMode() const {
    return m_stimMode;
}

void Ariadne::stimMode(ariadne_stimMode_t value) {
    m_stimMode = value;
}

void Ariadne::setAmplitude(const uint8_t data) {
    m_amplitude = data;
}

uint16_t Ariadne::duration(const DRV_port_t port) {
    return m_duration[(uint8_t)port];
}

uint8_t Ariadne::amp() {
    return m_amplitude;
}

uint8_t Ariadne::effect(const uint8_t port) {
    return m_effects[port];
}

void Ariadne::registerStimCallback(StimulationCallback callback) {
    m_callback = callback;
}

void Ariadne::setRecordingTimestamp_us() {
    m_recording_timestampOff_us = micros();
}

void Ariadne::sendLastTimestampUSB() {
    if (is_recording_acc) {
        char stim_time_header[8] = {'S', 'T', 'I', 'M', 'T', 'I', 'M', 'E'};
        usb_serial_jtag_write_bytes(stim_time_header, sizeof(stim_time_header), pdMS_TO_TICKS(1));
        usb_serial_jtag_write_bytes(&m_stim_timestamp_us, sizeof(long), pdMS_TO_TICKS(1));
        usb_serial_jtag_ll_txfifo_flush();
    }
}

// Set duration triggers stimulation with amplitude
void Ariadne::setDuration(const uint16_t durationA, const uint16_t durationB) {
    m_duration[0] = durationA;
    m_duration[1] = durationB;

    if (durationA) {
        is_A_activated = true;
    }
    if (durationB) {
        is_B_activated = true;
    }

    if (is_recording_acc) {
        m_stim_timestamp_us = micros() - m_recording_timestampOff_us;
    }
    // Invoke callback if registered
    if (m_callback) {
        m_callback();
    } else {
        log_e("Error: The stimulation callback has not been registered!");
    }
}
