#ifndef BLE_HANDLER_H
#define BLE_HANDLER_H

#include <Arduino.h>
#include <Ariadne.h>
#include <LEDS.h>
#include <NimBLEDevice.h>

typedef enum {
    BLE_SERVER_UUID = 0xF000,
    BLE_DEVICE_INFO_SERVICE_UUID = 0x180A,
    BLE_BATTERY_SERVICE_UUID = 0x180F,
    BLE_LRA_CONTROL_SERVICE_UUID = 0x1111,
    BLE_LRA_DIAG_SERVICE_UUID = 0x2220
} ServiceUUID_t;

typedef enum {
    // BATTERY_SERVICE
    BLE_BATTERY_LEVEL_CHARACTERISTIC_UUID = 0x2A19,
    // DEVICE_INFO_SERVICE
    // BLE_TX_POWER_LEVEL_CHARACTERISTIC_UUID = 0x2A07,
    BLE_FIRMWARE_REVISION_CHARACTERISTIC_UUID = 0x2A26,

    // DEVICE_CONTROL_SERVICE
    BLE_AMPLITUDE_CHARACTERISTIC_UUID = 0x1112,

    /*  For consistency A and B should be swapped. However, for historic reasons,
     *   to compatibility with scripts used inhouse A and B e.g left and right are
     *   are "B-first". The same applies for reading out accelerometer data via USB
     *   I recommend to swap A and B here and adapt the software
     *  When internal subject-trials have been completed this will probably be updated
     */
    BLE_DURATION_B_CHARACTERISTIC_UUID = 0x1113,
    BLE_DURATION_A_CHARACTERISTIC_UUID = 0x1114,

} CharacteristicUUID_t;

// not used but may be relevant for future use cases
typedef struct __attribute__((packed)) {
    uint8_t BLE_COMMAND_DURATION : 8;
    uint8_t BLE_COMMAND_CONTROL_VAL : 8;
    uint8_t BLE_COMMAND_SEQUENCE : 7;
    bool BLE_COMMAND_POS : 1;
} BLE_command_t;

/* Handler class for server events */
class ServerCallbacks : public NimBLEServerCallbacks {
    void onConnect(NimBLEServer *pServer, ble_gap_conn_desc *desc);

    void onDisconnect(NimBLEServer *pServer);
};

/** Handler class for characteristic actions */
class CharacteristicCallbacks : public NimBLECharacteristicCallbacks {
    uint8_t incrementer = 0;
    Ariadne &ariadne = Ariadne::getInstance();
    void onRead(NimBLECharacteristic *pCharacteristic);

    void onWrite(NimBLECharacteristic *pCharacteristic);
    /** Called before notification or indication is sent,
     *  the value can be changed here before sending if desired.
     */

    void onNotify(NimBLECharacteristic *pCharacteristic);

    /** The status returned in status is defined in NimBLECharacteristic.h.
     *  The value returned in code is the NimBLE host return code.
     */
    void onStatus(NimBLECharacteristic *pCharacteristic, Status status, int code);

    void onSubscribe(NimBLECharacteristic *pCharacteristic, ble_gap_conn_desc *desc, uint16_t subValue);

    //
    uint8_t handleAmplitudeWrite(uint8_t amplitude);

    void handleDurationWrite(uint16_t durationA, uint16_t durationB);
};
void ble_setup(const std::string &deviceName);

#endif  // BLE_HANDLER_H
