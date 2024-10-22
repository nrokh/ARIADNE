#include <BLE_HANDLER.h>
// Definition of static variables
static const char *TAG_BLE = "BLE-Interface";
static CharacteristicCallbacks chrCallbacks;

// Implementation of ServerCallbacks methods --------------------------------

void ServerCallbacks::onConnect(NimBLEServer *pServer, ble_gap_conn_desc *desc) {
    led_pressureMode();
    led_fade_to(127, LED_BLUE_CHANNEL);
    ESP_LOGI(TAG_BLE, "Client connected: %s\n", NimBLEAddress(desc->peer_ota_addr).toString().c_str());
};

void ServerCallbacks::onDisconnect(NimBLEServer *pServer) {
    led_breath();
    ESP_LOGI(TAG_BLE, "Client disconnected\n");
    // if still advertising we won't sleep yet.
    if (!pServer->getAdvertising()->isAdvertising()) {
        // ESP_LOGI(TAG_BLE, "Sleeping for %u seconds\n", 5);
        // esp_deep_sleep_start();
    }
}

// Implementation of CharacteristicCallbacks methods ------------------------------

void CharacteristicCallbacks::onRead(NimBLECharacteristic *pCharacteristic) {
    switch (pCharacteristic->getUUID().getNative()->u16.value) {
        case BLE_BATTERY_LEVEL_CHARACTERISTIC_UUID:
            incrementer++;
            pCharacteristic->setValue(ariadne.batteryLevel());
            ESP_LOGI(TAG_BLE, , "[%s] Battery level %d", pCharacteristic->getUUID().toString().c_str(), pCharacteristic->getValue()[0] + 48);
            break;
        default:

            ESP_LOGD(TAG_BLE, "Undefined BLE-Characteristic");
            break;
    }
}

void CharacteristicCallbacks::onWrite(NimBLECharacteristic *pCharacteristic) {
    const uint8_t *data;
    uint16_t duration;

    ESP_LOGI(TAG_BLE, "[%s] onWrite(), value: %d", pCharacteristic->getUUID().toString().c_str(), pCharacteristic->getValue().data()[0]);
    switch (pCharacteristic->getUUID().getNative()->u16.value) {
        case BLE_AMPLITUDE_CHARACTERISTIC_UUID:
            pCharacteristic->setValue(handleAmplitudeWrite(pCharacteristic->getValue().data()[0]));

            break;

        case BLE_DURATION_A_CHARACTERISTIC_UUID:
            duration = ((uint16_t)pCharacteristic->getValue().data()[1] << 8 | pCharacteristic->getValue().data()[0]);
            handleDurationWrite(duration, 0);
            break;

        case BLE_DURATION_B_CHARACTERISTIC_UUID:
            duration = ((uint16_t)pCharacteristic->getValue().data()[1] << 8 | pCharacteristic->getValue().data()[0]);
            handleDurationWrite(0, duration);
            break;

        default:

            ESP_LOGD(TAG_BLE, "Undefined BLE-Characteristic");
            break;
    }
};

/** Called before notification or indication is sent,
 *  the value can be changed here before sending if desired.
 */
void CharacteristicCallbacks::onNotify(NimBLECharacteristic *pCharacteristic) {
    ESP_LOGI(TAG_BLE, "Sending notification to clients");
};

/** The status returned in status is defined in NimBLECharacteristic.h.
 *  The value returned in code is the NimBLE host return code.
 */
void CharacteristicCallbacks::onStatus(NimBLECharacteristic *pCharacteristic, Status status, int code) {
    String str = ("Notification/Indication status code: ");
    str += status;
    str += ", return code: ";
    str += code;
    str += ", ";
    str += NimBLEUtils::returnCodeToString(code);
    ESP_LOGI(TAG_BLE, "%s", str);
};

void CharacteristicCallbacks::onSubscribe(NimBLECharacteristic *pCharacteristic, ble_gap_conn_desc *desc, uint16_t subValue) {
    String str = "Client ID: ";
    str += desc->conn_handle;
    str += " Address: ";
    str += std::string(NimBLEAddress(desc->peer_ota_addr)).c_str();
    if (subValue == 0) {
        str += " Unsubscribed to ";
    } else if (subValue == 1) {
        str += " Subscribed to notfications for ";
    } else if (subValue == 2) {
        str += " Subscribed to indications for ";
    } else if (subValue == 3) {
        str += " Subscribed to notifications and indications for ";
    }
    str += std::string(pCharacteristic->getUUID()).c_str();
    ESP_LOGI(TAG_BLE, "%s", str);
};

uint8_t CharacteristicCallbacks::handleAmplitudeWrite(uint8_t amplitude) {
    ESP_LOGD(TAG_BLE, "Amplitude set to %d", amplitude);
    if (ariadne.is_A_activated || ariadne.is_B_activated) {
        ESP_LOGE(TAG_BLE, "\e[0;34mCOMMAND IGNORED: Device still running\e[0m");
        digitalWrite(LED_ORANGE_PIN, HIGH);

    } else {
        ariadne.setAmplitude(amplitude);
    }
    return ariadne.amp();
}

void CharacteristicCallbacks::handleDurationWrite(uint16_t durationA, uint16_t durationB) {
    ESP_LOGD(TAG_BLE, "Duration command.\nDuration A: %dms\nDuration B: %dms", durationA, durationB);
    if (ariadne.is_A_activated || ariadne.is_B_activated) {
        ESP_LOGE("", "COMMAND IGNORED: Device was still running");
    } else if (durationA && durationB) {
        ESP_LOGW("", "It seems like the duration has been set for both devices.\nCurrently this is not supported and will cause both devices to activate for the duration 'B'");
    } else {
        ariadne.setDuration(durationA, durationB);
    }
}

// Implementation of BLE SETUP ------------------------------------------------------------

void ble_setup(const std::string &deviceName) {
    // Serial.begin(115200);
    // Serial.println("Starting Ariadne BLE - Server");
    NimBLEDevice::init(deviceName);
    //  NimBLEDevice::
#ifdef ESP_PLATFORM
    // NimBLEDevice::setPower(ESP_PWR_LVL_P9); /** +9db */
#else
    NimBLEDevice::setPower(9); /** +9db */
#endif

    NimBLEDevice::setSecurityAuth(true, true, true);
    //  NimBLEDevice::setSecurityPasskey(123456);
    //  NimBLEDevice::setSecurityIOCap(BLE_HS_IO_DISPLAY_ONLY);
    NimBLEDevice::setSecurityIOCap(BLE_HS_IO_NO_INPUT_OUTPUT);  // deactivates passkey -> Just works
    NimBLEServer *pServer = NimBLEDevice::createServer();

    NimBLEService *controlService = pServer->createService((uint16_t)BLE_LRA_CONTROL_SERVICE_UUID);
    NimBLEService *batteryService = pServer->createService((uint16_t)BLE_BATTERY_SERVICE_UUID);

    // NimBLEService *diagService = pServer->createService((uint16_t)BLE_LRA_DIAG_SERVICE_UUID);
    pServer->setCallbacks(new ServerCallbacks);
    // NimBLECharacteristic *pNonSecureCharacteristic = pService->createCharacteristic("1234", NIMBLE_PROPERTY::READ );
    // NimBLECharacteristic *pSecureCharacteristic = pService->createCharacteristic("1235", NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::READ_ENC | NIMBLE_PROPERTY::READ_AUTHEN);

    NimBLECharacteristic *BLE_Amplitude = controlService->createCharacteristic((uint16_t)BLE_AMPLITUDE_CHARACTERISTIC_UUID,
                                                                               NIMBLE_PROPERTY::READ |
                                                                                   NIMBLE_PROPERTY::WRITE,
                                                                               //  NIMBLE_PROPERTY::READ_ENC |
                                                                               //  NIMBLE_PROPERTY::WRITE_ENC,
                                                                               1);
    BLE_Amplitude->setCallbacks(&chrCallbacks);
    BLE_Amplitude->setValue((uint8_t)0x00);

    NimBLECharacteristic *BLE_Duration_A = controlService->createCharacteristic((uint16_t)BLE_DURATION_A_CHARACTERISTIC_UUID,
                                                                                NIMBLE_PROPERTY::READ |
                                                                                    NIMBLE_PROPERTY::WRITE,
                                                                                //  NIMBLE_PROPERTY::READ_ENC |
                                                                                //  NIMBLE_PROPERTY::WRITE_ENC,
                                                                                2);
    BLE_Duration_A->setCallbacks(&chrCallbacks);
    BLE_Duration_A->setValue((uint16_t)0x0000);

    NimBLECharacteristic *BLE_Duration_B = controlService->createCharacteristic((uint16_t)BLE_DURATION_B_CHARACTERISTIC_UUID,
                                                                                NIMBLE_PROPERTY::READ |
                                                                                    NIMBLE_PROPERTY::WRITE,
                                                                                //  NIMBLE_PROPERTY::READ_ENC |
                                                                                //  NIMBLE_PROPERTY::WRITE_ENC,
                                                                                2);
    BLE_Duration_B->setCallbacks(&chrCallbacks);
    BLE_Duration_B->setValue((uint16_t)0x0000);

    // Battery sertvice
    NimBLECharacteristic *BLE_Battery_Level = batteryService->createCharacteristic((uint16_t)BLE_BATTERY_LEVEL_CHARACTERISTIC_UUID,
                                                                                   NIMBLE_PROPERTY::READ,
                                                                                   1);
    BLE_Battery_Level->setCallbacks(&chrCallbacks);
    BLE_Battery_Level->setValue((uint8_t)0xFF);

    /*
      READ         =  BLE_GATT_CHR_F_READ,
      READ_ENC     =  BLE_GATT_CHR_F_READ_ENC,
      READ_AUTHEN  =  BLE_GATT_CHR_F_READ_AUTHEN,
      READ_AUTHOR  =  BLE_GATT_CHR_F_READ_AUTHOR,
      WRITE        =  BLE_GATT_CHR_F_WRITE,
      WRITE_NR     =  BLE_GATT_CHR_F_WRITE_NO_RSP,
      WRITE_ENC    =  BLE_GATT_CHR_F_WRITE_ENC,
      WRITE_AUTHEN =  BLE_GATT_CHR_F_WRITE_AUTHEN,
      WRITE_AUTHOR =  BLE_GATT_CHR_F_WRITE_AUTHOR,
      BROADCAST    =  BLE_GATT_CHR_F_BROADCAST,
      NOTIFY       =  BLE_GATT_CHR_F_NOTIFY,
      INDICATE     =  BLE_GATT_CHR_F_INDICATE*/
    controlService->start();
    batteryService->start();
    // diagService->start();
    //  pNonSecureCharacteristic->setValue("Hello Non Secure BLE");
    //  pSecureCharacteristic->setValue("Hello Secure BLE");
    NimBLEAdvertising *pAdvertising = NimBLEDevice::getAdvertising();
    pAdvertising->addServiceUUID((uint16_t)BLE_BATTERY_SERVICE_UUID);
    pAdvertising->addServiceUUID((uint16_t)BLE_LRA_CONTROL_SERVICE_UUID);

    pAdvertising->start();
    // register Ariadne callbacks
    auto &ariadne = Ariadne::getInstance();
    // ariadne.onCurrentPressureChanged([](uint16_t pressure)
    //                                   { ble_advertisePressure(pressure); });
    // esp_sleep_enable_timer_wakeup(sleepSeconds * 1000000);
}