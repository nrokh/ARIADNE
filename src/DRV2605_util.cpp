#include <DRV2605_util.h>

#include "LEDS.h"

DRV2605_UTIL &DRV2605_UTIL::getInstance() {
    static DRV2605_UTIL instance;
    return instance;
}
DRV2605_UTIL::DRV2605_UTIL() {
    m_ratedVoltage.bits.RATED_VOLTAGE = DRV_L_RV_LRA_CL_300US_2_0V_RMS;  // Rated Voltage [7:0] 2 Vrms
    m_OD_Clamp.bits.OD_CLAMP = DRV_L_LRA_OD_CLAMP_2_05V;                 // Overdrive Clamp Voltage: ODClamp [7:0] 2.05 Vpeak

    m_feedback_control.bits.N_ERM_LRA = DRV_LRA;
    m_feedback_control.bits.FB_BRAKE_FACTOR = DRV_FB_BRAKE_FACTOR_4X;
    m_feedback_control.bits.LOOP_GAIN = DRV_LOOP_GAIN_MEDIUM;
    m_feedback_control.bits.BEMF_GAIN = DRV_BEMF_GAIN_1_365X_15X;

    m_control1.bits.STARTUP_BOOST = DRV_STARTUP_BOOST_ON;
    m_control1.bits.AC_COUPLE = DRV_AC_COUPLE_DC;
    m_control1.bits.DRIVE_TIME = DRV_DRIVE_TIME_24;

    m_control2.bits.BIDIR_INPUT = DRV_BIDIR_INPUT_UNI;
    m_control2.bits.BRAKE_STABILIZE = DRV_BRAKE_STABILIZER_ON;
    m_control2.bits.SAMPLE_TIME = DRV_SAMPLE_TIME_300US;
    m_control2.bits.BLANKING_TIME = DRV_BLANKING_TIME_25_75US;
    m_control2.bits.IDISS_TIME = DRV_IDISS_TIME_25_75US;

    m_control3.bits.NG_THRESH = DRV_NG_THRESH_4PER;
    m_control3.bits.ERM_OPEN_LOOP = DRV_ERM_OPEN_LOOP;
    m_control3.bits.SUPPLY_COMP_DIS = DRV_SUPPLY_COMP_DIS_ON;
    m_control3.bits.DATA_FORMAT_RTP = DRV_DATA_FORMAT_RTP_SIGNED;
    m_control3.bits.LRA_DRIVE_MODE = DRV_LRA_DRIVE_MODE_ONCE;
    m_control3.bits.N_PWM_ANALOG = DRV_NPWM_ANALOG_PWM;
    m_control3.bits.LRA_OPEN_LOOP = DRV_LRA_AUTO_RES_MODE;

    m_control4.bits.ZC_DET_TIME = DRV_ZC_DET_TIME_100US;
    m_control4.bits.AUTO_CAL_TIME = DRV_AUTOCAL_TIME_1000MS;
    m_control4.bits.OTP_PROGRAM = DRV_OTP_PRG_OFF;

    m_control5.bits.AUTO_OL_CNT = DRV_AUTO_OL_CNT_5_ATTEMPTS;
    m_control5.bits.LRA_AUTO_OPEN_LOOP = DRV_LRA_AUTO_OPEN_LOOP_NEVER;
    m_control5.bits.PLAYBACK_INTERVAL = DRV_PLAYBACK_INTERVAL_5ms;
    m_control5.bits.BLANKING_TIME = DRV_BLANKING_TIME_MSB_DEFAULT;
    m_control5.bits.IDISS_TIME = DRV_IDISS_TIME_MSB_DEFAULT;

    m_LRA_open_loop_period.bits.OL_LRA_PERIOD = DRV_OL_LRA_PERIOD_175;  // Resonance-period of LRA in open loop

    auto &ariadne = Ariadne::getInstance();
    // register stimulate callback
    ariadne.registerStimCallback([this]() { this->stimulate(); });
}

void DRV2605_UTIL::stimulation_timer_callback(void *arg) {
    auto &ariadne = Ariadne::getInstance();
    auto &drv = DRV2605_UTIL::getInstance();
    switch (ariadne.stimMode()) {
        case ARIADNE_USERMODE_AMPLITUDE:
            drv.setRealtimeValue(0x00);
            break;
        case ARIADNE_USERMODE_EFFECT:
            drv.stop();
            break;
        default:
            break;
    }

    drv.disableAll();

    ariadne.is_A_activated = false;
    ariadne.is_B_activated = false;
    if (ariadne.is_recording_acc) {
        ariadne.sendLastTimestampUSB();  // send the stimulationTimestamp via usb
    }
    log_d("\n--------------STOPPED-------------------\n\n");
}

void DRV2605_UTIL::set_ratedVoltage(uint8_t ratedVoltage) {
    m_ratedVoltage.bits.RATED_VOLTAGE = ratedVoltage;

    bool state_A = m_enable_state[0];
    bool state_B = m_enable_state[1];
    enableAll();

    Adafruit_DRV2605::writeRegister8(DRV2605_REG_RATEDV, m_ratedVoltage.reg);

    if (!state_B) {
        disableB();
    }
    if (!state_A) {
        disableA();
    }
}
void DRV2605_UTIL::set_odClamp(uint8_t odClamp) {
    m_OD_Clamp.bits.OD_CLAMP = odClamp;
    bool state_A = m_enable_state[0];
    bool state_B = m_enable_state[1];
    enableAll();
    Adafruit_DRV2605::writeRegister8(DRV2605_REG_CLAMPV, m_OD_Clamp.reg);

    if (!state_B) {
        disableB();
    }
    if (!state_A) {
        disableA();
    }
}
void DRV2605_UTIL::autoCalibrate() {
    ESP_LOGI("DRV_UTIL", "START autocalibration");

    enableAll();

    Adafruit_DRV2605::setMode(DRV2605_MODE_AUTOCAL);

    Adafruit_DRV2605::go();
    uint8_t status;

    // Instead of polling the status we wait for a fixed delay.

    // Read the status register to check if the auto-calibration is complete
    //    while (status & 0x08)
    //    {
    //        status = Adafruit_DRV2605::readRegister8(DRV2605_REG_STATUS);
    //    }
    delay(1200);  // Autocalibration set to 1000ms

    ESP_LOGI(TAG_DRV, "END autocalibration");
}

void DRV2605_UTIL::init() {
    // create stimulation timer
    const esp_timer_create_args_t stimulation_timer_args = {
        .callback = &DRV2605_UTIL::stimulation_timer_callback,
        .name = "stimulation_timer"};

    ESP_ERROR_CHECK(esp_timer_create(&stimulation_timer_args, &stimulation_timer));
    enableAll();

    Adafruit_DRV2605::setMode(DRV2605_MODE_AUTOCAL);  //

    Adafruit_DRV2605::writeRegister8(DRV2605_REG_RATEDV, (uint8_t)m_ratedVoltage.reg);
    Adafruit_DRV2605::writeRegister8(DRV2605_REG_CLAMPV, (uint8_t)m_OD_Clamp.reg);

    Adafruit_DRV2605::writeRegister8(DRV2605_REG_FEEDBACK, (uint8_t)m_feedback_control.reg);

    Adafruit_DRV2605::writeRegister8(DRV2605_REG_CONTROL1, (uint8_t)m_control1.reg);
    Adafruit_DRV2605::writeRegister8(DRV2605_REG_CONTROL2, (uint8_t)m_control2.reg);
    Adafruit_DRV2605::writeRegister8(DRV2605_REG_CONTROL3, (uint8_t)m_control3.reg);
    Adafruit_DRV2605::writeRegister8(DRV2605_REG_CONTROL4, (uint8_t)m_control4.reg);

    DRV2605_UTIL::autoCalibrate();

    Adafruit_DRV2605::setRealtimeValue(0x00);
    Adafruit_DRV2605::setMode(DRV2605_MODE_REALTIME);

    // On default we use open loop since it oprovided more consistent results
    if (m_use_open_loop) {
        ESP_LOGI(TAG_DRV, "Using OPEN LOOP");
        m_control3.bits.LRA_OPEN_LOOP = DRV_LRA_OPEN_LOOP;
        m_control3.bits.ERM_OPEN_LOOP = DRV_ERM_OPEN_LOOP;
        Adafruit_DRV2605::writeRegister8(DRV2605_REG_CONTROL3, (uint8_t)m_control3.reg);
    }

    disableAll();
}

void DRV2605_UTIL::setTrigger(uint8_t triggerA, uint8_t triggerB) {
    m_trig_A = triggerA;
    m_trig_B = triggerB;

    pinMode(m_trig_A, OUTPUT);
    pinMode(m_trig_B, OUTPUT);

    digitalWrite(m_trig_A, LOW);
    digitalWrite(m_trig_B, LOW);
}
void DRV2605_UTIL::triggerA() {
    digitalWrite(m_trig_A, HIGH);
}
void DRV2605_UTIL::stopTriggerA() {
    digitalWrite(m_trig_A, LOW);
}
void DRV2605_UTIL::triggerB() {
    digitalWrite(m_trig_B, HIGH);
}
void DRV2605_UTIL::stopTriggerB() {
    digitalWrite(m_trig_B, LOW);
}

void DRV2605_UTIL::triggerStart() {
    digitalWrite(m_trig_A, HIGH);
    digitalWrite(m_trig_B, HIGH);
}

void DRV2605_UTIL::triggerStop() {
    digitalWrite(m_trig_A, LOW);
    digitalWrite(m_trig_B, LOW);
}

void DRV2605_UTIL::setEnablePins(uint8_t pinA, uint8_t pinB) {
    m_en_A = pinA;
    m_en_B = pinB;

    pinMode(m_en_A, OUTPUT);
    pinMode(m_en_B, OUTPUT);

    digitalWrite(m_en_A, LOW);
    digitalWrite(m_en_B, LOW);

    m_enable_state[0] = false;
    m_enable_state[1] = false;
}

void DRV2605_UTIL::enableAll() {
    enableA();
    enableB();
}
void DRV2605_UTIL::disableAll() {
    disableA();
    disableB();
}

void DRV2605_UTIL::enable(bool PortA, bool PortB) {
    if (PortA) {
        enableA();
    }
    if (PortB) {
        enableB();
    }
}
void DRV2605_UTIL::enableA() {
    digitalWrite(m_en_A, HIGH);
    m_enable_state[0] = true;
}
void DRV2605_UTIL::disableA() {
    digitalWrite(m_en_A, LOW);
    m_enable_state[0] = false;
}
void DRV2605_UTIL::enableB() {
    digitalWrite(m_en_B, HIGH);
    m_enable_state[1] = true;
}
void DRV2605_UTIL::disableB() {
    digitalWrite(m_en_B, LOW);
    m_enable_state[1] = false;
}

DRV2605_UTIL::drv_playbackInterval_t DRV2605_UTIL::playbackInterval() {
    return (drv_playbackInterval_t)m_control5.bits.PLAYBACK_INTERVAL;
}

void DRV2605_UTIL::setPlaybackInterval(drv_playbackInterval_t interval) {
    m_control5.bits.PLAYBACK_INTERVAL = (uint8_t)interval;
    Adafruit_DRV2605::writeRegister8(DRV2605_REG_CONTROL5, m_control5.reg);
}

void DRV2605_UTIL::stimulate() {
    // NOTE: when both A and B are activated both ports will be activated for the duration set by port "B"
    auto &ariadne = Ariadne::getInstance();
    DRV_port_t port;  // =0 -> A | =1 -> B

    switch (ariadne.stimMode()) {
        case ARIADNE_USERMODE_AMPLITUDE: {
            if (ariadne.is_A_activated) {
                enableA();
                port = drv_A;
            } else if (ariadne.is_B_activated) {
                enableB();
                port = drv_B;
            }
            setRealtimeValue(ariadne.amp());
            ESP_ERROR_CHECK(esp_timer_start_once(stimulation_timer, ariadne.duration(port) * 1000));  // time in us
            //  ESP_LOGD(TAG_DRV, "SetRealtimeValue = %d for %dms", ariadne.amp(), ariadne.duration());
            break;
        }
        case ARIADNE_USERMODE_EFFECT: {
            if (ariadne.is_A_activated) {
                enableA();
                port = drv_A;
            }
            if (ariadne.is_B_activated) {
                enableB();
                port = drv_B;
            }
            ESP_ERROR_CHECK(esp_timer_start_once(stimulation_timer, ariadne.duration(port) * 1000));  // time in us
            break;
        }

        case ARIADNE_USERMODE_DEMO00: {
            ESP_LOGI(TAG_DRV, "Userdemo: Alternatingly activate cuddles, repeating 3 'strong clicks' before switching side.");

            uint8_t effect = DRV_EFF_STRONG_CLK_100;
            enableAll();
            setMode(DRV2605_MODE_INTTRIG);
            for (uint8_t slot = 0; slot < 3; slot++) {
                setWaveform(slot, effect);
            }
            setWaveform(3, DRV_EFF_STOP_SEQ);
            disableAll();
            while (ariadne.stimMode() == ARIADNE_USERMODE_DEMO00) {
                disableB();
                enableA();
                Adafruit_DRV2605::go();

                delay(2000);
                disableA();
                enableB();
                Adafruit_DRV2605::go();
                delay(2000);
            }

            break;
        }
        default:
            ESP_LOGD(TAG_DRV, "Undefined Usermode");
            break;
    }
}
