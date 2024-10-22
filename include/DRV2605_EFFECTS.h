#ifndef DRV2605_EFFECTS_H
#define DRV2605_EFFECTS_H

/*!\enum DRV2605L_effect
** \brief Waveform Library Effects List
**/
typedef enum {
    DRV_EFF_STOP_SEQ = 0,                             //!< Strong Click - 100%
    DRV_EFF_STRONG_CLK_100 = 1,                       //!< Strong Click - 100%
    DRV_EFF_STRONG_CLK_60 = 2,                        //!< Strong Click - 60%
    DRV_EFF_STRONG_CLK_30 = 3,                        //!< Strong Click - 30%
    DRV_EFF_SHARP_CLK_100 = 4,                        //!< Sharp Click - 100%
    DRV_EFF_SHARP_CLK_60 = 5,                         //!< Sharp Click - 60%
    DRV_EFF_SHARP_CLK_30 = 6,                         //!< Sharp Click - 30%
    DRV_EFF_SOFT_BUMP_100 = 7,                        //!< Soft Bump - 100%
    DRV_EFF_SOFT_BUMP_60 = 8,                         //!< Soft Bump - 60%
    DRV_EFF_SOFT_BUMP_30 = 9,                         //!< Soft Bump - 30%
    DRV_EFF_DBL_CLK_100 = 10,                         //!< Double Click - 100%
    DRV_EFF_DBL_CLK_60 = 11,                          //!< Double Click - 60%
    DRV_EFF_DBL_CLK_30 = 12,                          //!< Triple Click - 100%
    DRV_EFF_SOFT_FUZZ_60 = 13,                        //!< Soft Fuzz - 60%
    DRV_EFF_STRONG_BUZZ_100 = 14,                     //!< Strong Buzz - 100%
    DRV_EFF_ALERT_750MS = 15,                         //!< 750ms Alert 100%
    DRV_EFF_ALERT_1000MS = 16,                        //!< 1000ms Alert 100%
    DRV_EFF_STRONG_CLK_1 = 17,                        //!< Strong Click 1 - 100%
    DRV_EFF_STRONG_CLK_2 = 18,                        //!< Strong Click 2 - 80%
    DRV_EFF_STRONG_CLK_3 = 19,                        //!< Strong Click 3 - 60%
    DRV_EFF_STRONG_CLK_4 = 20,                        //!< Strong Click 4 - 30%
    DRV_EFF_MEDIUM_CLK_1 = 21,                        //!< Medium Click 1 - 100%
    DRV_EFF_MEDIUM_CLK_2 = 22,                        //!< Medium Click 2 - 80%
    DRV_EFF_MEDIUM_CLK_3 = 23,                        //!< Medium Click 3 - 60%
    DRV_EFF_SHARP_TICK_1 = 24,                        //!< Sharp Tick 1 - 100%
    DRV_EFF_SHARP_TICK_2 = 25,                        //!< Sharp Tick 2 - 80%
    DRV_EFF_SHARP_TICK_3 = 26,                        //!< Sharp Tick 3 - 60%
    DRV_EFF_SHORT_DBL_CLK_STRONG_1 = 27,              //!< Short Double Click Strong 1 - 100%
    DRV_EFF_SHORT_DBL_CLK_STRONG_2 = 28,              //!< Short Double Click Strong 2 - 80%
    DRV_EFF_SHORT_DBL_CLK_STRONG_3 = 29,              //!< Short Double Click Strong 3 - 60%
    DRV_EFF_SHORT_DBL_CLK_STRONG_4 = 30,              //!< Short Double Click Strong 4 - 30%
    DRV_EFF_SHORT_DBL_CLK_MEDIUM_1 = 31,              //!< Short Double Click Medium 1 - 100%
    DRV_EFF_SHORT_DBL_CLK_MEDIUM_2 = 32,              //!< Short Double Click Medium 2 - 80%
    DRV_EFF_SHORT_DBL_CLK_MEDIUM_3 = 33,              //!< Short Double Click Medium 3 - 60%
    DRV_EFF_SHORT_DBL_SHARP_TICK_1 = 34,              //!< Short Double Sharp Tick 1 - 100%
    DRV_EFF_SHORT_DBL_SHARP_TICK_2 = 35,              //!< Short Double Sharp Tick 2 - 80%
    DRV_EFF_SHORT_DBL_SHARP_TICK_3 = 36,              //!< Short Double Sharp Tick 3 - 60%
    DRV_EFF_LONG_DBL_CLK_STRONG_1 = 37,               //!< Long Double Sharp Click Strong 1 - 100%
    DRV_EFF_LONG_DBL_CLK_STRONG_2 = 38,               //!< Long Double Sharp Click Strong 2 - 80%
    DRV_EFF_LONG_DBL_CLK_STRONG_3 = 39,               //!< Long Double Sharp Click Strong 3 - 60%
    DRV_EFF_LONG_DBL_CLK_STRONG_4 = 40,               //!< Long Double Sharp Click Strong 4 - 30%
    DRV_EFF_LONG_DBL_CLK_MEDIUM_1 = 41,               //!< Long Double Sharp Click Medium 1 - 100%
    DRV_EFF_LONG_DBL_CLK_MEDIUM_2 = 42,               //!< Long Double Sharp Click Medium 2 - 80%
    DRV_EFF_LONG_DBL_CLK_MEDIUM_3 = 43,               //!< Long Double Sharp Click Medium 3 - 60%
    DRV_EFF_LONG_DBL_SHARP_TICK_1 = 44,               //!< Long Double Sharp Tick 1 - 100%
    DRV_EFF_LONG_DBL_SHARP_TICK_2 = 45,               //!< Long Double Sharp Tick 2 - 80%
    DRV_EFF_LONG_DBL_SHARP_TICK_3 = 46,               //!< Long Double Sharp Tick 3 - 60%
    DRV_EFF_BUZZ_1 = 47,                              //!< Buzz 1 - 100%
    DRV_EFF_BUZZ_2 = 48,                              //!< Buzz 2 - 80%
    DRV_EFF_BUZZ_3 = 49,                              //!< Buzz 3 - 60%
    DRV_EFF_BUZZ_4 = 50,                              //!< Buzz 4 - 40%
    DRV_EFF_BUZZ_5 = 51,                              //!< Buzz 5 - 20%
    DRV_EFF_PULSING_STRONG_1 = 52,                    //!< Pulsing Strong 1 - 100%
    DRV_EFF_PULSING_STRONG_2 = 53,                    //!< Pulsing Strong 2 - 60%
    DRV_EFF_PULSING_MEDIUM_1 = 54,                    //!< Pulsing Medium 1 - 100%
    DRV_EFF_PULSING_MEDIUM_2 = 55,                    //!< Pulsing Medium 2 - 60%
    DRV_EFF_PULSING_SHARP_1 = 56,                     //!< Pulsing Sharp 1 - 100%
    DRV_EFF_PULSING_SHARP_2 = 57,                     //!< Pulsing Sharp 2 - 60%
    DRV_EFF_TRANS_CLK_1 = 58,                         //!< Transition Click 1 - 100%
    DRV_EFF_TRANS_CLK_2 = 59,                         //!< Transition Click 2 - 80%
    DRV_EFF_TRANS_CLK_3 = 60,                         //!< Transition Click 3 - 60%
    DRV_EFF_TRANS_CLK_4 = 61,                         //!< Transition Click 4 - 40%
    DRV_EFF_TRANS_CLK_5 = 62,                         //!< Transition Click 5 - 20%
    DRV_EFF_TRANS_CLK_6 = 63,                         //!< Transition Click 6 - 10%
    DRV_EFF_TRANS_HUM_1 = 64,                         //!< Transition Hum 1 - 100%
    DRV_EFF_TRANS_HUM_2 = 65,                         //!< Transition Hum 2 - 80%
    DRV_EFF_TRANS_HUM_3 = 66,                         //!< Transition Hum 3 - 60%
    DRV_EFF_TRANS_HUM_4 = 67,                         //!< Transition Hum 4 - 40%
    DRV_EFF_TRANS_HUM_5 = 68,                         //!< Transition Hum 5 - 20%
    DRV_EFF_TRANS_HUM_6 = 69,                         //!< Transition Hum 6 - 10%
    DRV_EFF_TRANS_RAMP_DWN_LONG_SMOOTH_1 = 70,        //!< Transition Ramp Down Long Smooth 1 - 100 to 0%
    DRV_EFF_TRANS_RAMP_DWN_LONG_SMOOTH_2 = 71,        //!< Transition Ramp Down Long Smooth 2 - 100 to 0%
    DRV_EFF_TRANS_RAMP_DWN_MEDIUM_SMOOTH_1 = 72,      //!< Transition Ramp Down Medium Smooth 1 - 100 to 0%
    DRV_EFF_TRANS_RAMP_DWN_MEDIUM_SMOOTH_2 = 73,      //!< Transition Ramp Down Medium Smooth 2 - 100 to 0%
    DRV_EFF_TRANS_RAMP_DWN_SHORT_SMOOTH_1 = 74,       //!< Transition Ramp Down Short Smooth 1 - 100 to 0%
    DRV_EFF_TRANS_RAMP_DWN_SHORT_SMOOTH_2 = 75,       //!< Transition Ramp Down Short Smooth 2 - 100 to 0%
    DRV_EFF_TRANS_RAMP_DWN_LONG_SHARP_1 = 76,         //!< Transition Ramp Down Long Sharp 1 - 100 t0 0%
    DRV_EFF_TRANS_RAMP_DWN_LONG_SHARP_2 = 77,         //!< Transition Ramp Down Long Sharp 2 - 100 to 0%
    DRV_EFF_TRANS_RAMP_DWN_MEDIUM_SHARP_1 = 78,       //!< Transition Ramp Down Medium Sharp 1 - 100 to 0%
    DRV_EFF_TRANS_RAMP_DWN_MEDIUM_SHARP_2 = 79,       //!< Transition Ramp Down Medium Sharp 2 - 100 to 0%
    DRV_EFF_TRANS_RAMP_DWN_SHORT_SHARP_1 = 80,        //!< Transition Ramp Down Short Sharp 1 - 100 to 0%
    DRV_EFF_TRANS_RAMP_DWN_SHORT_SHARP_2 = 81,        //!< Transition Ramp Down Short Sharp 2 - 100 to 0%
    DRV_EFF_TRANS_RAMP_UP_LONG_SMOOTH_1 = 82,         //!< Transition Ramp Up Long Smooth 1 - 0 to 100%
    DRV_EFF_TRANS_RAMP_UP_LONG_SMOOTH_2 = 83,         //!< Transition Ramp Up Long Smooth 2 - 0 to 100%
    DRV_EFF_TRANS_RAMP_UP_MEDIUM_SMOOTH_1 = 84,       //!< Transition Ramp Up Medium Smooth 1 - 0 to 100%
    DRV_EFF_TRANS_RAMP_UP_MEDIUM_SMOOTH_2 = 85,       //!< Transition Ramp Up Medium Smooth 2 - 0 to 100%
    DRV_EFF_TRANS_RAMP_UP_SHORT_SMOOTH_1 = 86,        //!< Transition Ramp Up Short Smooth 1 - 0 to 100%
    DRV_EFF_TRANS_RAMP_UP_SHORT_SMOOTH_2 = 87,        //!< Transition Ramp Up Short Smooth 2 - 0 to 100%
    DRV_EFF_TRANS_RAMP_UP_LONG_SHARP_1 = 88,          //!< Transition Ramp Up Long Sharp 1 - 0 to 100%
    DRV_EFF_TRANS_RAMP_UP_LONG_SHARP_2 = 89,          //!< Transition Ramp Up Long Sharp 2 - 0 to 100%
    DRV_EFF_TRANS_RAMP_UP_MEDIUM_SHARP_1 = 90,        //!< Transition Ramp Up Medium Sharp 1 - 0 to 100%
    DRV_EFF_TRANS_RAMP_UP_MEDIUM_SHARP_2 = 91,        //!< Transition Ramp Up Medium Sharp 2 - 0 to 100%
    DRV_EFF_TRANS_RAMP_UP_SHORT_SHARP_1 = 92,         //!< Transition Ramp Up Short Sharp 1 - 0 to 100%
    DRV_EFF_TRANS_RAMP_UP_SHORT_SHARP_2 = 93,         //!< Transition Ramp Up Short Sharp 2 - 0 to 100%
    DRV_EFF_TRANS_RAMP_DWN_LONG_SMOOTH_1_LOW = 94,    //!< Transition Ramp Down Long Smooth 1 - 50 to 0%
    DRV_EFF_TRANS_RAMP_DWN_LONG_SMOOTH_2_LOW = 95,    //!< Transition Ramp Down Long Smooth 2 - 50 to 0%
    DRV_EFF_TRANS_RAMP_DWN_MEDIUM_SMOOTH_1_LOW = 96,  //!< Transition Ramp Down Medium Smooth 1 - 50 to 0%
    DRV_EFF_TRANS_RAMP_DWN_MEDIUM_SMOOTH_2_LOW = 97,  //!< Transition Ramp Down Medium Smooth 2 - 50 to 0%
    DRV_EFF_TRANS_RAMP_DWN_SHORT_SMOOTH_1_LOW = 98,   //!< Transition Ramp Down Short Smooth 1 - 50 to 0%
    DRV_EFF_TRANS_RAMP_DWN_SHORT_SMOOTH_2_LOW = 99,   //!< Transition Ramp Down Short Smooth 2 - 50 to 0%
    DRV_EFF_TRANS_RAMP_DWN_LONG_SHARP_1_LOW = 100,    //!< Transition Ramp Down Long Sharp 1 - 50 to 0%
    DRV_EFF_TRANS_RAMP_DWN_LONG_SHARP_2_LOW = 101,    //!< Transition Ramp Down Long Sharp 2 - 50 to 0%
    DRV_EFF_TRANS_RAMP_DWN_MEDIUM_SHARP_1_LOW = 102,  //!< Transition Ramp Down Medium Sharp 1 - 50 to 0%
    DRV_EFF_TRANS_RAMP_DWN_MEDIUM_SHARP_2_LOW = 103,  //!< Transition Ramp Down Medium Sharp 2 - 50 to 0%
    DRV_EFF_TRANS_RAMP_DWN_SHORT_SHARP_1_LOW = 104,   //!< Transition Ramp Down Short Sharp 1 - 50 to 0%
    DRV_EFF_TRANS_RAMP_DWN_SHORT_SHARP_2_LOW = 105,   //!< Transition Ramp Down Short Sharp 2 - 50 to 0%
    DRV_EFF_TRANS_RAMP_UP_LONG_SMOOTH_1_LOW = 106,    //!< Transition Ramp Up Long Smooth 1 - 0 to 50%
    DRV_EFF_TRANS_RAMP_UP_LONG_SMOOTH_2_LOW = 107,    //!< Transition Ramp Up Long Smooth 2 - 0 to 50%
    DRV_EFF_TRANS_RAMP_UP_MEDIUM_SMOOTH_1_LOW = 108,  //!< Transition Ramp Up Medium Smooth 1 - 0 to 50%
    DRV_EFF_TRANS_RAMP_UP_MEDIUM_SMOOTH_2_LOW = 109,  //!< Transition Ramp Up Medium Smooth 2 - 0 to 50%
    DRV_EFF_TRANS_RAMP_UP_SHORT_SMOOTH_1_LOW = 110,   //!< Transition Ramp Up Short Smooth 1 - 0 to 50%
    DRV_EFF_TRANS_RAMP_UP_SHORT_SMOOTH_2_LOW = 111,   //!< Transition Ramp Up Short Smooth 2 - 0 to 50%
    DRV_EFF_TRANS_RAMP_UP_LONG_SHARP_1_LOW = 112,     //!< Transition Ramp Up Long Sharp 1 - 0 to 50%
    DRV_EFF_TRANS_RAMP_UP_LONG_SHARP_2_LOW = 113,     //!< Transition Ramp Up Long Sharp 2 - 0 to 50%
    DRV_EFF_TRANS_RAMP_UP_MEDIUM_SHARP_1_LOW = 114,   //!< Transition Ramp Up Medium Sharp 1 - 0 to 50%
    DRV_EFF_TRANS_RAMP_UP_MEDIUM_SHARP_2_LOW = 115,   //!< Transition Ramp Up Medium Sharp 2 - 0 to 50%
    DRV_EFF_TRANS_RAMP_UP_SHORT_SHARP_1_LOW = 116,    //!< Transition Ramp Up Short Sharp 1 - 0 to 50%
    DRV_EFF_TRANS_RAMP_UP_SHORT_SHARP_2_LOW = 117,    //!< Transition Ramp Up Short Sharp 2 - 0 to 50%
    DRV_EFF_LONG_BUZZ = 118,                          //!< Long buzz for programmatic stopping - 100%
    DRV_EFF_SMOOTH_HUM_1 = 119,                       //!< Smooth Hum 1 (No kick or brake pulse) - 50%
    DRV_EFF_SMOOTH_HUM_2 = 120,                       //!< Smooth Hum 2 (No kick or brake pulse) - 40%
    DRV_EFF_SMOOTH_HUM_3 = 121,                       //!< Smooth Hum 3 (No kick or brake pulse) - 30%
    DRV_EFF_SMOOTH_HUM_4 = 122,                       //!< Smooth Hum 4 (No kick or brake pulse) - 20%
    DRV_EFF_SMOOTH_HUM_5 = 123,                       //!< Smooth Hum 5 (No kick or brake pulse) - 10%
    DRV_NUM_EFFECTS
} DRV2605L_eff;

const char *const drv_effect_string_map[DRV_NUM_EFFECTS] = {
    "Stop Sequence",
    "Strong Click - 100%",
    "Strong Click - 60%",
    "Strong Click - 30%",
    "Sharp Click - 100%",
    "Sharp Click - 60%",
    "Sharp Click - 30%",
    "Soft Bump - 100%",
    "Soft Bump - 60%",
    "Soft Bump - 30%",
    "Double Click - 100%",
    "Double Click - 60%",
    "Triple Click - 100%",
    "Soft Fuzz - 60%",
    "Strong Buzz - 100%",
    "750ms Alert 100%",
    "1000ms Alert 100%",
    "Strong Click 1 - 100%",
    "Strong Click 2 - 80%",
    "Strong Click 3 - 60%",
    "Strong Click 4 - 30%",
    "Medium Click 1 - 100%",
    "Medium Click 2 - 80%",
    "Medium Click 3 - 60%",
    "Sharp Tick 1 - 100%",
    "Sharp Tick 2 - 80%",
    "Sharp Tick 3 - 60%",
    "Short Double Click Strong 1 - 100%",
    "Short Double Click Strong 2 - 80%",
    "Short Double Click Strong 3 - 60%",
    "Short Double Click Strong 4 - 30%",
    "Short Double Click Medium 1 - 100%",
    "Short Double Click Medium 2 - 80%",
    "Short Double Click Medium 3 - 60%",
    "Short Double Sharp Tick 1 - 100%",
    "Short Double Sharp Tick 2 - 80%",
    "Short Double Sharp Tick 3 - 60%",
    "Long Double Sharp Click Strong 1 - 100%",
    "Long Double Sharp Click Strong 2 - 80%",
    "Long Double Sharp Click Strong 3 - 60%",
    "Long Double Sharp Click Strong 4 - 30%",
    "Long Double Sharp Click Medium 1 - 100%",
    "Long Double Sharp Click Medium 2 - 80%",
    "Long Double Sharp Click Medium 3 - 60%",
    "Long Double Sharp Tick 1 - 100%",
    "Long Double Sharp Tick 2 - 80%",
    "Long Double Sharp Tick 3 - 60%",
    "Buzz 1 - 100%",
    "Buzz 2 - 80%",
    "Buzz 3 - 60%",
    "Buzz 4 - 40% ",
    "Buzz 5 - 20%",
    "Pulsing Strong 1 - 100%",
    "Pulsing Strong 2 - 60%",
    "Pulsing Medium 1 - 100%",
    "Pulsing Medium 2 - 60%",
    "Pulsing Sharp 1 - 100%",
    "Pulsing Sharp 2 - 60%",
    "Transition Click 1 - 100%",
    "Transition Click 2 - 80%",
    "Transition Click 3 - 60%",
    "Transition Click 4 - 40%",
    "Transition Click 5 - 20%",
    "Transition Click 6 - 10%",
    "Transition Hum 1 - 100%",
    "Transition Hum 2 - 80%",
    "Transition Hum 3 - 60%",
    "Transition Hum 4 - 40%",
    "Transition Hum 5 - 20%",
    "Transition Hum 6 - 10%",
    "Transition Ramp Down Long Smooth 1 - 100 to 0%",
    "Transition Ramp Down Long Smooth 2 - 100 to 0%",
    "Transition Ramp Down Medium Smooth 1 - 100 to 0%",
    "Transition Ramp Down Medium Smooth 2 - 100 to 0%",
    "Transition Ramp Down Short Smooth 1 - 100 to 0%",
    "Transition Ramp Down Short Smooth 2 - 100 to 0%",
    "Transition Ramp Down Long Sharp 1 - 100 t0 0%",
    "Transition Ramp Down Long Sharp 2 - 100 to 0%",
    "Transition Ramp Down Medium Sharp 1 - 100 to 0%",
    "Transition Ramp Down Medium Sharp 2 - 100 to 0%",
    "Transition Ramp Down Short Sharp 1 - 100 to 0%",
    "Transition Ramp Down Short Sharp 2 - 100 to 0%",
    "Transition Ramp Up Long Smooth 1 - 0 to 100%",
    "Transition Ramp Up Long Smooth 2 - 0 to 100%",
    "Transition Ramp Up Medium Smooth 1 - 0 to 100%",
    "Transition Ramp Up Medium Smooth 2 - 0 to 100%",
    "Transition Ramp Up Short Smooth 1 - 0 to 100%",
    "Transition Ramp Up Short Smooth 2 - 0 to 100%",
    "Transition Ramp Up Long Sharp 1 - 0 to 100%",
    "Transition Ramp Up Long Sharp 2 - 0 to 100%",
    "Transition Ramp Up Medium Sharp 1 - 0 to 100%",
    "Transition Ramp Up Medium Sharp 2 - 0 to 100%",
    "Transition Ramp Up Short Sharp 1 - 0 to 100%",
    "Transition Ramp Up Short Sharp 2 - 0 to 100%",
    "Transition Ramp Down Long Smooth 1 - 50 to 0%",
    "Transition Ramp Down Long Smooth 2 - 50 to 0%",
    "Transition Ramp Down Medium Smooth 1 - 50 to 0%",
    "Transition Ramp Down Medium Smooth 2 - 50 to 0%",
    "Transition Ramp Down Short Smooth 1 - 50 to 0%",
    "Transition Ramp Down Short Smooth 2 - 50 to 0%",
    "Transition Ramp Down Long Sharp 1 - 50 to 0%",
    "Transition Ramp Down Long Sharp 2 - 50 to 0%",
    "Transition Ramp Down Medium Sharp 1 - 50 to 0%",
    "Transition Ramp Down Medium Sharp 2 - 50 to 0%",
    "Transition Ramp Down Short Sharp 1 - 50 to 0%",
    "Transition Ramp Down Short Sharp 2 - 50 to 0%",
    "Transition Ramp Up Long Smooth 1 - 0 to 50%",
    "Transition Ramp Up Long Smooth 2 - 0 to 50%",
    "Transition Ramp Up Medium Smooth 1 - 0 to 50%",
    "Transition Ramp Up Medium Smooth 2 - 0 to 50%",
    "Transition Ramp Up Short Smooth 1 - 0 to 50%",
    "Transition Ramp Up Short Smooth 2 - 0 to 50%",
    "Transition Ramp Up Long Sharp 1 - 0 to 50%",
    "Transition Ramp Up Long Sharp 2 - 0 to 50%",
    "Transition Ramp Up Medium Sharp 1 - 0 to 50%",
    "Transition Ramp Up Medium Sharp 2 - 0 to 50%",
    "Transition Ramp Up Short Sharp 1 - 0 to 50%",
    "Transition Ramp Up Short Sharp 2 - 0 to 50%",
    "Long buzz for programmatic stopping - 100%",
    "Smooth Hum 1 (No kick or brake pulse) - 50%",
    "Smooth Hum 2 (No kick or brake pulse) - 40%",
    "Smooth Hum 3 (No kick or brake pulse) - 30%",
    "Smooth Hum 4 (No kick or brake pulse) - 20%",
    "Smooth Hum 5 (No kick or brake pulse) - 10%"};

#endif  // DRV2605_EFFECTS_H