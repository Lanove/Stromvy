#ifndef INDICATORCONTROLLER_H
#define INDICATORCONTROLLER_H
#include <Arduino.h>
#include <globals.h>

#define LED_R PB7
#define LED_G PB5
#define LED_B PB6

#define SOFT_PWM_FREQUENCY 15300

#define SINE_PERIOD 2000 // in ms
#define BLINK_PERIOD 250 // in ms

#define INDICATE_NUM 6 // Total of indicator mode in INDICATE typedef enum
#define R_POS 0        // position of R on indicatorLookup array
#define G_POS 1        // position of G on indicatorLookup array
#define B_POS 2        // position of B on indicatorLookup array

#define POSITIVE true
#define NEGATIVE false

typedef enum
{
    STANDBY,      // Constant-green
    RUNNING_CC,   // Sine-wave of purple (255,0,155)
    RUNNING_CV,   // Sine-wave of light-blue (0,150,255)
    PAUSED,       // Sine-wave of orange (255,40,0)
    TIMER_HALT,   // Blinking of orange (255,40,0) dut=50% f=2hz and 3 sec buzz
    OVERHEAT_HALT // Blinking of red (255,0,0) and 3 sec buzz dut=50% f=2hz
} INDICATE;

const uint8_t isinTable8[] = {
    0,
    4,
    9,
    13,
    18,
    22,
    27,
    31,
    35,
    40,
    44,
    49,
    53,
    57,
    62,
    66,
    70,
    75,
    79,
    83,
    87,
    91,
    96,
    100,
    104,
    108,
    112,
    116,
    120,
    124,
    128,

    131,
    135,
    139,
    143,
    146,
    150,
    153,
    157,
    160,
    164,
    167,
    171,
    174,
    177,
    180,
    183,
    186,
    190,
    192,
    195,
    198,
    201,
    204,
    206,
    209,
    211,
    214,
    216,
    219,
    221,

    223,
    225,
    227,
    229,
    231,
    233,
    235,
    236,
    238,
    240,
    241,
    243,
    244,
    245,
    246,
    247,
    248,
    249,
    250,
    251,
    252,
    253,
    253,
    254,
    254,
    254,
    255,
    255,
    255,
    255,
};

class indicatorControllerClass
{
private:
    INDICATE currentIndicator = INDICATE::STANDBY; // Default indicator mode is standby
    uint32_t beepMillis,
        firstBeepMillis,
        ledMillis;
    bool beepFlag, beepDirection, beepLoopFlag;
    int beepTon, beepToff, beepTotalTime;

    uint8_t r, g, b; // PWM value of each led color
    uint8_t captureCompare;

    int sine = 0;
    bool direction = POSITIVE;
    const int16_t ledPin[3] = {LED_R, LED_G, LED_B},
                  ledSineLoopPeriod = SINE_PERIOD / 90;
    const uint8_t indicatorLookup[INDICATE_NUM][3] = {
        {0, 255, 0},
        {255, 0, 155},
        {0, 155, 255},
        {255, 40, 0},
        {255, 40, 0},
        {255, 0, 0},
    };
public:
    void begin();
    void service();
    void setIndicator(INDICATE indicator);
    INDICATE getIndicator();
    void beepBuzzer(int ton, int toff = 0, bool looping = false, int totalTime = 0);
    void stopBuzzer();
    int getBeepTon();
    int getBeepToff();
    int getBeepTotalTime();
    bool getBeepFlag();
    bool getBeepLoopFlag();
    bool getBeepDirection();
    float isin(int x);
};
extern indicatorControllerClass indicator;
#endif