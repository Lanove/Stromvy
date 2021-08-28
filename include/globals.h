#include <Arduino.h>
#include <encoderController.h>
#include <lcdController.h>
#include <eepromController.h>
#include <indicatorController.h>
#include <Wire.h>
#include <ClickEncoder.h>
#include <Eeprom24C04_16.h>
#include "stm32f1xx_hal.h"
#include <RunningAverage.h>
#include "ADS1X15.h"
#include <IWatchdog.h>

#define WATCHDOG_TIMEOUT 4000000 // in uS

#define PIN_DEFINE // DO NOT DELETE

#define MODE_CC 0 // Used for opMode
#define MODE_CV 1 // Used for opMode

#define STATUS_ON 1  // Used for ldStatus
#define STATUS_OFF 0 // Used for ldStatus

#define ADS1115_ADDRESS 0x4A

#define OVERHEAT_TEMPERATURE 65 // maximum temperature for BJT to shutdown and goes OVERHEAT_HALT
#define LO2_MIN_CURRENT 0.0     // minimum current load for LO2 pwm map
#define LO2_MAX_CURRENT 2000.0  // maximum current load for LO2 pwm map

#define DAC_UPDATE_INTERVAL 10           // setCaptureCompare call interval, in msf
#define DAC_VOLTAGE_BASE_FACTOR 0.004619 // DAC to real voltage factor
#define DAC_CURRENT_BASE_FACTOR 1.095    // DAC to real current factor
#define DAC_CURRENT_OFFSET 19            // in mA

#define ADC_VOLTAGE_CHANNEL 0             // ADS1115 channel that is used to sense load voltage
#define ADC_CURRENT_CHANNEL 1             // ADS1115 channel that is used to sense load current
#define ADC_SAMPLE_INTERVAL 50            // ADC sample interval (also used for fan and energy)
#define ADC_SAMPLE_COUNT 20               // ADC running average total sample count (the bigger the value, the smoother the display but slower)
#define ADC_VOLTAGE_BASE_FACTOR 4.7299732 // Sensed voltage to real voltage factor
#define ADC_LSB_TO_CURRENT_MA 0.201560928 // ADC-LSB to real current
#define ADC_CURRENT_OFFSET 80             // ADC binary offset to cancel negative values

#define NTC_R1 10000.0         // The R1 of voltage divider for NTC
#define NTC_c1 1.009249522e-03 //c1 constant
#define NTC_c2 2.378405444e-04 //c2 constant
#define NTC_c3 2.019202697e-07 //c3 constant

#define MAX_V_DAC 4330 // Maximum DAC binary value (~20V)
#define MAX_I_DAC 2800 // Maximum DAC binary value (~3000mA)
#define MIN_V_DAC 100  // Minimum DAC binary value (~0.5V)
#define MIN_I_DAC 0    // Minimum DAC binary value (~19mA)

#define MAX_TIMER_DURATION 36000000 // in ms, 36000000 = 10 Hour
#define MIN_TIMER_DURATION 0
#define MAX_LOG_INTERVAL 60000 // in ms, 60000 = 1 Minute
#define MIN_LOG_INTERVAL 200   // in ms, 200 = 0.2 Second

#define MIN_FAN_PWM 0   // The absolute minimum for PWM value of fans
#define MAX_FAN_PWM 100 // The absolute maximum for PWM value of fans

#define MIN_FAN_TEMP 0  // The absolute minimum for bjt temperature for fan pwm map
#define MAX_FAN_TEMP 99 // The absolute maximum bjt temperature for fan pwm map

#define MAX_CALIBRATION_FACTOR 10.0000 // Self-explanatory
#define MIN_CALIBRATION_FACTOR 0.0001  // Self-explanatory

#define TIM4_ENCODER_PRESCALER 5 // prescaler for encoder routine call
#define TIM4_DAC_PRESCALER 2     // prescaler for dac increment/decrement routine call

#define LOGIC_OUTPUT1_TIMER_CHANNEL 3 // Timer channel used for LO1 associated with hardware pin (PB0 is T3C3)
#define LOGIC_OUTPUT2_TIMER_CHANNEL 4 // Timer channel used for LO2 associated with hardware pin (PB1 is T3C4)

#define I_SET_TIMER_CHANNEL 3 // Timer channel used for current adjustment associated with hardware pin (PB8 is T4C3)
#define V_SET_TIMER_CHANNEL 4 // Timer channel used for current adjustment associated with hardware pin (PB8 is T4C4)

#ifdef PIN_DEFINE

#define LED_R PB7
#define LED_G PB5
#define LED_B PB6
#define LED_PC13 PC13

#define ENC_CLK PB4
#define ENC_DT PB3
#define ENC_BTN PA15
#define ENC_STEPS 4

#define LCD_SDA PB11
#define LCD_SCL PB10

#define BUZZER_PIN PB14

#define LD_BTN PA13
#define LD_EN PA14

#define LOGIC_OUTPUT1 PB0
#define LOGIC_OUTPUT2 PB1

#define NTC1 PA2
#define NTC2 PA3

#define CC_IND PB12
#define CV_IND PB13

#define I_SET PB8
#define V_SET PB9
#endif

extern HardwareTimer *ledTimer; // used for indicatorControllerClass

extern int presetVoltageDAC, // Variable to store digital value (0~5000) of preset voltage
    presetCurrentDAC;        // Variable to store digital value (0~5000) of preset current
// SCREEN_MAIN VARIABLES
extern float sensedVoltage, // variable to store real sensed voltage on terminal, used on main and lcdController
    sensedCurrent,          // variable to store real sensed current flow to load, used on main and lcdController
    sensedPower,            // variable to store real power dissipated by load, used on main and lcdController
    presetVoltage,          // The preset value of real voltage
    presetCurrent,          // The preset value of real current
    bjtTemp;                // The measured temperature of the output BJT
extern bool ldStatus,       // Load status, whether it's ON or OFF
    opMode,                 // Current operation of Power supply which is CC or CV
    timerStatus;            // Timer status, whether it's ON or OFF
// SCREEN_MENU VARIABLES
extern uint32_t timerDuration; // variable to store timer duration that user intended, if value is 0 then timer is OFF
// SCREEN_ENERGY VARIABLES
extern float mWhTotal,       // Total energy used by load in mWh
    mAhTotal;                // Total energy used by load in mAh
extern uint32_t timeRunning; // variable to keep track total time running, recorded when ldStatus is ON and paused when ldStatus is OFF
// SCREEN_LOG VARIABLES
extern uint32_t logInterval; // The interval logging value spit out on UART port
extern bool logStatus;       // the status of logging, enabled or disable
// SCREEN_FAN VARIABLES
extern int8_t minPWMLO1, // EEPROM address 0. Stored on EEPROM on save, minimum PWM value for logic output 1
    maxPWMLO1,           // EEPROM address 1. Stored on EEPROM on save, maximum PWM value for logic output 1
    minPWMLO2,           // EEPROM address 2. Stored on EEPROM on save, minimum PWM value for logic output 2
    maxPWMLO2;           // EEPROM address 3. Stored on EEPROM on save, maximum PWM value for logic output 2
extern float maxTemp,    // EEPROM address 4. Stored on EEPROM on save, maximum temperature for logic output (logic output will put out maximum PWM value if temperature is higher or equal maxTemp)
    minTemp;             // EEPROM address 8. Stored on EEPROM on save, minimum temperature for logic output (logic output will put out minimum PWM value if temperature is lower or equal to minTemp)
// SCREEN_CAL VARIABLES
extern float presetVoltageFactor, // EEPROM address 12. Stored on EEPROM on save, preset voltage will be multiplied by this factor before shown on LCD
    presetCurrentFactor,          // EEPROM address 16. Stored on EEPROM on save, preset current will be multiplied by this factor before shown on LCD
    sensedCurrentFactor,          // EEPROM address 20. Stored on EEPROM on save, sensed voltage will be multiplied by this factor before shown on LCD
    sensedVoltageFactor;          // EEPROM address 24. Stored on EEPROM on save, sensed voltage will be multiplied by this factor before shown on LCD