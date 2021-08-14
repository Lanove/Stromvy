#include <Arduino.h>
#include <encoderController.h>
#include <lcdController.h>
#include <Wire.h>
#include <ClickEncoder.h>
#include <Eeprom24C04_16.h>
#include "stm32f1xx_hal.h"
#include <RunningAverage.h>
#include "ADS1X15.h"

#define PIN_DEFINE

#define MODE_CC 0
#define MODE_CV 1

#define STATUS_ON 1
#define STATUS_OFF 0

#define EEPROM_ADDRESS 0x50

#define MAX_V_ADC 4095
#define MAX_I_ADC 4095
#define MIN_FAN_PWM 0
#define MAX_FAN_PWM 100
#define MIN_FAN_TEMP 0
#define MAX_FAN_TEMP 125
#define MAX_CALIBRATION_FACTOR 10.0000
#define MIN_CALIBRATION_FACTOR 0.0001

#ifdef PIN_DEFINE

#define LED_LDON PB7
#define LED_ST2 PB6
#define LED_ST1 PB5 //problem
#define LED_PC13 PC13

#define ENC_CLK PB4
#define ENC_DT PB3
#define ENC_BTN PA15
#define ENC_STEPS 4

#define LCD_SDA PB11
#define LCD_SCL PB10

#define BUZZER PB14

#define LD_BTN PA13
#define LD_EN PA14

#define LOGIC_OUTPUT1 PB0
#define LOGIC_OUTPUT2 PB1

#define LOGIC_OUTPUT1_TIMER_CHANNEL 3
#define LOGIC_OUTPUT2_TIMER_CHANNEL 4

#define NTC1 PA2
#define NTC2 PA3

#define NTC1_ADC_CHANNEL ADC_CHANNEL_2
#define NTC2_ADC_CHANNEL ADC_CHANNEL_3

#define CC_IND PB12
#define CV_IND PB13

#define I_SET PB8
#define V_SET PB9

#define I_SET_TIMER_CHANNEL 3
#define V_SET_TIMER_CHANNEL 4
#endif

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
extern uint8_t minPWMLO1, // Stored on EEPROM on save, minimum PWM value for logic output 1
    maxPWMLO1,            // Stored on EEPROM on save, maximum PWM value for logic output 1
    minPWMLO2,            // Stored on EEPROM on save, minimum PWM value for logic output 2
    maxPWMLO2;            // Stored on EEPROM on save, maximum PWM value for logic output 2
extern float maxTemp,     // Stored on EEPROM on save, maximum temperature for logic output (logic output will put out maximum PWM value if temperature is higher or equal maxTemp)
    minTemp;              // Stored on EEPROM on save, minimum temperature for logic output (logic output will put out minimum PWM value if temperature is lower or equal to minTemp)
// SCREEN_CAL VARIABLES
extern float presetVoltageFactor, // Stored on EEPROM on save, preset voltage will be multiplied by this factor before shown on LCD
    presetCurrentFactor,          // Stored on EEPROM on save, preset current will be multiplied by this factor before shown on LCD
    sensedCurrentFactor,          // Stored on EEPROM on save, sensed voltage will be multiplied by this factor before shown on LCD
    sensedVoltageFactor;          // Stored on EEPROM on save, sensed voltage will be multiplied by this factor before shown on LCD