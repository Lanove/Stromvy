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

#define EEPROM_ADDRESS 0x50

#ifdef PIN_DEFINE
#define MAX_V_ADC 4095
#define MAX_I_ADC 4095
#define MIN_FAN_PWM 0
#define MAX_FAN_PWM 100
#define MIN_FAN_TEMP 0
#define MAX_FAN_TEMP 125
#define MAX_CALIBRATION_FACTOR 10.0000
#define MIN_CALIBRATION_FACTOR 0.0001

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

extern float sensedVoltage, // variable to store real sensed voltage on terminal, used on main and lcdController
    sensedCurrent,          // variable to store real sensed current flow to load, used on main and lcdController
    sensedPower,            // variable to store real power dissipated by load, used on main and lcdController
    mWhTotal,               // variable to keep track total watthour has been used, used on main and lcdController
    mAhTotal,               // variable to keep track total amphour has been used,used on main and lcdController
    presetValue;            // the preset value of the load whether it's current/voltage/power or resistance, used on main and lcdController
extern uint8_t opMode,      // current operation of the load(SB/CC/CV/CP/CR/ER/PD/CU/TN), used on main and lcdController
    ldMode;                 // intended operation of the load (CC/CV/CP/CR), used on main and lcdController
extern float fetTemperature;
extern float CULV,          // Low voltage cut-off value, load operation will set to CU if voltage goes below this value, CUV_MIN is ignored
             CUHV,          // High voltage cut-off value, load operation will set to CU if voltage goes above this value, CUV_MAX or above is ignored
             CULA,          // Low current cut-off value, load operation will set to CU if current goes below this value, CUA_MIN is ignored
             CUHA,          // High current cut-off value, load operation will set to CU if current goes above this value, CUA_MAX or above is ignored
             CULP,          // Low power cut-off value, load operation will set to CU if power goes below this value, CUP_MIN is ignored
             CUHP;          // High power cut-off value, load operation will set to CU if power goes above this value, CUP_MAX or above is ignored
extern float calibrationVoltageFactor, // Stored on EEPROM on save, sensed voltage will be multiplied by this factor before shown on LCD
             calibrationCurrentFactor; // Stored on EEPROM on save, sensed current will be multiplied by this factor before shown on LCD
extern float overTempPoint, // Stored on EEPROM on save, if the temperature of FET goes above this, system will set operation mode to ERROR
             fanTempPoint, // Stored on EEPROM on save, if the temperature of FET goes above this, fan will be turned on
             fanTempHysteresis; // Stored on EEPROM on save, if the temperature goes below fanTempPoint-fanTempHysteresis fan will be turned off
extern uint32_t timeRunning; // variable to keep track total time running, used on main and lcdController
extern uint32_t timerDuration; // variable to store timer duration that user intended, if value is 0 then no timer