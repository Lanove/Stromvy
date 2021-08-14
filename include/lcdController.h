#ifndef LCDCONTROLLER_H
#define LCDCONTROLLER_H
#include <LiquidCrystal_I2C.h>
#include <ClickEncoder.h>
#include <globals.h>

#define LCD_REFRESH_RATE 333 // LCD refresh rate in ms
#define LCDI2C_ADDRESS 0x27
#define LCD_ROWS 20
#define LCD_COLS 4

#define ARROW_SYMBOL (char)B01111110
#define DEGREE_SYMBOL (char)223
#define BLOCK_SYMBOL (char)0xFF

typedef enum
{
    SCREEN_MAIN,
    SCREEN_MENU,
    SCREEN_ENERGY,
    SCREEN_LOG,
    SCREEN_FAN,
    SCREEN_CAL
} LCD_SCREEN;

class lcdControllerClass
{
private:
    uint32_t refreshMillis;
    uint32_t blinkMillis;
    LiquidCrystal_I2C *clcd;
    LCD_SCREEN screen;
    int cursor;
    bool cursorBlink;

    // Variables to keep last value of x to be used on lcd refresh (only display the changed value on LCD)
    bool lastCursorBlink;
    int lastCursor;

    float lastSensedVoltage,
        lastSensedCurrent,
        lastSensedPower,
        lastPresetVoltage,
        lastPresetCurrent,
        lastBjtTemp;
    bool lastLdStatus,
        lastOpMode,
        lastTimerStatus;
    uint32_t lastTimerDuration;
    float lastMWhTotal,
        lastMAhTotal;
    uint32_t lastTimeRunning;
    uint32_t lastLogInterval;
    bool lastLogStatus;
    uint8_t lastMinPWMLO1,
        lastMaxPWMLO1,
        lastMinPWMLO2,
        lastMaxPWMLO2;
    float lastMaxTemp,
        lastMinTemp;
    float lastCalibrationVoltageFactor,
        lastCalibrationCurrentFactor;

public:
    lcdControllerClass();
    void begin();
    void service();

    void setScreen(LCD_SCREEN screen);
    LCD_SCREEN getScreen() { return screen; }

    void setCursor(int cursor);
    int getCursor() { return cursor; }

    void setArrowBlink(bool blink);
    bool getArrowBlink() { return cursorBlink; }
};

extern lcdControllerClass lcd;
#endif