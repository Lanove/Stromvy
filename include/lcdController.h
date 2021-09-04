#ifndef LCDCONTROLLER_H
#define LCDCONTROLLER_H
#include <LiquidCrystal_I2C.h>
#include <ClickEncoder.h>
#include <globals.h>

#define LCD_REFRESH_INTERVAL 200 // LCD refresh rate in ms
#define LCDI2C_ADDRESS 0x27
#define LCD_ROWS 20
#define LCD_COLS 4

#define ARROW_BLINK_INTERVAL 500
#define ARROW_SYMBOL (char)B01111110
#define DEGREE_SYMBOL (char)223
#define BLOCK_SYMBOL (char)0xFF

#define LCD_SCREEN_COUNT 7
#define LCD_HIGHEST_CURSOR_COUNT 8

typedef enum
{
    SCREEN_MAIN,
    SCREEN_MENU,
    SCREEN_ENERGY,
    SCREEN_LOG,
    SCREEN_FAN,
    SCREEN_CAL,
    SCREEN_DEBUG
} LCD_SCREEN;

class lcdControllerClass
{
private:
    LiquidCrystal_I2C *clcd; // Variable to store the lcd object
    uint32_t refreshMillis;  // Variable to store the milis for lcd refresh
    uint32_t blinkMillis;    // Variable to store the millis for cursor blink
    LCD_SCREEN screen;       // Variable to store the lcd screen position
    int8_t cursor;           // Variable to store the cursor position
    bool cursorBlink;        // Variable to store whether cursor should be blinked or not
    bool cursorBlinkFlag;    // Variable to store the display condition of arrow (1 == arrow displayed), used to blink the arrow of cursor

    const int8_t cursorCoordinate[LCD_SCREEN_COUNT][LCD_HIGHEST_CURSOR_COUNT][2] = { // Variable to store the coordinate of each cursor position on each screen
        {                                                                            // SCREEN_MAIN
         {8, 1},
         {8, 2},
         {8, 3}},
        {// SCREEN_MENU
         {0, 0},
         {0, 1},
         {0, 2},
         {0, 3},
         {16, 0},
         {16, 1},
         {16, 2},
         {16, 3}},
        {// SCREEN_ENERGY
         {0, 2},
         {0, 3}},
        {// SCREEN_LOG
         {0, 1},
         {0, 2},
         {0, 3}},
        {// SCREEN_FAN
         {0, 0},
         {0, 1},
         {0, 2},
         {0, 3},
         {10, 0},
         {10, 1},
         {12, 2}},
        {// SCREEN_CAL
         {0, 2},
         {0, 3},
         {8, 0},
         {8, 1},
         {8, 2},
         {8, 3},
         {15, 0}},
        {// SCREEN_DEBUG
         {15, 3}}};
    const int8_t cursorCount[LCD_SCREEN_COUNT] = {3, 8, 2, 3, 7, 7, 1}; // Variable to store maximum cursor count of each screen

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
    float lastPresetVoltageFactor,
        lastPresetCurrentFactor;
    float lastSensedVoltageFactor,
        lastSensedCurrentFactor;
    void drawCursor(int8_t position, bool display = true);

public:
    lcdControllerClass();
    ~lcdControllerClass();

    void begin();
    void service();

    void setScreen(LCD_SCREEN screen_);
    LCD_SCREEN getScreen() { return screen; }

    void incrementCursor();
    void decrementCursor();
    void setCursor(int8_t position);
    int getCursor() { return cursor; }

    void setArrowBlink(bool blink);
    bool getArrowBlink() { return cursorBlink; }
};

extern lcdControllerClass lcd;
#endif