#ifndef LCDCONTROLLER_H
#define LCDCONTROLLER_H
#include <LiquidCrystal_I2C.h>
#include <ClickEncoder.h>
#include <globals.h>

#define LCD_REFRESH_RATE 333 // LCD refresh rate in ms
#define LCDI2C_ADDRESS 0x27
#define LCD_ROWS 20
#define LCD_COLS 4

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
    LiquidCrystal_I2C *lcd_;
    LCD_SCREEN screen;
    int cursor;
    bool cursorBlink;

    bool lastCursorBlink;
    int lastCursor;
public:
    lcdControllerClass();
    void begin();
    void service();

    void setScreen(LCD_SCREEN screen);
    LCD_SCREEN getScreen(){return screen;}

    void setCursor(int cursor);
    int getCursor(){return cursor;}

    void setArrowBlink(bool blink);
    bool getArrowBlink(){return cursorBlink;}
};

extern lcdControllerClass lcd;
#endif