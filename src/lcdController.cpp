
#include <lcdController.h>

lcdControllerClass::lcdControllerClass() {}

void lcdControllerClass::begin()
{
    clcd = new LiquidCrystal_I2C(LCDI2C_ADDRESS, LCD_ROWS, LCD_COLS);
    // initialize the LCD
    clcd->begin();
    // Turn on the blacklight
    clcd->backlight();
    setScreen(SCREEN_MAIN);
}

void lcdControllerClass::service()
{
}

void lcdControllerClass::setScreen(LCD_SCREEN screen)
{
    screen = screen;
    setCursor(0);
    setArrowBlink(false);
    clcd->clear();
    if (screen == SCREEN_MAIN)
    {
        clcd->setCursor(0, 0);
        clcd->printf("Preset %c Actual%c%s", BLOCK_SYMBOL, BLOCK_SYMBOL, (ldStatus == STATUS_ON) ? "ON" : "OFF");
        clcd->setCursor(0, 1);
        clcd->printf("%05.2fV %c %05.2fV%c%s", presetVoltage, BLOCK_SYMBOL, sensedVoltage, BLOCK_SYMBOL, (opMode == MODE_CC) ? "CC" : "CV");
        clcd->setCursor(0, 2);
        clcd->printf("%04.0fmA %c %04.0fmA%c%2.0f%cC", presetCurrent, BLOCK_SYMBOL, sensedCurrent, BLOCK_SYMBOL, bjtTemp, DEGREE_SYMBOL);
        clcd->setCursor(0, 3);
        if (sensedPower >= 10.00)
            clcd->printf("%05.2fW %c MENU  %c%s", sensedPower, BLOCK_SYMBOL, BLOCK_SYMBOL, (timerDuration == 0) ? "Toff" : "Ton");
        else
            clcd->printf("%05.3fW %c MENU  %c%s", sensedPower, BLOCK_SYMBOL, BLOCK_SYMBOL, (timerDuration == 0) ? "Toff" : "Ton");
    }
    else if (screen == SCREEN_MENU)
    {
        clcd->setCursor(0, 0);
        clcd->printf(" Main Screen     Fan");
        clcd->setCursor(0, 1);
        clcd->printf(" Energy Info     Cal");
        clcd->setCursor(0, 2);
        if (timerDuration == 0)
            clcd->printf(" Timer: Off      Rst");
        else
            clcd->printf(" Timer: %02d:%02d:%02d Rst", (int)timerDuration / 3600, (int)timerDuration % 3600 / 60, (int)timerDuration % 60);
        clcd->setCursor(0, 3);
        clcd->printf(" Data Logging");
    }
    else if (screen == SCREEN_ENERGY)
    {
        clcd->setCursor(0, 0);
        clcd->printf("%08.0fmWh %02d:%02d:%02d", mWhTotal, (int)timeRunning / 3600, (int)timeRunning % 3600 / 60, (int)timeRunning % 60);
        clcd->setCursor(0, 1);
        clcd->printf("%08.0fmAh", mAhTotal);
        clcd->setCursor(0, 2);
        clcd->printf(" Reset Record");
        clcd->setCursor(0, 3);
        clcd->printf(" Back");
    }
    else if (screen == SCREEN_LOG)
    {
        clcd->setCursor(0, 0);
        clcd->printf("DATA LOGGING (UART)");
        clcd->setCursor(0, 1);
        clcd->printf(" %s", (logStatus) ? "Enabled" : "Disabled");
        clcd->setCursor(0, 2);
        clcd->printf(" Interval:%5dms", logInterval);
        clcd->setCursor(0, 3);
        clcd->printf(" Back");
    }
    else if (screen == SCREEN_FAN)
    {
        clcd->setCursor(0, 0);
        clcd->printf(" MinP1:%2d%% MaxT:%2f%cC", minPWMLO1, maxTemp, DEGREE_SYMBOL);
        clcd->setCursor(0, 1);
        clcd->printf(" MinP2:%2d%% MinT:%2f%cC", minPWMLO2, minTemp, DEGREE_SYMBOL);
        clcd->setCursor(0, 2);
        if (maxPWMLO1 == 100)
            clcd->printf(" MaxP1:%3d%%  Back", maxPWMLO1);
        else
            clcd->printf(" MaxP1:%3d%%   Back", maxPWMLO1);
        clcd->setCursor(0, 3);
        clcd->printf(" MaxP2:%2d%%", maxPWMLO2);
    }
    else if (screen == SCREEN_CAL)
    {
        clcd->setCursor(0, 0);
        clcd->printf("%05.2fV   %05.2fV Back", sensedVoltage, presetVoltage);
        clcd->setCursor(0, 1);
        clcd->printf("%04.0fmA   %04.0fmA", minPWMLO2, minTemp, DEGREE_SYMBOL);
        clcd->setCursor(0, 2);
        clcd->printf("");
        clcd->setCursor(0, 3);
        clcd->printf("");
    }
}

void lcdControllerClass::setCursor(int cursor)
{
    cursor = cursor;
}

void lcdControllerClass::setArrowBlink(bool blink)
{
    cursorBlink = blink;
}