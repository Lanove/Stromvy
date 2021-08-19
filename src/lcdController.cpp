
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
    if (millis() - refreshMillis >= LCD_REFRESH_INTERVAL)
    {
        refreshMillis = millis();
        if (screen == SCREEN_MAIN)
        {
            if (sensedVoltage != lastSensedVoltage)
            {
                clcd->setCursor(0, 1);
                clcd->printf("%05.2fV", sensedVoltage);
            }
            if (sensedCurrent != lastSensedCurrent)
            {
                clcd->setCursor(0, 2);
                clcd->printf("%04.0fmA", sensedCurrent);
            }
            if (sensedPower != lastSensedPower)
            {
                clcd->setCursor(0, 3);
                if (sensedPower >= 10.00)
                    clcd->printf("%05.2fW", sensedPower);
                else
                    clcd->printf("%05.3fW", sensedPower);
            }
            if (presetVoltage != lastPresetVoltage)
            {
                clcd->setCursor(9, 1);
                clcd->printf("%05.2fV", presetVoltage);
            }
            if (presetCurrent != lastPresetCurrent)
            {
                clcd->setCursor(9, 2);
                clcd->printf("%04.0fmA", presetCurrent);
            }
            if (ldStatus != lastLdStatus)
            {
                clcd->setCursor(16, 0);
                clcd->printf("%s", (ldStatus == STATUS_ON) ? "ON " : "OFF");
            }
            if (opMode != lastOpMode)
            {
                clcd->setCursor(16, 1);
                clcd->printf("%s", (opMode == MODE_CC) ? "CC" : "CV");
            }
            if (bjtTemp != lastBjtTemp)
            {
                clcd->setCursor(16, 2);
                clcd->printf("%2.0f%cC", bjtTemp, DEGREE_SYMBOL);
            }
            if (timerDuration != lastTimerDuration)
            {
                clcd->setCursor(16, 3);
                clcd->printf("%s", (timerDuration == 0) ? "Toff" : "Ton ");
            }
        }
        else if (screen == SCREEN_MENU)
        {
            if (timerDuration != lastTimerDuration)
            {
                clcd->setCursor(8, 2);
                if (timerDuration == 0)
                    clcd->printf("Off     ");
                else
                    clcd->printf("%02d:%02d:%02d", (int)timerDuration / 3600, (int)timerDuration % 3600 / 60, (int)timerDuration % 60);
            }
        }
        else if (screen == SCREEN_ENERGY)
        {
            if (mWhTotal != lastMWhTotal)
            {
                clcd->setCursor(0, 0);
                clcd->printf("%8.0fmWh", mWhTotal);
            }
            if (mAhTotal != lastMAhTotal)
            {
                clcd->setCursor(0, 1);
                clcd->printf("%8.0fmAh", mAhTotal);
            }
            if (timeRunning != lastTimeRunning)
            {
                clcd->setCursor(12, 0);
                clcd->printf("%02d:%02d:%02d", (int)timeRunning / 3600, (int)timeRunning % 3600 / 60, (int)timeRunning % 60);
            }
        }
        else if (screen == SCREEN_LOG)
        {
            if (logStatus != lastLogStatus)
            {
                clcd->setCursor(1, 1);
                clcd->printf("%s", (logStatus) ? "Enabled " : "Disabled");
            }
            if (logInterval != lastLogInterval)
            {
                clcd->setCursor(10, 2);
                clcd->printf("%5dms", logInterval);
            }
        }
        else if (screen == SCREEN_FAN)
        {
            if (minPWMLO1 != lastMinPWMLO1)
            {
                clcd->setCursor(7, 0);
                clcd->printf("%2d%% ", minPWMLO1);
            }
            if (minPWMLO2 != lastMinPWMLO2)
            {
                clcd->setCursor(7, 1);
                clcd->printf("%2d%% ", minPWMLO2);
            }
            if (maxPWMLO1 != lastMaxPWMLO1)
            {
                clcd->setCursor(7, 2);
                clcd->printf("%2d%% ", maxPWMLO1);
            }
            if (maxPWMLO2 != lastMaxPWMLO2)
            {
                clcd->setCursor(7, 3);
                clcd->printf("%2d%% ", maxPWMLO2);
            }
            if (maxTemp != lastMaxTemp)
            {
                clcd->setCursor(16, 0);
                clcd->printf("%2.0f%cC", maxTemp, DEGREE_SYMBOL);
            }
            if (minTemp != lastMinTemp)
            {
                clcd->setCursor(16, 1);
                clcd->printf("%2.0f%cC", minTemp, DEGREE_SYMBOL);
            }
        }else if(screen == SCREEN_CAL){
            if(sensedVoltageFactor != lastSensedVoltageFactor){
                clcd->setCursor(1, 2);
                clcd->printf("%6.4f", sensedVoltageFactor);
            }
            if(sensedCurrentFactor != lastSensedCurrentFactor){
                clcd->setCursor(1, 3);
                clcd->printf("%6.4f", sensedCurrentFactor);
            }
            if(presetVoltageFactor != lastPresetVoltageFactor){
                clcd->setCursor(9, 2);
                clcd->printf("%6.4f", presetVoltageFactor);
            }
            if(presetCurrentFactor != lastPresetCurrentFactor){
                clcd->setCursor(9, 3);
                clcd->printf("%6.4f", presetCurrentFactor);
            }
            if (presetVoltage != lastPresetVoltage)
            {
                clcd->setCursor(9, 0);
                clcd->printf("%05.2fV", presetVoltage);
            }
            if (presetCurrent != lastPresetCurrent)
            {
                clcd->setCursor(9, 1);
                clcd->printf("%04.0fmA", presetCurrent);
            }
            if (sensedVoltage != lastSensedVoltage)
            {
                clcd->setCursor(0, 0);
                clcd->printf("%05.2fV", sensedVoltage);
            }
            if (sensedCurrent != lastSensedCurrent)
            {
                clcd->setCursor(0, 1);
                clcd->printf("%04.0fmA",sensedCurrent);
            }
        }

        lastCursorBlink = cursorBlink;
        lastCursor = cursor;
        lastSensedVoltage = sensedVoltage;
        lastSensedCurrent = sensedCurrent;
        lastSensedPower = sensedPower;
        lastPresetVoltage = presetVoltage;
        lastPresetCurrent = presetCurrent;
        lastBjtTemp = bjtTemp;
        lastLdStatus = ldStatus;
        lastOpMode = opMode;
        lastTimerStatus = timerStatus;
        lastTimerDuration = timerDuration;
        lastMWhTotal = mWhTotal;
        lastMAhTotal = mAhTotal;
        lastTimeRunning = timeRunning;
        lastLogInterval = logInterval;
        lastLogStatus = logStatus;
        lastMinPWMLO1 = minPWMLO1;
        lastMaxPWMLO1 = maxPWMLO1;
        lastMinPWMLO2 = minPWMLO2;
        lastMaxPWMLO2 = maxPWMLO2;
        lastMaxTemp = maxTemp;
        lastMinTemp = minTemp;
        lastPresetVoltageFactor = presetVoltageFactor;
        lastPresetCurrentFactor = presetCurrentFactor;
        lastSensedVoltageFactor = sensedVoltageFactor;
        lastSensedCurrentFactor = sensedCurrentFactor;
    }
    if (millis() - blinkMillis >= ARROW_BLINK_INTERVAL && cursorBlink)
    {
        blinkMillis = millis();
        cursorBlinkFlag = !cursorBlinkFlag; // Invert the blink flag every interval
        clcd->setCursor(cursorCoordinate[screen][cursor][0], cursorCoordinate[screen][cursor][1]);
        drawCursor(cursor, cursorBlinkFlag);
    }
}

void lcdControllerClass::setScreen(LCD_SCREEN screen_)
{
    screen = screen_;
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
        clcd->printf("%8.0fmWh %02d:%02d:%02d", mWhTotal, (int)timeRunning / 3600, (int)timeRunning % 3600 / 60, (int)timeRunning % 60);
        clcd->setCursor(0, 1);
        clcd->printf("%8.0fmAh", mAhTotal);
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
        clcd->printf(" MinP1:%2d%% MaxT:%2.0f%cC", minPWMLO1, maxTemp, DEGREE_SYMBOL);
        clcd->setCursor(0, 1);
        clcd->printf(" MinP2:%2d%% MinT:%2.0f%cC", minPWMLO2, minTemp, DEGREE_SYMBOL);
        clcd->setCursor(0, 2);
        if (maxPWMLO1 == 100)
            clcd->printf(" MaxP1:%3d%%  Back", maxPWMLO1);
        else
            clcd->printf(" MaxP1:%2d%%   Back", maxPWMLO1);
        clcd->setCursor(0, 3);
        clcd->printf(" MaxP2:%2d%%", maxPWMLO2);
    }
    else if (screen == SCREEN_CAL)
    {
        clcd->setCursor(0, 0);
        clcd->printf("%05.2fV   %05.2fV Back", sensedVoltage, presetVoltage);
        clcd->setCursor(0, 1);
        clcd->printf("%04.0fmA   %04.0fmA", sensedCurrent, presetCurrent);
        clcd->setCursor(0, 2);
        clcd->printf(" %06.4f  %06.4f", sensedVoltageFactor, presetVoltageFactor);
        clcd->setCursor(0, 3);
        clcd->printf(" %06.4f  %06.4f", sensedCurrentFactor, presetCurrentFactor);
    }
    setCursor(0);
    setArrowBlink(false);
}

void lcdControllerClass::drawCursor(int8_t position, bool display)
{
    if (cursor == position && display)
        clcd->print(ARROW_SYMBOL);
    else
        clcd->print(" ");
}

void lcdControllerClass::setCursor(int8_t position)
{
    cursor = position;
    blinkMillis = millis(); // reset blink millis
    cursorBlinkFlag = 1;    // Display the arrow on cursor
    // out of bound handler :
    if (cursor < 0)
        cursor = 0;
    else if (cursor > cursorCount[screen] - 1) // cursor starts at 0 while indexsize count from 1, so size-1
        cursor = cursorCount[screen] - 1;
    //
    // Clear all cursor printed and draw the arrow on new cursor position
    for (int i = 0; i < cursorCount[screen]; i++)
    {
        clcd->setCursor(cursorCoordinate[screen][i][0], cursorCoordinate[screen][i][1]);
        drawCursor(i);
    }
}

void lcdControllerClass::incrementCursor()
{
    setCursor(cursor + 1);
}

void lcdControllerClass::decrementCursor()
{
    setCursor(cursor - 1);
}

void lcdControllerClass::setArrowBlink(bool blink)
{
    cursorBlink = blink;
    if(!cursorBlink){ // Redraw the arrow after changing cursorBlink flag, because the arrow might disappear when flag changed whilst arrow is undisplayed
        clcd->setCursor(cursorCoordinate[screen][cursor][0], cursorCoordinate[screen][cursor][1]);
        drawCursor(cursor);
    }
}