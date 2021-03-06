#include "encoderController.h"

ClickEncoder enc(ENC_CLK, ENC_DT, ENC_BTN, ENC_STEPS);

void encoderControllerClass::begin()
{
    enc.setAccelerationEnabled(false);
}

void encoderControllerClass::service()
{
    LCD_SCREEN screen = lcd.getScreen();
    uint8_t cursor = lcd.getCursor();
    bool blinkFlag = lcd.getArrowBlink();

    // Apply acceleration only when changing value of variable, not when navigating the interface
    enc.setAccelerationEnabled(blinkFlag);

    // Store variables
    encoderValue += enc.getValue();
    encoderDelta = encoderValue - oldEncoderValue;
    if (encoderValue != oldEncoderValue)
        isMoved = true;
    else
        isMoved = false;
    btnState = enc.getButton();

    if (btnState == ClickEncoder::Clicked) // Screen move navigations (consult to LCD User Interface.xlsx)
    {
        if (screen == SCREEN_MENU)
        {
            if (cursor != 2) // Beep buzzer if we are not on cursor 2 while clicking (because cursor 2 is timer value)
                indicator.beepBuzzer(50);

            // Move to screen x accordingly
            if (cursor == 0)
                lcd.setScreen(SCREEN_MAIN);
            else if (cursor == 1)
                lcd.setScreen(SCREEN_ENERGY);
            else if (cursor == 3)
                lcd.setScreen(SCREEN_LOG);
            else if (cursor == 4)
                lcd.setScreen(SCREEN_FAN);
            else if (cursor == 5)
                lcd.setScreen(SCREEN_CAL);
            else if (cursor == 7)
                lcd.setScreen(SCREEN_DEBUG);
        }
        // Back buttons to SCREEN_MENU
        if ((screen == SCREEN_MAIN && cursor == 2) ||
            (screen == SCREEN_ENERGY && cursor == 1) ||
            (screen == SCREEN_LOG && cursor == 2) ||
            (screen == SCREEN_ENERGY && cursor == 1) ||
            (screen == SCREEN_FAN && cursor == 6) ||
            (screen == SCREEN_CAL && cursor == 6) ||
            (screen == SCREEN_DEBUG && cursor == 0))
        {
            lcd.setScreen(SCREEN_MENU);
            indicator.beepBuzzer(50);
        }
    }
    else if (btnState == ClickEncoder::DoubleClicked)
    {
        eeprom.update(); // Try to update EEPROM every double click
        // cursor and screen position that is used for changing value of variables
        if ((screen == SCREEN_MAIN && (cursor == 0 || cursor == 1)) ||
            (screen == SCREEN_MENU && cursor == 2) ||
            (screen == SCREEN_LOG && cursor == 1) ||
            (screen == SCREEN_FAN && cursor != 6) ||
            (screen == SCREEN_CAL && cursor != 6))
        {
            lcd.setArrowBlink(!blinkFlag);
            indicator.beepBuzzer(100);
        }
        // Double clicking on following screen and cursor will reset the microcontroller
        else if (screen == SCREEN_MENU && cursor == 6)
            NVIC_SystemReset();
        // Double clicking on following screen and cursor will reset the energy and timeRunning values to 0
        else if (screen == SCREEN_ENERGY && cursor == 0)
        {
            mWhTotal = 0;
            mAhTotal = 0;
            timeRunning = 0;
            indicator.setIndicator(STANDBY);
            indicator.beepBuzzer(100);
        }
        // Double clicking on following screen and cursor will invert logStatus value
        else if (screen == SCREEN_LOG && cursor == 0)
        {
            logStatus = !logStatus;
            indicator.beepBuzzer(100);
        }
    }

    // Move the cursor according to direction when we are on navigation mode
    if (encoderDelta > 0 && !blinkFlag)
        lcd.incrementCursor();
    else if (encoderDelta < 0 && !blinkFlag)
        lcd.decrementCursor();
    // Change variable values when encoder is moved whilst on blinkFlag (consult to LCD User Interface.xlsx)
    else if (encoderDelta != 0 && blinkFlag)
    {
        if (screen == SCREEN_MAIN)
        {
            if (cursor == 0)
                presetVoltageDAC = presetVoltageDAC + encoderDelta;
            else if (cursor == 1)
                presetCurrentDAC = presetCurrentDAC + encoderDelta;
        }
        else if (screen == SCREEN_MENU && cursor == 2)
            timerDuration = timerDuration + encoderDelta;
        else if (screen == SCREEN_LOG && cursor == 1)
            logInterval = logInterval + encoderDelta;
        else if (screen == SCREEN_FAN)
        {
            if (cursor == 0)
                minPWMLO1 = minPWMLO1 + encoderDelta;
            else if (cursor == 1)
                minPWMLO2 = minPWMLO2 + encoderDelta;
            else if (cursor == 2)
                maxPWMLO1 = maxPWMLO1 + encoderDelta;
            else if (cursor == 3)
                maxPWMLO2 = maxPWMLO2 + encoderDelta;
            else if (cursor == 4)
                maxTemp = maxTemp + float(encoderDelta);
            else if (cursor == 5)
                minTemp = minTemp + float(encoderDelta);
        }
        else if (screen == SCREEN_CAL)
        {
            if (cursor == 0)
                sensedVoltageFactor = sensedVoltageFactor + (float(encoderDelta) * 0.0001);
            else if (cursor == 1)
                sensedCurrentFactor = sensedCurrentFactor + (float(encoderDelta) * 0.0001);
            else if (cursor == 2)
                presetVoltageDAC = presetVoltageDAC + encoderDelta;
            else if (cursor == 3)
                presetCurrentDAC = presetCurrentDAC + encoderDelta;
            else if (cursor == 4)
                presetVoltageFactor = presetVoltageFactor + (float(encoderDelta) * 0.0001);
            else if (cursor == 5)
                presetCurrentFactor = presetCurrentFactor + (float(encoderDelta) * 0.0001);
        }
    }

    // Limits
    if (presetVoltageDAC < MIN_V_DAC)
        presetVoltageDAC = MIN_V_DAC;
    else if (presetVoltageDAC > MAX_V_DAC)
        presetVoltageDAC = MAX_V_DAC;
    if (presetCurrentDAC < MIN_I_DAC)
        presetCurrentDAC = MIN_I_DAC;
    else if (presetCurrentDAC > MAX_I_DAC)
        presetCurrentDAC = MAX_I_DAC;
    if (timerDuration > MAX_TIMER_DURATION)
        timerDuration = MIN_TIMER_DURATION;
    if (logInterval < MIN_LOG_INTERVAL)
        logInterval = MIN_LOG_INTERVAL;
    else if (logInterval > MAX_LOG_INTERVAL)
        logInterval = MAX_LOG_INTERVAL;
    if (minPWMLO1 < MIN_FAN_PWM - 1)
        minPWMLO1 = MIN_FAN_PWM - 1;
    else if (minPWMLO1 > MAX_FAN_PWM)
        minPWMLO1 = MAX_FAN_PWM;
    if (minPWMLO2 < MIN_FAN_PWM - 1)
        minPWMLO2 = MIN_FAN_PWM - 1;
    else if (minPWMLO2 > MAX_FAN_PWM)
        minPWMLO2 = MAX_FAN_PWM;
    if (maxPWMLO1 < MIN_FAN_PWM)
        maxPWMLO1 = MIN_FAN_PWM;
    else if (maxPWMLO1 > MAX_FAN_PWM)
        maxPWMLO1 = MAX_FAN_PWM;
    if (maxPWMLO2 < MIN_FAN_PWM)
        maxPWMLO2 = MIN_FAN_PWM;
    else if (maxPWMLO2 > MAX_FAN_PWM)
        maxPWMLO2 = MAX_FAN_PWM;
    if (minTemp < MIN_FAN_TEMP)
        minTemp = MIN_FAN_TEMP;
    else if (minTemp > MAX_FAN_TEMP)
        minTemp = MAX_FAN_TEMP;
    if (maxTemp < MIN_FAN_TEMP)
        maxTemp = MIN_FAN_TEMP;
    else if (maxTemp > MAX_FAN_TEMP)
        maxTemp = MAX_FAN_TEMP;
    if (presetVoltageFactor < MIN_CALIBRATION_FACTOR)
        presetVoltageFactor = MIN_CALIBRATION_FACTOR;
    else if (presetVoltageFactor > MAX_CALIBRATION_FACTOR)
        presetVoltageFactor = MAX_CALIBRATION_FACTOR;
    if (presetCurrentFactor < MIN_CALIBRATION_FACTOR)
        presetCurrentFactor = MIN_CALIBRATION_FACTOR;
    else if (presetCurrentFactor > MAX_CALIBRATION_FACTOR)
        presetCurrentFactor = MAX_CALIBRATION_FACTOR;
    if (sensedVoltageFactor < MIN_CALIBRATION_FACTOR)
        sensedVoltageFactor = MIN_CALIBRATION_FACTOR;
    else if (sensedVoltageFactor > MAX_CALIBRATION_FACTOR)
        sensedVoltageFactor = MAX_CALIBRATION_FACTOR;
    if (sensedCurrentFactor < MIN_CALIBRATION_FACTOR)
        sensedCurrentFactor = MIN_CALIBRATION_FACTOR;
    else if (sensedCurrentFactor > MAX_CALIBRATION_FACTOR)
        sensedCurrentFactor = MAX_CALIBRATION_FACTOR;

    oldEncoderValue = encoderValue;
}

bool encoderControllerClass::isEncoderMoved()
{
    return isMoved;
}
int encoderControllerClass::getEncoderDelta()
{
    return encoderDelta;
}
int encoderControllerClass::getEncoderValue()
{
    return encoderValue;
}
ClickEncoder::Button encoderControllerClass::getButtonState()
{
    return btnState;
}