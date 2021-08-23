#include "indicatorController.h"

HardwareTimer *ledTimer = new HardwareTimer(TIM2);

indicatorControllerClass::indicatorControllerClass()
{
}

void indicatorControllerClass::begin()
{
    ledTimer->setOverflow(20000, HERTZ_FORMAT); // in Hertz
    callback_function_t h = std::bind(&indicatorControllerClass::service, this);
    ledTimer->attachInterrupt(h);
    ledTimer->resume();
}

void indicatorControllerClass::service()
{
    unsigned long mil = millis();
    if (currentIndicator != STANDBY)
    {
        captureCompare++;
        if (captureCompare >= r)
            digitalWrite(LED_R, LOW);
        if (captureCompare >= g)
            digitalWrite(LED_G, LOW);
        if (captureCompare >= b)
            digitalWrite(LED_B, LOW);
        if (captureCompare == 255)
        {
            captureCompare = 0;
            digitalWrite(LED_R, HIGH);
            digitalWrite(LED_G, HIGH);
            digitalWrite(LED_B, HIGH);
        }
    }
    else
    {
        digitalWrite(LED_R, LOW);
        digitalWrite(LED_G, HIGH);
        digitalWrite(LED_B, LOW);
    }
    if (currentIndicator == RUNNING_CC || currentIndicator == RUNNING_CV || currentIndicator == PAUSED)
    {
        if (mil - ledMillis >= ledSineLoopPeriod)
        {
            ledMillis = mil;
            if (direction == POSITIVE)
                sine++;
            else
                sine--;
            if (sine == 0 || sine == 90)
                direction = !direction;
            float sineFactor = isin(sine);
            r = float(indicatorLookup[currentIndicator][R_POS]) * sineFactor,
            g = float(indicatorLookup[currentIndicator][G_POS]) * sineFactor,
            b = float(indicatorLookup[currentIndicator][B_POS]) * sineFactor;
            // Serial.printf("R%d;G%d;B%d\n",r,g,b);
        }
    }
    else if (currentIndicator == OVERHEAT_HALT || currentIndicator == TIMER_HALT)
    {
        if (mil - ledMillis >= 250)
        {
            ledMillis = mil;
            direction = !direction;
            r = (direction) ? 0 : indicatorLookup[currentIndicator][R_POS];
            g = (direction) ? 0 : indicatorLookup[currentIndicator][G_POS];
            b = (direction) ? 0 : indicatorLookup[currentIndicator][B_POS];
        }
    }
    else if (currentIndicator == STANDBY)
    {
        r = 0;
        g = 255;
        b = 0;
    }
    if (beepFlag)
    {
        if (mil - beepMillis >= ((beepDirection) ? beepTon : beepToff))
        {
            beepMillis = mil;
            beepDirection = !beepDirection;
            digitalWrite(BUZZER_PIN, beepDirection);
            if (!beepLoopFlag || (beepTotalTime != 0 && mil - firstBeepMillis >= beepTotalTime))
                stopBuzzer();
        }
    }
}

void indicatorControllerClass::setIndicator(INDICATE indicator)
{
    currentIndicator = indicator;
    captureCompare = 0;
    r = 0;
    g = 0;
    b = 0;
    sine = 0;
    direction = POSITIVE;
    ledMillis = millis();
}

INDICATE indicatorControllerClass::getIndicator()
{
    return currentIndicator;
}

void indicatorControllerClass::beepBuzzer(int ton, int toff, bool looping, int totalTime)
{
    stopBuzzer();
    beepTon = ton;
    beepToff = toff;
    beepLoopFlag = looping;
    beepFlag = true;
    beepDirection = POSITIVE;
    beepTotalTime = totalTime;
    beepMillis = millis();
    firstBeepMillis = millis();
    digitalWrite(BUZZER_PIN, beepDirection);
}

void indicatorControllerClass::stopBuzzer()
{
    beepFlag = false;
    digitalWrite(BUZZER_PIN, LOW);
}

int indicatorControllerClass::getBeepTon()
{
    return beepTon;
}
int indicatorControllerClass::getBeepToff()
{
    return beepToff;
}
int indicatorControllerClass::getBeepTotalTime()
{
    return beepTotalTime;
}
bool indicatorControllerClass::getBeepFlag()
{
    return beepFlag;
}
bool indicatorControllerClass::getBeepLoopFlag()
{
    return beepLoopFlag;
}
bool indicatorControllerClass::getBeepDirection()
{
    return beepDirection;
}

inline float indicatorControllerClass::isin(int x)
{
    boolean pos = true; // positive - keeps an eye on the sign.
    if (x < 0)
    {
        x = -x;
        pos = !pos;
    }
    if (x >= 360)
        x %= 360;
    if (x > 180)
    {
        x -= 180;
        pos = !pos;
    }
    if (x > 90)
        x = 180 - x;
    if (pos)
        return isinTable8[x] * 0.003921568627; // = /255.0
    return isinTable8[x] * -0.003921568627;
}