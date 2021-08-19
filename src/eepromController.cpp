#include <eepromController.h>

eepromControllerClass::eepromControllerClass() {}
void eepromControllerClass::begin()
{
    eep = new Eeprom24C04_16(EEPROM_ADDRESS);
    eep->initialize();
}
void eepromControllerClass::fetch()
{
    eep->readBytes(0, EEPROM_TOTAL_ADDRESS, outputBuffer);
    memcpy(&minPWMLO1, &outputBuffer[0], sizeof(int8_t));
    memcpy(&maxPWMLO1, &outputBuffer[1], sizeof(int8_t));
    memcpy(&minPWMLO2, &outputBuffer[2], sizeof(int8_t));
    memcpy(&maxPWMLO2, &outputBuffer[3], sizeof(int8_t));
    memcpy(&maxTemp, &outputBuffer[4], sizeof(float));
    memcpy(&minTemp, &outputBuffer[8], sizeof(float));
    memcpy(&presetVoltageFactor, &outputBuffer[12], sizeof(float));
    memcpy(&presetCurrentFactor, &outputBuffer[16], sizeof(float));
    memcpy(&sensedVoltageFactor, &outputBuffer[20], sizeof(float));
    memcpy(&sensedCurrentFactor, &outputBuffer[24], sizeof(float));
    lastMinPWMLO1 = minPWMLO1;
    lastMaxPWMLO1 = maxPWMLO1;
    lastMinPWMLO2 = minPWMLO2;
    lastMaxPWMLO2 = maxPWMLO2;
    lastMaxTemp = maxTemp;
    lastMinTemp = minTemp;
    lastPresetVoltageFactor = presetVoltageFactor;
    lastPresetCurrentFactor = presetCurrentFactor;
    lastSensedCurrentFactor = sensedVoltageFactor;
    lastSensedVoltageFactor = sensedVoltageFactor;
    Serial.println("readBytes;");
}

void eepromControllerClass::update()
{
    if (lastMinPWMLO1 != minPWMLO1 ||
        lastMaxPWMLO1 != maxPWMLO1 ||
        lastMinPWMLO2 != minPWMLO2 ||
        lastMaxPWMLO2 != maxPWMLO2 ||
        lastMaxTemp != maxTemp ||
        lastMinTemp != minTemp ||
        lastPresetVoltageFactor != presetVoltageFactor ||
        lastPresetCurrentFactor != presetCurrentFactor ||
        lastSensedCurrentFactor != sensedVoltageFactor ||
        lastSensedVoltageFactor != sensedVoltageFactor)
    {
        memcpy(&outputBuffer[0], &minPWMLO1, sizeof(int8_t));
        memcpy(&outputBuffer[1], &maxPWMLO1, sizeof(int8_t));
        memcpy(&outputBuffer[2], &minPWMLO2, sizeof(int8_t));
        memcpy(&outputBuffer[3], &maxPWMLO2, sizeof(int8_t));
        memcpy(&outputBuffer[4], &maxTemp, sizeof(float));
        memcpy(&outputBuffer[8], &minTemp, sizeof(float));
        memcpy(&outputBuffer[12], &presetVoltageFactor, sizeof(float));
        memcpy(&outputBuffer[16], &presetCurrentFactor, sizeof(float));
        memcpy(&outputBuffer[20], &sensedVoltageFactor, sizeof(float));
        memcpy(&outputBuffer[24], &sensedCurrentFactor, sizeof(float));
        eep->writeBytes(0, EEPROM_TOTAL_ADDRESS, outputBuffer);
        Serial.println("writeBytes;");
        fetch();
    }
}
