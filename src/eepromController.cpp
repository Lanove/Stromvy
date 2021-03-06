#include <eepromController.h>

Eeprom24C04_16 _eeprom(EEPROM_ADDRESS);

void eepromControllerClass::begin()
{
    bool eepromUninitialized = true;
    _eeprom.initialize();
    _eeprom.readBytes(0, EEPROM_TOTAL_ADDRESS, outputBuffer.data());
    // if EEPROM is empty (all value is 0xFF) then set each used address bytes to 0x00
    if (count(outputBuffer.begin(), outputBuffer.end(), 0xFF) >= EEPROM_TOTAL_ADDRESS)
    {
        fill(outputBuffer.begin(), outputBuffer.end(), 0x00);
        _eeprom.writeBytes(0, EEPROM_TOTAL_ADDRESS, outputBuffer.data());
    }
    _eeprom.readBytes(100, sizeof(highestHeapUsage), buffer.data());
    if (count(buffer.begin(), buffer.end(), 0xFF) >= sizeof(highestHeapUsage))
    {
        fill(buffer.begin(), buffer.end(), 0x00);
        _eeprom.writeBytes(100, sizeof(highestHeapUsage), buffer.data());
    }
}

void eepromControllerClass::fetch()
{
    // Store to outputBuffer
    _eeprom.readBytes(0, EEPROM_TOTAL_ADDRESS, outputBuffer.data());

    // Copy binary values from EEPROM that is stored on outputBuffer to each associated variables
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
    // Update last fetched variables
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
}

void eepromControllerClass::update()
{
    // Only write to EEPROM when one or more variable below is changed
    if (lastMinPWMLO1 != minPWMLO1 ||
        lastMaxPWMLO1 != maxPWMLO1 ||
        lastMinPWMLO2 != minPWMLO2 ||
        lastMaxPWMLO2 != maxPWMLO2 ||
        lastMaxTemp != maxTemp ||
        lastMinTemp != minTemp ||
        lastPresetVoltageFactor != presetVoltageFactor ||
        lastPresetCurrentFactor != presetCurrentFactor ||
        lastSensedCurrentFactor != sensedCurrentFactor ||
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
        _eeprom.writeBytes(0, EEPROM_TOTAL_ADDRESS, outputBuffer.data());
        fetch();
    }
}

void eepromControllerClass::fetchMallinfo()
{
    size_t _size = sizeof(highestHeapUsage);
    _eeprom.readBytes(100, _size, buffer.data());
    memcpy(&highestHeapUsage, buffer.data(), _size);
    lastHighestHeapUsage = highestHeapUsage;
}

void eepromControllerClass::writeMallinfo()
{
    if (lastHighestHeapUsage != highestHeapUsage)
    {
        size_t _size = sizeof(highestHeapUsage);
        memcpy(buffer.data(), &highestHeapUsage, _size);
        _eeprom.writeBytes(100, _size, buffer.data());
        fetchMallinfo();
    }
}
