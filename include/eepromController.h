#ifndef EEPROMCONTROLLER_H
#define EEPROMCONTROLLER_H
#include <Eeprom24C04_16.h>
#include <globals.h>

#define EEPROM_ADDRESS 0x50
#define EEPROM_TOTAL_ADDRESS 30

class eepromControllerClass
{
private:
    Eeprom24C04_16 *eep;

    // Variables to store last fetched value from EEPROM
    int8_t lastMinPWMLO1,                // EEPROM address 0.
        lastMaxPWMLO1,                   // EEPROM address 1.
        lastMinPWMLO2,                   // EEPROM address 2.
        lastMaxPWMLO2;                   // EEPROM address 3.
    float lastMaxTemp,                   // EEPROM address 4.
        lastMinTemp;                     // EEPROM address 8.
    float lastPresetVoltageFactor, // EEPROM address 12.
        lastPresetCurrentFactor,   // EEPROM address 16.
        lastSensedCurrentFactor,   // EEPROM address 20.
        lastSensedVoltageFactor;   // EEPROM address 24.
    byte outputBuffer[EEPROM_TOTAL_ADDRESS];
public:
    eepromControllerClass();
    void begin();
    void fetch();
    void update();
};

extern eepromControllerClass eeprom;
#endif