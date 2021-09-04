#ifndef EEPROMCONTROLLER_H
#define EEPROMCONTROLLER_H
#include <Eeprom24C04_16.h>
#include <globals.h>

#define EEPROM_TOTAL_ADDRESS 30 // USED address memory, not EEPROM real total size
#define EEPROM_ADDRESS 0x50     // I2C Address

class eepromControllerClass
{
private:

    // Variables to store last fetched value from EEPROM, so that update() only writes to EEPROM when there is change on below variables
    int8_t lastMinPWMLO1,          // EEPROM address 0.
        lastMaxPWMLO1,             // EEPROM address 1.
        lastMinPWMLO2,             // EEPROM address 2.
        lastMaxPWMLO2;             // EEPROM address 3.
    float lastMaxTemp,             // EEPROM address 4.
        lastMinTemp;               // EEPROM address 8.
    float lastPresetVoltageFactor, // EEPROM address 12.
        lastPresetCurrentFactor,   // EEPROM address 16.
        lastSensedCurrentFactor,   // EEPROM address 20.
        lastSensedVoltageFactor;   // EEPROM address 24.
    byte outputBuffer[EEPROM_TOTAL_ADDRESS]; // used to store temporary values on readBytes and writeBytes

public:
    void begin(); // Init
    void fetch(); // Read the content of EEPROM and store it on associated variables
    void update(); // Check and Writes the content of EEPROM if there is changes
};

extern eepromControllerClass eeprom;
#endif