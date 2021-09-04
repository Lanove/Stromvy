#include <globals.h>

int presetVoltageDAC = MAX_V_DAC / 2, // Variable to store digital value (0~5000) of preset voltage
    presetCurrentDAC = MAX_I_DAC;     // Variable to store digital value (0~5000) of preset current
float sensedVoltage,                  // variable to store real sensed voltage on terminal, used on main and lcdController
    sensedCurrent,                    // variable to store real sensed current flow to load, used on main and lcdController
    sensedPower,                      // variable to store real power dissipated by load, used on main and lcdController
    presetVoltage,                    // The preset value of real voltage
    presetCurrent,                    // The preset value of real current
    bjtTemp;                          // The measured temperature of the output BJT
bool ldStatus,                        // Load status, whether it's ON or OFF
    opMode,                           // Current operation of Power supply which is CC or CV
    timerStatus;                      // Timer status, whether it's ON or OFF
uint32_t timerDuration;               // variable to store timer duration that user intended, if value is 0 then timer is OFF
float mWhTotal,                       // Total energy used by load in mWh
    mAhTotal;                         // Total energy used by load in mAh
uint32_t timeRunning;                 // variable to keep track total time running, recorded when ldStatus is ON and paused when ldStatus is OFF
uint32_t logInterval;                 // The interval logging value spit out on UART port
bool logStatus;                       // the status of logging, enabled or disabled
int8_t minPWMLO1,                     // EEPROM address 0. Stored on EEPROM on save, minimum PWM value for logic output 1
    maxPWMLO1,                        // EEPROM address 1. Stored on EEPROM on save, maximum PWM value for logic output 1
    minPWMLO2,                        // EEPROM address 2. Stored on EEPROM on save, minimum PWM value for logic output 2
    maxPWMLO2;                        // EEPROM address 3. Stored on EEPROM on save, maximum PWM value for logic output 2
float maxTemp,                        // EEPROM address 4. Stored on EEPROM on save, maximum temperature for logic output (logic output will put out maximum PWM value if temperature is higher or equal maxTemp)
    minTemp;                          // EEPROM address 8. Stored on EEPROM on save, minimum temperature for logic output (logic output will put out minimum PWM value if temperature is lower or equal to minTemp)
float presetVoltageFactor,            // EEPROM address 12. Stored on EEPROM on save, preset voltage will be multiplied by this factor before shown on LCD
    presetCurrentFactor,              // EEPROM address 16. Stored on EEPROM on save, preset current will be multiplied by this factor before shown on LCD
    sensedCurrentFactor,              // EEPROM address 20. Stored on EEPROM on save, sensed voltage will be multiplied by this factor before shown on LCD
    sensedVoltageFactor;              // EEPROM address 24. Stored on EEPROM on save, sensed voltage will be multiplied by this factor before shown on LCD

DigitalButton loadButton(LD_BTN); // Object for load button

lcdControllerClass lcd;             // Object for lcdController
encoderControllerClass encoder;     // Object for encoderController
eepromControllerClass eeprom;       // Object for eepromController
indicatorControllerClass indicator; // Object for indicatorController

HardwareTimer pwmTimer(TIM4); // Object pointer for pwmTimer on TIMER4
HardwareTimer fanTimer(TIM3); // Object pointer for fanTimer on TIMER3

ADS1115 ADS(ADS1115_ADDRESS);               // Object for ADS1115 16-bit ADC
RunningAverage raVoltage(ADC_SAMPLE_COUNT); // Object for sensed voltage running average
RunningAverage raCurrent(ADC_SAMPLE_COUNT); // Object for sensed current running average

const float adcLSBSize = ADS.toVoltage(1); // the size of lsb for ADS1115 ADC, the gain of ADS1115 is 1, so 1 lsb is 125uV
unsigned long adcMillis,                   // Variable to store millis for adc, fan and energy counter routine
    dacMillis,                             // Variable to store millis for dac routine
    logMillis,                             // Variable to store millis for log routine
    lastMillis,                            // Variable to store millis for energy and time running calculation
    i2cRefreshMillis;
unsigned long dacVoltage,   // Digital value of voltage DAC (0~5000) range
    dacCurrent;             // Digital value of current DAC (0~5000) range
uint8_t encoderCounter = 0; // Variable to store the prescaler counter for encoder routine on TIM4
uint8_t dacCounter = 0;     // Variable to store the prescaler counter for DAC increment/decrement routine on TIM4
bool lastOpMode;            // Used to detect change in CC/CV mode to beep status buzzer

int16_t voltageADC, // Variable to store ADC value of voltage channel
    currentADC;     // Variable to store ADC value of current channel
uint16_t adcTimeoutCounter = 0; // Variable to store total count of ADC timeout occurrence

void setupPWM();                                                               // Function to begin setup of PWM and timers
static void MX_GPIO_Init(void);                                                // Function to begin setup of every GPIO
void timer4Interrupt(void);                                                    // Function that is called every TIM4 overflow (interrupt function of TIM4)
void fanHandler();                                                             // Function to handle the PWM value of fan based on bjt temperature (LO1) and sensed load current (LO2)
float mapf(float x, float in_min, float in_max, float out_min, float out_max); // float version of map()
void display_mallinfo(void);
void refreshI2C();

void setup()
{
  Serial.begin(115200);
  Wire.setSDA(LCD_SDA);
  Wire.setSCL(LCD_SCL);
  MX_GPIO_Init();
  analogReadResolution(12); // set ADC resolution to 12 bit (0~4095) range

  lcd.begin();
  eeprom.begin();
  eeprom.fetch();
  encoder.begin();
  setupPWM();
  indicator.begin();
  if (!ADS.begin())
  {
    indicator.beepBuzzer(1000, 250, true, 5000); // Long beep on start-up mean ADS1115 ADC fail to init
    Serial.println("Failed to initialize ADS.");
    delay(6000);
    NVIC_SystemReset();
  }

  ADS.setGain(1);            // Use 0~4.096V range
  indicator.beepBuzzer(200); // Power supply ready beep
  IWatchdog.begin(WATCHDOG_TIMEOUT);
  if (IWatchdog.isReset())
  {
    indicator.beepBuzzer(2000, 1000, true, 5000); // Long beep on start-up mean ADS1115 ADC fail to init
    IWatchdog.clearReset();
  }
}

void loop()
{
  opMode = digitalRead(CC_IND);
  lcd.service();
  encoder.service();

  ClickEncoder::Button b = loadButton.getButton();
  // Refresh I2C Line every I2C_REFRESH_INTERVAL (30m) or when ldButton double click
  if (millis() - i2cRefreshMillis >= I2C_REFRESH_INTERVAL || b == ClickEncoder::DoubleClicked)
  {
    i2cRefreshMillis = millis();
    refreshI2C();
    display_mallinfo();
  }

  // If there is opMode change and ldStatus is ON and no current active buzzer beep, then beep the buzzer
  if (opMode != lastOpMode && ldStatus == STATUS_ON && !indicator.getBeepFlag())
    indicator.beepBuzzer(150, 150, true, 1200);

  // If load button clicked
  if (b == ClickEncoder::Clicked)
  {
    // Invert load status
    ldStatus = !ldStatus;
    digitalWrite(LD_EN, ldStatus);

    // Beep buzzer for 500ms
    indicator.beepBuzzer(500);

    // If load status switched to ON and there are running timer then use the following beep instead of 500ms beep
    if (ldStatus == STATUS_ON && timerDuration != 0)
      indicator.beepBuzzer(100, 100, true, 1000);

    // reset each DAC value (for smooth start-up when ldStatus switched on)
    dacVoltage = 0;
    dacCurrent = 0;

    // Reset indicator status (will altered by next if statement)
    indicator.setIndicator(STANDBY); // reset the indicator (to reset overheat and timer halt status)
  }

  // While the current status is not OVERHEAT or TIMER_HALT then, change the indicator status based on current condition
  if (indicator.getIndicator() != OVERHEAT_HALT && indicator.getIndicator() != TIMER_HALT)
  {
    // When the time running is still 0 and load status is off then STANDBY
    if (timeRunning == 0 && ldStatus == STATUS_OFF && indicator.getIndicator() != STANDBY)
      indicator.setIndicator(STANDBY);
    // When the load status is ON and opMode is CC then set status to RUNNING_CC
    else if (ldStatus == STATUS_ON && opMode == MODE_CC && indicator.getIndicator() != RUNNING_CC)
      indicator.setIndicator(RUNNING_CC);
    // When the load status is ON and opMode is CV then set status to RUNNING_CV
    else if (ldStatus == STATUS_ON && opMode == MODE_CV && indicator.getIndicator() != RUNNING_CV)
      indicator.setIndicator(RUNNING_CV);
    // When the time running is more than 0 and load status is off then PAUSED
    else if (timeRunning != 0 && ldStatus == STATUS_OFF && indicator.getIndicator() != PAUSED)
      indicator.setIndicator(PAUSED);
  }

  // When the bjtTemp is more than OVERHEAT_TEMPERATURE and timeRunning is getting past timerDuration then
  if (bjtTemp > OVERHEAT_TEMPERATURE || (timeRunning / 1000 >= timerDuration && timerDuration != 0))
  {
    // Disble the load status
    ldStatus = STATUS_OFF;
    digitalWrite(LD_EN, ldStatus);
    // reset each DAC value (for smooth start-up when ldStatus switched on)
    dacVoltage = 0;
    dacCurrent = 0;

    // If it's overheat condition then set indicator status to overheat and beep buzzer
    if (bjtTemp > OVERHEAT_TEMPERATURE && indicator.getIndicator() != OVERHEAT_HALT)
    {
      indicator.setIndicator(OVERHEAT_HALT);
      indicator.beepBuzzer(250, 250, true, 3000);
    }
    // If it's timer run out condition then set indicator status to timer_halt and beep buzzer
    else if (timeRunning / 1000 >= timerDuration && timerDuration != 0 && indicator.getIndicator() != TIMER_HALT)
    {
      indicator.setIndicator(TIMER_HALT);
      indicator.beepBuzzer(250, 250, true, 3000);
    }
  }

  // data logging routine
  if (millis() - logMillis >= logInterval && logStatus)
  {
    logMillis = millis();
    // Send following variables value to serial port every logInterval with CSV format
    // sensedVoltage,sensedCurrent,sensedPower,presetVoltage,presetCurrent,bjtTemp,mWhTotal,mAhTotal,timeRunning (hh:mm:ss)
    Serial.printf("%5.2f;%4.0f;%5.3f;%5.2f;%4.0f;%1.1f;%1.0f;%1.0f;%02d:%02d:%02d\n", sensedVoltage, sensedCurrent, sensedPower, presetVoltage, presetCurrent, bjtTemp, mWhTotal, mAhTotal, (int)timeRunning / 3600000, (int)(timeRunning / 1000) % 3600 / 60, (int)(timeRunning / 1000) % 60);
  }

  // PWM-DAC routine
  if (millis() - dacMillis >= DAC_UPDATE_INTERVAL)
  {
    dacMillis = millis();
    pwmTimer.setCaptureCompare(I_SET_TIMER_CHANNEL, (ldStatus == STATUS_ON) ? uint32_t(float(dacCurrent)) : 0);
    pwmTimer.setCaptureCompare(V_SET_TIMER_CHANNEL, (ldStatus == STATUS_ON) ? uint32_t(float(dacVoltage)) : 0);
  }

  // ADC-FAN-energy counter routine
  if (millis() - adcMillis >= ADC_SAMPLE_INTERVAL)
  {
    adcMillis = millis();

    fanHandler();

    // Read ADC of voltage, current and NTC1
    int16_t vread = ADS.readADC(ADC_VOLTAGE_CHANNEL);                      // Read ADC voltage
    int16_t iread = ADS.readADC(ADC_CURRENT_CHANNEL) + ADC_CURRENT_OFFSET; // Add current offset to make calculation work (because 0mA output negative voltage and may mess calculation)
    if (vread != ADS.getErrorValue())
      voltageADC = vread;
    if (iread != ADS.getErrorValue())
      currentADC = iread;
    if (vread == ADS.getErrorValue() || iread == ADS.getErrorValue())
    {
      adcTimeoutCounter++;
      if (adcTimeoutCounter % 10 == 0)
        refreshI2C();
      if (adcTimeoutCounter >= 10000)
        adcTimeoutCounter = 0;
    }
    int16_t ntc1 = analogRead(NTC1); // 12-bit

    if (raVoltage.getCount() >= 65500)
    {
      float temp = raVoltage.getAverage();
      raVoltage.clear();
      raVoltage.addValue(temp);
    }

    if (raCurrent.getCount() >= 65500)
    {
      float temp = raCurrent.getAverage();
      raCurrent.clear();
      raCurrent.addValue(temp);
    }

    // Add new read out to running average
    raVoltage.addValue(float(voltageADC) * adcLSBSize * ADC_VOLTAGE_BASE_FACTOR * sensedVoltageFactor); // Uses gain to compute actual result on output terminal
    raCurrent.addValue(float(currentADC) * ADC_LSB_TO_CURRENT_MA * sensedCurrentFactor);                // in mA, Directly transform ADC read out to current by multiplying with ADC_LSB_TO_CURRENT_MA

    // Calculate each sensed voltage,current and power
    sensedVoltage = raVoltage.getAverage();
    sensedCurrent = raCurrent.getAverage();
    sensedPower = sensedVoltage * (sensedCurrent / 1000); // P = V*I, since I is on mA we need to divide by 1000

    // Convert binary presetDAC to real voltage/current value
    presetVoltage = DAC_VOLTAGE_BASE_FACTOR * presetVoltageDAC * presetVoltageFactor;
    presetCurrent = (DAC_CURRENT_BASE_FACTOR * presetCurrentDAC * presetCurrentFactor) + DAC_CURRENT_OFFSET;

    // Bottom limits
    if (sensedVoltage < 0)
      sensedVoltage = 0;
    if (sensedCurrent < 0)
      sensedCurrent = 0;
    if (mAhTotal < 0)
      mAhTotal = 0;
    if (mWhTotal < 0)
      mWhTotal = 0;

    // Calculate temperature of bjt (NTC)
    const float R2 = NTC_R1 * (4095.0 / (float)ntc1 - 1.0);
    const float logR2 = log(R2);
    bjtTemp = (1.0 / (NTC_c1 + NTC_c2 * logR2 + NTC_c3 * logR2 * logR2 * logR2));
    bjtTemp = bjtTemp - 273.15;

    // Calculate and integrate energy and time when load is on
    unsigned long cMillis = millis();
    float elapsedTime = (cMillis - lastMillis) / 1000.0; // in normal second not millisecond
    if (ldStatus == STATUS_ON)
    {
      mAhTotal += (sensedCurrent * elapsedTime) / 3600;
      mWhTotal += (sensedPower * 1000.0 * elapsedTime) / 3600;
      timeRunning += uint32_t(elapsedTime * 1000.0);
    }
    lastMillis = cMillis;
  }
  // Store current variable to detect changes on next loop
  lastOpMode = opMode;
  // Reload watchdog counter
  IWatchdog.reload();
}

void fanHandler()
{
  // Convert temperature range and current range to pwm percentage
  unsigned long pwmLO1 = (unsigned long)mapf(bjtTemp, minTemp, maxTemp, float(minPWMLO1), float(maxPWMLO1)),
                pwmLO2 = (unsigned long)mapf(sensedCurrent, LO2_MIN_CURRENT, LO2_MAX_CURRENT, float(minPWMLO2), float(maxPWMLO2));
  // Limits
  if (bjtTemp < minTemp)
    pwmLO1 = 0;
  if (pwmLO1 > 100)
    pwmLO1 = 100;
  if (pwmLO2 > 100)
    pwmLO2 = 100;
  // Apply changes
  fanTimer.setCaptureCompare(LOGIC_OUTPUT1_TIMER_CHANNEL, pwmLO1, PERCENT_COMPARE_FORMAT);
  fanTimer.setCaptureCompare(LOGIC_OUTPUT2_TIMER_CHANNEL, pwmLO2, PERCENT_COMPARE_FORMAT);
}

void timer4Interrupt(void)
{
  encoderCounter++;
  dacCounter++;
  // Run DAC increment/decrement routine every TIM4 overflow divided by TIM4_DAC_PRESCALER
  if (dacCounter >= TIM4_DAC_PRESCALER)
  {
    if (dacVoltage < presetVoltageDAC)
      dacVoltage++;
    else if (dacVoltage > presetVoltageDAC)
      dacVoltage--;
    if (dacCurrent < presetCurrentDAC)
      dacCurrent++;
    else if (dacCurrent > presetCurrentDAC)
      dacCurrent--;
  }
  // Run encoder routine every TIM4 overflow divided by TIM4_DAC_PRESCALER
  if (encoderCounter >= TIM4_ENCODER_PRESCALER)
  {
    encoderCounter = 0;
    enc.service();
    loadButton.service();
  }
}

void setupPWM()
{
  // == TIM_OCMODE_PWM1             pin high when counter < channel compare, high otherwise
  pwmTimer.setMode(I_SET_TIMER_CHANNEL, TIMER_OUTPUT_COMPARE_PWM1, I_SET);
  pwmTimer.setMode(V_SET_TIMER_CHANNEL, TIMER_OUTPUT_COMPARE_PWM1, V_SET);
  fanTimer.setMode(LOGIC_OUTPUT1_TIMER_CHANNEL, TIMER_OUTPUT_COMPARE_PWM1, LOGIC_OUTPUT1);
  fanTimer.setMode(LOGIC_OUTPUT2_TIMER_CHANNEL, TIMER_OUTPUT_COMPARE_PWM1, LOGIC_OUTPUT2);
  // Pwm frequency is calculated with by dividing PWM Clock (72MHz) with ARR/overflow tick
  // set PWM frequency to 14.4kHz (72MHz/5000=14.4KHz)
  pwmTimer.setOverflow(5000); // in TICK FORMAT
  pwmTimer.attachInterrupt(timer4Interrupt);
  // set fan PWM frequency to 100Hz
  fanTimer.setOverflow(100, HERTZ_FORMAT); // in Hertz

  pwmTimer.setCaptureCompare(I_SET_TIMER_CHANNEL, 0);
  pwmTimer.setCaptureCompare(V_SET_TIMER_CHANNEL, 0);

  pwmTimer.resume();
  fanTimer.resume();
}

void refreshI2C()
{
  IWatchdog.reload();                         // Reset watchdog counter before, because following procedures might take times
  LCD_SCREEN currentScreen = lcd.getScreen(); // Store current screen because begin reset screen to SCREEN_MAIN
  lcd.begin();                                // LCD begin already cover I2C Wire.begin();
  lcd.setScreen(currentScreen);
}

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_All, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_All, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_All, GPIO_PIN_RESET);

  // Configure Analog Input pins from PORT A
  GPIO_InitStruct.Pin = GPIO_PIN_2 | GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  // Configure Output pins from PORT C
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  // Configure Output pins from PORT B
  GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  // Configure Output pins from PORT A
  GPIO_InitStruct.Pin = GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  // Configure input pins (without internal pull) from PORT B
  GPIO_InitStruct.Pin = GPIO_PIN_12 | GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  // Configure input pins (without internal pull) from PORT A
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

float mapf(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void display_mallinfo(void)
{

  static char *ramstart = &_sdata;
  static char *ramend = &_estack;
  static char *minSP = (char *)(ramend - &_Min_Stack_Size);
  char *heapend = (char *)sbrk(0);
  char *stack_ptr = (char *)__get_MSP();
  struct mallinfo mi = mallinfo();

  Serial.print("Total non-mmapped bytes (arena):       ");
  Serial.println(mi.arena);
  Serial.print("# of free chunks (ordblks):            ");
  Serial.println(mi.ordblks);
  Serial.print("# of free fastbin blocks (smblks):     ");
  Serial.println(mi.smblks);
  Serial.print("# of mapped regions (hblks):           ");
  Serial.println(mi.hblks);
  Serial.print("Bytes in mapped regions (hblkhd):      ");
  Serial.println(mi.hblkhd);
  Serial.print("Max. total allocated space (usmblks):  ");
  Serial.println(mi.usmblks);
  Serial.print("Free bytes held in fastbins (fsmblks): ");
  Serial.println(mi.fsmblks);
  Serial.print("Total allocated space (uordblks):      ");
  Serial.println(mi.uordblks);
  Serial.print("Total free space (fordblks):           ");
  Serial.println(mi.fordblks);
  Serial.print("Topmost releasable block (keepcost):   ");
  Serial.println(mi.keepcost);

  Serial.print("RAM Start at:       0x");
  Serial.println((unsigned long)ramstart, HEX);
  Serial.print("Data/Bss end at:    0x");
  Serial.println((unsigned long)&_end, HEX);
  Serial.print("Heap end at:        0x");
  Serial.println((unsigned long)heapend, HEX);
  Serial.print("Stack Ptr end at:   0x");
  Serial.println((unsigned long)stack_ptr, HEX);
  Serial.print("RAM End at:         0x");
  Serial.println((unsigned long)ramend, HEX);

  Serial.print("Heap RAM Used:      ");
  Serial.println(mi.uordblks);
  Serial.print("Program RAM Used:   ");
  Serial.println(&_end - ramstart);
  Serial.print("Stack RAM Used:     ");
  Serial.println(ramend - stack_ptr);
  Serial.print("Estimated Free RAM: ");
  Serial.println(((stack_ptr < minSP) ? stack_ptr : minSP) - heapend + mi.fordblks);
}