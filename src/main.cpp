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

DigitalButton loadButton(LD_BTN);

lcdControllerClass lcd;
encoderControllerClass encoder;
eepromControllerClass eeprom;

HardwareTimer *pwmTimer = new HardwareTimer(TIM4);
HardwareTimer *fanTimer = new HardwareTimer(TIM3);

ADS1115 ADS(ADS1115_ADDRESS);
RunningAverage raVoltage(ADC_SAMPLE_COUNT);
RunningAverage raCurrent(ADC_SAMPLE_COUNT);

const float adcLSBSize = ADS.toVoltage(1);

void setupPWM();
static void MX_GPIO_Init(void);
void encoderService(void);

unsigned long adcMillis;

void setup()
{
  Serial.begin(115200);
  Serial.printf("Halo");
  Wire.setSDA(LCD_SDA);
  Wire.setSCL(LCD_SCL);
  MX_GPIO_Init();
  pinMode(NTC1, INPUT_ANALOG);
  pinMode(NTC2, INPUT_ANALOG);
  digitalWrite(LD_EN, HIGH);
  analogReadResolution(12);

  lcd.begin();
  eeprom.begin();
  eeprom.fetch();
  encoder.begin();
  if (!ADS.begin())
  {
    Serial.println("Failed to initialize ADS.");
    while (1)
      ;
  }
  ADS.setGain(1);
  setupPWM();
}

void loop()
{
  lcd.service();
  encoder.service();
  if (millis() - adcMillis >= ADC_SAMPLE_INTERVAL)
  {
    adcMillis = millis();
    int16_t val_0 = ADS.readADC(0);
    int16_t val_1 = ADS.readADC(1) + 84;
    int16_t ntc1 = analogRead(NTC1);
    raVoltage.addValue(val_0);
    raCurrent.addValue(val_1);
    sensedVoltage = raVoltage.getAverage() * adcLSBSize * ADC_VOLTAGE_BASE_FACTOR * sensedVoltageFactor;
    sensedCurrent = raCurrent.getAverage() * ADC_LSB_TO_CURRENT_MA * sensedCurrentFactor;
    if (sensedVoltage < 0)
      sensedVoltage = 0;
    if (sensedCurrent < 0)
      sensedCurrent = 0;
    pwmTimer->setCaptureCompare(I_SET_TIMER_CHANNEL, uint32_t(float(presetCurrentDAC)));
    pwmTimer->setCaptureCompare(V_SET_TIMER_CHANNEL, uint32_t(float(presetVoltageDAC)));

    const float R2 = 10000 * (4095.0 / (float)ntc1 - 1.0);
    const float logR2 = log(R2);
    bjtTemp = (1.0 / (1.009249522e-03 + 2.378405444e-04 * logR2 + 2.019202697e-07 * logR2 * logR2 * logR2));
    bjtTemp = bjtTemp - 273.15;

    presetVoltage = 0.004619 * presetVoltageDAC * presetVoltageFactor;
    presetCurrent = 1.095 * presetCurrentDAC * presetCurrentFactor;
    // Serial.printf("I_DAC : %d ; V_DAC : %d\n", presetCurrentDAC,presetVoltageDAC);
  }
}

uint8_t encoderPrescaler = 0;
void encoderService(void)
{
  encoderPrescaler++;
  if (encoderPrescaler >= 5)
  {
    encoderPrescaler = 0;
    encoder.enc->service();
  }
}

void setupPWM()
{
  // == TIM_OCMODE_PWM1             pin high when counter < channel compare, high otherwise
  pwmTimer->setMode(I_SET_TIMER_CHANNEL, TIMER_OUTPUT_COMPARE_PWM1, I_SET);
  pwmTimer->setMode(V_SET_TIMER_CHANNEL, TIMER_OUTPUT_COMPARE_PWM1, V_SET);
  fanTimer->setMode(LOGIC_OUTPUT1_TIMER_CHANNEL, TIMER_OUTPUT_COMPARE_PWM1, LOGIC_OUTPUT1);
  fanTimer->setMode(LOGIC_OUTPUT2_TIMER_CHANNEL, TIMER_OUTPUT_COMPARE_PWM1, LOGIC_OUTPUT2);
  // Pwm frequency is calculated with by dividing PWM Clock (72MHz) with ARR/overflow tick
  // set PWM frequency to 14.4kHz (72MHz/5000=14.4KHz)
  pwmTimer->setOverflow(5000); // in TICK FORMAT
  pwmTimer->attachInterrupt(encoderService);
  // set fan PWM frequency to 100Hz
  fanTimer->setOverflow(100, HERTZ_FORMAT); // in Hertz

  fanTimer->setCaptureCompare(LOGIC_OUTPUT2_TIMER_CHANNEL, 70, PERCENT_COMPARE_FORMAT); // 50%

  pwmTimer->setCaptureCompare(I_SET_TIMER_CHANNEL, 0);
  pwmTimer->setCaptureCompare(V_SET_TIMER_CHANNEL, 0);

  pwmTimer->resume();
  fanTimer->resume();
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