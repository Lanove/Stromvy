#include <globals.h>

ClickEncoder encoder(ENC_CLK, ENC_DT, ENC_BTN, ENC_STEPS);
DigitalButton loadButton(LD_BTN);
static Eeprom24C04_16 eeprom(EEPROM_ADDRESS);

int16_t oldEncPos, encPos;
uint8_t buttonState;

HardwareTimer *pwmTimer = new HardwareTimer(TIM4);
HardwareTimer *fanTimer = new HardwareTimer(TIM3);
ADS1115 ADS(0x4A);
RunningAverage voltageAverage(10);
RunningAverage currentAverage(10);

bool polarity;
unsigned long recorder, period, counter;

void setupPWM();
void testCallback();
static void MX_GPIO_Init(void);

void setup()
{
  Serial.begin(115200);
  Serial.printf("Halo");
  Wire.setSDA(LCD_SDA);
  Wire.setSCL(LCD_SCL);
  MX_GPIO_Init();
  pinMode(NTC1,INPUT_ANALOG);
  pinMode(NTC2,INPUT_ANALOG);
  digitalWrite(LD_EN, HIGH);
  setupPWM();
  analogReadResolution(12);
  encoder.setAccelerationEnabled(true);

  Serial.print("Acceleration is ");
  Serial.println((encoder.getAccelerationEnabled()) ? "enabled" : "disabled");

  oldEncPos = -1;
  HAL_Delay(1000);
  while (!Serial)
    ;
  if (!ADS.begin())
  {
    Serial.println("Failed to initialize ADS.");
    while (1)
      ;
  }
  ADS.setGain(1);
}

bool lastButtonReading;
unsigned long milliser, miles;
bool nota;

void loop()
{
  if (millis() - miles >= 200)
  {
    miles = millis();
    int ntc1 = analogRead(NTC1_ADC_CHANNEL);
    int ntc2 = analogRead(NTC2_ADC_CHANNEL);
    
    int16_t val_0 = ADS.readADC(0);
    int16_t val_1 = ADS.readADC(1);

    voltageAverage.addValue(val_0);
    currentAverage.addValue(val_1);
    float f = ADS.toVoltage(1); // voltage factor
    // Serial.printf("V_int:%f ; I_int:%f\n", float(vread)*0.0008056640625, float(iread)*0.0008056640625);
  }
  if (millis() - milliser >= 5000)
  {
    milliser = millis();
    nota = !nota;

    // digitalWrite(LED_PC13, nota);
    // digitalWrite(LED_LDON, nota);
    // digitalWrite(LED_ST1, nota);
    // digitalWrite(LED_ST2, nota);
    // // digitalWrite(BUZZER, nota);
    // digitalWrite(LD_EN, nota);
    // digitalWrite(LOGIC_OUTPUT1, nota);
    // digitalWrite(LOGIC_OUTPUT2, nota);
  }
  encoder.service();
  encPos += encoder.getValue();
  bool button = digitalRead(LD_BTN);
  if (button != lastButtonReading)
    Serial.println(button);
  lastButtonReading = button;
  if (encPos != oldEncPos)
  {
    oldEncPos = encPos;
    Serial.print("Encoder Value: ");
    Serial.println(encPos);
  }

  buttonState = encoder.getButton();

  if (buttonState != 0)
  {
    // Serial.print("Button: ");
    // Serial.println(buttonState);
    switch (buttonState)
    {
    case ClickEncoder::Open: //0
      break;

    case ClickEncoder::Closed: //1
      break;

    case ClickEncoder::Pressed: //2
      break;

    case ClickEncoder::Held: //3
      break;

    case ClickEncoder::Released: //4
      break;

    case ClickEncoder::Clicked: //5
      break;

    case ClickEncoder::DoubleClicked: //6
      break;
    }
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
  // set fan PWM frequency to 100Hz
  fanTimer->setOverflow(100, HERTZ_FORMAT); // in Hertz FORMAT

  fanTimer->setCaptureCompare(LOGIC_OUTPUT2_TIMER_CHANNEL, 70, PERCENT_COMPARE_FORMAT); // 50%

  pwmTimer->setCaptureCompare(I_SET_TIMER_CHANNEL, 5000); // 100%
  pwmTimer->setCaptureCompare(V_SET_TIMER_CHANNEL, 2500); // 50%

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