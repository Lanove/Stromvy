#include <Arduino.h>
#include <LiquidCrystal_I2C.h>
#include <Wire.h>
#include <ClickEncoder.h>
#include <Eeprom24C04_16.h>
#include "stm32f1xx_hal.h"
#define EEPROM_ADDRESS 0x50

#define LED_LDON PB7
#define LED_ST2 PB6
#define LED_ST1 PB5 //problem
#define LED_PC13 PC13

#define ENC_CLK PB4
#define ENC_DT PB3
#define ENC_BTN PA15
#define ENC_STEPS 4

#define LCD_SDA PB11
#define LCD_SCL PB10

#define BUZZER PB14

#define LD_BTN PA13
#define LD_EN PA14

#define LOGIC_OUTPUT1 PB0
#define LOGIC_OUTPUT2 PB1

#define LOGIC_OUTPUT1_TIMER_CHANNEL 3
#define LOGIC_OUTPUT2_TIMER_CHANNEL 4

#define I_READ PA0
#define V_READ PA1
#define NTC1 PA2
#define NTC2 PA3

#define I_READ_ADC_CHANNEL ADC_CHANNEL_0
#define V_READ_ADC_CHANNEL ADC_CHANNEL_1
#define NTC1_ADC_CHANNEL ADC_CHANNEL_2
#define NTC2_ADC_CHANNEL ADC_CHANNEL_3

#define CC_IND PB12
#define CV_IND PB13

#define I_SET PB8
#define V_SET PB9

#define I_SET_TIMER_CHANNEL 3
#define V_SET_TIMER_CHANNEL 4

ClickEncoder encoder(ENC_CLK, ENC_DT, ENC_BTN, ENC_STEPS);
DigitalButton loadButton(LD_BTN);
LiquidCrystal_I2C lcd(0x27, 20, 4);
static Eeprom24C04_16 eeprom(EEPROM_ADDRESS);

int16_t oldEncPos, encPos;
uint8_t buttonState;

HardwareTimer *pwmTimer = new HardwareTimer(TIM4);
HardwareTimer *fanTimer = new HardwareTimer(TIM3);
ADC_HandleTypeDef adcHandler;

bool polarity;
unsigned long recorder, period, counter;

void setupPWM();
void testCallback();
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
uint16_t adcRead(uint32_t channel);

void setup()
{
  Serial.begin(115200);
  Serial.printf("Halo");
  Wire.setSDA(LCD_SDA);
  Wire.setSCL(LCD_SCL);
  lcd.begin();
  lcd.print("Hello World!");
  MX_GPIO_Init();
  MX_ADC1_Init();
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
}

bool lastButtonReading;
unsigned long milliser, miles;
bool nota;

void loop()
{
  if (millis() - miles >= 200)
  {
    miles = millis();
    // Serial.println();
    // Serial.println(analogRead(V_READ));
    // Serial.println(analogRead(I_READ));
    // Serial.println(analogRead(NTC1));
    // Serial.println(analogRead(NTC2));
    // Serial.println(analogRead(PA4));
    int vread = adcRead(V_READ_ADC_CHANNEL);
    int iread = adcRead(I_READ_ADC_CHANNEL);
    int ntc1 = adcRead(NTC1_ADC_CHANNEL);
    int ntc2 = adcRead(NTC2_ADC_CHANNEL);
    lcd.setCursor(0, 1);
    lcd.printf("V_READ : %04d", vread);
    lcd.setCursor(0, 2);
    lcd.printf("I_READ : %04d", iread);
    Serial.printf("V:%d ; I:%d ; NTC1:%d ; NTC2:%d\n", vread, iread, ntc1, ntc2);
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
    Serial.print("Button: ");
    Serial.println(buttonState);
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
  fanTimer->setOverflow(100, HERTZ_FORMAT); // in TICK FORMAT

  fanTimer->setCaptureCompare(LOGIC_OUTPUT2_TIMER_CHANNEL, 50, PERCENT_COMPARE_FORMAT); // 50%

  pwmTimer->setCaptureCompare(I_SET_TIMER_CHANNEL, 5000); // 100%
  pwmTimer->setCaptureCompare(V_SET_TIMER_CHANNEL, 2500); // 50%

  pwmTimer->resume();
  fanTimer->resume();
}

uint16_t adcRead(uint32_t channel)
{
  ADC_ChannelConfTypeDef adcConfig = {0};
  uint16_t adcResult = 0;
  adcConfig.Channel = channel;
  adcConfig.Rank = ADC_REGULAR_RANK_1;
  adcConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&adcHandler, &adcConfig) != HAL_OK)
  {
    Serial.printf("HAL_ADC_ConfigChannel%d\n", channel);
    return 0;
  }
  if (HAL_ADCEx_Calibration_Start(&adcHandler) != HAL_OK)
  {
    Serial.println("HAL_ADCEx_Calibration FAIL");
    return 0;
  }
  if (HAL_ADC_Start(&adcHandler) != HAL_OK)
  {
    Serial.println("HAL_ADC_Start FAIL");
    return 0;
  }
  if (HAL_ADC_PollForConversion(&adcHandler, 10) == HAL_OK)
    adcResult = HAL_ADC_GetValue(&adcHandler);
  else
  {
    Serial.println("HAL_ADC_PollForConversion FAIL");
    return 0;
  }
  if (HAL_ADC_Stop(&adcHandler) != HAL_OK)
  {
    Serial.println("HAL_ADC_Stop FAIL");
    return 0;
  }
  return adcResult;
}

static void MX_ADC1_Init(void)
{
  adcHandler.Instance = ADC1;
  adcHandler.Init.ScanConvMode = ADC_SCAN_DISABLE;
  adcHandler.Init.ContinuousConvMode = DISABLE;
  adcHandler.Init.DiscontinuousConvMode = DISABLE;
  adcHandler.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  adcHandler.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  adcHandler.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&adcHandler) != HAL_OK)
    Serial.println("HAL_ADC_INIT FAIL");
  if (HAL_ADCEx_Calibration_Start(&adcHandler) != HAL_OK)
    Serial.println("HAL_ADCEx_Calibration FAIL");
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
  GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3;
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