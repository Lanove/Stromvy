#include <Arduino.h>
#include <LiquidCrystal_I2C.h>
#include <ClickEncoder.h>

#define LED_LDON PB7
#define LED_ST2 PB6
#define LED_ST1 PB5 //problem
#define LED_PC13 PC13

#define ENC_CLK PB4
#define ENC_DT PB3
#define ENC_BTN PA15
#define ENC_STEPS 4

#define LCD_SDA PB10
#define LCD_SCL PB11

#define BUZZER PB14

#define NTC1 PA2
#define NTC2 PA3

#define LD_BTN PA13
#define LD_EN PA14

#define LOGIC_OUTPUT1 PB0
#define LOGIC_OUTPUT2 PB1

#define V_READ PA4
#define I_READ PA0

#define CC_IND PB12
#define CV_IND PB13

#define I_SET PB8
#define V_SET PB9

ClickEncoder encoder(ENC_CLK, ENC_DT, ENC_BTN, ENC_STEPS);
DigitalButton loadButton(LD_BTN);

int16_t oldEncPos, encPos;
uint8_t buttonState;


void setup()
{
  Serial.begin(115200);
  Serial.printf("Halo");
  // pinMode(LED_PC13, OUTPUT);
  // pinMode(LED_LDON, OUTPUT);
  // pinMode(LED_ST1, OUTPUT);
  // pinMode(LED_ST2, OUTPUT);
  // pinMode(BUZZER, OUTPUT);
  pinMode(LD_EN, OUTPUT);
  digitalWrite(LD_EN,HIGH);
  //   pinMode(LOGIC_OUTPUT1, OUTPUT);
  //   digitalWrite(LOGIC_OUTPUT1,HIGH);
  // pinMode(LOGIC_OUTPUT2, OUTPUT);
  //   digitalWrite(LOGIC_OUTPUT2,HIGH);

  // pinMode(I_SET, PWM);
  // pinMode(V_SET, PWM);
  // pinMode(V_READ,INPUT_ANALOG);
  // pinMode(I_READ,INPUT_ANALOG);
  // pinMode(NTC1,INPUT_ANALOG);
  // pinMode(NTC2,INPUT_ANALOG);
  analogWrite(I_SET, 127);
  analogWrite(V_SET, 127);

  encoder.setAccelerationEnabled(true);

  Serial.print("Acceleration is ");
  Serial.println((encoder.getAccelerationEnabled()) ? "enabled" : "disabled");

  oldEncPos = -1;
  HAL_Delay(1000);
  // put your setup code here, to run once:
}

bool lastButtonReading;
unsigned long milliser,miles;
bool nota;

void loop()
{
  if(millis() - miles >= 200)
  {
    miles = millis();
    Serial.println();
    Serial.println(analogRead(V_READ));
    Serial.println(analogRead(I_READ));
    Serial.println(analogRead(NTC1));
    Serial.println(analogRead(NTC2));
    Serial.println(analogRead(PA4));
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