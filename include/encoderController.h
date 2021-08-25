#ifndef ENCODERCONTROLLER_H
#define ENCODERCONTROLLER_H
#include <ClickEncoder.h>
#include <globals.h>

class encoderControllerClass
{
private:
  int encoderValue;         // The integral value of encoder
  int oldEncoderValue = -1; // Used to determine encoder delta
  int encoderDelta;         // How much encoder moved from before
  bool isMoved;
  ClickEncoder::Button btnState;

public:
  ClickEncoder *enc;        // Object pointer is in public because service() need to be called on 1ms interval (on TIM4 with prescaler interrupt)
  encoderControllerClass(); // Constructor
  void begin();             // Init
  void service();           // Only call if you want to updates encoder variables
  int getEncoderDelta();
  int getEncoderValue();
  bool isEncoderMoved();
  ClickEncoder::Button getButtonState();
};
extern encoderControllerClass encoder;
#endif