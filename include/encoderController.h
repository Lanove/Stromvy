#ifndef ENCODERCONTROLLER_H
#define ENCODERCONTROLLER_H
#include <ClickEncoder.h>
#include <globals.h>

extern ClickEncoder enc;

class encoderControllerClass
{
private:
  int encoderValue;         // The integral value of encoder
  int oldEncoderValue = -1; // Used to determine encoder delta
  int encoderDelta;         // How much encoder moved from before
  bool isMoved;
  ClickEncoder::Button btnState;

public:
  void begin();             // Init
  void service();           // Only call if you want to updates encoder variables
  int getEncoderDelta();
  int getEncoderValue();
  bool isEncoderMoved();
  ClickEncoder::Button getButtonState();
};
extern encoderControllerClass encoder;
#endif