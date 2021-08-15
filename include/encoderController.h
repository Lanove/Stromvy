#ifndef ENCODERCONTROLLER_H
#define ENCODERCONTROLLER_H
#include <ClickEncoder.h>
#include <globals.h>


#define VERBOSECASE(label)      \
    case label:                 \
        Serial.println(#label); \
        break;

class encoderControllerClass{
  private:
    int encoderValue;
    int oldEncoderValue = -1;
    int encoderDelta;
    bool isMoved;
    ClickEncoder *enc;
    ClickEncoder::Button btnState;
  public:
    encoderControllerClass();
    void begin();
    void service();  
    int getEncoderDelta();
    int getEncoderValue();
    bool isEncoderMoved();
    ClickEncoder::Button getButtonState();
};
extern encoderControllerClass encoder;
#endif