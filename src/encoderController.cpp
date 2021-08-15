#include "encoderController.h"

encoderControllerClass::encoderControllerClass()
{
}

void encoderControllerClass::begin()
{
    enc = new ClickEncoder(ENC_CLK, ENC_DT, ENC_BTN, ENC_STEPS);
    enc->setAccelerationEnabled(true);
}

void encoderControllerClass::service()
{
    enc->service();
    encoderValue += enc->getValue();
    encoderDelta = encoderValue - oldEncoderValue;
    if (encoderValue != oldEncoderValue)
        isMoved = true;
    else
        isMoved = false;
    btnState = enc->getButton();
    oldEncoderValue = encoderValue;
    
}

bool encoderControllerClass::isEncoderMoved()
{
    return isMoved;
}
int encoderControllerClass::getEncoderDelta()
{
    return encoderDelta;
}
int encoderControllerClass::getEncoderValue()
{
    return encoderValue;
}
ClickEncoder::Button encoderControllerClass::getButtonState()
{
    return btnState;
}