#ifndef EncoderLibrarys_H
#define EncoderLibrarys_H

#include "Arduino.h"

class EncoderLibrarys {
  public:
    // Constructor
    EncoderLibrarys(int pinA1, int pinB1, int pinA2, int pinB2);

    // Method to setup encoder
    void setupEncoder();

    // Methods to get encoder positions
    int Poss_L();
    int Poss_R();

    // Method to reset encoder positions
    void resetEncoders();

  private:
    int _encoderPinA1, _encoderPinB1;
    int _encoderPinA2, _encoderPinB2;

    volatile int encoderPoss1;
    volatile int encoderPoss2;

    static void encoderInterrupt1();
    static void encoderInterrupt2();

    static EncoderLibrarys* _instance;
};

#endif
