#include <Arduino.h>
#include <Adafruit_TiCoServo.h>
#include <Adafruit_NeoPixel.h>

#define DEBUG 0

// #define PIXEL_PIN 4
// #define PIXEL_NUM 16
// #define ENCODER_A_PIN 2
// #define ENCODER_B_PIN 3
// #define ENCODER_BUTTON_PIN 7
// #define SERVO_PIN 10
// #define SERVO_CLOSED_MICROS 1800
// #define SERVO_OPEN_MICROS 1800

void PinA(void);
void PinB(void);
void encoderPinAHandler(void);
void encoderPinBHandler(void);
void updateFlower(void);
void setWheel(byte WheelPos, float brightness);
const size_t serialPrintf(const char *szFormat, ...);
