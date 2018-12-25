#include <Arduino.h>
#include <Adafruit_TiCoServo.h>
#include <Adafruit_NeoPixel.h>

#define DEBUG 0

#if DEBUG
#define PRINTS(s)   { Serial.print(F(s)); }
#define PRINT(s,v)  { Serial.print(F(s)); Serial.print(v); }
#else
#define PRINTS(s)
#define PRINT(s,v)
#endif

#define PIXEL_PIN 4
#define PIXEL_NUM 16
#define ENCODER_A_PIN 2
#define ENCODER_B_PIN 3
#define ENCODER_BUTTON_PIN 7

#define SERVO_PIN 10
#define SERVO_CLOSED_MICROS 1800
#define SERVO_OPEN_MICROS 1400

void encoderPinAISR(void);
void encoderPinBISR(void);
void updateFlower(void);
void setWheel(byte WheelPos, float brightness);
const size_t serialPrintf(const char *szFormat, ...);
void rotateHandler(uint8_t, uint8_t);
