#include "main.h"

Adafruit_NeoPixel strip = Adafruit_NeoPixel(PIXEL_NUM, PIXEL_PIN, NEO_GRB + NEO_KHZ800);
Adafruit_TiCoServo petalServo;

volatile uint8_t aFlag = 0;
volatile uint8_t bFlag = 0;
volatile uint8_t encoderPosition = 0;
volatile uint8_t oldEncoderPosition = 0;
volatile uint8_t reading = 0;

uint8_t flowerGoalState = HIGH;
uint8_t buttonState;
uint8_t lastButtonState = LOW;

uint_fast16_t servoPosition = SERVO_CLOSED_MICROS;

int frameDuration = 3000;         //number of milliseconds for complete movement
int frameElapsed = 0;             //how much of the frame has gone by, will range from 0 to frameDuration
int interval = 0;
unsigned long previousMillis = 0; //the last time we ran the servoPositionition interpolation
unsigned long currentMillis = 0;  //current time, we will update this continuosly

int8_t movementDirection = 0; //0 = stopped, 1 opening, -1 closing

volatile float brightness = 0.0;
long lastDebounceTime = 0;
long debounceDelay = 50;

void setup()
{
  pinMode(ENCODER_A_PIN, INPUT_PULLUP);
  pinMode(ENCODER_B_PIN, INPUT_PULLUP);
  pinMode(ENCODER_BUTTON_PIN, INPUT_PULLUP);

  attachInterrupt(0, encoderPinAISR, RISING);
  attachInterrupt(1, encoderPinBISR, RISING);

  strip.begin();
  strip.show();

  petalServo.attach(SERVO_PIN);

#if DEBUG
  Serial.begin(115200);
#endif
}

void loop()
{
  if (oldEncoderPosition != encoderPosition)
  {
    Serial.println(encoderPosition);
    updateFlower();
    oldEncoderPosition = encoderPosition;
  }

  int reading = digitalRead(ENCODER_BUTTON_PIN);

  if (reading != lastButtonState)
    lastDebounceTime = millis();

  if ((millis() - lastDebounceTime) > debounceDelay)
  {
    if (reading != buttonState)
    {
      buttonState = reading;

      if (buttonState == HIGH)
      {
        if (movementDirection == 0)
        {
          flowerGoalState = !flowerGoalState;
          movementDirection = (flowerGoalState == HIGH) ? 1 : -1;
          petalServo.attach(SERVO_PIN);

          PRINT("\nmovement direction:", movementDirection);
        }
        PRINTS("\nPush button pushed");
      }
    }
  }

  lastButtonState = reading;
  updateFlower();
}

// Input a value 0 to 255 to get a color value.
// The colours are a transition r - g - b - back to r.
uint32_t Wheel(byte WheelPos, float brightness)
{
  WheelPos = 255 - WheelPos;

  PRINT("\nratio:", brightness);

  if (WheelPos < 85)
    return strip.Color(brightness * (255 - WheelPos * 3), 0, brightness * (WheelPos * 3));

  if (WheelPos < 170)
  {
    WheelPos -= 85;
    return strip.Color(0, brightness * (WheelPos * 3), brightness * (255 - WheelPos * 3));
  }

  WheelPos -= 170;
  return strip.Color(brightness * (WheelPos * 3), brightness * (255 - WheelPos * 3), 0);
}

//apply colours from the wheel to the pixels
void setWheel(byte WheelPos, float brightness)
{
  for (uint16_t i = 0; i < strip.numPixels() / 2; i++)
    strip.setPixelColor(i, Wheel(WheelPos, brightness));

  for (uint16_t i = strip.numPixels() / 2; i < strip.numPixels(); i++)
    strip.setPixelColor(i, Wheel(128 - WheelPos, brightness));

  strip.show();
}

void updateFlower()
{
  unsigned long currentMillis = millis();
  interval = currentMillis - previousMillis;
  previousMillis = currentMillis;

  frameElapsed += movementDirection * interval;
  float frameElapsedRatio = float(frameElapsed) / float(frameDuration);
  brightness = frameElapsedRatio;

  if (frameElapsed < 0)
  {
    movementDirection = 0;
    frameElapsed = 0;
    PRINTS("\nclosed");
    brightness = 0.0;
    petalServo.detach();
  }
  if (frameElapsed > frameDuration)
  {
    movementDirection = 0;
    frameElapsed = frameDuration;
    PRINTS("\nopened");
    brightness = 1.0;
    petalServo.detach();
  }

  if (movementDirection != 0)
  {
    int newServoMicros = (SERVO_CLOSED_MICROS + int(frameElapsedRatio * (SERVO_OPEN_MICROS - SERVO_CLOSED_MICROS)));
    petalServo.write(newServoMicros);
  }

  setWheel(encoderPosition, brightness);
}

void rotateHandler(uint8_t bit, uint8_t flag) {
  cli();

  reading = PIND & 0xC;

  if (reading == B00001100 && flag)
  { 
    (bit == 4) ? (encoderPosition -= 5) : (encoderPosition += 5);
    bFlag = 0;
    aFlag = 0;
  }
  else if (reading == bit)
    (bit == 4) ? (bFlag = 1): (aFlag = 1);

  sei();
}

void encoderPinAISR() { rotateHandler(B00000100, aFlag); }
void encoderPinBISR() { rotateHandler(B00001000, bFlag); }
