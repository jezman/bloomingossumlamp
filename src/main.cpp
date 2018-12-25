#include "main.h"

volatile byte aFlag = 0;      // let's us know when we're expecting a rising edge on pinA to signal that the encoder has arrived at a detent
volatile byte bFlag = 0;      // let's us know when we're expecting a rising edge on pinB to signal that the encoder has arrived at a detent (opposite direction to when aFlag is set)
volatile byte encoderPos = 0; //this variable stores our current value of encoder position. Change to int or uin16_t instead of byte if you want to record a larger range than 0-255
volatile byte oldEncPos = 0;  //stores the last encoder position value so we can compare to the current reading and see if it has changed (so we know when to print to the serial monitor)
volatile byte reading = 0;    //somewhere to store the direct values we read from our interrupt pins before checking to see if we have moved a whole detent

int flowerGoalState = HIGH; // HIGH = Open, LOW = Closed
int buttonState;            // the current reading from the input pin
int lastButtonState = LOW;  // the previous reading from the input pin

long lastDebounceTime = 0; // the last time the output pin was toggled
long debounceDelay = 50;   // the debounce time; increase if the output flickers

static int servoPin = 10; //variable to store which pin is the signal pin
static int servoClosedMicros = 1800;
static int servoOpenMicros = 1400;
int pos = servoClosedMicros;

//petal animation timing variables
int frameDuration = 3000;         //number of milliseconds for complete movement
int frameElapsed = 0;             //how much of the frame has gone by, will range from 0 to frameDuration
unsigned long previousMillis = 0; //the last time we ran the position interpolation
unsigned long currentMillis = 0;  //current time, we will update this continuosly
int interval = 0;

//petal animation status variables
int movementDirection = 0; //0 = stopped, 1 opening, -1 closing

volatile float brightness = 0.0;

Adafruit_NeoPixel strip = Adafruit_NeoPixel(PIXEL_NUM, PIXEL_PIN, NEO_GRB + NEO_KHZ800);
Adafruit_TiCoServo petalServo;

void setup()
{
  pinMode(ENCODER_A_PIN, INPUT_PULLUP);
  pinMode(ENCODER_B_PIN, INPUT_PULLUP);
  pinMode(ENCODER_BUTTON_PIN, INPUT_PULLUP);

  attachInterrupt(0, encoderPinAHandler, RISING);
  attachInterrupt(1, encoderPinBHandler, RISING);

  strip.begin();
  strip.show();

  petalServo.attach(servoPin);

#if DEBUG
  Serial.begin(115200);
#endif
}

void encoderPinAHandler()
{
  cli();                //stop interrupts happening before we read pin values

  reading = PIND & 0xC; // read all eight pin values then strip away all but pinA and pinB's values

  if (reading == B00001100 && aFlag)
  {                  //check that we have both pins at detent (HIGH) and that we are expecting detent on this pin's rising edge
    encoderPos -= 5; //decrement the encoder's position count
    bFlag = 0;       //reset flags for the next turn
    aFlag = 0;       //reset flags for the next turn
  }
  else if (reading == B00000100)
    bFlag = 1; //signal that we're expecting pinB to signal the transition to detent from free rotation

  sei();       //restart interrupts
}

void encoderPinBHandler()
{
  cli();                //stop interrupts happening before we read pin values

  reading = PIND & 0xC; //read all eight pin values then strip away all but pinA and pinB's values

  if (reading == B00001100 && bFlag)
  {                  //check that we have both pins at detent (HIGH) and that we are expecting detent on this pin's rising edge
    encoderPos += 5; //increment the encoder's position count
    bFlag = 0;       //reset flags for the next turn
    aFlag = 0;       //reset flags for the next turn
  }
  else if (reading == B00001000)
    aFlag = 1; //signal that we're expecting pinA to signal the transition to detent from free rotation

  sei();       //restart interrupts
}

// Input a value 0 to 255 to get a color value.
// The colours are a transition r - g - b - back to r.
uint32_t Wheel(byte WheelPos, float brightness)
{
  WheelPos = 255 - WheelPos;

  PRINT("\nratio:", brightness);

  if (WheelPos < 85)
  {
    return strip.Color(brightness * (255 - WheelPos * 3), 0, brightness * (WheelPos * 3));
  }

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
  {
    strip.setPixelColor(i, Wheel(WheelPos, brightness));
  }

  for (uint16_t i = strip.numPixels() / 2; i < strip.numPixels(); i++)
  {
    strip.setPixelColor(i, Wheel(128 - WheelPos, brightness));
  }

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
    int newServoMicros = (servoClosedMicros + int(frameElapsedRatio * (servoOpenMicros - servoClosedMicros)));
    petalServo.write(newServoMicros);
  }

  setWheel(encoderPos, brightness);
}

void loop()
{
  if (oldEncPos != encoderPos)
  {
    Serial.println(encoderPos);
    updateFlower();
    oldEncPos = encoderPos;
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
          if (flowerGoalState == HIGH)
          {
            movementDirection = 1;
          }
          else
          {
            movementDirection = -1;
          }
          PRINT("\nmovement direction:", movementDirection);
          petalServo.attach(servoPin);
        }
        PRINTS("\nPush button pushed");
      }
    }
  }

  lastButtonState = reading;
  updateFlower();
}
