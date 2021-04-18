#include <FastLED.h>

// Wallflower by Jason Stiles

// named constant for phototransistor
const int lightSensorPin = A0;
int lightSensorPinValue = 0; // variable to hold current light sensor value
int lightSensorHighValue = 0; // variable to calibrate light high value
int lightSensorLowValue = 1023; // variable to calibrate light sensor low value

// Named constants for LED light strip pins
#define REDPIN   3
#define GREENPIN 10
#define BLUEPIN  11

// named constants for the Ultrasonic sensor
const int pingPin = 7; // Trigger Pin of Ultrasonic Sensor
const int echoPin = 6; // Echo Pin of Ultrasonic Sensor

// named constants for motor pins
const int motorPinOne =  9; // the number of the motor pin 1
const int motorPinTwo =  5; // the number of the motor pin 2
int motorsRunning = 0; // 0 = motors running, 1 = motors not running

// number of seconds to push the water through
int drainMs = 5000;

// furthest number of inches the person or object must be in order to run the motors
int distanceInInches = 30;

// values for ultrasonic sensor
long duration, inches, cm;

void setup() {
  // setup logging
  Serial.begin(9600); // Starting Serial Terminal

  // set light sensor pin to output
  pinMode(lightSensorPin, OUTPUT);
  // calibrate the light sensor
  calibrateLightSensor();

  // initialize the motor pin 1 as an output:
  pinMode(motorPinOne, OUTPUT);
  // initialize the motor pin 2 as an output:
  pinMode(motorPinTwo, OUTPUT);

  // LED light strip configuration
  pinMode(REDPIN,   OUTPUT);
  pinMode(GREENPIN, OUTPUT);
  pinMode(BLUEPIN,  OUTPUT);

  // Flash the "hello" color sequence: R, G, B, black.
  showColorBars();
}

void loop() {
  runLightSensor();
  runUltrasonicSensor();
  runMotors();

//  Serial.println("Light Sensor Pin Value: ");
//  Serial.println(lightSensorPinValue);

  if (lightSensorPinValue == 0 || lightSensorPinValue < lightSensorLowValue) {
    showRainbow();
  } else {
    turnOffLEDs();
  }
}

void calibrateLightSensor() {
  // calibrate for the first five seconds after program runs
  while (millis() < 5000) {
    lightSensorPinValue = analogRead(lightSensorPin);
    delay(500);

    // record the maximum sensor value
    if (lightSensorPinValue > lightSensorHighValue) {
      lightSensorHighValue = lightSensorPinValue;
    }

    // record the minimum sensor value
    if (lightSensorPinValue < lightSensorLowValue) {
      lightSensorLowValue = lightSensorPinValue;
    }
  }

//  Serial.println("Sensor high is:");
//  Serial.println(lightSensorHighValue);
//  Serial.println("Sensor low is:");
//  Serial.println(lightSensorLowValue);
}

void runLightSensor() {
  lightSensorPinValue = analogRead(lightSensorPin);
}

void runMotors() {

  // check if the switch is pressed.
  if (inches <= distanceInInches) {
    Serial.print(inches);
    Serial.print("in");
    Serial.println();

    showOrange();
    motorsRunning = 1;

    // turn motor on:
    digitalWrite(motorPinOne, HIGH);
    delay(drainMs);
    // turn motor off:
    digitalWrite(motorPinOne, LOW);

    // turn 2nd motor on
    digitalWrite(motorPinTwo, HIGH);
    delay(drainMs);
    // turn second motor off
    digitalWrite(motorPinTwo, LOW);

    motorsRunning = 0;
    turnOffLEDs();
  }

}

void runUltrasonicSensor() {
  pinMode(pingPin, OUTPUT);
  digitalWrite(pingPin, LOW);
  delayMicroseconds(2);
  digitalWrite(pingPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(pingPin, LOW);
  pinMode(echoPin, INPUT);
  duration = pulseIn(echoPin, HIGH);
  inches = microsecondsToInches(duration);
  cm = microsecondsToCentimeters(duration);
  delay(100);
}

long microsecondsToInches(long microseconds) {
  return microseconds / 74 / 2;
}

long microsecondsToCentimeters(long microseconds) {
  return microseconds / 29 / 2;
}

// showAnalogRGB: this is like FastLED.show(), but outputs on
// analog PWM output pins instead of sending data to an intelligent,
// pixel-addressable LED strip.
//
// This function takes the incoming RGB values and outputs the values
// on three analog PWM output pins to the r, g, and b values respectively.
void showAnalogRGB( const CRGB& rgb)
{
  analogWrite(REDPIN,   rgb.r );
  analogWrite(GREENPIN, rgb.g );
  analogWrite(BLUEPIN,  rgb.b );
}

void showRainbow() {
  static uint8_t hue;
  hue = hue + 1;
  // Use FastLED automatic HSV->RGB conversion 10 = Blue, 100 = Green, 150 = Yellow, 155 = Orange, 160 = Red
  showAnalogRGB( CHSV( hue, 255, 255) );
  Serial.print("HUE is ");
  Serial.print(hue);
  Serial.println();

  delay(20);
}

void showOrange() {
  showAnalogRGB(CHSV(155, 255, 255));
}

void turnOffLEDs() {
  showAnalogRGB( CRGB::Black );
}

// showColorBars: flashes Red, then Green, then Blue, then Black.
// Helpful for diagnosing if you've mis-wired which is which.
void showColorBars()
{
  showAnalogRGB( CRGB::Red );   delay(500);
  showAnalogRGB( CRGB::Green ); delay(500);
  showAnalogRGB( CRGB::Blue );  delay(500);
  showAnalogRGB( CRGB::Black ); delay(500);
}


