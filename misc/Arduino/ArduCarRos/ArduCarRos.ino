#include <NewPing.h>
#include <ArduinoJson.h>

#define SERIALBAUDRATE 9600
//HC-SR04 specification
#define TRIGGER_PIN  2  // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN     3  // Arduino pin tied to echo pin on the ultrasonic sensor.
#define MIN_DISTANCE 10 // Minimum distance we want to ping for (in centimeters).
#define MAX_DISTANCE 300 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.

//L298N
//Motor A
const int motorPin1  = 9;
const int motorPin2  = 10;
//Motor B
const int motorPin3  = 11;
const int motorPin4  = 12;

NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); // NewPing setup of pins and maximum distance.

void setup() {
  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);
  pinMode(motorPin3, OUTPUT);
  pinMode(motorPin4, OUTPUT);
  Serial.begin(SERIALBAUDRATE);
}

void publishFrontUltrasound() {
  unsigned int range = sonar.ping_cm();
  if (range == 0) {
    return;
  }
  String message = "";
  Serial.println(message + MIN_DISTANCE + "," + MAX_DISTANCE + "," + range);
  if (range < MIN_DISTANCE) {
      Serial.flush();
      analogWrite(motorPin1, 200);
      analogWrite(motorPin2, 0);
      analogWrite(motorPin3, 0);
      analogWrite(motorPin4, 200);
      delay(500);
      analogWrite(motorPin1, 0);
      analogWrite(motorPin2, 0);
      analogWrite(motorPin3, 0);
      analogWrite(motorPin4, 0);
  }
  delay(50);
}

void loop() {
  publishFrontUltrasound();
  if (Serial.available() > 0) {
    char serialbuffer[72] = "";
    DynamicJsonBuffer jsonBuffer;

    // If anything comes in Serial (USB)
    Serial.readBytes(serialbuffer, sizeof(serialbuffer));

    // Test if parsing succeeds.
    JsonObject& root = jsonBuffer.parseObject(serialbuffer);
    if (!root.success())
    {
      // Nothing is sent
      return;
    }
    else
    {
      analogWrite(motorPin1, root["pin1"]);
      analogWrite(motorPin2, root["pin2"]);
      analogWrite(motorPin3, root["pin3"]);
      analogWrite(motorPin4, root["pin4"]);
    }
  }
}

