#include <NewPing.h>
#include <ArduinoJson.h>

#define SERIALBAUDRATE 9600
#define TRIGGER_PIN  2  // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN     3  // Arduino pin tied to echo pin on the ultrasonic sensor.
#define MAX_DISTANCE 450 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.

//L298N
//Motor A
const int motorPin1  = 9;
const int motorPin2  = 10;
//Motor B
const int motorPin3  = 11;
const int motorPin4  = 12;

NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); // NewPing setup of pins and maximum distance.

void setup() {
  // put your setup code here, to run once:
  Serial.begin(SERIALBAUDRATE);
  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);
  pinMode(motorPin3, OUTPUT);
  pinMode(motorPin4, OUTPUT);
}

void publishFrontUltrasound() {
  unsigned int distance = sonar.ping_cm(); // Send ping, get ping time in microseconds (uS).
  Serial.println(distance);
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
      delay(50);     
    }
  }
}

