#include <NewPing.h>

#define TRIGGER_PIN  2  // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN     3  // Arduino pin tied to echo pin on the ultrasonic sensor.
#define MAX_DISTANCE 400 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.
#define MIN_DISTANCE 5 // Minimum distance we want to ping to avoid collision.

//L298N
//Motor A
const int motorPin1  = 9;
const int motorPin2  = 10;
//Motor B
const int motorPin3  = 5;
const int motorPin4  = 6;

NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); // NewPing setup of pins and maximum distance.

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);
  pinMode(motorPin3, OUTPUT);
  pinMode(motorPin4, OUTPUT);
}
void forward() {
  digitalWrite(motorPin1, LOW);
  digitalWrite(motorPin2, HIGH);
  digitalWrite(motorPin3, HIGH);
  digitalWrite(motorPin4, LOW);
}
void reverse() {
  digitalWrite(motorPin1, HIGH);
  digitalWrite(motorPin2, LOW);
  digitalWrite(motorPin3, LOW);
  digitalWrite(motorPin4, HIGH);
}
void nomove() {
  digitalWrite(motorPin1, LOW);
  digitalWrite(motorPin2, LOW);
  digitalWrite(motorPin3, LOW);
  digitalWrite(motorPin4, LOW);
}
void right() {
  digitalWrite(motorPin1, HIGH);
  digitalWrite(motorPin2, LOW);
  digitalWrite(motorPin3, HIGH);
  digitalWrite(motorPin4, LOW);
}
void left() {
  digitalWrite(motorPin1, LOW);
  digitalWrite(motorPin2, HIGH);
  digitalWrite(motorPin3, LOW);
  digitalWrite(motorPin4, HIGH);
}
void avoidCollision() {
  unsigned int uS = sonar.ping(); // Send ping, get ping time in microseconds (uS).
  delay(25);
  unsigned int uS2 = sonar.ping(); // Send ping, get ping time in microseconds (uS).
  int distance = max(uS / US_ROUNDTRIP_CM, uS2 / US_ROUNDTRIP_CM);
  if (distance != 0 && distance <= MIN_DISTANCE) {
    nomove();
  }
  delay(25);
}
void loop() {
  unsigned int uS = 0;
  avoidCollision();
  if (Serial.available() > 0) {
    switch (Serial.read()) {
      case 48: // 0 Sonar value
        nomove();
        uS = sonar.ping(); // Send ping, get ping time in microseconds (uS).
        Serial.print(uS / US_ROUNDTRIP_CM); // Convert ping time to distance and print result (LOW = outside set distance range, no ping echo)
        Serial.println(" cm");
        break;
      case 56: // 8 Forward
        forward();
        break;
      case 50: // 2 Reverse
        reverse();
        delay(500);
        nomove();
        break;
      case 53: // 5 Stop
        nomove();
        break;
      case 52: // 4 Left
        left();
        delay(150);
        nomove();
        break;
      case 54: // 6 Right
        right();
        delay(150);
        nomove();
        break;
      default: // Stop
        nomove();
        break;
    }
  }

}
