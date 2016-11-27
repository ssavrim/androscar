#include <NewPing.h>
#include <ArduinoJson.h>

#define SERIALBAUDRATE 9600
#define MAX_MILLIS_TO_WAIT 500
#define PAYLOAD_SIZE 8 // Size of the payload which contains data to set pin of the motors.
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
void emergencyStop() {
  // Stop the robot
  analogWrite(motorPin1, 0);
  analogWrite(motorPin2, 0);
  analogWrite(motorPin3, 0);
  analogWrite(motorPin4, 0);
  // Empty serial buffer
  while(Serial.available())
    Serial.read();
  // Reverse the robot
  analogWrite(motorPin1, 255);
  analogWrite(motorPin2, 0);
  analogWrite(motorPin3, 0);
  analogWrite(motorPin4, 255);
  delay(250);
  analogWrite(motorPin1, 0);
  analogWrite(motorPin2, 0);
  analogWrite(motorPin3, 0);
  analogWrite(motorPin4, 0);
}
void publishFrontUltrasound() {
  unsigned int range = sonar.ping_cm();
  if (range == 0) {
    return;
  }
  String message = "";
  Serial.println(message + MIN_DISTANCE + "," + MAX_DISTANCE + "," + range);
  if (range <= MIN_DISTANCE) {
    emergencyStop();
  }
  delay(50);
}
int hex2int(byte input) {
  int result = -1;
  // Value shall be between [0-9] or [A-F]
  switch (input) {
    case 'A':
      result = 10;
      break;
    case 'B':
      result = 11;
      break;
    case 'C':
      result = 12;
      break;
    case 'D':
      result = 13;
      break;
    case 'E':
      result = 14;
      break;
    case 'F':
      result = 15;
      break;
    default:
      String res = "";
      res += (char) input;
      result = res.toInt();
  }
  return result;
}
void loop() {
  //unsigned long starttime;
  int availableBytes = Serial.available();
  if (availableBytes == PAYLOAD_SIZE) {
    int myPinValues[] = {0, 0, 0, 0};
    //starttime = millis();
    for(int n=0; n<PAYLOAD_SIZE/2; n++) {
      myPinValues[n] = hex2int(Serial.read()) * 16 + hex2int(Serial.read());
    }
    //Serial.println(millis() - starttime);
    analogWrite(motorPin1, myPinValues[0]);
    analogWrite(motorPin2, myPinValues[1]);
    analogWrite(motorPin3, myPinValues[2]);
    analogWrite(motorPin4, myPinValues[3]);
  } else {
    // Simply empty the buffer
    for(int n=0; n<availableBytes; n++)
      Serial.read();
  }
  publishFrontUltrasound();
  /*if (Serial.available() > 0) {  
    // If anything comes in Serial (USB)
    int start = millis();
    for(int n=0; n<8; n++)
        Serial.print(Serial.read()); // Then: Get them.
    int stop = millis();
    Serial.println("");
   
    Serial.println(stop - start);
    analogWrite(motorPin1, 0);
    analogWrite(motorPin2, 0);
    analogWrite(motorPin3, 0);
    analogWrite(motorPin4, 0);
  }*/
}

