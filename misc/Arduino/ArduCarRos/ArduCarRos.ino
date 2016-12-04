#include <NewPing.h>

#define SERIALBAUDRATE 9600
#define PAYLOAD_SIZE 8 // Size of the payload which contains data to set pin of the motors.
//HC-SR04 specification
#define SONAR_NUM     2 // Number of sensors.2
#define LEFT_TRIGGER_PIN  3  // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define LEFT_ECHO_PIN     2  // Arduino pin tied to echo pin on the ultrasonic sensor.
#define RIGHT_TRIGGER_PIN  4  // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define RIGHT_ECHO_PIN     5  // Arduino pin tied to echo pin on the ultrasonic sensor.
#define MIN_DISTANCE 6 // Minimum distance we want to ping for (in centimeters).
#define MAX_DISTANCE 300 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.
#define PING_INTERVAL 33 // Milliseconds between sensor pings (29ms is about the min to avoid cross-sensor echo).

//L298N
//Motor A
const int motorPin1  = 9;
const int motorPin2  = 10;
//Motor B
const int motorPin3  = 11;
const int motorPin4  = 12;


unsigned long pingTimer[SONAR_NUM]; // Holds the times when the next ping should happen for each sensor.
unsigned int cm[SONAR_NUM];         // Where the ping distances are stored.
uint8_t currentSensor = 0;          // Keeps track of which sensor is active.

String sonarName[SONAR_NUM] = {"left", "right"};
NewPing sonar[SONAR_NUM] = {     // Sensor object array.
  NewPing(LEFT_TRIGGER_PIN, LEFT_ECHO_PIN, MAX_DISTANCE), // Each sensor's trigger pin, echo pin, and max distance to ping.
  NewPing(RIGHT_TRIGGER_PIN, RIGHT_ECHO_PIN, MAX_DISTANCE)
};

void setup() {
  
  // Initialize sonars
  pingTimer[0] = millis() + 75;           // First ping starts at 75ms, gives time for the Arduino to chill before starting.
  for (uint8_t i = 1; i < SONAR_NUM; i++) // Set the starting time for each sensor.
    pingTimer[i] = pingTimer[i - 1] + PING_INTERVAL;
  // Initialier motor pins
  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);
  pinMode(motorPin3, OUTPUT);
  pinMode(motorPin4, OUTPUT);
  Serial.begin(SERIALBAUDRATE);
}
void echoCheck() { // If ping received, set the sensor distance to array.
  if (sonar[currentSensor].check_timer())
    cm[currentSensor] = sonar[currentSensor].ping_result / US_ROUNDTRIP_CM;
}
void sonarLoop() {
  for (uint8_t i = 0; i < SONAR_NUM; i++) { // Loop through all the sensors.
    if (millis() >= pingTimer[i]) {         // Is it this sensor's time to ping?
      pingTimer[i] += PING_INTERVAL * SONAR_NUM;  // Set next time this sensor will be pinged.
      if (i == 0 && currentSensor == SONAR_NUM - 1) publishSonarValues(); // Sensor ping cycle complete, do something with the results.
      sonar[currentSensor].timer_stop();          // Make sure previous timer is canceled before starting a new ping (insurance).
      currentSensor = i;                          // Sensor being accessed.
      cm[currentSensor] = 0;                      // Make distance zero in case there's no ping echo for this sensor.
      sonar[currentSensor].ping_timer(echoCheck); // Do the ping (processing continues, interrupt will call echoCheck to look for echo).
    }
  }
}
void publishSonarValues() {
// Sensor ping cycle complete, do something with the results.
  // The following code would be replaced with your code that does something with the ping results.
  for (uint8_t i = 0; i < SONAR_NUM; i++) {
    Serial.println(sonarName[i] + "," + MIN_DISTANCE + "," + MAX_DISTANCE + "," + cm[i]);
  }
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
  sonarLoop();
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
  }
}

