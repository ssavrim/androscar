#include <NewPing.h>

#define SERIALBAUDRATE 9600
#define PAYLOAD_SIZE 4 // Size of the payload which contains data to set pin of the motors.
//HC-SR04 specification
#define SONAR_NUM     3 // Number of sensors.
#define LEFT_TRIGGER_PIN  3  // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define LEFT_ECHO_PIN     2  // Arduino pin tied to echo pin on the ultrasonic sensor.
#define CENTER_TRIGGER_PIN  7  // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define CENTER_ECHO_PIN     6  // Arduino pin tied to echo pin on the ultrasonic sensor.
#define RIGHT_TRIGGER_PIN  4  // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define RIGHT_ECHO_PIN     5  // Arduino pin tied to echo pin on the ultrasonic sensor.
#define MIN_DISTANCE 10 // Minimum distance we want to ping for (in centimeters).
#define MAX_DISTANCE 150 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.
#define PING_INTERVAL 30 // Milliseconds between sensor pings (29ms is about the min to avoid cross-sensor echo).
#define SAMPLING_SIZE 3 // Sampling of ping size. The objective is to reduce noise.

//L298N
//Motor A
const int motorPin1  = 9;
const int motorPin2  = 10;
//Motor B
const int motorPin3  = 11;
const int motorPin4  = 12;

unsigned long pingTimer[SONAR_NUM]; // Holds the times when the next ping should happen for each sensor.
unsigned int cm[SONAR_NUM];         // Last ping distances.
unsigned int sampling[SONAR_NUM][SAMPLING_SIZE];   // Sampling of ping distances.
uint8_t currentSensor = 0;          // Keeps track of which sensor is active.

String sonarName[SONAR_NUM] = {"left", "right", "center"};
NewPing sonar[SONAR_NUM] = {     // Sensor object array.
  NewPing(LEFT_TRIGGER_PIN, LEFT_ECHO_PIN, MAX_DISTANCE), // Each sensor's trigger pin, echo pin, and max distance to ping.
  NewPing(RIGHT_TRIGGER_PIN, RIGHT_ECHO_PIN, MAX_DISTANCE),
  NewPing(CENTER_TRIGGER_PIN, CENTER_ECHO_PIN, MAX_DISTANCE)
};

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
      cm[currentSensor] = MAX_DISTANCE;           // Make distance to maximum. We consider that in this case the maximum distance is reached.
      sonar[currentSensor].ping_timer(echoCheck); // Do the ping (processing continues, interrupt will call echoCheck to look for echo).
    }
  }
}
void publishSonarValues() {
    unsigned int range;
    // Sensor ping cycle complete, do something with the results.
    for (uint8_t i = 0; i < SONAR_NUM; i++) {
        sampling[i][SAMPLING_SIZE-1] = cm[i];
        range = cm[i];
        for (uint8_t j = 0; j < SAMPLING_SIZE-1; j++) {
            sampling[i][j] = sampling[i][j+1];
            range += sampling[i][j];
        }
        Serial.println(sonarName[i] + "," + MIN_DISTANCE + "," + MAX_DISTANCE + "," + range/SAMPLING_SIZE);
        //Serial.print(range/SAMPLING_SIZE);
        //Serial.print(",");
        avoidCollision(range/SAMPLING_SIZE);
    }
    //Serial.println("");
}
void avoidCollision(unsigned int distance)
{
    if (distance <= MIN_DISTANCE) {
        analogWrite(motorPin1, 0);
        analogWrite(motorPin2, 0);
        analogWrite(motorPin3, 0);
        analogWrite(motorPin4, 0);
        while(Serial.available() > 0) {
            // Empty the serial buffer
            Serial.read();
        }
    }
}
void motorLoop() {
  int availableBytes = Serial.available();
  if (Serial.available() == PAYLOAD_SIZE) {
    analogWrite(motorPin1, Serial.read());
    analogWrite(motorPin2, Serial.read());
    analogWrite(motorPin3, Serial.read());
    analogWrite(motorPin4, Serial.read());
  }
}
void setup() {
  // Initialize sonars
  pingTimer[0] = millis() + 75;           // First ping starts at 75ms, gives time for the Arduino to chill before starting.
  for (uint8_t i = 1; i < SONAR_NUM; i++) // Set the starting time for each sensor.
    pingTimer[i] = pingTimer[i - 1] + PING_INTERVAL;

  for (uint8_t i = 0; i < SONAR_NUM; i++) {
      for (uint8_t j = 0; j < SAMPLING_SIZE; j++) {
          sampling[i][j] = MAX_DISTANCE;
      }
  }
  // Initialize motor pins
  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);
  pinMode(motorPin3, OUTPUT);
  pinMode(motorPin4, OUTPUT);
  Serial.begin(SERIALBAUDRATE);
}
void loop() {
    sonarLoop();
    motorLoop();
}

