#include <ESP32Servo.h>

// Define motor control pins
#define ENA 25
#define IN1 23
#define IN2 33
#define ENB 26
#define IN3 21
#define IN4 22

// Define motor PWM channels
#define ENA_CHANNEL 0
#define ENB_CHANNEL 1
#define PWM_FREQ 1000 // 1 kHz PWM frequency
#define PWM_RES 8     // 8-bit resolution (0-255)

// Define servo and ultrasonic sensor pins
#define trigPin 4
#define echoPin 5
#define distanceMin 20
#define servoPin 12

// Variables for distance calculation
int duration, distance;
int dist_left, dist_right;
Servo myservo;

void setup() {
  Serial.begin(9600);

  // Attach the servo to the pin
  myservo.attach(servoPin);
  myservo.write(90); // Center the servo

  // Configure motor pins as outputs
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // Set up PWM channels for motors
  ledcSetup(ENA_CHANNEL, PWM_FREQ, PWM_RES);
  ledcAttachPin(ENA, ENA_CHANNEL);

  ledcSetup(ENB_CHANNEL, PWM_FREQ, PWM_RES);
  ledcAttachPin(ENB, ENB_CHANNEL);

  // Configure ultrasonic sensor pins
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
}

void loop() {
  myservo.write(90); // Center servo
  delay(500);        // Wait for servo to move
  int dist_front = calculateDistance();

  if (dist_front < distanceMin) {
    stopMoving();
    myservo.write(180); // Rotate servo to the left
    delay(1000);
    dist_left = calculateDistance();

    myservo.write(0); // Rotate servo to the right
    delay(1000);
    dist_right = calculateDistance();

    if (dist_left > distanceMin || dist_right > distanceMin) {
      if (dist_right > dist_left) {
        spinRight();
      } else {
        spinLeft();
      }
    } else {
      moveBackward();
      delay(500);
    }
  } else {
    moveForward();
    delay(1000);
  }
}

// Function to calculate distance using ultrasonic sensor
int calculateDistance() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  distance = duration * 0.034 / 2;
  Serial.print("Distance: ");
  Serial.println(distance);
  return distance;
}

// Motor control functions using ledcWrite()
void moveForward() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  ledcWrite(ENA_CHANNEL, 200); // Set speed for motor A
  ledcWrite(ENB_CHANNEL, 200); // Set speed for motor B
}

void moveBackward() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  ledcWrite(ENA_CHANNEL, 200); // Set speed for motor A
  ledcWrite(ENB_CHANNEL, 200); // Set speed for motor B
}

void spinRight() {
  Serial.println("Spinning right...");
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  ledcWrite(ENA_CHANNEL, 200); // Set speed for motor A
  ledcWrite(ENB_CHANNEL, 200); // Set speed for motor B
}

void spinLeft() {
  Serial.println("Spinning left...");
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  ledcWrite(ENA_CHANNEL, 200); // Set speed for motor A
  ledcWrite(ENB_CHANNEL, 200); // Set speed for motor B
}

void stopMoving() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  ledcWrite(ENA_CHANNEL, 0); // Stop motor A
  ledcWrite(ENB_CHANNEL, 0); // Stop motor B
}
