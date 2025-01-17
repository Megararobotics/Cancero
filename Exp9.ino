#include <Servo.h>  // Include Servo library

// Ultrasonic sensor pins
const int trigPin = 17;
const int echoPin = 16;

// Motor control pins
const int motor1A = 20;
const int motor1B = 1;
#define ENA 2
const int motor2A = 6;
const int motor2B = 9;
#define ENB 5

// Servo setup
Servo gripperServo1;  // Create a Servo object for the first gripper
Servo gripperServo2;  // Create a Servo object for the second gripper
const int servoPin1 = 3;  // Pin for the first gripper (PWM capable)
const int servoPin2 = 4;  // Pin for the second gripper (PWM capable)

// Variables
long duration, distance;
bool obstacleMode = false;  // Track if 'A' command is active

void setup() {
  // Pin modes for ultrasonic sensor
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  // Pin modes for motor control
  pinMode(motor1A, OUTPUT);
  pinMode(motor1B, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(motor2A, OUTPUT);
  pinMode(motor2B, OUTPUT);
  pinMode(ENB, OUTPUT);

  // Attach the servos
  gripperServo1.attach(servoPin1);
  gripperServo2.attach(servoPin2);

  // Initialize Serial communication (shared with Bluetooth on pins 0 and 1)
  Serial2.begin(9600);  

  Serial2.println("Setup Complete.");
}

void loop() {
  distance = getDistance();  // Continuously monitor distance
  if (obstacleMode) {
    followDistanceLogic();  // Adjust motor behavior based on distance if 'A' is active
  }
  waitForBluetoothCommand();  // Check for new Bluetooth commands at all times
}

// Function to read distance from ultrasonic sensor
long getDistance() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH);
  long distance = (duration / 2) / 28.5;  // Convert to cm
  return distance;
}

// Bluetooth command handler
void waitForBluetoothCommand() {
  if (Serial2.available() > 0) {
    char command = Serial2.read();  // Read Bluetooth command
    executeCommand(command);
  }
}

void executeCommand(char command) {
  switch (command) {
    case 'F':
      obstacleMode = false;  // Disable obstacle mode if 'F' is received
      moveMotors(150);
      break;
    case 'B':
      obstacleMode = false;
      moveBackward();
      break;
    case 'L':
      obstacleMode = false;
      moveLeft();
      break;
    case 'R':
      obstacleMode = false;
      moveRight();
      break;
    case 'S':
      obstacleMode = false;
      stopMotors();
      break;
    case 'O':
      openGripper();
      break;
    case 'C':
      closeGripper();
      break;
    case 'U':
      gripperUp();
      break;
    case 'A':
      obstacleMode = true;  // Enable obstacle-aware movement
      Serial2.println("Obstacle mode activated");
      break;
    default:
      Serial2.println("Invalid Command");
      break;
  }
}

// Function to move motors at a given speed
void moveMotors(int speed) {
  digitalWrite(motor1A, HIGH);
  digitalWrite(motor1B, LOW);
  digitalWrite(motor2A, HIGH);
  digitalWrite(motor2B, LOW);
  analogWrite(ENA, speed);
  analogWrite(ENB, speed);
}

void stopMotors() {
  digitalWrite(motor1A, LOW);
  digitalWrite(motor1B, LOW);
  digitalWrite(motor2A, LOW);
  digitalWrite(motor2B, LOW);
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}

// Function to control motors based on distance logic
void followDistanceLogic() {
  Serial2.print("Distance: ");
  Serial2.println(distance);

  if (distance >= 80) {
    moveMotors(255);
    Serial2.println("Obstacle is at far distance");
  } else if (distance >= 50) {
    moveMotors(175);
    Serial2.println("Obstacle is at distance >=50");
  } else if (distance >= 10) {
    moveMotors(100);
    Serial2.println("Obstacle is at distance >=10");
  } else {
    stopMotors();
    Serial2.println("Obstacle is nearby");
  }
}

// Gripper control functions
void openGripper() {
  gripperServo1.write(150);
  Serial2.println("Gripper Open");
}

void closeGripper() {
  gripperServo1.write(10);
  Serial2.println("Gripper Closed");
}

void gripperUp() {
  gripperServo2.write(65);
  Serial2.println("Gripper Up");
}

void gripperDown() {
  gripperServo2.write(90);
  Serial2.println("Gripper Down");
}

// Functions to control robot movement
void moveForward() {
  digitalWrite(motor1A, HIGH);
  digitalWrite(motor1B, LOW);
  digitalWrite(motor2A, HIGH);
  digitalWrite(motor2B, LOW);
  analogWrite(ENA, 140);  // Adjust PWM value if needed
  analogWrite(ENB, 165);  // Adjust PWM value if needed
  Serial2.println("Robot moving forward");
}
 
void moveBackward() {
  digitalWrite(motor1A, LOW);
  digitalWrite(motor1B, HIGH);
  digitalWrite(motor2A, LOW);
  digitalWrite(motor2B, HIGH);
  analogWrite(ENA, 145);  // Adjust PWM value if needed
  analogWrite(ENB, 165);  // Adjust PWM value if needed
  Serial2.println("Robot moving backward");
}
 
void moveLeft() {
  digitalWrite(motor1A, HIGH);
  digitalWrite(motor1B, LOW);
  digitalWrite(motor2A, LOW);
  digitalWrite(motor2B, HIGH);
  analogWrite(ENA, 150);  // Adjust PWM value if needed
  analogWrite(ENB, 150);  // Adjust PWM value if needed
  Serial2.println("Robot moving left");
}
 
void moveRight() {
  digitalWrite(motor1A, LOW);
  digitalWrite(motor1B, HIGH);
  digitalWrite(motor2A, HIGH);
  digitalWrite(motor2B, LOW);
  analogWrite(ENA, 150);  // Adjust PWM value if needed
  analogWrite(ENB, 150);  // Adjust PWM value if needed
  Serial2.println("Robot moving right");
}
