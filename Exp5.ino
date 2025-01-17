#include <Servo.h>  // Include the Servo library
// Define motor control pins
const int motor1PWM = 2;   // PWM input for Motor 1
const int motor1Dir1 = 20;  // Direction input 1 for Motor 1
const int motor1Dir2 = 1;  // Direction input 2 for Motor 1
const int motor2PWM = 5;  // PWM input for Motor 2
const int motor2Dir1 = 6;  // Direction input 1 for Motor 2
const int motor2Dir2 = 9;  // Direction input 2 for Motor 2
// Define servo control for grippers
const int servoPin1 = 3;  // Servo pin for the gripper (PWM capable)
Servo gripperServo1;       // Create a Servo object to control the first gripper
const int servoPin2 = 4;  // Servo pin for the second gripper (PWM capable)
Servo gripperServo2;       // Create a Servo object to control the second gripper
void setup() {
  // Set motor control pins as OUTPUT
  pinMode(motor1PWM, OUTPUT);
  pinMode(motor1Dir1, OUTPUT);
  pinMode(motor1Dir2, OUTPUT);
  pinMode(motor2PWM, OUTPUT);
  pinMode(motor2Dir1, OUTPUT);
  pinMode(motor2Dir2, OUTPUT);
  // Attach servos to their respective pins
  gripperServo1.attach(servoPin1);
  gripperServo2.attach(servoPin2);
  // Initialize servos to default positions
  //gripperServo1.write(90);  // Middle position (neutral)
  //gripperServo2.write(90);  // Middle position (neutral)
  Serial.begin(9600);  // Initialize Bluetooth communication on Serial1
}
void loop() {
  // Wait for Bluetooth input
  if (Serial.available() > 0) {
    char command = Serial.read();  // Read from Bluetooth
    executeCommand(command);
  }
}
void executeCommand(char command) {
  switch (command) {
    case 'B':  // Move Backward
      moveBackward();
      break;
    case 'F':  // Move Forward
      Serial.println("Moving Forward");  // Send response to Bluetooth
      moveForward();
      break;
    case 'L':  // Move Left
      moveLeft();
      break;
    case 'R':  // Move Right
      moveRight();
      break;
    case 'S':  // Stop Motors
      stopMotors();
      break;
    case 'O':  // Open the gripper
      openGripper();
      break;
    case 'C':  // Close the gripper
      closeGripper();
      break;
    case 'U':  // Lift the gripper
      gripperUp();
      break;
    case 'D':  // Lower the gripper
      gripperDown();
      break;
    default:
      break;
  }
}
// Rest of the movement and gripper functions remain the same

// Functions to control robot movement
void moveForward() {
  digitalWrite(motor1Dir1, HIGH);
  digitalWrite(motor1Dir2, LOW);
  digitalWrite(motor2Dir1, HIGH);
  digitalWrite(motor2Dir2, LOW);
  analogWrite(motor1PWM, 140);  // Adjust PWM value if needed
  analogWrite(motor2PWM, 175);  // Adjust PWM value if needed
  Serial.println("Robot moving forward");
}
void moveBackward() {
  digitalWrite(motor1Dir1, LOW);
  digitalWrite(motor1Dir2, HIGH);
  digitalWrite(motor2Dir1, LOW);
  digitalWrite(motor2Dir2, HIGH);
  analogWrite(motor1PWM, 130);  // Adjust PWM value if needed
  analogWrite(motor2PWM, 170);  // Adjust PWM value if needed
  Serial.println("Robot moving backward");
}
void moveRight() {
  digitalWrite(motor1Dir1, HIGH);
  digitalWrite(motor1Dir2, LOW);
  digitalWrite(motor2Dir1, LOW);
  digitalWrite(motor2Dir2, HIGH);
  analogWrite(motor1PWM, 150);  // Adjust PWM value if needed
  analogWrite(motor2PWM, 150);  // Adjust PWM value if needed
  Serial.println("Robot moving left");
}
void moveLeft() {
  digitalWrite(motor1Dir1, LOW);
  digitalWrite(motor1Dir2, HIGH);
  digitalWrite(motor2Dir1, HIGH);
  digitalWrite(motor2Dir2, LOW);
  analogWrite(motor1PWM, 150);  // Adjust PWM value if needed
  analogWrite(motor2PWM, 150);  // Adjust PWM value if needed
  Serial.println("Robot moving right");
}
void stopMotors() {
  digitalWrite(motor1Dir1, LOW);
  digitalWrite(motor1Dir2, LOW);
  digitalWrite(motor2Dir1, LOW);
  digitalWrite(motor2Dir2, LOW);
  analogWrite(motor1PWM, 0);
  analogWrite(motor2PWM, 0);
  Serial.println("Robot stops moving");
}
// Functions to control the grippers
void openGripper() {
  gripperServo1.write(150);  // Open position for first gripper
  Serial.println("Gripper Open");
}
void closeGripper() {
  gripperServo1.write(10);  // Close position for first gripper
  Serial.println("Gripper Close");
}
void gripperUp() {
  gripperServo2.write(65);  // Up position for second gripper
  Serial.println("Gripper Up");
}
void gripperDown() {
  gripperServo2.write(90);  // Down position for second gripper
  Serial.println("Gripper Down");
}
