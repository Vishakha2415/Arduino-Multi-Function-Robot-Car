/*obstacle avoiding, Bluetooth control, voice control robot car.
   Home Page
*/

// Include necessary libraries
#include <Servo.h>     // Library for controlling the servo motor
#include <AFMotor.h>   // Library for controlling the motors via L293D shield

// Define pin connections and constants
#define Echo A0        // Ultrasonic sensor Echo pin connected to A0
#define Trig A1        // Ultrasonic sensor Trig pin connected to A1
#define motor 10       // Servo motor connected to digital pin 10
#define Speed 170      // Motor speed value (range 0-255)
#define spoint 103     // Servo center position/starting point (degrees)

// Variable declarations
char value;           // Stores incoming character from Bluetooth
int distance;         // Stores distance measured by ultrasonic sensor
int Left;             // Stores distance measurement from left side
int Right;            // Stores distance measurement from right side
int L = 0;            // Temporary variable for left distance in obstacle mode
int R = 0;            // Temporary variable for right distance in obstacle mode
int L1 = 0;           // Additional variable (not currently used in code)
int R1 = 0;           // Additional variable (not currently used in code)

// Create objects for controlling hardware
Servo servo;          // Create servo object to control the servo motor
AF_DCMotor M1(1);     // Create motor object M1 connected to motor port 1 on shield
AF_DCMotor M2(2);     // Create motor object M2 connected to motor port 2 on shield
AF_DCMotor M3(3);     // Create motor object M3 connected to motor port 3 on shield
AF_DCMotor M4(4);     // Create motor object M4 connected to motor port 4 on shield

void setup() {
  // Initialize serial communication at 9600 baud rate (for Bluetooth)
  Serial.begin(9600);
  
  // Configure ultrasonic sensor pins
  pinMode(Trig, OUTPUT);  // Trig pin as output (sends ultrasonic pulse)
  pinMode(Echo, INPUT);   // Echo pin as input (receives reflected pulse)
  
  // Attach servo motor to the defined pin
  servo.attach(motor);
  
  // Set initial speed for all motors
  M1.setSpeed(Speed);
  M2.setSpeed(Speed);
  M3.setSpeed(Speed);
  M4.setSpeed(Speed);
}

void loop() {
  // Main program loop - only ONE function should be uncommented at a time
  //Obstacle();           // Uncomment for autonomous obstacle avoidance mode
  //Bluetoothcontrol();   // Uncomment for Bluetooth remote control mode
  //voicecontrol();       // Uncomment for voice control mode
}

// Bluetooth Control Function - handles remote control via Bluetooth
void Bluetoothcontrol() {
  // Check if data is available from Bluetooth module
  if (Serial.available() > 0) {
    value = Serial.read();     // Read the incoming character
    Serial.println(value);     // Echo back for debugging (optional)
  }
  
  // Execute movement based on received character
  if (value == 'F') {          // 'F' received - move forward
    forward();
  } else if (value == 'B') {   // 'B' received - move backward
    backward();
  } else if (value == 'L') {   // 'L' received - turn left
    left();
  } else if (value == 'R') {   // 'R' received - turn right
    right();
  } else if (value == 'S') {   // 'S' received - stop all motors
    Stop();
  }
}

// Obstacle Avoidance Function - autonomous navigation avoiding obstacles
void Obstacle() {
  // Get distance to obstacle in front
  distance = ultrasonic();
  
  // If obstacle is within 12cm
  if (distance <= 12) {
    Stop();                // 1. Stop immediately
    backward();            // 2. Move backward briefly
    delay(100);
    Stop();                // 3. Stop again
    
    // 4. Check distance on left side
    L = leftsee();         // Turn servo left and measure distance
    servo.write(spoint);   // Return servo to center position
    delay(800);            // Wait for servo movement
    
    // 5. Check distance on right side
    R = rightsee();        // Turn servo right and measure distance
    servo.write(spoint);   // Return servo to center position
    
    // 6. Decide which direction has more space
    if (L < R) {           // More space on right side
      right();             // Turn right
      delay(500);          // Turn for 500ms
      Stop();
      delay(200);
    } else if (L > R) {    // More space on left side
      left();              // Turn left
      delay(500);          // Turn for 500ms
      Stop();
      delay(200);
    }
  } else {
    forward();             // No obstacle detected, keep moving forward
  }
}

// Voice Control Function - handles voice commands via Bluetooth
void voicecontrol() {
  // Check if voice command data is available
  if (Serial.available() > 0) {
    value = Serial.read();     // Read the incoming character
    Serial.println(value);     // Echo back for debugging
    
    // Execute command based on special character mapping
    if (value == '^') {        // '^' received - move forward
      forward();
    } else if (value == '-') { // '-' received - move backward
      backward();
    } else if (value == '<') { // '<' received - turn left
      L = leftsee();           // First check left side distance
      servo.write(spoint);     // Return servo to center
      if (L >= 10 ) {          // If at least 10cm space available
        left();                // Turn left
        delay(500);            // Turn for 500ms
        Stop();
      } else if (L < 10) {     // If less than 10cm space
        Stop();                // Don't turn, just stop
      }
    } else if (value == '>') { // '>' received - turn right
      R = rightsee();          // First check right side distance
      servo.write(spoint);     // Return servo to center
      if (R >= 10 ) {          // If at least 10cm space available
        right();               // Turn right
        delay(500);            // Turn for 500ms
        Stop();
      } else if (R < 10) {     // If less than 10cm space
        Stop();                // Don't turn, just stop
      }
    } else if (value == '*') { // '*' received - stop all motors
      Stop();
    }
  }
}

// Ultrasonic sensor distance reading function
int ultrasonic() {
  // Send a 10 microsecond pulse to trigger the sensor
  digitalWrite(Trig, LOW);
  delayMicroseconds(4);
  digitalWrite(Trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(Trig, LOW);
  
  // Measure the time it takes for the echo to return
  long t = pulseIn(Echo, HIGH);
  
  // Convert time to distance in centimeters
  // Formula: distance = (time in microseconds) / 29 / 2
  // 29 comes from speed of sound (340 m/s ≈ 29 cm/ms)
  // Divide by 2 because sound travels to object and back
  long cm = t / 29 / 2;
  
  return cm;  // Return distance in cm
}

// Movement Functions - control the 4 motors for different movements

// Move forward: all 4 motors rotate forward
void forward() {
  M1.run(FORWARD);
  M2.run(FORWARD);
  M3.run(FORWARD);
  M4.run(FORWARD);
}

// Move backward: all 4 motors rotate backward
void backward() {
  M1.run(BACKWARD);
  M2.run(BACKWARD);
  M3.run(BACKWARD);
  M4.run(BACKWARD);
}

// Turn right: left side motors backward, right side motors forward
// This causes the robot to pivot/turn right
void right() {
  M1.run(BACKWARD);  // Motor 1 (front left) backward
  M2.run(BACKWARD);  // Motor 2 (front right) backward
  M3.run(FORWARD);   // Motor 3 (rear left) forward
  M4.run(FORWARD);   // Motor 4 (rear right) forward
}

// Turn left: right side motors backward, left side motors forward
// This causes the robot to pivot/turn left
void left() {
  M1.run(FORWARD);   // Motor 1 (front left) forward
  M2.run(FORWARD);   // Motor 2 (front right) forward
  M3.run(BACKWARD);  // Motor 3 (rear left) backward
  M4.run(BACKWARD);  // Motor 4 (rear right) backward
}

// Stop all motors: release/stop all 4 motors
void Stop() {
  M1.run(RELEASE);
  M2.run(RELEASE);
  M3.run(RELEASE);
  M4.run(RELEASE);
}

// Look right function: turn servo to 20 degrees and measure distance
int rightsee() {
  servo.write(20);           // Turn servo to look right (20 degrees)
  delay(800);                // Wait for servo to reach position
  Left = ultrasonic();       // Measure distance (actually measures left side when looking right)
  return Left;               // Return measured distance
}

// Look left function: turn servo to 180 degrees and measure distance
int leftsee() {
  servo.write(180);          // Turn servo to look left (180 degrees)
  delay(800);                // Wait for servo to reach position
  Right = ultrasonic();      // Measure distance (actually measures right side when looking left)
  return Right;              // Return measured distance
}
