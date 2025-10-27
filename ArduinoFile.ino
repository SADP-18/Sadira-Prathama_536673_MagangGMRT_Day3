/*
 * Gadjah Mada Robotics Team 2026
 * Day 3 Assignment - Servo Control with MPU6050 and PIR
 * Programmer: Sadira Prathama
 * NIM: 24/536673/TK/59550
 */

#include <Arduino.h>
#include <Wire.h> 
#include <ESP32Servo.h> 
#include <MPU6050.h>

// Configure the PIN used for Servo, PIR Sensor, MPU6050 Sensor and LED
#define SERVO1_PIN 13
#define SERVO2_PIN 12
#define SERVO3_PIN 14
#define SERVO4_PIN 27
#define SERVO5_PIN 26
#define PIR_PIN    33
#define LED_PIN    2
#define I2C_SDA    22 
#define I2C_SCL    21 

// MPU6050 I2C address
#define MPU6050_ADDR 0x68

// Object Servo
Servo servo1, servo2, servo3, servo4, servo5;

// Control variable
float roll = 0, pitch = 0, yaw = 0; // Orientation angles
unsigned long lastUpdate = 0; // Timing yaw updates
unsigned long motionTimer = 0; // Timer for motion event
bool motionActive = false; // Indicates motion state in PIR

const int initialPos = 90; // Initialize center position or starting position

// Initial setup
void setup() {
  Serial.begin(115200);
  Wire.begin(I2C_SDA, I2C_SCL); // Starting I2C for MPU6050
  
  // Initialize MPU6050
  setupMPU6050();
  
  pinMode(PIR_PIN, INPUT); // PIR sensor as input
  pinMode(LED_PIN, OUTPUT); // LED as output
  digitalWrite(LED_PIN, LOW); // Setting initial condition for LED

  // Attach the servo to their respective pins
  servo1.attach(SERVO1_PIN);
  servo2.attach(SERVO2_PIN);
  servo3.attach(SERVO3_PIN);
  servo4.attach(SERVO4_PIN);
  servo5.attach(SERVO5_PIN);

  // Set all of the servo in the initial position (90 degrees)
  servo1.write(initialPos);
  servo2.write(initialPos);
  servo3.write(initialPos);
  servo4.write(initialPos);
  servo5.write(initialPos);

  lastUpdate = millis(); // Start time tracking
}

void setupMPU6050() {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x6B); // PWR_MGMT_1 register
  Wire.write(0);    // Wake up MPU6050
  Wire.endTransmission(true);
  
  // Test connection
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x75); 
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDR, 1, true);
  byte whoAmI = Wire.read();
  
  if (whoAmI == 0x68) {
    Serial.println("MPU6050 Successfully Connected!");
  } else {
    Serial.println("MPU6050 Failed to Connect!"); // Stop the program if connection failed
    while (1);
  }
}

// Read MPU6050 data
void readMPU6050(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz) {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x3B); // Starting register for accelerometer data
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDR, 14, true); // Read 14 bytes
  
  // Read accelerometer data
  *ax = Wire.read() << 8 | Wire.read();
  *ay = Wire.read() << 8 | Wire.read();
  *az = Wire.read() << 8 | Wire.read();
  
  // Read temperature
  Wire.read() << 8 | Wire.read();
  
  // Read gyroscope data
  *gx = Wire.read() << 8 | Wire.read();
  *gy = Wire.read() << 8 | Wire.read();
  *gz = Wire.read() << 8 | Wire.read();
}

// Main loop function
void loop() {
  // Read the MPU6050 data
  int16_t ax, ay, az, gx, gy, gz;
  readMPU6050(&ax, &ay, &az, &gx, &gy, &gz);

  // Calculate roll and pitch
  roll = atan2(ay, az) * 180.0 / PI;
  pitch = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / PI; 

  // Calculate yaw from gyro
  unsigned long now = millis();
  float dt = (now - lastUpdate) / 1000.0; // Time difference in seconds
  lastUpdate = now;
  yaw += (gz / 131.0) * dt; // Integrate gyro rate to get yaw
  yaw = constrain(yaw, -90, 90); // Condition: Limit the yaw between -90 and 90 degrees

  // Roll Motion
  if (abs(roll) > 5 && !motionActive) { // If theres tilt detected and not interrupted by PIR
    int offset = map((int)roll, -90, 90, -45, 45);
    // Opposite direction for balance
    servo1.write(constrain(initialPos - offset, 0, 180));
    servo2.write(constrain(initialPos + offset, 0, 180));
  } else if (!motionActive) {
    // If no tilt detected, return to neutral
    servo1.write(initialPos);
    servo2.write(initialPos);
  }

  // Pitch Motion
  if (abs(pitch) > 5 && !motionActive) { // Check if the pitch angle is larger than 5 degree
    int offset = map((int)pitch, -90, 90, -45, 45); // Map pitch value (-90 to 90) and movement range (-45 to 45).
    // Move servo 3 and 4 in the same direction and keeping the final position between 0 and 180 degree
    servo3.write(constrain(initialPos + offset, 0, 180));
    servo4.write(constrain(initialPos + offset, 0, 180));
  } else if (!motionActive) { // If theres no significant pitch movement and system is not responding PIR
    // Reset servo 3 and 4 to their centered position which is 90
    servo3.write(initialPos);
    servo4.write(initialPos);
  }

 // Yaw Motion
  static unsigned long yawStart = 0; // Saving the time when yaw movement starts
  static bool yawMoving = false; 

  if (abs(yaw) > 5 && !motionActive && !yawMoving) { // Check whether the tilt is more than 5 degree
    int offset = map((int)yaw, -90, 90, -45, 45); // Converts yaw angle (-90 to 90) into smaller servo range (-45 to 45)
    servo5.write(constrain(initialPos + offset, 0, 180)); // Use constrain to ensures servo position stays in its range
    // Mark the yaw movement in progress and the current time
    yawMoving = true;
    yawStart = millis();
  }

  // After 1 seconds, back to the initial position
  if (yawMoving && millis() - yawStart > 1000) {
    servo5.write(initialPos);
    yawMoving = false;
  }

  // Detect external motion using PIR
  int pirVal = digitalRead(PIR_PIN);
  if (pirVal == HIGH && !motionActive) {
    Serial.println("External Motion Detected!");
    motionActive = true;
    digitalWrite(LED_PIN, HIGH); // Turn the LED on to indicates motion detection
    motionTimer = millis();

    // Move to free position (we decide the degree)
    servo1.write(45);
    servo2.write(135);
    servo3.write(45);
    servo4.write(135);
    servo5.write(60);
  }

  // After 2 seconds, all the servos back to the initial position
  if (motionActive && millis() - motionTimer > 2000) {
    servo1.write(initialPos);
    servo2.write(initialPos);
    servo3.write(initialPos);
    servo4.write(initialPos);
    servo5.write(initialPos);
    digitalWrite(LED_PIN, LOW); // Turn the indicator light off
    motionActive = false;
    Serial.println("Back to the original position.");
  }

  // Debug and update the data every 0.5 seconds
  static unsigned long debugTimer = 0; // Stores the last time data was print
  if (millis() - debugTimer > 500) { // Check whether the last data print is more than 500 miliseconds ago
    Serial.print("Roll: "); Serial.print(roll);
    Serial.print(" | Pitch: "); Serial.print(pitch);
    Serial.print(" | Yaw: "); Serial.println(yaw);
    debugTimer = millis(); // Reset the timer
  }

  delay(50); // For stability add delay
}
