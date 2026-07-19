#include <WiFi.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include <ESP32Servo.h> 

// --- Wi-Fi & UDP Settings ---
const char* ssid = "TEST_RIG_003"; // Connect your computer to this Wi-Fi network
const char* password = "password123";
WiFiUDP udp;
const int udpPort = 8888;
IPAddress remoteIP; 
uint16_t remotePort;

// --- Hardware Pins ---
const int MOTOR_LEFT_PIN = 18;
const int MOTOR_RIGHT_PIN = 19;

// --- Objects ---
Servo motorLeft;
Servo motorRight;

// --- PID & Control Variables ---
float kp = 0.0, ki = 0.0, kd = 0.0;
float setpoint = 0.0;
float current_angle = 0.0;

float error = 0.0;
float previous_error = 0.0;
float integral = 0.0;

unsigned long last_time = 0;
unsigned long last_send_time = 0;

// Base throttle to get the motors spinning but not flying away
// 1000 = Motors off, 2000 = Max throttle
int BASE_THROTTLE = 1150; 

// --- MPU6050 Variables ---
const int MPU_ADDR = 0x68;
unsigned long mpu_last_time = 0;

void setup() {
  Serial.begin(115200);
  Wire.begin(); // Start I2C for MPU6050 (Pins 21 & 22)

  // 1. Setup MPU6050 (Wake it up)
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); // Power management register
  Wire.write(0);    // Clear sleep mode
  Wire.endTransmission(true);

  // 2. Setup Wi-Fi Access Point
  Serial.println("Starting Wi-Fi Access Point...");
  WiFi.softAP(ssid, password);
  Serial.print("ESP32 IP Address: ");
  Serial.println(WiFi.softAPIP()); // This will be 192.168.4.1

  // 3. Start UDP "Mailbox"
  udp.begin(udpPort);
  Serial.println("UDP Mailbox Open on Port 8888");

  // 4. Setup ESCs (Strict Limits & Arming sequence)
  motorLeft.attach(MOTOR_LEFT_PIN, 1000, 2000);
  motorRight.attach(MOTOR_RIGHT_PIN, 1000, 2000);
  
  Serial.println("Arming ESCs... Keep rig completely still!");
  motorLeft.writeMicroseconds(1000); 
  motorRight.writeMicroseconds(1000);
  delay(3000); // Wait 3 seconds for ESCs to beep and arm
  Serial.println("ESCs Armed. System Ready.");

  last_time = millis();
  mpu_last_time = micros();
}

void loop() {
  // 1. READ SENSOR (MPU6050 Complementary Filter)
  readMPU6050();

  // 2. CHECK MAILBOXES (Listen for Python GUI)
  checkMailboxes();

  // 3. RUN PID MATH
  unsigned long current_time = millis();
  float dt = (current_time - last_time) / 1000.0; // Time step in seconds
  
  if (dt > 0) {
    error = setpoint - current_angle;
    integral += (error * dt);
    
    // Anti-Windup: Cap the I-term so it doesn't grow out of control
    integral = constrain(integral, -400, 400); 

    float derivative = (error - previous_error) / dt;
    float pid_output = (kp * error) + (ki * integral) + (kd * derivative);

    previous_error = error;
    last_time = current_time;

    // 4. APPLY MATH TO MOTORS
    // Assuming positive angle means left side is down: Left pushes harder, right backs off.
    int pwm_left = BASE_THROTTLE + pid_output;
    int pwm_right = BASE_THROTTLE - pid_output;

    // EMERGENCY KILL SWITCH: If you set Kp, Ki, Kd to 0 in Python, motors shut off.
    if (kp == 0.0 && ki == 0.0 && kd == 0.0) {
      pwm_left = 900;
      pwm_right = 900;
      integral = 0; // Reset math
    }

    // Strict Safety Limits: Never drop below 1000 or exceed a safe testing max (1500)
    pwm_left = constrain(pwm_left, 1000, 1500); 
    pwm_right = constrain(pwm_right, 1000, 1500);

    // Send signals to ESCs
    motorLeft.writeMicroseconds(pwm_left);
    motorRight.writeMicroseconds(pwm_right);
  }

  // 5. SEND DATA BACK TO PYTHON GUI (Every 20ms / 50Hz)
  if (current_time - last_send_time >= 20) {
    char payload[15];
    snprintf(payload, sizeof(payload), "%.2f\n", current_angle);

    // Print to Serial (so you can watch it in the Arduino IDE Plotter too)
    Serial.print("Angle:");
    Serial.println(current_angle);

    // Send to Wi-Fi (Only if Python script has pinged us and we saved its IP)
    if (remoteIP) {
      udp.beginPacket(remoteIP, remotePort);
      udp.print(payload);
      udp.endPacket();
    }
    
    last_send_time = current_time;
  }
}

// ==========================================
// HELPER FUNCTIONS
// ==========================================

void checkMailboxes() {
  char incomingPacket[255];
  bool dataReceived = false;

  // Check UDP (Wi-Fi)
  int packetSize = udp.parsePacket();
  if (packetSize) {
    remoteIP = udp.remoteIP();     // Save Python's Return Address
    remotePort = udp.remotePort(); // Save Python's Return Door
    
    int len = udp.read(incomingPacket, 255);
    if (len > 0) incomingPacket[len] = 0; 
    dataReceived = true;
  }
  
  // Check Serial (USB)
  else if (Serial.available() > 0) {
    int len = Serial.readBytesUntil('\n', incomingPacket, 255);
    if (len > 0) incomingPacket[len] = 0; 
    dataReceived = true;
  }

  // Parse the incoming string "Kp,Ki,Kd,Setpoint"
  if (dataReceived) {
    if (strncmp(incomingPacket, "PING", 4) != 0) { // Ignore the setup ping
      float new_p, new_i, new_d, new_sp;
      if (sscanf(incomingPacket, "%f,%f,%f,%f", &new_p, &new_i, &new_d, &new_sp) == 4) {
        kp = new_p;
        ki = new_i;
        kd = new_d;
        setpoint = new_sp;
      }
    }
  }
}

void readMPU6050() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B); 
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 14, true);

  int16_t accX = Wire.read() << 8 | Wire.read();
  int16_t accY = Wire.read() << 8 | Wire.read();
  int16_t accZ = Wire.read() << 8 | Wire.read();
  
  Wire.read(); Wire.read(); // Skip Temperature
  
  int16_t gyroX = Wire.read() << 8 | Wire.read();

  unsigned long current_micros = micros();
  float dt = (current_micros - mpu_last_time) / 1000000.0;
  mpu_last_time = current_micros;

  // --- HARDWARE OFFSET APPLIED HERE ---
  // The + 9.1 corrects the physical tilt of the sensor on your rig
  float accel_pitch = (atan2(accY, accZ) * 180.0 / PI); 
  
  float gyro_rate = gyroX / 131.0; 

  // Complementary Filter
  current_angle = 0.98 * (current_angle + (gyro_rate * dt)) + 0.02 * accel_pitch;
}
