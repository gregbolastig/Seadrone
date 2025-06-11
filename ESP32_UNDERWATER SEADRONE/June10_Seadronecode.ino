#include "Wire.h"
#include <WiFi.h> // Include for Wi-Fi capabilities
#include <WebServer.h> // Include for web server

// MPU-6050 Constants
const int MPU_ADDR = 0x68;  // I2C address of the MPU-6050
const float ACCEL_SENSITIVITY = 16384.0;  // LSB/g for ±2g range

// Pin definitions for ESP32
const int RELAY_LEFT_1_PIN = 27;
const int RELAY_LEFT_2_PIN = 14;
const int RELAY_RIGHT_1_PIN = 25;
const int RELAY_RIGHT_2_PIN = 26;
const int RELAY_FORWARD_1_PIN = 32;  // New forward relay 1
const int RELAY_FORWARD_2_PIN = 33;  // New forward relay 2
const float ACCEL_THRESHOLD = 0.200;  // Threshold for acceleration in g

// Ultrasonic sensor pins
const int TRIG_PIN = 5;   // GPIO5 for trigger
const int ECHO_PIN = 18; // GPIO18 for echo
const int TRIG_PIN_2 = 4;   // GPIO4 for trigger of the second sensor (obstacle detection)
const int ECHO_PIN_2 = 19; // GPIO19 for echo of the second sensor (obstacle detection)
const int DISTANCE_THRESHOLD = 45; // Distance threshold in cm
const int DISTANCE_THRESHOLD_2 = 45; // Distance threshold in cm for obstacle detection

// RELAY CONTROL LOGIC
const bool RELAY_ON = LOW;   // Most relay modules are active LOW
const bool RELAY_OFF = HIGH; // Relays are off when pin is HIGH

// I2C pins for ESP32 (optional - ESP32 has default I2C pins)
const int SDA_PIN = 21;   // Default SDA pin on ESP32
const int SCL_PIN = 22;   // Default SCL pin on ESP32

// Autonomous forward movement variables
const unsigned long INITIAL_COUNTDOWN = 30000; //30 seconds initial countdown
const unsigned long FORWARD_ON_TIME = 3000;    // 3 seconds ON time
const unsigned long FORWARD_OFF_TIME = 6000;   // 6 seconds OFF time
unsigned long forwardStartTime = 0;
unsigned long forwardCycleStartTime = 0;
bool initialCountdownComplete = false;
bool forwardCycleActive = false; // true = ON phase, false = OFF phase
enum ForwardState { FORWARD_INITIAL_COUNTDOWN, FORWARD_ON_PHASE, FORWARD_OFF_PHASE };
ForwardState forwardState = FORWARD_INITIAL_COUNTDOWN;

// Variables for raw sensor data
int16_t accel_x_raw, accel_y_raw, accel_z_raw;

// Variables for processed data
float accel_x, accel_y, accel_z;  // in g-forces

// Ultrasonic sensor variables
long duration, duration2;
int distance, distance2;

// Calibration variables
float accel_x_offset = 0, accel_y_offset = 0, accel_z_offset = 0;
const int CALIBRATION_SAMPLES = 1000;

// Auto-ON after 2 minutes variables
const unsigned long AUTO_ON_DELAY = 120000; // 2 minutes in milliseconds
unsigned long startMillis = 0;
bool autoOnActive = false;

// --- Web Server Variables ---
const char* ssid = "HUAWEI-9ZEM";         // REPLACE WITH YOUR WIFI SSID
const char* password = "daWHKx5A"; // REPLACE WITH YOUR WIFI PASSWORD

WebServer server(80); // Create a web server on port 80

// Global variable to control the main logic
bool mainLogicEnabled = false; // Starts OFF by default

// --- HTML for the web page ---
String createControlPage() {
  String html = "<!DOCTYPE html><html><head>";
  html += "<title>Seadrone Monitor Control</title>";
  html += "<meta name='viewport' content='width=device-width, initial-scale=1'>";
  html += "<meta http-equiv='refresh' content='5'>"; // Auto-refresh every 5 seconds
  html += "<style>";

  // CSS Styles
  html += "* { margin: 0; padding: 0; box-sizing: border-box; }";
  html += "body { font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif; ";
  html += "background: linear-gradient(135deg, #667eea 0%, #764ba2 100%); ";
  html += "min-height: 100vh; color: white; padding: 20px; }";

  html += ".container { max-width: 1200px; margin: 0 auto; }";

  html += ".header { text-align: center; margin-bottom: 30px; }";
  html += ".header h1 { font-size: 2.5em; margin-bottom: 10px; text-shadow: 2px 2px 4px rgba(0,0,0,0.3); }";
  html += ".header .subtitle { font-size: 1.2em; opacity: 0.9; }";

  html += ".status-grid { display: grid; grid-template-columns: repeat(auto-fit, minmax(280px, 1fr)); gap: 20px; margin-bottom: 30px; }";

  html += ".status-card { background: rgba(255,255,255,0.1); backdrop-filter: blur(10px); ";
  html += "border-radius: 15px; padding: 20px; border: 1px solid rgba(255,255,255,0.2); }";

  html += ".status-card h3 { margin-bottom: 15px; font-size: 1.3em; display: flex; align-items: center; }";
  html += ".status-card .icon { margin-right: 10px; font-size: 1.5em; }";

  html += ".status-item { display: flex; justify-content: space-between; align-items: center; ";
  html += "margin-bottom: 10px; padding: 8px; background: rgba(255,255,255,0.1); border-radius: 8px; }";

  html += ".status-badge { padding: 4px 12px; border-radius: 20px; font-size: 0.9em; font-weight: bold; }";
  html += ".status-on { background: #4CAF50; color: white; }";
  html += ".status-off { background: #f44336; color: white; }";
  html += ".status-active { background: #ff9800; color: white; }";
  html += ".status-inactive { background: #9e9e9e; color: white; }";

  html += ".control-panel { text-align: center; margin: 30px 0; }";
  html += ".main-button { background: linear-gradient(45deg, #4CAF50, #45a049); ";
  html += "border: none; color: white; padding: 20px 40px; font-size: 1.3em; ";
  html += "border-radius: 50px; cursor: pointer; text-decoration: none; display: inline-block; ";
  html += "box-shadow: 0 8px 25px rgba(0,0,0,0.2); transition: all 0.3s ease; ";
  html += "text-transform: uppercase; font-weight: bold; letter-spacing: 1px; }";

  html += ".main-button:hover { transform: translateY(-2px); box-shadow: 0 12px 35px rgba(0,0,0,0.3); }";
  html += ".main-button.off { background: linear-gradient(45deg, #f44336, #d32f2f); }";

  html += ".sensor-grid { display: grid; grid-template-columns: repeat(auto-fit, minmax(200px, 1fr)); gap: 15px; margin-top: 20px; }";
  html += ".sensor-item { background: rgba(255,255,255,0.1); padding: 15px; border-radius: 10px; text-align: center; }";
  html += ".sensor-value { font-size: 1.5em; font-weight: bold; margin-bottom: 5px; }";
  html += ".sensor-label { font-size: 0.9em; opacity: 0.8; }";

  html += ".relay-grid { display: grid; grid-template-columns: repeat(auto-fit, minmax(120px, 1fr)); gap: 10px; }";
  html += ".relay-item { padding: 10px; border-radius: 8px; text-align: center; font-size: 0.9em; }";
  html += ".relay-on { background: rgba(76, 175, 80, 0.8); }";
  html += ".relay-off { background: rgba(158, 158, 158, 0.8); }";

  html += ".footer { text-align: center; margin-top: 40px; padding-top: 20px; ";
  html += "border-top: 1px solid rgba(255,255,255,0.2); opacity: 0.7; }";

  html += "@media (max-width: 768px) { ";
  html += ".header h1 { font-size: 2em; } ";
  html += ".main-button { padding: 15px 30px; font-size: 1.1em; } ";
  html += ".status-grid { grid-template-columns: 1fr; } }";

  html += "</style></head><body>";

  // HTML Content
  html += "<div class='container'>";

  // Header
  html += "<div class='header'>";
  html += "<h1>Seadrone Monitor Control</h1>";
  html += "<div class='subtitle'>Autonomous Monitoring & Control System</div>";
  html += "</div>";

  // Status Grid
  html += "<div class='status-grid'>";

  // System Status Card
  html += "<div class='status-card'>";
  html += "<h3>System Status</h3>";
  html += "<div class='status-item'>";
  html += "<span>Main Logic</span>";
  html += "<span class='status-badge ";
  html += (mainLogicEnabled ? "status-on'>ON" : "status-off'>OFF");
  html += "</span></div>";

  html += "<div class='status-item'>";
  html += "<span>Auto-ON Timer</span>";
  html += "<span class='status-badge ";
  html += (autoOnActive ? "status-active'>ACTIVE" : "status-inactive'>WAITING");
  html += "</span></div>";
  html += "</div>";

  // Sensor Data Card
  html += "<div class='status-card'>";
  html += "<h3>Sensor Data</h3>";
  html += "<div class='sensor-grid'>";
  html += "<div class='sensor-item'>";
  html += "<div class='sensor-value'>" + String(distance) + "</div>";
  html += "<div class='sensor-label'>Distance 1 (cm)</div>";
  html += "</div>";
  html += "<div class='sensor-item'>";
  html += "<div class='sensor-value'>" + String(distance2) + "</div>";
  html += "<div class='sensor-label'>Distance 2 (cm)</div>";
  html += "</div>";
  html += "</div></div>";

  // Relay Status Card
  html += "<div class='status-card'>";
  html += "<h3>Relay Status</h3>";
  html += "<div class='relay-grid'>";

  // Check each relay and display status
  html += "<div class='relay-item ";
  html += (digitalRead(RELAY_LEFT_1_PIN) == RELAY_ON ? "relay-on'>L1 (27) ON" : "relay-off'>L1 (27) OFF");
  html += "</div>";

  html += "<div class='relay-item ";
  html += (digitalRead(RELAY_LEFT_2_PIN) == RELAY_ON ? "relay-on'>L2 (14) ON" : "relay-off'>L2 (14) OFF");
  html += "</div>";

  html += "<div class='relay-item ";
  html += (digitalRead(RELAY_RIGHT_1_PIN) == RELAY_ON ? "relay-on'>R1 (25) ON" : "relay-off'>R1 (25) OFF");
  html += "</div>";

  html += "<div class='relay-item ";
  html += (digitalRead(RELAY_RIGHT_2_PIN) == RELAY_ON ? "relay-on'>R2 (26) ON" : "relay-off'>R2 (26) OFF");
  html += "</div>";

  html += "<div class='relay-item ";
  html += (digitalRead(RELAY_FORWARD_1_PIN) == RELAY_ON ? "relay-on'>F1 (32) ON" : "relay-off'>F1 (32) OFF");
  html += "</div>";

  html += "<div class='relay-item ";
  html += (digitalRead(RELAY_FORWARD_2_PIN) == RELAY_ON ? "relay-on'>F2 (33) ON" : "relay-off'>F2 (33) OFF");
  html += "</div>";

  html += "</div></div>";
  html += "</div>"; // End status-grid

  // Control Panel
  html += "<div class='control-panel'>";
  html += "<a href='/toggle'><button class='main-button ";
  html += (mainLogicEnabled ? "off" : "");
  html += "'>";
  html += (mainLogicEnabled ? "STOP SYSTEM" : "START SYSTEM");
  html += "</button></a>";
  html += "</div>";

  // Footer
  html += "<div class='footer'>";
  html += "<p>Auto-refresh every 5 seconds | ";
  html += "IP: " + WiFi.localIP().toString() + "</p>";
  html += "</div>";

  html += "</div>"; // End container
  html += "</body></html>";

  return html;
}

// --- Web Server Handlers ---
void handleRoot() {
  server.send(200, "text/html", createControlPage());
}

void handleToggle() {
  mainLogicEnabled = !mainLogicEnabled; // Toggle the state
  Serial.print("Main Logic is now: ");
  Serial.println(mainLogicEnabled ? "ON" : "OFF");

  // If logic is turned OFF, ensure all relays are off
  if (!mainLogicEnabled) {
    digitalWrite(RELAY_LEFT_1_PIN, RELAY_OFF);
    digitalWrite(RELAY_LEFT_2_PIN, RELAY_OFF);
    digitalWrite(RELAY_RIGHT_1_PIN, RELAY_OFF);
    digitalWrite(RELAY_RIGHT_2_PIN, RELAY_OFF);
    digitalWrite(RELAY_FORWARD_1_PIN, RELAY_OFF);
    digitalWrite(RELAY_FORWARD_2_PIN, RELAY_OFF);

    autoOnActive = false; // Reset auto-ON if turned off manually

    // Reset forward movement system
    forwardState = FORWARD_INITIAL_COUNTDOWN;
    initialCountdownComplete = false;
    forwardCycleActive = false;

    Serial.println("All outputs turned OFF - Main logic disabled");
  } else {
    // If logic is turned ON, restart all timers
    startMillis = millis(); // Re-start auto-ON timer

    // Start forward movement system
    forwardStartTime = millis();
    forwardState = FORWARD_INITIAL_COUNTDOWN;
    initialCountdownComplete = false;
    forwardCycleActive = false;

    Serial.println("Auto-ON timer re-started.");
    Serial.println("Starting autonomous forward movement: 30s countdown...");
  }

  server.sendHeader("Location", "/"); // Redirect back to the main page
  server.send(303);
}

void handleNotFound() {
  server.send(404, "text/plain", "Not Found");
}

void setup() {
  Serial.begin(115200);

  // Initialize I2C on ESP32 specified pins
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);

  // Initialize relay pins - MAKE SURE RELAYS START IN OFF STATE
  pinMode(RELAY_LEFT_1_PIN, OUTPUT);
  pinMode(RELAY_LEFT_2_PIN, OUTPUT);
  pinMode(RELAY_RIGHT_1_PIN, OUTPUT);
  pinMode(RELAY_RIGHT_2_PIN, OUTPUT);
  pinMode(RELAY_FORWARD_1_PIN, OUTPUT);
  pinMode(RELAY_FORWARD_2_PIN, OUTPUT);

  // Initialize ultrasonic sensor pins
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(TRIG_PIN_2, OUTPUT);
  pinMode(ECHO_PIN_2, INPUT);

  // Ensure all pins are OFF initially
  digitalWrite(RELAY_LEFT_1_PIN, RELAY_OFF);
  digitalWrite(RELAY_LEFT_2_PIN, RELAY_OFF);
  digitalWrite(RELAY_RIGHT_1_PIN, RELAY_OFF);
  digitalWrite(RELAY_RIGHT_2_PIN, RELAY_OFF);
  digitalWrite(RELAY_FORWARD_1_PIN, RELAY_OFF);
  digitalWrite(RELAY_FORWARD_2_PIN, RELAY_OFF);

  delay(100);

  // Initialize MPU-6050
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);   // PWR_MGMT_1 register
  Wire.write(0);      // Wake up the MPU-6050
  Wire.endTransmission(true);

  // Configure accelerometer range to ±2g
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1C);   // ACCEL_CONFIG register
  Wire.write(0x00);   // 0x00 = ±2g
  Wire.endTransmission(true);

  Serial.println("Calibrating sensors. Keep the MPU-6050 still...");
  calibrateSensors();
  Serial.println("Calibration complete!");
  Serial.println("All relays should now be OFF. If they're still ON, check your relay module's logic.");

  // --- Wi-Fi and Web Server Setup ---
  Serial.print("Connecting to WiFi: ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected.");
  Serial.print("Access your control page at http://");
  Serial.println(WiFi.localIP());

  server.on("/", handleRoot);
  server.on("/toggle", handleToggle);
  server.onNotFound(handleNotFound);

  server.begin();
  Serial.println("HTTP server started");

  // Initial state of mainLogicEnabled is false, so everything is OFF
  Serial.println("Main logic is currently OFF. Visit the IP address to turn it ON.");
}

void handleForwardMovement() {
  if (!mainLogicEnabled) return;

  unsigned long currentTime = millis();
  bool obstacleDetected = (distance2 <= DISTANCE_THRESHOLD_2 && distance2 > 0);

  switch (forwardState) {
    case FORWARD_INITIAL_COUNTDOWN:
      if (currentTime - forwardStartTime >= INITIAL_COUNTDOWN) {
        Serial.println("Initial countdown complete. Starting forward movement cycle.");
        forwardState = FORWARD_ON_PHASE;
        forwardCycleStartTime = currentTime;
        forwardCycleActive = true;
        initialCountdownComplete = true;
      } else {
        // Print countdown every 2 seconds
        static unsigned long lastCountdownPrint = 0;
        if (currentTime - lastCountdownPrint >= 2000) {
          lastCountdownPrint = currentTime;
          int secondsRemaining = (INITIAL_COUNTDOWN - (currentTime - forwardStartTime)) / 1000;
          Serial.print("Forward movement countdown: ");
          Serial.print(secondsRemaining);
          Serial.println(" seconds");
        }
      }
      break;

    case FORWARD_ON_PHASE:
      if (currentTime - forwardCycleStartTime >= FORWARD_ON_TIME) {
        Serial.println("Forward ON phase complete. Starting OFF phase.");
        forwardState = FORWARD_OFF_PHASE;
        forwardCycleStartTime = currentTime;
        forwardCycleActive = false;
      }
      break;

    case FORWARD_OFF_PHASE:
      if (currentTime - forwardCycleStartTime >= FORWARD_OFF_TIME) {
        Serial.println("Forward OFF phase complete. Starting ON phase.");
        forwardState = FORWARD_ON_PHASE;
        forwardCycleStartTime = currentTime;
        forwardCycleActive = true;
      }
      break;
  }

  // Control forward relays based on state and obstacle detection
  if (forwardState == FORWARD_INITIAL_COUNTDOWN) {
    // During countdown, both relays are OFF unless obstacle detected
    if (obstacleDetected) {
      digitalWrite(RELAY_FORWARD_1_PIN, RELAY_ON);  // Only pin 32 ON for obstacle
      digitalWrite(RELAY_FORWARD_2_PIN, RELAY_OFF);
    } else {
      digitalWrite(RELAY_FORWARD_1_PIN, RELAY_OFF);
      digitalWrite(RELAY_FORWARD_2_PIN, RELAY_OFF);
    }
  } else if (forwardCycleActive) {
    // During ON phase
    if (obstacleDetected) {
      digitalWrite(RELAY_FORWARD_1_PIN, RELAY_ON);  // Only pin 32 ON for obstacle
      digitalWrite(RELAY_FORWARD_2_PIN, RELAY_OFF);
    } else {
      digitalWrite(RELAY_FORWARD_1_PIN, RELAY_ON);  // Both pins ON for normal forward
      digitalWrite(RELAY_FORWARD_2_PIN, RELAY_ON);
    }
  } else {
    // During OFF phase
    if (obstacleDetected) {
      digitalWrite(RELAY_FORWARD_1_PIN, RELAY_ON);  // Only pin 32 ON for obstacle
      digitalWrite(RELAY_FORWARD_2_PIN, RELAY_OFF);
    } else {
      digitalWrite(RELAY_FORWARD_1_PIN, RELAY_OFF); // Both pins OFF
      digitalWrite(RELAY_FORWARD_2_PIN, RELAY_OFF);
    }
  }
}

void calibrateSensors() {
  float accel_x_sum = 0, accel_y_sum = 0, accel_z_sum = 0;

  for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
    readRawSensorData(); // Read only raw data for calibration
    accel_x_sum += accel_x_raw / ACCEL_SENSITIVITY;
    accel_y_sum += accel_y_raw / ACCEL_SENSITIVITY;
    // For Z-axis, subtract 1.0g to account for gravity
    accel_z_sum += accel_z_raw / ACCEL_SENSITIVITY - 1.0;
    delay(1);
  }

  // Calculate average offsets
  accel_x_offset = accel_x_sum / CALIBRATION_SAMPLES;
  accel_y_offset = accel_y_sum / CALIBRATION_SAMPLES;
  accel_z_offset = accel_z_sum / CALIBRATION_SAMPLES;

  Serial.print("Calibration offsets: X=");
  Serial.print(accel_x_offset, 3);
  Serial.print(" Y=");
  Serial.print(accel_y_offset, 3);
  Serial.print(" Z=");
  Serial.println(accel_z_offset, 3);
}

void readRawSensorData() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);   // Starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 6, true);   // Request only accelerometer registers

  // Read accelerometer data
  accel_x_raw = Wire.read() << 8 | Wire.read();
  accel_y_raw = Wire.read() << 8 | Wire.read();
  accel_z_raw = Wire.read() << 8 | Wire.read();
}

void readSensorData() {
  readRawSensorData();

  // Convert raw values to physical units with calibration
  accel_x = accel_x_raw / ACCEL_SENSITIVITY - accel_x_offset;
  accel_y = accel_y_raw / ACCEL_SENSITIVITY - accel_y_offset;
  accel_z = accel_z_raw / ACCEL_SENSITIVITY - accel_z_offset;
}

// Function to read the ultrasonic sensor
void readUltrasonic() {
  // Clear the trigPin by setting it LOW
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(5);

  // Trigger the sensor by setting the trigPin high for 10 microseconds
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // Read the echoPin. pulseIn() returns the duration (length of the pulse) in microseconds
  duration = pulseIn(ECHO_PIN, HIGH);

  // Calculate the distance
  distance = (duration/2) / 29.1;
}

// Function to read the second ultrasonic sensor (obstacle detection)
void readUltrasonic2() {
  // Clear the trigPin by setting it LOW
  digitalWrite(TRIG_PIN_2, LOW);
  delayMicroseconds(5);

  // Trigger the sensor by setting the trigPin high for 10 microseconds
  digitalWrite(TRIG_PIN_2, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN_2, LOW);

  // Read the echoPin. pulseIn() returns the duration (length of the pulse) in microseconds
  duration2 = pulseIn(ECHO_PIN_2, HIGH);

  // Calculate the distance
  distance2 = (duration2/2) / 29.1;
}

// Function to check if any relay is active
bool isAnyRelayActive() {
  return (digitalRead(RELAY_LEFT_1_PIN) == RELAY_ON ||
          digitalRead(RELAY_LEFT_2_PIN) == RELAY_ON ||
          digitalRead(RELAY_RIGHT_1_PIN) == RELAY_ON ||
          digitalRead(RELAY_RIGHT_2_PIN) == RELAY_ON ||
          digitalRead(RELAY_FORWARD_1_PIN) == RELAY_ON ||
          digitalRead(RELAY_FORWARD_2_PIN) == RELAY_ON);
}

void checkSensorsAndControl() {
  // CRITICAL: If main logic is disabled, turn everything OFF and return
  if (!mainLogicEnabled) {
    digitalWrite(RELAY_LEFT_1_PIN, RELAY_OFF);
    digitalWrite(RELAY_LEFT_2_PIN, RELAY_OFF);
    digitalWrite(RELAY_RIGHT_1_PIN, RELAY_OFF);
    digitalWrite(RELAY_RIGHT_2_PIN, RELAY_OFF);
    digitalWrite(RELAY_FORWARD_1_PIN, RELAY_OFF);
    digitalWrite(RELAY_FORWARD_2_PIN, RELAY_OFF);
    return;
  }

  // Apply a small deadzone to prevent noise from triggering relays
  const float DEADZONE = 0.02;
  float deadzone_lower = ACCEL_THRESHOLD - DEADZONE;

  // Variables to track relay status for X and Y axes
  bool x_left_active = false;
  bool x_right_active = false;
  bool y_pos_active = false;
  bool y_neg_active = false;
  bool proximity_detected = false;

  // Check if distance is below threshold for the first sensor
  if (distance <= DISTANCE_THRESHOLD && distance > 0) {   // Added > 0 to filter invalid readings
    proximity_detected = true;
  }

  // Check X acceleration against thresholds with hysteresis
  if (accel_x <= -ACCEL_THRESHOLD) {
    x_left_active = true;
  }
  else if (accel_x >= ACCEL_THRESHOLD) {
    x_right_active = true;
  }
  // If in between deadzone_lower and ACCEL_THRESHOLD, maintain previous state (hysteresis)

  // Check Y acceleration against thresholds with hysteresis
  if (accel_y >= ACCEL_THRESHOLD) {
    // Positive Y acceleration detected
    y_pos_active = true;
  }
  else if (accel_y <= -ACCEL_THRESHOLD) {
    // Negative Y acceleration detected
    y_neg_active = true;
  }

  // Check if any acceleration is detected - this is for the auto-ON override
  bool anyAccelDetected = x_left_active || x_right_active || y_pos_active || y_neg_active;

  // If auto-ON is active and no acceleration is detected, all original relay pins should be ON
  if (autoOnActive && !anyAccelDetected) {
    digitalWrite(RELAY_LEFT_1_PIN, RELAY_ON);
    digitalWrite(RELAY_LEFT_2_PIN, RELAY_ON);
    digitalWrite(RELAY_RIGHT_1_PIN, RELAY_ON);
    digitalWrite(RELAY_RIGHT_2_PIN, RELAY_ON);
    return; // Skip the normal control logic for original relays
  }

  // If proximity is detected by the first sensor, activate all original relays regardless of acceleration
  if (proximity_detected) {
    digitalWrite(RELAY_LEFT_1_PIN, RELAY_ON);
    digitalWrite(RELAY_LEFT_2_PIN, RELAY_ON);
    digitalWrite(RELAY_RIGHT_1_PIN, RELAY_ON);
    digitalWrite(RELAY_RIGHT_2_PIN, RELAY_ON);
    return; // Skip the normal acceleration-based control logic
  }

  // Control relays based on both X and Y acceleration
  // For positive Y: activate PIN 27, 25
  // For negative Y: activate PIN 14, 26

  // Control RELAY_LEFT_1_PIN (PIN 27) - activated by X-left OR Y-positive
  digitalWrite(RELAY_LEFT_1_PIN, (x_left_active || y_pos_active) ? RELAY_ON : RELAY_OFF);

  // Control RELAY_LEFT_2_PIN (PIN 14) - activated by X-left OR Y-negative
  digitalWrite(RELAY_LEFT_2_PIN, (x_left_active || y_neg_active) ? RELAY_ON : RELAY_OFF);

  // Control RELAY_RIGHT_1_PIN (PIN 25) - activated by X-right OR Y-positive
  digitalWrite(RELAY_RIGHT_1_PIN, (x_right_active || y_pos_active) ? RELAY_ON : RELAY_OFF);

  // Control RELAY_RIGHT_2_PIN (PIN 26) - activated by X-right OR Y-negative
  digitalWrite(RELAY_RIGHT_2_PIN, (x_right_active || y_neg_active) ? RELAY_ON : RELAY_OFF);
}

void checkAutoOnTimer() {
  // CRITICAL: If main logic is disabled, don't check auto-on timer
  if (!mainLogicEnabled) {
    return;
  }

  unsigned long currentMillis = millis();

  // Check if 2 minutes have passed and auto-ON hasn't activated yet
  if (!autoOnActive && (currentMillis - startMillis >= AUTO_ON_DELAY)) {
    autoOnActive = true;
    Serial.println("Auto-ON activated! All original relay pins (27, 14, 25, 26) will now stay ON.");

    // Set all original relay pins to ON state when auto-ON activates
    digitalWrite(RELAY_LEFT_1_PIN, RELAY_ON);
    digitalWrite(RELAY_LEFT_2_PIN, RELAY_ON);
    digitalWrite(RELAY_RIGHT_1_PIN, RELAY_ON);
    digitalWrite(RELAY_RIGHT_2_PIN, RELAY_ON);
  }

  // Print a countdown every 10 seconds before auto-ON activates
  static unsigned long lastAutoOnPrint = 0;
  if (!autoOnActive && (currentMillis - lastAutoOnPrint >= 10000)) {
    lastAutoOnPrint = currentMillis;
    int secondsRemaining = (AUTO_ON_DELAY - (currentMillis - startMillis)) / 1000;
    Serial.print("Auto-ON in: ");
    Serial.print(secondsRemaining);
    Serial.println(" seconds");
  }
}

void loop() {
  server.handleClient(); // Handle incoming web requests

  if (mainLogicEnabled) { // ONLY RUN THE MAIN LOGIC IF ENABLED
    // Handle autonomous forward movement
    handleForwardMovement();

    readSensorData();   // Read accelerometer data
    readUltrasonic();   // Read first ultrasonic sensor data
    readUltrasonic2(); // Read second ultrasonic sensor data (obstacle detection)

    checkAutoOnTimer(); // Check if it's time to activate auto-ON
    checkSensorsAndControl(); // Control relay pins based on sensors

    // Print acceleration data
    Serial.print("Accel (g): X=");
    Serial.print(accel_x, 3);
    Serial.print(" Y=");
    Serial.print(accel_y, 3);
    Serial.print(" Z=");
    Serial.print(accel_z, 3);

    // Print distance data for both sensors
    Serial.print(" | Distance1 = ");
    Serial.print(distance);
    Serial.print(" cm | Distance2 = ");
    Serial.print(distance2);
    Serial.print(" cm");

    // Print forward movement status
    Serial.print(" | Forward: ");
    switch (forwardState) {
      case FORWARD_INITIAL_COUNTDOWN:
        Serial.print("Countdown");
        break;
      case FORWARD_ON_PHASE:
        Serial.print("ON");
        break;
      case FORWARD_OFF_PHASE:
        Serial.print("OFF");
        break;
    }

    // Print relay status
    Serial.print(" | Relays: ");

    bool anyRelayActive = isAnyRelayActive();

    if (digitalRead(RELAY_LEFT_1_PIN) == RELAY_ON) {
      Serial.print("L1(27) ");
    }

    if (digitalRead(RELAY_LEFT_2_PIN) == RELAY_ON) {
      Serial.print("L2(14) ");
    }

    if (digitalRead(RELAY_RIGHT_1_PIN) == RELAY_ON) {
      Serial.print("R1(25) ");
    }

    if (digitalRead(RELAY_RIGHT_2_PIN) == RELAY_ON) {
      Serial.print("R2(26) ");
    }

    if (digitalRead(RELAY_FORWARD_1_PIN) == RELAY_ON) {
      Serial.print("F1(32) ");
    }

    if (digitalRead(RELAY_FORWARD_2_PIN) == RELAY_ON) {
      Serial.print("F2(33) ");
    }

    if (!anyRelayActive) {
      Serial.print("ALL OFF");
    }

    // Print trigger information
    Serial.print(" | Triggers: ");
    if (autoOnActive) {
      Serial.print("AUTO-ON ");
    }

    if (distance <= DISTANCE_THRESHOLD && distance > 0) {
      Serial.print("PROXIMITY1 ");
    }

    if (distance2 <= DISTANCE_THRESHOLD_2 && distance2 > 0) {
      Serial.print("OBSTACLE ");
    }

    if (accel_x <= -ACCEL_THRESHOLD) {
      Serial.print("X-LEFT ");
    }
    else if (accel_x >= ACCEL_THRESHOLD) {
      Serial.print("X-RIGHT ");
    }

    if (accel_y >= ACCEL_THRESHOLD) {
      Serial.print("Y-POS ");
    }
    else if (accel_y <= -ACCEL_THRESHOLD) {
      Serial.print("Y-NEG ");
    }

    Serial.println();
  } else {
    // CRITICAL: When main logic is OFF, ensure ALL outputs are forced OFF
    digitalWrite(RELAY_LEFT_1_PIN, RELAY_OFF);
    digitalWrite(RELAY_LEFT_2_PIN, RELAY_OFF);
    digitalWrite(RELAY_RIGHT_1_PIN, RELAY_OFF);
    digitalWrite(RELAY_RIGHT_2_PIN, RELAY_OFF);
    digitalWrite(RELAY_FORWARD_1_PIN, RELAY_OFF);
    digitalWrite(RELAY_FORWARD_2_PIN, RELAY_OFF);

    // If mainLogicEnabled is false, print a message indicating it's off
    static unsigned long lastOffMessagePrint = 0;
    if (millis() - lastOffMessagePrint >= 2000) { // Print every 2 seconds
      Serial.println("Main logic is OFF. All outputs forced OFF. Visit the web page to turn it ON.");
      lastOffMessagePrint = millis();
    }
  }

  delay(1000);
}
