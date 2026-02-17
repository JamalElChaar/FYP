/*
 * ESP32 Direct PWM Servo Controller
 *
 * This firmware receives joint angle commands from ROS2 via WiFi/TCP
 * and controls servos DIRECTLY via PWM on GPIO pins.
 *
 * Hardware Setup:
 * - ESP32 DevKit V1
 * - MG995 Servos connected directly to GPIO pins
 * - External 5V power supply for servos (DO NOT power from ESP32!)
 *
 * Protocol:
 * - Commands from ROS2: "POS:angle1,angle2,angle3,angle4,angle5,angle6\n"
 * - Response to GET: "POS:angle1,angle2,angle3,angle4,angle5,angle6\n"
 * - Angles are in degrees (0-180)
 */

#include <ESP32Servo.h>
#include <WiFi.h>

// ========== CONFIGURATION - EDIT THESE ==========
const char *WIFI_SSID = "YOUR_WIFI_SSID";         // Your WiFi network name
const char *WIFI_PASSWORD = "YOUR_WIFI_PASSWORD"; // Your WiFi password
const int TCP_PORT = 5000;                        // TCP port to listen on

// ========== GPIO PIN MAPPING ==========
// Set to -1 to disable a joint
// Joint 1 = base rotation, Joint 6 = gripper
const int SERVO_PINS[6] = {
    -1, // Joint 1 - DISABLED (set GPIO pin number to enable, e.g., 32)
    13, // Joint 2 - GPIO 13
    12, // Joint 3 - GPIO 12
    14, // Joint 4 - GPIO 14
    27, // Joint 5 - GPIO 27
    26  // Joint 6 - GPIO 26 (gripper)
};

// ========== SERVO CALIBRATION ==========
// MG995 typical pulse range: 500-2400 microseconds
// Adjust these if your servos don't reach full range
const int SERVO_MIN_US = 500;  // Pulse width for 0 degrees
const int SERVO_MAX_US = 2400; // Pulse width for 180 degrees

// ========== GLOBALS ==========
WiFiServer tcpServer(TCP_PORT);
WiFiClient client;
Servo servos[6];

// Current positions in degrees
float currentPositions[6] = {90.0, 90.0, 90.0, 90.0, 90.0, 90.0};

// Buffer for receiving commands
String inputBuffer = "";

// ========== SERVO FUNCTIONS ==========
void initServos() {
  // Allow allocation of all timers for servos
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  for (int i = 0; i < 6; i++) {
    if (SERVO_PINS[i] >= 0) {
      servos[i].setPeriodHertz(50); // Standard 50Hz servo
      servos[i].attach(SERVO_PINS[i], SERVO_MIN_US, SERVO_MAX_US);
      servos[i].write(90); // Start at center position
      Serial.printf("Joint %d: Attached to GPIO %d\n", i + 1, SERVO_PINS[i]);
    } else {
      Serial.printf("Joint %d: DISABLED\n", i + 1);
    }
  }
  Serial.println("Servo initialization complete!");
}

void setServoAngle(int joint, float angle) {
  // joint is 0-indexed (0-5)
  if (joint < 0 || joint >= 6)
    return;
  if (SERVO_PINS[joint] < 0)
    return; // Skip disabled joints

  // Clamp angle to valid range
  if (angle < 0)
    angle = 0;
  if (angle > 180)
    angle = 180;

  currentPositions[joint] = angle;
  servos[joint].write((int)angle);

  Serial.printf("Joint %d -> %.1f degrees\n", joint + 1, angle);
}

void setAllServos(float positions[6]) {
  Serial.println("Setting all servos:");
  for (int i = 0; i < 6; i++) {
    setServoAngle(i, positions[i]);
  }
}

// ========== WIFI & TCP ==========
void setupWiFi() {
  Serial.print("Connecting to WiFi: ");
  Serial.println(WIFI_SSID);

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 30) {
    delay(500);
    Serial.print(".");
    attempts++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\n========================================");
    Serial.println("WiFi connected!");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());
    Serial.print("TCP Port: ");
    Serial.println(TCP_PORT);
    Serial.println("========================================");
  } else {
    Serial.println("\nWiFi connection FAILED!");
    Serial.println("Please check credentials and restart");
  }
}

// ========== COMMAND PROCESSING ==========
void processCommand(String cmd) {
  cmd.trim();
  Serial.print("Received command: ");
  Serial.println(cmd);

  if (cmd == "INIT") {
    // Initialize/reset servos to center
    float centerPos[6] = {90.0, 90.0, 90.0, 90.0, 90.0, 90.0};
    setAllServos(centerPos);
    client.println("OK:INIT");
    Serial.println("Servos initialized to center position");
  } else if (cmd == "GET") {
    // Return current positions
    String response = "POS:";
    for (int i = 0; i < 6; i++) {
      response += String(currentPositions[i], 1);
      if (i < 5)
        response += ",";
    }
    client.println(response);
    Serial.print("Sent: ");
    Serial.println(response);
  } else if (cmd.startsWith("POS:")) {
    // Parse position command: POS:a1,a2,a3,a4,a5,a6
    String posStr = cmd.substring(4);
    float newPositions[6];
    int jointIndex = 0;

    // Parse comma-separated values
    int startIdx = 0;
    for (int i = 0; i <= posStr.length() && jointIndex < 6; i++) {
      if (i == posStr.length() || posStr.charAt(i) == ',') {
        String valStr = posStr.substring(startIdx, i);
        newPositions[jointIndex] = valStr.toFloat();
        jointIndex++;
        startIdx = i + 1;
      }
    }

    if (jointIndex == 6) {
      setAllServos(newPositions);
      client.println("OK:POS");
    } else {
      client.println("ERROR:Invalid position format");
      Serial.printf("Parse error: got %d values, expected 6\n", jointIndex);
    }
  } else if (cmd.startsWith("J")) {
    // Single joint command: J1:90 or J2:45
    int jointNum = cmd.substring(1, 2).toInt();
    int colonIdx = cmd.indexOf(':');
    if (colonIdx > 0 && jointNum >= 1 && jointNum <= 6) {
      float angle = cmd.substring(colonIdx + 1).toFloat();
      setServoAngle(jointNum - 1, angle); // Convert to 0-indexed
      client.println("OK:J" + String(jointNum));
    } else {
      client.println("ERROR:Invalid joint command");
    }
  } else if (cmd == "PING") {
    client.println("PONG");
  } else if (cmd == "STATUS") {
    String status = "STATUS:";
    status += "IP=" + WiFi.localIP().toString();
    status += ",RSSI=" + String(WiFi.RSSI());
    status += ",JOINTS=";
    for (int i = 0; i < 6; i++) {
      status += (SERVO_PINS[i] >= 0) ? "1" : "0";
    }
    client.println(status);
  } else {
    client.println("ERROR:Unknown command");
    Serial.println("Unknown command received");
  }
}

// ========== MAIN ==========
void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("\n========================================");
  Serial.println("ESP32 Direct PWM Servo Controller");
  Serial.println("========================================\n");

  // Initialize servos FIRST
  initServos();

  // Then connect to WiFi
  setupWiFi();

  // Start TCP server
  tcpServer.begin();
  Serial.println("TCP server started, waiting for connections...");
}

void loop() {
  // Handle WiFi reconnection
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi disconnected, reconnecting...");
    setupWiFi();
    return;
  }

  // Check for new client connection
  if (!client || !client.connected()) {
    client = tcpServer.available();
    if (client) {
      Serial.println("========================================");
      Serial.print("New client connected from: ");
      Serial.println(client.remoteIP());
      Serial.println("========================================");
      inputBuffer = "";
    }
  }

  // Read data from client
  while (client && client.connected() && client.available()) {
    char c = client.read();
    if (c == '\n') {
      processCommand(inputBuffer);
      inputBuffer = "";
    } else if (c != '\r') {
      inputBuffer += c;
    }
  }

  // Small delay to prevent CPU hogging
  delay(1);
}
