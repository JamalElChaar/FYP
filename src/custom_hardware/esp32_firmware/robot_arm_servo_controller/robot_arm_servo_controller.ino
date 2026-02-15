/*
 * ESP32 Robot Arm Servo Controller
 *
 * This firmware receives joint angle commands from ROS2 via WiFi/TCP
 * and forwards them to a 16-channel UART servo controller.
 *
 * Hardware Setup:
 * - ESP32 DevKit V1
 * - 16-channel UART Servo Controller (connected to Serial2)
 * - MG995 Servos (6 joints)
 *
 * Protocol:
 * - Commands from ROS2: "POS:angle1,angle2,angle3,angle4,angle5,angle6\n"
 * - Response to GET: "POS:angle1,angle2,angle3,angle4,angle5,angle6\n"
 * - Angles are in degrees (0-180)
 *
 * WiFi Configuration:
 * - Set your WiFi credentials below
 * - Default TCP port: 5000
 */

#include <WiFi.h>

// ========== CONFIGURATION - EDIT THESE ==========
const char *WIFI_SSID = "YOUR_WIFI_SSID";         // Your WiFi network name
const char *WIFI_PASSWORD = "YOUR_WIFI_PASSWORD"; // Your WiFi password
const int TCP_PORT = 5000;                        // TCP port to listen on

// Serial2 pins for UART servo controller
#define SERVO_RX_PIN 16 // GPIO16 - connect to TX of servo controller
#define SERVO_TX_PIN 17 // GPIO17 - connect to RX of servo controller
#define SERVO_BAUD 115200

// Number of joints/servos
#define NUM_JOINTS 6

// Servo channel mapping (which servo controller channel for each joint)
// Adjust these based on how you wired your servos
const int SERVO_CHANNELS[NUM_JOINTS] = {0, 1, 2, 3, 4, 5};

// ========== GLOBALS ==========
WiFiServer tcpServer(TCP_PORT);
WiFiClient client;

// Current servo positions (in degrees)
float servoPositions[NUM_JOINTS] = {90.0, 90.0, 90.0, 90.0, 90.0, 90.0};

// Buffer for receiving commands
String inputBuffer = "";

// ========== UART SERVO CONTROLLER PROTOCOL ==========
// This is a generic implementation - you may need to modify based on your
// specific servo controller's protocol (LewanSoul, Hiwonder, Torobot, etc.)

void initServoController() {
  Serial2.begin(SERVO_BAUD, SERIAL_8N1, SERVO_RX_PIN, SERVO_TX_PIN);
  delay(100);
  Serial.println("Servo controller initialized on Serial2");
}

// Generic servo command - modify based on your servo controller protocol
void setServoAngle(int channel, float angle) {
  // Clamp angle to valid range
  if (angle < 0)
    angle = 0;
  if (angle > 180)
    angle = 180;

  // Convert angle to pulse width (typical: 500-2500us for 0-180°)
  // MG995 typical range: 500-2400us
  int pulseWidth = map(angle * 10, 0, 1800, 500, 2500);

  // Option 1: Simple ASCII protocol
  // Format: #<channel>P<pulse>T<time>\r\n
  // Example: #0P1500T100\r\n (channel 0, pulse 1500us, time 100ms)
  char cmd[32];
  sprintf(cmd, "#%dP%dT100\r\n", channel, pulseWidth);
  Serial2.print(cmd);

  // Option 2: LewanSoul/Hiwonder LX-16A protocol (uncomment if using)
  // sendLewanSoulCommand(channel, pulseWidth);

  // Option 3: Direct PWM value for some controllers
  // sendPWMCommand(channel, pulseWidth);
}

// Move all servos to specified positions
void setAllServos(float positions[NUM_JOINTS]) {
  for (int i = 0; i < NUM_JOINTS; i++) {
    servoPositions[i] = positions[i];
    setServoAngle(SERVO_CHANNELS[i], positions[i]);
  }
}

// Alternative protocols - uncomment and modify as needed:

/*
// LewanSoul/Hiwonder LX-16A Protocol
void sendLewanSoulCommand(int id, int position) {
  // Position: 0-1000 (maps to 0-240 degrees)
  int pos = map(position, 500, 2500, 0, 1000);
  uint8_t buf[10];
  buf[0] = 0x55;  // Header
  buf[1] = 0x55;  // Header
  buf[2] = id;    // Servo ID
  buf[3] = 7;     // Length
  buf[4] = 1;     // CMD_SERVO_MOVE
  buf[5] = pos & 0xFF;
  buf[6] = (pos >> 8) & 0xFF;
  buf[7] = 100 & 0xFF;   // Time low byte
  buf[8] = (100 >> 8) & 0xFF;  // Time high byte
  buf[9] = ~(buf[2] + buf[3] + buf[4] + buf[5] + buf[6] + buf[7] + buf[8]);  //
Checksum Serial2.write(buf, 10);
}
*/

/*
// Simple serial protocol: "S<channel>,<angle>\n"
void sendSimpleSerialCommand(int channel, float angle) {
  Serial2.print("S");
  Serial2.print(channel);
  Serial2.print(",");
  Serial2.println(angle);
}
*/

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
    Serial.println("\nWiFi connected!");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());
    Serial.print("TCP Port: ");
    Serial.println(TCP_PORT);
  } else {
    Serial.println("\nWiFi connection FAILED!");
    Serial.println("Please check credentials and restart");
  }
}

void startTCPServer() {
  tcpServer.begin();
  Serial.println("TCP server started");
}

// ========== COMMAND PROCESSING ==========
void processCommand(String cmd) {
  cmd.trim();

  if (cmd.startsWith("INIT")) {
    // Initialization command
    Serial.println("ROS2 client initialized");
    client.println("OK:INIT");

    // Move to home position
    float home[NUM_JOINTS] = {90, 90, 90, 90, 90, 90};
    setAllServos(home);

  } else if (cmd.startsWith("POS:")) {
    // Position command: "POS:angle1,angle2,angle3,angle4,angle5,angle6"
    String data = cmd.substring(4);
    float newPositions[NUM_JOINTS];

    int idx = 0;
    int lastComma = -1;
    for (int i = 0; i <= data.length() && idx < NUM_JOINTS; i++) {
      if (i == data.length() || data[i] == ',') {
        String angleStr = data.substring(lastComma + 1, i);
        newPositions[idx] = angleStr.toFloat();
        idx++;
        lastComma = i;
      }
    }

    if (idx == NUM_JOINTS) {
      setAllServos(newPositions);
      Serial.print("Set positions: ");
      for (int i = 0; i < NUM_JOINTS; i++) {
        Serial.print(newPositions[i]);
        Serial.print(" ");
      }
      Serial.println();
    } else {
      Serial.println("Invalid position command");
      client.println("ERR:INVALID_POS");
    }

  } else if (cmd.startsWith("GET")) {
    // Get current positions
    String response = "POS:";
    for (int i = 0; i < NUM_JOINTS; i++) {
      response += String(servoPositions[i], 1);
      if (i < NUM_JOINTS - 1)
        response += ",";
    }
    client.println(response);

  } else if (cmd.startsWith("STOP")) {
    // Stop/disable servos (if supported)
    Serial.println("Stop command received");
    client.println("OK:STOP");

  } else if (cmd.startsWith("HOME")) {
    // Move to home position
    float home[NUM_JOINTS] = {90, 90, 90, 90, 90, 90};
    setAllServos(home);
    client.println("OK:HOME");

  } else if (cmd.length() > 0) {
    Serial.print("Unknown command: ");
    Serial.println(cmd);
    client.println("ERR:UNKNOWN_CMD");
  }
}

void handleClient() {
  // Check for new client
  if (!client || !client.connected()) {
    WiFiClient newClient = tcpServer.available();
    if (newClient) {
      client = newClient;
      Serial.println("New client connected!");
      inputBuffer = "";
    }
  }

  // Handle connected client
  if (client && client.connected()) {
    while (client.available()) {
      char c = client.read();
      if (c == '\n') {
        processCommand(inputBuffer);
        inputBuffer = "";
      } else if (c != '\r') {
        inputBuffer += c;
        // Prevent buffer overflow
        if (inputBuffer.length() > 200) {
          inputBuffer = "";
        }
      }
    }
  }
}

// ========== SETUP & LOOP ==========
void setup() {
  // Initialize serial for debugging
  Serial.begin(115200);
  delay(1000);

  Serial.println("\n================================");
  Serial.println("ESP32 Robot Arm Servo Controller");
  Serial.println("================================\n");

  // Initialize servo controller
  initServoController();

  // Move to home position
  Serial.println("Moving to home position...");
  float home[NUM_JOINTS] = {90, 90, 90, 90, 90, 90};
  setAllServos(home);
  delay(1000);

  // Connect to WiFi
  setupWiFi();

  // Start TCP server
  if (WiFi.status() == WL_CONNECTED) {
    startTCPServer();
    Serial.println("\nReady for ROS2 connection!");
    Serial.println("Use this IP in your URDF: " + WiFi.localIP().toString());
  }
}

void loop() {
  // Handle TCP client
  handleClient();

  // Reconnect WiFi if disconnected
  static unsigned long lastWiFiCheck = 0;
  if (millis() - lastWiFiCheck > 10000) {
    lastWiFiCheck = millis();
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("WiFi disconnected, reconnecting...");
      setupWiFi();
      if (WiFi.status() == WL_CONNECTED) {
        startTCPServer();
      }
    }
  }

  // Small delay to prevent watchdog issues
  delay(1);
}
