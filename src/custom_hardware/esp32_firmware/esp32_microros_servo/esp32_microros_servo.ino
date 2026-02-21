/*
 * ESP32 micro-ROS Servo Controller (WiFi UDP)
 *
 * This firmware makes the ESP32 a native ROS2 node using micro-ROS over WiFi.
 * It subscribes to joint commands and publishes joint states.
 *
 * Hardware Setup:
 * - ESP32 DevKit V1
 * - MG995 Servos connected directly to GPIO pins
 * - WiFi connection to same network as PC
 *
 * micro-ROS Topics:
 * - Subscribes: /esp32/joint_commands (std_msgs/Float64MultiArray)
 * - Publishes:  /esp32/joint_states (std_msgs/Float64MultiArray)
 *
 * Installation:
 * 1. Install micro_ros_arduino library in Arduino IDE
 * 2. Edit WiFi credentials and Agent IP below
 * 3. Select ESP32 board and upload
 * 4. Run micro-ROS agent on PC: ros2 run micro_ros_agent micro_ros_agent udp4
 * --port 8888
 */

#include <ESP32Servo.h>
#include <WiFi.h>
#include <micro_ros_arduino.h>

#include <rcl/error_handling.h>
#include <rcl/init_options.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <stdio.h>

#include <std_msgs/msg/float64_multi_array.h>

// ========== LED PIN (ESP32 built-in LED) ==========
#define LED_PIN 2 // GPIO 2 is the built-in LED on most ESP32 boards

// ========== WIFI CONFIGURATION - EDIT THESE ==========
const char *WIFI_SSID = "YOUR_WIFI_SSID";         // Your WiFi network name
const char *WIFI_PASSWORD = "YOUR_WIFI_PASSWORD"; // Your WiFi password

// micro-ROS Agent IP (your laptop's IP address on the network)
// Find it with: hostname -I (Linux) or ipconfig (Windows)
char agent_ip[] = "192.168.1.100"; // CHANGE THIS to your laptop's IP
const uint16_t agent_port = 8888;
const size_t ROS_DOMAIN_ID = 0; // Must match ROS_DOMAIN_ID in your ROS2 shell

// ========== GPIO PIN MAPPING ==========
// Set to -1 to disable a joint
const int SERVO_PINS[6] = {
    -1, // Joint 1 - DISABLED (set GPIO pin number to enable, e.g., 32)
    13, // Joint 2 - GPIO 13
    12, // Joint 3 - GPIO 12
    14, // Joint 4 - GPIO 14
    27, // Joint 5 - GPIO 27
    26  // Joint 6 - GPIO 26 (gripper)
};

// ========== SERVO CALIBRATION ==========
const int SERVO_MIN_US = 500;  // Pulse width for 0 degrees
const int SERVO_MAX_US = 2400; // Pulse width for 180 degrees

// ========== MICRO-ROS OBJECTS ==========
rcl_subscription_t subscriber;
rcl_publisher_t publisher;
std_msgs__msg__Float64MultiArray cmd_msg;
std_msgs__msg__Float64MultiArray state_msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

// ========== SERVO OBJECTS ==========
Servo servos[6];
float currentPositions[6] = {90.0, 90.0, 90.0, 90.0, 90.0, 90.0};

// ========== ERROR HANDLING ==========
#define RCCHECK(fn)                                                            \
  {                                                                            \
    rcl_ret_t temp_rc = fn;                                                    \
    if ((temp_rc != RCL_RET_OK)) {                                             \
      error_loop();                                                            \
    }                                                                          \
  }
#define RCSOFTCHECK(fn)                                                        \
  {                                                                            \
    rcl_ret_t temp_rc = fn;                                                    \
    if ((temp_rc != RCL_RET_OK)) {                                             \
    }                                                                          \
  }

void error_loop() {
  while (1) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

// ========== WIFI SETUP ==========
void setupWiFi() {
  Serial.begin(115200);
  Serial.println();
  Serial.print("Connecting to WiFi: ");
  Serial.println(WIFI_SSID);

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 30) {
    delay(500);
    Serial.print(".");
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    attempts++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println();
    Serial.println("========================================");
    Serial.println("WiFi connected!");
    Serial.print("ESP32 IP: ");
    Serial.println(WiFi.localIP());
    Serial.print("Agent IP: ");
    Serial.print(agent_ip);
    Serial.print(":");
    Serial.println(agent_port);
    Serial.println("========================================");
  } else {
    Serial.println();
    Serial.println("WiFi connection FAILED!");
    error_loop();
  }
}

// ========== SERVO FUNCTIONS ==========
void initServos() {
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  for (int i = 0; i < 6; i++) {
    if (SERVO_PINS[i] >= 0) {
      servos[i].setPeriodHertz(50);
      servos[i].attach(SERVO_PINS[i], SERVO_MIN_US, SERVO_MAX_US);
      servos[i].write(90); // Start at center
    }
  }
}

void setServoAngle(int joint, float angle) {
  if (joint < 0 || joint >= 6)
    return;
  if (SERVO_PINS[joint] < 0)
    return; // Skip disabled joints

  // Clamp angle
  if (angle < 0)
    angle = 0;
  if (angle > 180)
    angle = 180;

  currentPositions[joint] = angle;
  servos[joint].write((int)angle);
}

// ========== MICRO-ROS CALLBACKS ==========
void subscription_callback(const void *msgin) {
  const std_msgs__msg__Float64MultiArray *msg =
      (const std_msgs__msg__Float64MultiArray *)msgin;

  // Check if we have 6 values
  if (msg->data.size >= 6) {
    for (int i = 0; i < 6; i++) {
      setServoAngle(i, msg->data.data[i]);
    }
  }
}

void timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    // Publish current joint states
    for (int i = 0; i < 6; i++) {
      state_msg.data.data[i] = currentPositions[i];
    }
    RCSOFTCHECK(rcl_publish(&publisher, &state_msg, NULL));
  }
}

// ========== SETUP ==========
void setup() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  // Initialize servos first
  initServos();

  // Connect to WiFi
  setupWiFi();

  // Set up micro-ROS WiFi UDP transport
  set_microros_wifi_transports((char *)WIFI_SSID, (char *)WIFI_PASSWORD,
                               agent_ip, agent_port);

  delay(2000); // Wait for micro-ROS agent

  allocator = rcl_get_default_allocator();

  // Create init options and set ROS domain explicitly
  rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
  RCCHECK(rcl_init_options_init(&init_options, allocator));
  RCCHECK(rcl_init_options_set_domain_id(&init_options, ROS_DOMAIN_ID));
  RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options,
                                         &allocator));
  RCCHECK(rcl_init_options_fini(&init_options));

  // Create node
  RCCHECK(rclc_node_init_default(&node, "esp32_servo_node", "", &support));

  // Create subscriber for joint commands
  RCCHECK(rclc_subscription_init_default(
      &subscriber, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64MultiArray),
      "/esp32/joint_commands"));

  // Create publisher for joint states
  RCCHECK(rclc_publisher_init_default(
      &publisher, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64MultiArray),
      "/esp32/joint_states"));

  // Create timer for publishing states (50Hz)
  const unsigned int timer_timeout = 20; // 20ms = 50Hz
  RCCHECK(rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(timer_timeout),
                                  timer_callback));

  // Create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &cmd_msg,
                                         &subscription_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));

  // Allocate memory for messages
  cmd_msg.data.capacity = 6;
  cmd_msg.data.size = 6;
  cmd_msg.data.data = (double *)malloc(6 * sizeof(double));

  state_msg.data.capacity = 6;
  state_msg.data.size = 6;
  state_msg.data.data = (double *)malloc(6 * sizeof(double));

  // Initialize state message with current positions
  for (int i = 0; i < 6; i++) {
    state_msg.data.data[i] = currentPositions[i];
  }

  digitalWrite(LED_PIN, LOW); // LED off = ready
}

// ========== MAIN LOOP ==========
void loop() {
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));
}