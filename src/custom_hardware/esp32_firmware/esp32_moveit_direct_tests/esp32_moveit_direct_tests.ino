#include <ESP32Servo.h>
#include <WiFi.h>
#include <math.h>
#include <micro_ros_arduino.h>

#include <rcl/error_handling.h>
#include <rcl/init_options.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <std_msgs/msg/float64_multi_array.h>

#define LED_PIN 2
#define NUM_JOINTS 6

// Edit these before uploading.
const char * WIFI_SSID = "VUZ.HOTEL";
const char * WIFI_PASSWORD = "Vuz@1324";
char agent_ip[] = "10.5.50.141";
const uint16_t agent_port = 8888;
const size_t ROS_DOMAIN_ID = 0;

// Replace -1 with the verified GPIO for joint 1 before testing that joint.
const int SERVO_PINS[NUM_JOINTS] = {-1, 13, 32, 33, 27, 26};
const int SERVO_MIN_US = 500;
const int SERVO_MAX_US = 2400;

rcl_subscription_t subscriber;
rcl_publisher_t publisher;
rcl_timer_t timer;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
std_msgs__msg__Float64MultiArray command_message;
std_msgs__msg__Float64MultiArray state_message;

Servo servos[NUM_JOINTS];
float currentPositions[NUM_JOINTS] = {90.0, 90.0, 90.0, 90.0, 90.0, 90.0};

void error_loop()
{
  while (true) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

#define RCCHECK(function_call)                                                  \
  do {                                                                          \
    const rcl_ret_t return_code = (function_call);                               \
    if (return_code != RCL_RET_OK) {                                             \
      error_loop();                                                              \
    }                                                                            \
  } while (0)

#define RCSOFTCHECK(function_call)                                               \
  do {                                                                          \
    const rcl_ret_t return_code = (function_call);                               \
    (void)return_code;                                                           \
  } while (0)

void initialize_servos()
{
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  for (int joint = 0; joint < NUM_JOINTS; ++joint) {
    if (SERVO_PINS[joint] < 0) {
      continue;
    }
    servos[joint].setPeriodHertz(50);
    servos[joint].attach(SERVO_PINS[joint], SERVO_MIN_US, SERVO_MAX_US);
    servos[joint].write(static_cast<int>(currentPositions[joint]));
  }
}

void set_servo_angle(int joint, float angle)
{
  if (joint < 0 || joint >= NUM_JOINTS || SERVO_PINS[joint] < 0) {
    return;
  }

  angle = constrain(angle, 0.0F, 180.0F);
  currentPositions[joint] = angle;
  servos[joint].write(static_cast<int>(angle));
}

void command_callback(const void * incoming_message)
{
  const auto * message =
    static_cast<const std_msgs__msg__Float64MultiArray *>(incoming_message);
  if (message->data.size < NUM_JOINTS) {
    return;
  }

  for (int joint = 0; joint < NUM_JOINTS; ++joint) {
    const double received = message->data.data[joint];

    // NaN explicitly means: reuse the ESP32's last command for this joint.
    const float requested = isnan(received) ? currentPositions[joint] : received;
    if (!isfinite(requested)) {
      continue;
    }
    set_servo_angle(joint, requested);
  }
}

void state_timer_callback(rcl_timer_t * timer_handle, int64_t last_call_time)
{
  RCLC_UNUSED(last_call_time);
  if (timer_handle == nullptr) {
    return;
  }

  for (int joint = 0; joint < NUM_JOINTS; ++joint) {
    state_message.data.data[joint] = currentPositions[joint];
  }
  RCSOFTCHECK(rcl_publish(&publisher, &state_message, nullptr));
}

void connect_wifi()
{
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 30) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(500);
    ++attempts;
  }

  if (WiFi.status() != WL_CONNECTED) {
    error_loop();
  }

  Serial.print("ESP32 IP: ");
  Serial.println(WiFi.localIP());
  Serial.print("Agent: ");
  Serial.print(agent_ip);
  Serial.print(":");
  Serial.println(agent_port);
}

void allocate_messages()
{
  command_message.data.capacity = NUM_JOINTS;
  command_message.data.size = NUM_JOINTS;
  command_message.data.data =
    static_cast<double *>(malloc(NUM_JOINTS * sizeof(double)));

  state_message.data.capacity = NUM_JOINTS;
  state_message.data.size = NUM_JOINTS;
  state_message.data.data =
    static_cast<double *>(malloc(NUM_JOINTS * sizeof(double)));

  if (command_message.data.data == nullptr || state_message.data.data == nullptr) {
    error_loop();
  }

  for (int joint = 0; joint < NUM_JOINTS; ++joint) {
    command_message.data.data[joint] = currentPositions[joint];
    state_message.data.data[joint] = currentPositions[joint];
  }
}

void setup()
{
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  initialize_servos();
  connect_wifi();
  set_microros_wifi_transports(
    const_cast<char *>(WIFI_SSID), const_cast<char *>(WIFI_PASSWORD), agent_ip,
    agent_port);
  delay(2000);

  allocator = rcl_get_default_allocator();
  rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
  RCCHECK(rcl_init_options_init(&init_options, allocator));
  RCCHECK(rcl_init_options_set_domain_id(&init_options, ROS_DOMAIN_ID));
  RCCHECK(rclc_support_init_with_options(&support, 0, nullptr, &init_options, &allocator));
  RCCHECK(rcl_init_options_fini(&init_options));

  RCCHECK(rclc_node_init_default(&node, "esp32_moveit_test_node", "", &support));
  RCCHECK(rclc_subscription_init_default(
      &subscriber, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64MultiArray),
      "/esp32/joint_commands"));
  RCCHECK(rclc_publisher_init_default(
      &publisher, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64MultiArray),
      "/esp32/joint_states"));

  const unsigned int state_period_ms = 20;
  RCCHECK(rclc_timer_init_default(
      &timer, &support, RCL_MS_TO_NS(state_period_ms), state_timer_callback));

  allocate_messages();
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_subscription(
      &executor, &subscriber, &command_message, command_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));

  digitalWrite(LED_PIN, LOW);
  Serial.println("Ready for direct MoveIt tests");
}

void loop()
{
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));
}
