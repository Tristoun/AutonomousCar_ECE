#include <Arduino.h>
#include <WiFi.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/string.h>

#include <sensor_msgs/msg/point_cloud2.h>

#define LED_PIN 2
#define WIFI_SSID "Arecetri"
#define WIFI_PASSWORD "arece1234"
#define AGENT_IP   IPAddress(10, 150, 62, 183)
#define AGENT_PORT 8888

// ROS 2 Objects
rcl_publisher_t publisher;
std_msgs__msg__String msg;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

enum states {
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
} state;

void connectWiFi() {
  Serial.print("Connecting to WiFi...");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConnected to Wi-Fi!");
  Serial.printf("IP Address: %s\n", WiFi.localIP().toString().c_str());
}

bool ping_agent() {
  // Try to initialize micro-ROS and check if agent responds
  allocator = rcl_get_default_allocator();
  rcl_ret_t ret = rclc_support_init(&support, 0, NULL, &allocator);
  
  if (ret == RCL_RET_OK) {
    return true;
  }
  
  // Clean up failed attempt
  rcl_ret_t rc = rclc_support_fini(&support);
  return false;
}

void create_entities() {
  // Create node
  if (rclc_node_init_default(&node, "esp32_platformio_node", "", &support) != RCL_RET_OK) {
    Serial.println("Error initializing ROS node");
    state = WAITING_AGENT;
    return;
  }

  // Create publisher
  if (rclc_publisher_init_default(
        &publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
        "platformio_hello_world") != RCL_RET_OK) {
    Serial.println("Error initializing publisher");
    state = WAITING_AGENT;
    return;
  }

  // Allocate message memory
  static char buffer[50];
  msg.data.data = buffer;
  msg.data.capacity = 50;
  msg.data.size = 0;
  
  state = AGENT_CONNECTED;
  Serial.println("** micro-ROS entities created successfully! **");
}

void destroy_entities() {
  // Clean up ROS entities
  rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
  (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);
  
  rcl_publisher_fini(&publisher, &node);
  rcl_node_fini(&node);
  rclc_support_fini(&support);
}

void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  
  connectWiFi();
  
  // Set micro-ROS WiFi transport
  set_microros_wifi_transports(WIFI_SSID, WIFI_PASSWORD, AGENT_IP, AGENT_PORT);
  
  state = WAITING_AGENT;
}

int counter = 0;
unsigned long last_blink = 0;
unsigned long last_publish = 0;

void loop() {


  switch (state) {
    case WAITING_AGENT:
      Serial.println("Waiting for micro-ROS agent...");
      state = AGENT_AVAILABLE;
      break;
      
    case AGENT_AVAILABLE:
      // Check WiFi
      if (WiFi.status() != WL_CONNECTED) {
        Serial.println("WiFi connection lost!");
        connectWiFi();
      }
      
      // Attempt to ping agent
      if (ping_agent()) {
        Serial.println("micro-ROS agent found!");
        create_entities();
      } else {
        delay(1000);  // Wait 1 second before retrying
      }
      break;
      
    case AGENT_CONNECTED:
      // Publish messages
      if (millis() - last_publish > 1000) {
        last_publish = millis();
        
        sprintf(msg.data.data, "Hello from PlatformIO! %d", counter++);
        msg.data.size = strlen(msg.data.data);
        
        rcl_ret_t ret = rcl_publish(&publisher, &msg, NULL);
        
        if (ret != RCL_RET_OK) {
          Serial.println("Error publishing - agent may be disconnected");
          destroy_entities();
          state = WAITING_AGENT;
        } else {
          Serial.printf("Published: %s\n", msg.data.data);
        }
      }
      break;
      
    default:
      break;
  }
}