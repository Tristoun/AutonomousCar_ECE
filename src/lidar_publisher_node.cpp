#include <Arduino.h>
#include <WiFi.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <sensor_msgs/msg/point_cloud2.h>
#include <rosidl_runtime_c/string_functions.h>
// IMPORTANT: Required for the ping function
#include <rmw_microros/rmw_microros.h> 

#include "lidar_header.hpp"

// --- CONFIGURATION ---
#define WIFI_SSID "Arecetri"
#define WIFI_PASSWORD "arece1234"
#define AGENT_IP IPAddress(10, 150, 62, 183)
#define AGENT_PORT 8888


// --- ROS OBJECTS ---
rcl_publisher_t lidar_pub;
sensor_msgs__msg__PointCloud2 cloud;
rcl_node_t node;
rclc_support_t support;
rcl_allocator_t allocator;
rclc_executor_t executor;

// --- MEMORY ---
#define MAX_POINTS 460 
uint8_t cloud_data_buffer[MAX_POINTS * 16]; 
sensor_msgs__msg__PointField field_buffer[4];

// --- STATE MACHINE ---
enum states {
  WAITING_AGENT,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
} state;

// --- FUNCTIONS ---

// 1. Static Message Initialization (Run once)
void init_point_cloud_msg() {
    rosidl_runtime_c__String__assign(&cloud.header.frame_id, "base_link");
    cloud.height = 1;
    cloud.is_bigendian = false;
    cloud.is_dense = true;
    cloud.point_step = 16;
    cloud.fields.data = field_buffer;
    cloud.fields.size = 4;
    cloud.fields.capacity = 4;
    cloud.data.data = cloud_data_buffer;
    cloud.data.capacity = sizeof(cloud_data_buffer);

    // X, Y, Z, Intensity mapping
    rosidl_runtime_c__String__assign(&cloud.fields.data[0].name, "x");
    cloud.fields.data[0].offset = 0; cloud.fields.data[0].datatype = 7; cloud.fields.data[0].count = 1;
    rosidl_runtime_c__String__assign(&cloud.fields.data[1].name, "y");
    cloud.fields.data[1].offset = 4; cloud.fields.data[1].datatype = 7; cloud.fields.data[1].count = 1;
    rosidl_runtime_c__String__assign(&cloud.fields.data[2].name, "z");
    cloud.fields.data[2].offset = 8; cloud.fields.data[2].datatype = 7; cloud.fields.data[2].count = 1;
    rosidl_runtime_c__String__assign(&cloud.fields.data[3].name, "intensity");
    cloud.fields.data[3].offset = 12; cloud.fields.data[3].datatype = 7; cloud.fields.data[3].count = 1;
}

// 2. Create ROS Entities
bool create_entities() {
    allocator = rcl_get_default_allocator();
    
    // Initialize Support
    rclc_support_init(&support, 0, NULL, &allocator);

    // Create Node
    if (rclc_node_init_default(&node, "esp32_lidar", "", &support) != RCL_RET_OK) return false;

    // Create Publisher
    if (rclc_publisher_init_default(&lidar_pub, &node, 
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, PointCloud2), "scan_cloud") != RCL_RET_OK) return false;

    // Create Executor
    executor = rclc_executor_get_zero_initialized_executor();
    rclc_executor_init(&executor, &support.context, 1, &allocator);
    
    init_point_cloud_msg();
    return true;
}

// 3. Destroy Entities (Cleanup on disconnect)
void destroy_entities() {
    rcl_publisher_fini(&lidar_pub, &node);
    rcl_node_fini(&node);
    rclc_executor_fini(&executor);
    rclc_support_fini(&support);
}

// --- SETUP ---
void setup() {
    Serial.begin(115200);
    
    // Increase Lidar Buffer
    Serial2.setRxBufferSize(4096);
    Serial2.begin(230400, SERIAL_8N1, 16, 17);

    // Setup WiFi Transport
    Serial.print("Connecting to WiFi...");
    set_microros_wifi_transports(WIFI_SSID, WIFI_PASSWORD, AGENT_IP, AGENT_PORT);
    
    // Wait for WiFi
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("\nWiFi Connected! IP: " + WiFi.localIP().toString());

    state = WAITING_AGENT;
}

// --- LOOP ---
std::vector<LidarPoint> scanBuffer;
float last_angle = 0;

void loop() {
    // ------------------------------------------------
    // STATE MACHINE: CONNECTION HANDLING
    // ------------------------------------------------
    switch (state) {
        case WAITING_AGENT:
            // "Knock on door" - Check if Agent is reachable
            // Timeout 100ms, 1 attempt
            if (rmw_uros_ping_agent(100, 1) == RMW_RET_OK) {
                Serial.println("Agent found! Creating entities...");
                if (create_entities()) {
                    state = AGENT_CONNECTED;
                } else {
                    Serial.println("Creation failed.");
                    destroy_entities();
                }
            } else {
                // While waiting, drain Lidar buffer so it doesn't overflow
                while(Serial2.available()) Serial2.read(); 
            }
            break;

        case AGENT_CONNECTED:
            // Check WiFi health
            if (WiFi.status() != WL_CONNECTED) {
                state = AGENT_DISCONNECTED;
            }
            // Check Agent health
            if (rmw_uros_ping_agent(100, 1) != RMW_RET_OK) {
                // Count missed pings if you want, or disconnect immediately
                // keeping it simple:
                // state = AGENT_DISCONNECTED; 
            }

            rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1));
            break;

        case AGENT_DISCONNECTED:
            destroy_entities();
            state = WAITING_AGENT;
            break;
    }

    // ------------------------------------------------
    // LIDAR READING (Only process if Connected)
    // ------------------------------------------------
    if (state == AGENT_CONNECTED) {
        while (Serial2.available() > 47) {
            std::vector<LidarPoint> newPoints = getPoints();
            if (newPoints.empty()) continue;

            float current_angle = newPoints.back().angle();

            for (auto &p : newPoints) {
                if (p.distance() > 50 && p.distance() < 12000) {
                    scanBuffer.push_back(p);
                }
            }

            // Detect End of Scan (Angle wrap 360 -> 0)
            if (current_angle < last_angle - 20000) {
                
                // --- PUBLISH ---
                int count = scanBuffer.size();
                if (count > MAX_POINTS) count = MAX_POINTS;

                // Sync Time
                int64_t time_ns = rmw_uros_epoch_nanos();
                cloud.header.stamp.sec = time_ns / 1000000000;
                cloud.header.stamp.nanosec = time_ns % 1000000000;
                cloud.width = count;
                cloud.row_step = cloud.point_step * cloud.width;
                cloud.data.size = cloud.row_step;

                for (int i = 0; i < count; i++) {
                    float angle_rad = (scanBuffer[i].angle() / 100.0f) * (PI / 180.0f);
                    float dist_m = scanBuffer[i].distance() / 1000.0f;
                    
                    // Coordinate Transform (Standard ROS: X forward, Y left)
                    float x = dist_m * cos(angle_rad);
                    float y = dist_m * sin(angle_rad);
                    float z = 0.0f;
                    float intensity = (float)scanBuffer[i].intensity();

                    int offset = i * 16;
                    memcpy(&cloud_data_buffer[offset], &x, 4);
                    memcpy(&cloud_data_buffer[offset+4], &y, 4);
                    memcpy(&cloud_data_buffer[offset+8], &z, 4);
                    memcpy(&cloud_data_buffer[offset+12], &intensity, 4);
                }

                rcl_publish(&lidar_pub, &cloud, NULL);
                
                scanBuffer.clear();
                scanBuffer.reserve(MAX_POINTS);
            }
            last_angle = current_angle;
        }
    }
}