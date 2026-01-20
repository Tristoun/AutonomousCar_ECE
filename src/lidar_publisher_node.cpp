#include <Arduino.h>
#include <WiFi.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <sensor_msgs/msg/point_cloud2.h>
#include <sensor_msgs/msg/imu.h>
#include <rosidl_runtime_c/string_functions.h>
#include <rmw_microros/rmw_microros.h> 
#include <Wire.h>

// --- HEADER INCLUDES ---
// Ensure this file exists in your src folder or include path
#include "lidar_header.hpp"

// --- CONFIGURATION ---
const int MPU_ADDR = 0x68;  // I2C address of MPU6050
#define WIFI_SSID "Arecetri"
#define WIFI_PASSWORD "arece1234"
#define AGENT_IP IPAddress(10, 150, 62, 183)
#define AGENT_PORT 8888

// --- ROS OBJECTS ---
rcl_node_t node;
rclc_support_t support;
rcl_allocator_t allocator;
rclc_executor_t executor;

// Lidar Objects
rcl_publisher_t lidar_pub;
sensor_msgs__msg__PointCloud2 cloud;

// IMU Objects
rcl_publisher_t imu_pub;
sensor_msgs__msg__Imu imu_msg;

// --- MEMORY & BUFFERS ---
#define MAX_POINTS 460 
uint8_t cloud_data_buffer[MAX_POINTS * 16]; 
sensor_msgs__msg__PointField field_buffer[4];

// --- TIMING ---
unsigned long last_imu_time = 0;
const unsigned long IMU_INTERVAL = 50; // 20Hz (50ms)

// --- STATE MACHINE ---
enum states {
  WAITING_AGENT,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
} state;

// --- INITIALIZATION FUNCTIONS ---

void init_imu_msg() {
    rosidl_runtime_c__String__assign(&imu_msg.header.frame_id, "imu_link");
    // Covariance[0] = -1 indicates orientation is not provided (only raw data)
    imu_msg.orientation_covariance[0] = -1;
}

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

    // Define Fields: x, y, z, intensity
    rosidl_runtime_c__String__assign(&cloud.fields.data[0].name, "x");
    cloud.fields.data[0].offset = 0; cloud.fields.data[0].datatype = 7; cloud.fields.data[0].count = 1;
    rosidl_runtime_c__String__assign(&cloud.fields.data[1].name, "y");
    cloud.fields.data[1].offset = 4; cloud.fields.data[1].datatype = 7; cloud.fields.data[1].count = 1;
    rosidl_runtime_c__String__assign(&cloud.fields.data[2].name, "z");
    cloud.fields.data[2].offset = 8; cloud.fields.data[2].datatype = 7; cloud.fields.data[2].count = 1;
    rosidl_runtime_c__String__assign(&cloud.fields.data[3].name, "intensity");
    cloud.fields.data[3].offset = 12; cloud.fields.data[3].datatype = 7; cloud.fields.data[3].count = 1;
}

// --- DATA PROCESSING FUNCTIONS ---

void publish_imu() {
    int16_t AcX, AcY, AcZ, GyX, GyY, GyZ;

    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x3B); // Starting register for Accel Readings
    Wire.endTransmission(false);
    
    // Request 14 bytes: Accel(6) + Temp(2) + Gyro(6)
    if (Wire.requestFrom(MPU_ADDR, 14, true) == 14) {
        AcX = Wire.read() << 8 | Wire.read();
        AcY = Wire.read() << 8 | Wire.read();
        AcZ = Wire.read() << 8 | Wire.read();
        Wire.read(); Wire.read(); // Ignore Temperature
        GyX = Wire.read() << 8 | Wire.read();
        GyY = Wire.read() << 8 | Wire.read();
        GyZ = Wire.read() << 8 | Wire.read();

        // 1. Sync Time with ROS Agent
        int64_t time_ns = rmw_uros_epoch_nanos();
        imu_msg.header.stamp.sec = time_ns / 1000000000;
        imu_msg.header.stamp.nanosec = time_ns % 1000000000;

        // 2. Convert Raw to Physical Units
        // Accel: +/- 2g scale -> 16384 LSB/g. Multiply by 9.806 for m/s^2
        imu_msg.linear_acceleration.x = (float)AcX / 16384.0f * 9.806f;
        imu_msg.linear_acceleration.y = (float)AcY / 16384.0f * 9.806f;
        imu_msg.linear_acceleration.z = (float)AcZ / 16384.0f * 9.806f;

        // Gyro: +/- 250 deg/s scale -> 131 LSB/deg/s. Convert to rad/s
        imu_msg.angular_velocity.x = (float)GyX / 131.0f * (PI / 180.0f);
        imu_msg.angular_velocity.y = (float)GyY / 131.0f * (PI / 180.0f);
        imu_msg.angular_velocity.z = (float)GyZ / 131.0f * (PI / 180.0f);

        rcl_publish(&imu_pub, &imu_msg, NULL);
    }
}

// --- LIFECYCLE MANAGEMENT ---

bool create_entities() {
    allocator = rcl_get_default_allocator();
    
    // Init Support
    rclc_support_init(&support, 0, NULL, &allocator);

    // Init Node
    if (rclc_node_init_default(&node, "esp32_lidar", "", &support) != RCL_RET_OK) return false;

    // Init Lidar Publisher
    if (rclc_publisher_init_default(&lidar_pub, &node, 
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, PointCloud2), "scan_cloud") != RCL_RET_OK) return false;

    // Init IMU Publisher
    if (rclc_publisher_init_default(&imu_pub, &node, 
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), "imu/data_raw") != RCL_RET_OK) return false;

    // Init Executor (for background tasks)
    executor = rclc_executor_get_zero_initialized_executor();
    rclc_executor_init(&executor, &support.context, 1, &allocator);
    
    // Init Message Structures
    init_point_cloud_msg();
    init_imu_msg();

    // Sync Time
    rmw_uros_sync_session(1000);

    return true;
}

void destroy_entities() {
    rcl_publisher_fini(&lidar_pub, &node);
    rcl_publisher_fini(&imu_pub, &node);
    rcl_node_fini(&node);
    rclc_executor_fini(&executor);
    rclc_support_fini(&support);
}

// --- MAIN SETUP ---
void setup() {
    Serial.begin(115200);
    
    // Lidar Serial
    Serial2.setRxBufferSize(4096);
    Serial2.begin(230400, SERIAL_8N1, 16, 17);

    // WiFi & Transport
    Serial.print("Connecting to WiFi...");
    set_microros_wifi_transports(WIFI_SSID, WIFI_PASSWORD, AGENT_IP, AGENT_PORT);
    
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("\nWiFi Connected! IP: " + WiFi.localIP().toString());

    // IMU Setup
    Wire.begin(4, 5);
    Wire.setClock(400000); // Set I2C to 400kHz for faster reads
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x6B);       // PWR_MGMT_1 register
    Wire.write(0);          // Wake up
    Wire.endTransmission();

    state = WAITING_AGENT;
}

// --- MAIN LOOP ---
std::vector<LidarPoint> scanBuffer;
float last_angle = 0;

void loop() {
    // ------------------------------------------------
    // STATE MACHINE
    // ------------------------------------------------
    switch (state) {
        case WAITING_AGENT:
            // "Knock on door" - Check if Agent is reachable
            if (rmw_uros_ping_agent(100, 1) == RMW_RET_OK) {
                Serial.println("Agent found! Creating entities...");
                if (create_entities()) {
                    state = AGENT_CONNECTED;
                } else {
                    Serial.println("Creation failed.");
                    destroy_entities();
                }
            } else {
                // Drain Lidar buffer while waiting
                while(Serial2.available()) Serial2.read(); 
            }
            break;

        case AGENT_CONNECTED:
            // Check WiFi
            if (WiFi.status() != WL_CONNECTED) {
                state = AGENT_DISCONNECTED;
                break;
            }

            // Handle micro-ROS background tasks
            rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1));

            // --- CHECK 1: Publish IMU (Priority Check) ---
            if (millis() - last_imu_time > IMU_INTERVAL) {
                publish_imu();
                last_imu_time = millis();
            }

            // --- READ LIDAR STREAM ---
            while (Serial2.available() > 47) {
                
                // --- CHECK 2: Interleaved IMU Check ---
                // Prevents Lidar stream from blocking IMU updates
                if (millis() - last_imu_time > IMU_INTERVAL) {
                    publish_imu();
                    last_imu_time = millis();
                }

                std::vector<LidarPoint> newPoints = getPoints();
                if (newPoints.empty()) continue;

                float current_angle = newPoints.back().angle();

                // Filter valid points
                for (auto &p : newPoints) {
                    if (p.distance() > 50 && p.distance() < 12000) {
                        scanBuffer.push_back(p);
                    }
                }

                // Detect End of Scan (Angle wrap 360 -> 0)
                if (current_angle < last_angle - 20000) {
                    
                    // --- PUBLISH LIDAR CLOUD ---
                    int count = scanBuffer.size();
                    if (count > MAX_POINTS) count = MAX_POINTS;

                    // Sync Time
                    int64_t time_ns = rmw_uros_epoch_nanos();
                    cloud.header.stamp.sec = time_ns / 1000000000;
                    cloud.header.stamp.nanosec = time_ns % 1000000000;
                    
                    cloud.width = count;
                    cloud.row_step = cloud.point_step * cloud.width;
                    cloud.data.size = cloud.row_step;

                    // Fill Data Buffer
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
            break;

        case AGENT_DISCONNECTED:
            destroy_entities();
            state = WAITING_AGENT;
            break;
    }
}