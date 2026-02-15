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
#include <std_msgs/msg/int16_multi_array.h>
#include <Wire.h>
#include <vector>

#include "lidar_header.hpp"
#include "motor_header.hpp"

// --- CONFIGURATION ---
const int MPU_ADDR = 0x68;
#define WIFI_SSID "Arecetri"
#define WIFI_PASSWORD "arece1234"
#define AGENT_IP IPAddress(10, 36, 21, 183) //WARNING : set your computer IP ADDR

#define AGENT_PORT 8888
#define MAX_POINTS 460 

//Struct for imu data
typedef struct {
    float acc[3];
    float gyro[3];
    uint64_t timestamp;
} imu_data_t;

//Struct for lidar data
typedef struct {
    float x[MAX_POINTS];
    float y[MAX_POINTS];
    float z[MAX_POINTS];
    float intensity[MAX_POINTS];
    int count;
    uint64_t timestamp;
} lidar_scan_t;

//Queue : Similar to FIFO to send data forever
QueueHandle_t imu_queue;
QueueHandle_t lidar_queue; 

rcl_node_t node;
rclc_support_t support;
rcl_allocator_t allocator;
rclc_executor_t executor;
rcl_publisher_t lidar_pub;
rcl_publisher_t imu_pub;
rcl_subscription_t motor_sub;

sensor_msgs__msg__PointCloud2 cloud;
sensor_msgs__msg__Imu imu_msg;
uint8_t cloud_data_buffer[MAX_POINTS * 16]; 
sensor_msgs__msg__PointField field_buffer[4];
std_msgs__msg__Int16MultiArray motor_msg;

enum states { WAITING_AGENT, AGENT_CONNECTED, AGENT_DISCONNECTED } state; //State machine

//Imu calib variables
float gyro_bias_x = 0, gyro_bias_y = 0, gyro_bias_z = 0;
float accel_bias_x = 0, accel_bias_y = 0, accel_bias_z = 0;


void init_imu_msg() {
    rosidl_runtime_c__String__assign(&imu_msg.header.frame_id, "imu_link");
    imu_msg.orientation_covariance[0] = -1;
}

void init_point_cloud_msg() {
    rosidl_runtime_c__String__assign(&cloud.header.frame_id, "laser_link");
    cloud.height = 1;
    cloud.is_bigendian = false;
    cloud.is_dense = true;
    cloud.point_step = 16;
    cloud.fields.data = field_buffer;
    cloud.fields.size = 4;
    cloud.fields.capacity = 4;
    cloud.data.data = cloud_data_buffer;
    cloud.data.capacity = sizeof(cloud_data_buffer);

    rosidl_runtime_c__String__assign(&cloud.fields.data[0].name, "x");
    cloud.fields.data[0].offset = 0; cloud.fields.data[0].datatype = 7; cloud.fields.data[0].count = 1; 
    rosidl_runtime_c__String__assign(&cloud.fields.data[1].name, "y");
    cloud.fields.data[1].offset = 4; cloud.fields.data[1].datatype = 7; cloud.fields.data[1].count = 1;
    rosidl_runtime_c__String__assign(&cloud.fields.data[2].name, "z");
    cloud.fields.data[2].offset = 8; cloud.fields.data[2].datatype = 7; cloud.fields.data[2].count = 1;
    rosidl_runtime_c__String__assign(&cloud.fields.data[3].name, "intensity");
    cloud.fields.data[3].offset = 12; cloud.fields.data[3].datatype = 7; cloud.fields.data[3].count = 1;
}

void configure_mpu6050() {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x6B); Wire.write(0x00);
    Wire.endTransmission();
    delay(100);
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x1B); Wire.write(0x00);
    Wire.endTransmission();
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x1C); Wire.write(0x00);
    Wire.endTransmission();
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x1A); Wire.write(0x03);
    Wire.endTransmission();
}

void calibrate_mpu6050(int samples = 500) {
    long acc_x = 0, acc_y = 0, acc_z = 0, gyr_x = 0, gyr_y = 0, gyr_z = 0;
    for (int i = 0; i < samples; i++) {
        Wire.beginTransmission(MPU_ADDR);
        Wire.write(0x3B);
        Wire.endTransmission(false);
        Wire.requestFrom(MPU_ADDR, 14, true);
        int16_t AcX = Wire.read() << 8 | Wire.read();
        int16_t AcY = Wire.read() << 8 | Wire.read();
        int16_t AcZ = Wire.read() << 8 | Wire.read();
        Wire.read(); Wire.read();
        int16_t GyX = Wire.read() << 8 | Wire.read();
        int16_t GyY = Wire.read() << 8 | Wire.read();
        int16_t GyZ = Wire.read() << 8 | Wire.read();
        acc_x += AcX; acc_y += AcY; acc_z += AcZ - 16384;
        gyr_x += GyX; gyr_y += GyY; gyr_z += GyZ;
        delay(5);
    }
    accel_bias_x = (float)acc_x / samples; accel_bias_y = (float)acc_y / samples; accel_bias_z = (float)acc_z / samples;
    gyro_bias_x = (float)gyr_x / samples; gyro_bias_y = (float)gyr_y / samples; gyro_bias_z = (float)gyr_z / samples;
}

void motor_pwm_callback(const void * msg_in) {
    const std_msgs__msg__Int16MultiArray * msg = (const std_msgs__msg__Int16MultiArray *)msg_in;
    if (msg->data.size < 2) return;  // Expect at least 2 elements: left, right

    int left_pwm = msg->data.data[0];  // -255 -> 255
    int right_pwm = msg->data.data[1]; // -255 -> 255

    // Map the PWM values to motor driver function
    channel_A_Ctrl(left_pwm);   // Left motor
    channel_B_Ctrl(right_pwm);  // Right motor
}


void setup() {
    Serial.begin(115200);
    delay(2000);
    // Lidar Serial
    Serial2.setRxBufferSize(4096);
    Serial2.begin(230400, SERIAL_8N1, 16, 17);

    // --- Publishers ---
    rclc_publisher_init_default(&lidar_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, PointCloud2), "point_cloud");
    rclc_publisher_init_default(&imu_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), "imu_data");
    
    // --- Subscriber ---
    rclc_subscription_init_default(
        &motor_sub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16MultiArray),
        "motor_pwm"
    );

    // --- CRITICAL FIX: Initialize Subscriber Message Memory ---
    // We must provide a buffer for the executor to write the incoming data into.
    // Since create_entities might be called multiple times (reconnections), 
    // we use a static buffer to avoid memory leaks or re-allocation.
    static int16_t motor_data_buffer[4]; // Buffer for up to 4 values (Left, Right, etc.)
    motor_msg.data.capacity = 4;
    motor_msg.data.data = motor_data_buffer;
    motor_msg.data.size = 0;

    // --- Executor ---
    executor = rclc_executor_get_zero_initialized_executor();
    // Handles: 1 subscription. (Publishers don't count as handles in the executor)
    rclc_executor_init(&executor, &support.context, 1, &allocator);
    
    rclc_executor_add_subscription(
        &executor,
        &motor_sub,
        &motor_msg,
        &motor_pwm_callback,
        ON_NEW_DATA
    );

    init_point_cloud_msg();
    init_imu_msg();
    rmw_uros_sync_session(1000);
    return true;
}


void destroy_entities() {
    //Ending the ros2 node
    rcl_publisher_fini(&lidar_pub, &node);
    rcl_publisher_fini(&imu_pub, &node);
    rcl_node_fini(&node);
    rclc_executor_fini(&executor);
    rclc_support_fini(&support);
}


void sensor_motor_task(void * pvParameters) {
    /*Task from RTOS
    * We get data from all of our sensors and even using the wheels here
    */ 
    std::vector<LidarPoint> scanBuffer;
    float last_angle = 0;
    unsigned long last_imu_time = 0;
    const unsigned long IMU_INTERVAL = 50; 

    // Initialize data structure for Lidar
    lidar_scan_t* current_scan = new lidar_scan_t();
    current_scan->count = 0;

    for (;;) {
        // 2. IMU SAMPLING
        if (millis() - last_imu_time > IMU_INTERVAL) {
            imu_data_t imu_raw;
            Wire.beginTransmission(MPU_ADDR);
            Wire.write(0x3B);
            Wire.endTransmission(false);
            if (Wire.requestFrom(MPU_ADDR, 14, true) == 14) { //Reading imu data
                int16_t AcX = Wire.read() << 8 | Wire.read();
                int16_t AcY = Wire.read() << 8 | Wire.read();
                int16_t AcZ = Wire.read() << 8 | Wire.read();
                Wire.read(); Wire.read();
                int16_t GyX = Wire.read() << 8 | Wire.read();
                int16_t GyY = Wire.read() << 8 | Wire.read();
                int16_t GyZ = Wire.read() << 8 | Wire.read();

                imu_raw.acc[0] = ((float)AcX - accel_bias_x) / 16384.0f * 9.806f;
                imu_raw.acc[1] = ((float)AcY - accel_bias_y) / 16384.0f * 9.806f;
                imu_raw.acc[2] = ((float)AcZ - accel_bias_z) / 16384.0f * 9.806f;
                imu_raw.gyro[0] = ((float)GyX - gyro_bias_x) / 131.0f * (PI / 180.0f);
                imu_raw.gyro[1] = ((float)GyY - gyro_bias_y) / 131.0f * (PI / 180.0f);
                imu_raw.gyro[2] = ((float)GyZ - gyro_bias_z) / 131.0f * (PI / 180.0f);
                imu_raw.timestamp = rmw_uros_epoch_nanos();

                xQueueSend(imu_queue, &imu_raw, 0);
            }
            last_imu_time = millis();
        }

        // Lidar collector data
        while (Serial2.available() > 47) {
            std::vector<LidarPoint> newPoints = getPoints();
            if (newPoints.empty()) continue;

            float current_angle = newPoints.back().angle();
            for (auto &p : newPoints) {
                if (p.distance() > 50 && p.distance() < 12000 && current_scan->count < MAX_POINTS) {
                    float angle_rad = (p.angle() / 100.0f) * (PI / 180.0f);
                    float dist_m = p.distance() / 1000.0f;
                    
                    int i = current_scan->count;
                    current_scan->x[i] = dist_m * cos(angle_rad);
                    current_scan->y[i] = dist_m * sin(angle_rad);
                    current_scan->z[i] = 0.0f;
                    current_scan->intensity[i] = (float)p.intensity();
                    current_scan->count++;
                }
            } else {
                // Drain Lidar buffer while waiting
                while(Serial2.available()) Serial2.read(); 
                Serial.println(".");
            }

            if (current_angle < last_angle - 20000) {
                current_scan->timestamp = rmw_uros_epoch_nanos();
                // Push POINTER to queue
                if(xQueueSend(lidar_queue, &current_scan, 0) == pdPASS) { //Check if all good
                    current_scan = new lidar_scan_t(); // Memory for next scan
                    current_scan->count = 0;
                } else {
                    current_scan->count = 0; // Drop data if queue full
                }
            }
            last_angle = current_angle;
        }
        vTaskDelay(pdMS_TO_TICKS(1)); 
    }
}

//Micro ros communication core
void microros_task(void * pvParameters) {
    imu_data_t incoming_imu;
    lidar_scan_t* incoming_lidar = NULL;

    for (;;) {
        switch (state) {
            case WAITING_AGENT:
                if (rmw_uros_ping_agent(100, 1) == RMW_RET_OK) {
                    if (create_entities()) state = AGENT_CONNECTED;
                }
                break;

            case AGENT_CONNECTED:
                if (WiFi.status() != WL_CONNECTED) {
                    state = AGENT_DISCONNECTED;
                    break;
                }

                // 1. Check IMU Queue
                if (xQueueReceive(imu_queue, &incoming_imu, 0) == pdPASS) {
                    imu_msg.header.stamp.sec = incoming_imu.timestamp / 1000000000;
                    imu_msg.header.stamp.nanosec = incoming_imu.timestamp % 1000000000;
                    imu_msg.linear_acceleration.x = incoming_imu.acc[0];
                    imu_msg.linear_acceleration.y = incoming_imu.acc[1];
                    imu_msg.linear_acceleration.z = incoming_imu.acc[2];
                    imu_msg.angular_velocity.x = incoming_imu.gyro[0];
                    imu_msg.angular_velocity.y = incoming_imu.gyro[1];
                    imu_msg.angular_velocity.z = incoming_imu.gyro[2];
                    rcl_publish(&imu_pub, &imu_msg, NULL);
                }

                // 2. Check Lidar Queue (Pointer)
                if (xQueueReceive(lidar_queue, &incoming_lidar, 0) == pdPASS) {
                    cloud.header.stamp.sec = incoming_lidar->timestamp / 1000000000;
                    cloud.header.stamp.nanosec = incoming_lidar->timestamp % 1000000000;
                    cloud.width = incoming_lidar->count;
                    cloud.row_step = cloud.point_step * cloud.width;
                    cloud.data.size = cloud.row_step;

                    for (int i = 0; i < incoming_lidar->count; i++) {
                        int offset = i * 16;
                        memcpy(&cloud_data_buffer[offset], &incoming_lidar->x[i], 4);
                        memcpy(&cloud_data_buffer[offset+4], &incoming_lidar->y[i], 4);
                        memcpy(&cloud_data_buffer[offset+8], &incoming_lidar->z[i], 4);
                        memcpy(&cloud_data_buffer[offset+12], &incoming_lidar->intensity[i], 4);
                    }
                    rcl_publish(&lidar_pub, &cloud, NULL);
                    delete incoming_lidar; // Free memory allocated in Core 1
                }

                rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
                break;

            case AGENT_DISCONNECTED:
                destroy_entities();
                state = WAITING_AGENT;
                break;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

void setup() {
    Serial.begin(115200);
    Serial2.begin(230400, SERIAL_8N1, 16, 17);
    
    set_microros_wifi_transports(WIFI_SSID, WIFI_PASSWORD, AGENT_IP, AGENT_PORT);
    Wire.begin(4, 5);
    Wire.setClock(400000);
    configure_mpu6050();
    calibrate_mpu6050();

    // Motor Pins
    pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT); pinMode(PWMA, OUTPUT);
    pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT); pinMode(PWMB, OUTPUT);
    ledcSetup(channel_A, freq, resolution); ledcAttachPin(PWMA, channel_A);
    ledcSetup(channel_B, freq, resolution); ledcAttachPin(PWMB, channel_B);

    // Create Queues
    imu_queue = xQueueCreate(10, sizeof(imu_data_t)); //Size of queue
    lidar_queue = xQueueCreate(2, sizeof(lidar_scan_t*)); 

    state = WAITING_AGENT;

    // Launch Tasks
    xTaskCreatePinnedToCore(sensor_motor_task, "Sensors", 8192, NULL, 3, NULL, 1);
    xTaskCreatePinnedToCore(microros_task, "microRos", 12288, NULL, 2, NULL, 0);
}

void loop() {
    // Empty - All logic moved to tasks
    vTaskDelete(NULL);
}